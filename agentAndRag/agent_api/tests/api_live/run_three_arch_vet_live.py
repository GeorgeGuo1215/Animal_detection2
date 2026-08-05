from __future__ import annotations

import argparse
import asyncio
import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

import httpx


ROOT = Path(__file__).resolve().parents[3]
REPORT_ROOT = Path(__file__).resolve().parent / "reports"
MODELS = ("agent-plan-solve", "agent-multi-turn", "agent-moe")
QUESTIONS = {
    "flutd": (
        "5岁英短公猫，已绝育。今天频繁进出猫砂盆，每次蹲很久，仅见少量尿团，"
        "食欲下降、精神差并持续舔舐会阴。既往有尿血和疑似膀胱炎，近期家中来客后明显应激。"
        "请以接诊兽医视角总结病例，给出问题列表、鉴别诊断、检查与紧急处理优先级。"
        "请使用英语检索本地兽医知识库，并联网核对当前仍适用的猫下泌尿道疾病或尿道梗阻证据；"
        "最终只引用实际检索结果。"
    ),
    "bulldog": (
        "3岁法国斗牛犬公犬，已绝育，12 kg。主人计划每天剧烈跑步或追球1小时，"
        "但近期气温较高时散步即明显张口喘、不愿继续走，休息后缓解。"
        "请以兽医视角评估短吻犬气道与热损伤风险、需要排查的问题，并制定安全运动边界。"
        "请使用英语检索本地兽医知识库，并联网核对当前短吻犬运动和热风险证据；"
        "最终只引用实际检索结果。"
    ),
}


def load_dotenv() -> None:
    path = ROOT / ".env"
    if not path.exists():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def default_api_key() -> str:
    path = ROOT / "agent_api" / "keys.txt"
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            value = line.strip()
            if value and not value.startswith("#"):
                return value
    return os.getenv("AGENT_API_KEY", "")


def tool_records(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    for event in events:
        status = event.get("status")
        detail = event.get("detail") or {}
        if status == "tool_complete" and detail.get("tool_name"):
            records.append({
                "expert": detail.get("expert"),
                "tool_name": detail.get("tool_name"),
                "arguments": detail.get("arguments"),
                "result": detail.get("result"),
                "latency_ms": detail.get("latency_ms"),
                "hits_count": detail.get("hits_count"),
                "round": detail.get("round"),
            })
        opinion = detail.get("opinion") if status == "expert_complete" else None
        if isinstance(opinion, dict):
            for item in opinion.get("tool_results") or []:
                if isinstance(item, dict):
                    records.append({"expert": opinion.get("expert"), **item})
    return records


def llm_outputs(events: List[Dict[str, Any]], answer: str) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    for event in events:
        status = event.get("status")
        detail = event.get("detail") or {}
        if status == "plan_complete":
            records.append({"stage": "planner", "output": detail.get("plan")})
        elif status == "decision_complete":
            records.append({
                "stage": f"decision:round:{detail.get('round')}",
                "output": detail.get("output"),
            })
        elif status == "llm_trace":
            records.extend(detail.get("calls") or [])
    if not any(record.get("stage") == "aggregator" for record in records):
        records.append({"stage": "final_generation", "output": answer})
    return records


async def run_case(
    client: httpx.AsyncClient,
    *,
    base_url: str,
    api_key: str,
    case_id: str,
    question: str,
    model: str,
    max_tokens: int,
) -> Dict[str, Any]:
    headers = {"Authorization": f"Bearer {api_key}", "Accept": "text/event-stream"}
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": question}],
        "stream": True,
        "temperature": 0.2,
        "max_tokens": max_tokens,
        "user_role": "veterinarian",
        "debug_timing": True,
    }
    started = time.perf_counter()
    events: List[Dict[str, Any]] = []
    answer: List[str] = []
    finish_reason = None
    async with client.stream(
        "POST", f"{base_url.rstrip('/')}/v1/chat/completions", headers=headers, json=payload,
    ) as response:
        response.raise_for_status()
        async for line in response.aiter_lines():
            if not line.startswith("data: "):
                continue
            data = line[6:]
            if data == "[DONE]":
                break
            chunk = json.loads(data)
            elapsed_ms = round((time.perf_counter() - started) * 1000, 1)
            choice = (chunk.get("choices") or [{}])[0]
            delta = choice.get("delta") or {}
            status = chunk.get("agent_status")
            content = delta.get("content") or ""
            if status == "streaming" and content:
                answer.append(content)
            if choice.get("finish_reason"):
                finish_reason = choice["finish_reason"]
            events.append({
                "elapsed_ms": elapsed_ms,
                "status": status,
                "detail": chunk.get("agent_detail") or {},
                "content": content,
                "finish_reason": choice.get("finish_reason"),
            })
    elapsed_ms = round((time.perf_counter() - started) * 1000, 1)
    final_answer = "".join(answer)
    return {
        "case_id": case_id,
        "model": model,
        "question": question,
        "answer": final_answer,
        "finish_reason": finish_reason,
        "elapsed_ms": elapsed_ms,
        "events": events,
        "tools": tool_records(events),
        "llm_outputs": llm_outputs(events, final_answer),
    }


def render_markdown(results: List[Dict[str, Any]]) -> str:
    lines = [
        "# Three-Architecture Veterinary Live Test",
        "",
        "| Case | Model | Duration | Finish | Tools | Answer chars |",
        "|---|---|---:|---|---|---:|",
    ]
    for result in results:
        tools = ", ".join(dict.fromkeys(
            str(item.get("tool_name") or "") for item in result["tools"] if item.get("tool_name")
        )) or "none"
        lines.append(
            f"| {result['case_id']} | {result['model']} | {result['elapsed_ms']:.1f} ms | "
            f"{result['finish_reason']} | {tools} | {len(result['answer'])} |"
        )
    for result in results:
        lines.extend([
            "",
            f"## {result['case_id']} / {result['model']}",
            "",
            "### Timeline",
            "",
        ])
        for event in result["events"]:
            if event["status"] and event["status"] != "streaming":
                lines.append(
                    f"- `{event['elapsed_ms']:.1f} ms` `{event['status']}` "
                    f"`{json.dumps(event['detail'], ensure_ascii=False, default=str)[:1200]}`"
                )
        lines.extend(["", "### Tool Records", "", "```json"])
        lines.append(json.dumps(result["tools"], ensure_ascii=False, indent=2, default=str))
        lines.extend(["```", "", "### LLM Outputs", "", "```json"])
        lines.append(json.dumps(result["llm_outputs"], ensure_ascii=False, indent=2, default=str))
        lines.extend(["```", "", "### Answer", "", result["answer"] or "(empty)"])
    return "\n".join(lines) + "\n"


async def async_main(args: argparse.Namespace) -> int:
    load_dotenv()
    api_key = args.api_key or default_api_key()
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    report_dir = REPORT_ROOT / f"three_arch_vet_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    report_dir.mkdir(parents=True, exist_ok=True)
    results: List[Dict[str, Any]] = []
    timeout = httpx.Timeout(connect=20, read=args.timeout, write=30, pool=30)
    async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
        selected_cases = {args.case: QUESTIONS[args.case]} if args.case else QUESTIONS
        selected_models = (args.model,) if args.model else MODELS
        for case_id, question in selected_cases.items():
            for model in selected_models:
                print(f"[run] {case_id} / {model}", flush=True)
                result = await run_case(
                    client, base_url=args.base_url, api_key=api_key,
                    case_id=case_id, question=question, model=model,
                    max_tokens=args.max_tokens,
                )
                results.append(result)
                print(
                    f"[done] {result['elapsed_ms']:.1f} ms finish={result['finish_reason']} "
                    f"tools={len(result['tools'])} chars={len(result['answer'])}",
                    flush=True,
                )
    (report_dir / "results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2, default=str), encoding="utf-8",
    )
    (report_dir / "REPORT.md").write_text(render_markdown(results), encoding="utf-8")
    print(f"[report] {report_dir}")
    return 0 if all(result["answer"] and result["finish_reason"] for result in results) else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--api-key", default="")
    parser.add_argument("--max-tokens", type=int, default=2500)
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--case", choices=tuple(QUESTIONS))
    parser.add_argument("--model", choices=MODELS)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(async_main(parse_args())))
