"""兽医视角 MoE：对已启动的后端发 HTTP 请求，落盘 SSE 事件与终答。

用法：
    # 先启动服务（agentAndRag 目录）
    #   python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000
    # 或 start_agent.bat

    python tests/moe/run_moe_vet_live.py
    python tests/moe/run_moe_vet_live.py --base-url http://127.0.0.1:8000 --case bulldog

输出：tests/moe/reports/vet_live_<timestamp>/
  - case_*.md（请求、SSE 进度、终答、风格验收）
  - case_*_answer.txt
  - SUMMARY.md
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import httpx

_THIS = Path(__file__).resolve()
_AGENT_API = _THIS.parents[2]
_AGENTANDRAG = _THIS.parents[3]
for _p in (str(_AGENT_API), str(_AGENTANDRAG)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_dotenv() -> None:
    """从项目 .env 读取环境变量（不覆盖已有值）。"""
    env_path = _AGENTANDRAG / ".env"
    if not env_path.exists():
        return
    try:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k, v = k.strip(), v.strip().strip('"').strip("'")
            if k and k not in os.environ:
                os.environ[k] = v
    except Exception:  # noqa: BLE001
        pass


_load_dotenv()
os.environ.setdefault("HTTPX_TRUST_ENV", "0")

_FLUTD = (
    "5 岁的英短，公猫，已经绝育。今天从早上开始就一直往猫砂盆跑，差不多十几分钟去一次，"
    "每次蹲很久，但是宠主看不出来到底有没有尿出来。猫砂盆里好像只有一点点尿团，比平时少很多。"
    "它今天不太爱吃东西，平时早上会主动来要罐头，今天只舔了几口。精神也差一些，老是趴着，还会舔下面。"
    "没有明显呕吐，但是刚才好像干呕了一下。昨天晚上还挺正常的。最近没有换粮，喝水感觉和平时差不多。"
    "宠主家里最近来了客人，它有点紧张，躲了两天。疫苗应该是去年打的，驱虫不太记得。"
    "之前有过一次尿血，可能是膀胱炎，吃药后好了。今天还没有去医院，也没有吃药。"
)

_IMHA = (
    "6 岁英国可卡犬，雌性，已绝育，体重 12.8 kg。近 3 天进行性精神沉郁、食欲下降，"
    "今天出现呼吸加快和明显乏力。体检：T 39.2°C，HR 156 次/分，RR 44 次/分，"
    "黏膜苍白伴轻度黄染，CRT 1.5 秒，未见明显外出血。CBC：HCT 16%，Hb 5.4 g/dL，"
    "网织红细胞升高，球形红细胞 2+，盐水凝集试验阳性；血小板 152×10^9/L。"
    "生化：总胆红素 48 μmol/L，ALT 轻度升高，肌酐正常。胸腹部快速超声未见明显出血，"
    "尚未输血或使用糖皮质激素。"
)

CASES: List[Dict[str, Any]] = [
    {
        "id": "flutd_diagnosis",
        "title": "英短病例 + 请做出诊断（应出病例工作流）",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "检查与治疗方案", "风险提示"],
        "question": _FLUTD + "\n请对以上病例做出诊断。",
    },
    {
        "id": "flutd_organize",
        "title": "英短病例 + 请整理病例格式（应出病例工作流）",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "风险提示"],
        "question": _FLUTD + "\n请对以上病例做出整理，输出统一的病例格式。",
    },
    {
        "id": "flutd_latest_evidence",
        "title": "英短尿闭风险 + 本地知识库与最新网络指南交叉核对",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "检查与治疗方案", "风险提示"],
        "question": (
            _FLUTD
            + "\n请总结病例并给出鉴别诊断、检查与紧急处置优先级。"
            "请让相关专家分别检索本地兽医知识库，并联网核对 2025-2026 年仍适用的猫下泌尿道疾病/尿道梗阻指南；"
            "最终只引用实际检索到的来源，明确区分本地知识证据与最新网络证据。"
        ),
    },
    {
        "id": "canine_imha_latest_evidence",
        "title": "犬疑似 IMHA + 输血、免疫抑制与血栓预防证据核对",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "检查与治疗方案", "风险提示"],
        "question": (
            _IMHA
            + "\n请以接诊兽医会诊形式总结病例，给出问题列表、鉴别与确诊路径，"
            "并排序输血、免疫抑制、血栓预防和监测的紧急处置。"
            "请让相关专家分别使用英语检索本地兽医知识库，并联网核对 2024-2026 年仍适用的犬 IMHA 共识或指南；"
            "最终只引用实际检索结果，区分本地知识证据和网络证据。"
        ),
    },
    {
        "id": "bulldog",
        "title": "法斗运动评估（不应强制病例三件套）",
        "expect_concrete": False,
        "forbid_required_sections": True,
        "expect_keywords": ["短吻", "热", "运动", "呼吸"],
        "question": (
            "3岁法斗（法国斗牛犬）公犬，已绝育，体重12kg。"
            "主人想每天带它剧烈跑步或追球1小时，最近热天遛弯就张口喘、不愿走。"
            "从兽医角度评估运动建议与风险边界，并说明与普通中型犬的差异。"
        ),
    },
    {
        "id": "vague",
        "title": "笼统对照：猫尿血怎么办",
        "expect_concrete": False,
        "forbid_required_sections": True,
        "question": "猫尿血怎么办",
    },
]


def _default_api_key() -> str:
    """解析默认 API 密钥。"""
    keys_path = _AGENT_API / "keys.txt"
    if keys_path.exists():
        for line in keys_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                return line
    return os.getenv("AGENT_API_KEY") or "sk-petmind-default-key-2026"


def _style_checks(case: Dict[str, Any], answer: str) -> List[str]:
    """检查回答文风是否符合约定。"""
    issues: List[str] = []
    for sec in case.get("expect_sections") or []:
        if sec not in answer:
            issues.append(f"终答缺少分节关键词: {sec}")
    if case.get("forbid_required_sections"):
        if all(s in answer for s in ("病例整理", "问题列表", "检查与治疗方案")):
            issues.append("非诊断/整理意图不应强制完整病例三件套")
    kws = case.get("expect_keywords") or []
    if kws and not any(k in answer for k in kws):
        issues.append(f"终答未命中任一特异化关键词: {kws}")
    return issues


def _parse_sse_block(block: str) -> Optional[Dict[str, Any]]:
    """解析一块 SSE 数据。"""
    data_lines = []
    for line in block.splitlines():
        if line.startswith("data:"):
            data_lines.append(line[5:].lstrip())
    if not data_lines:
        return None
    payload = "\n".join(data_lines).strip()
    if not payload or payload == "[DONE]":
        return {"done": True}
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return {"raw": payload}


def _chat_stream(
    *,
    base_url: str,
    api_key: str,
    question: str,
    max_tokens: int,
    timeout_s: float,
) -> Tuple[str, List[Dict[str, Any]]]:
    """发起聊天流式请求并收集事件。"""
    url = base_url.rstrip("/") + "/v1/chat/completions"
    body = {
        "model": "agent-moe",
        "stream": True,
        "temperature": 0.3,
        "max_tokens": max_tokens,
        "user_role": "veterinarian",
        "messages": [{"role": "user", "content": question}],
    }
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
    }
    events: List[Dict[str, Any]] = []
    answer_parts: List[str] = []
    with httpx.Client(timeout=httpx.Timeout(connect=10, read=timeout_s, write=30, pool=30), trust_env=False) as client:
        with client.stream("POST", url, headers=headers, json=body) as resp:
            if resp.status_code >= 400:
                err = resp.read().decode("utf-8", errors="replace")
                raise RuntimeError(f"HTTP {resp.status_code}: {err[:800]}")
            buf = ""
            for chunk in resp.iter_text():
                buf += chunk
                while "\n\n" in buf:
                    block, buf = buf.split("\n\n", 1)
                    obj = _parse_sse_block(block)
                    if not obj:
                        continue
                    if obj.get("done"):
                        events.append({"type": "done"})
                        continue
                    events.append(obj)
                    # OpenAI chunk: content under streaming status; also collect any delta content
                    status = obj.get("agent_status")
                    detail = obj.get("agent_detail") or {}
                    choices = obj.get("choices") or []
                    content = ""
                    if choices:
                        content = ((choices[0].get("delta") or {}).get("content")) or ""
                    if status == "streaming" and content:
                        answer_parts.append(content)
                    elif status and status != "streaming":
                        # keep progress breadcrumbs in events only
                        _ = detail
    return "".join(answer_parts), events


def _expert_opinions(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从追踪记录取出专家意见。"""
    opinions: List[Dict[str, Any]] = []
    seen = set()
    for event in events:
        if event.get("agent_status") != "expert_complete":
            continue
        detail = event.get("agent_detail") or {}
        opinion = detail.get("opinion")
        if not isinstance(opinion, dict):
            continue
        expert = str(opinion.get("expert") or detail.get("expert") or "")
        if expert in seen:
            continue
        seen.add(expert)
        opinions.append(opinion)
    return opinions


def _web_search_records(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从追踪记录取出网页检索记录。"""
    records: List[Dict[str, Any]] = []
    for opinion in _expert_opinions(events):
        for tool_result in opinion.get("tool_results") or []:
            if not isinstance(tool_result, dict):
                continue
            tool_name = str(tool_result.get("tool_name") or "")
            if not tool_name.startswith("mcp.web_search"):
                continue
            records.append({
                "expert": opinion.get("expert"),
                "expert_name": opinion.get("name_zh"),
                **tool_result,
            })
    return records


def _web_result_items(record: Dict[str, Any]) -> List[Dict[str, Any]]:
    """抽出网页检索结果条目。"""
    result = record.get("result")
    if not isinstance(result, dict):
        return []
    items = result.get("results") or result.get("hits") or []
    return [item for item in items if isinstance(item, dict)] if isinstance(items, list) else []


def _render_web_search_section(records: List[Dict[str, Any]]) -> List[str]:
    """渲染报告里的网页检索章节。"""
    lines = ["## Web Search 结果", ""]
    if not records:
        lines.append("_本次没有记录到 Web Search 调用。_")
        lines.append("")
        return lines
    for index, record in enumerate(records, start=1):
        arguments = record.get("arguments") if isinstance(record.get("arguments"), dict) else {}
        lines.append(f"### {index}. {record.get('expert_name') or record.get('expert') or 'expert'}")
        lines.append("")
        lines.append(f"- query: `{arguments.get('query') or ''}`")
        lines.append(f"- ok: `{bool(record.get('ok'))}`")
        lines.append(f"- latency_ms: `{record.get('latency_ms') or 0}`")
        if record.get("error"):
            lines.append(f"- error: `{record.get('error')}`")
        lines.append("")
        items = _web_result_items(record)
        if not items:
            lines.append("_无结果。_")
            lines.append("")
            continue
        for item_index, item in enumerate(items, start=1):
            title = str(item.get("title") or item.get("url") or f"result-{item_index}")
            url = str(item.get("url") or "")
            snippet = str(item.get("content") or item.get("snippet") or item.get("text") or "")
            snippet = " ".join(snippet.split())[:500]
            lines.append(f"{item_index}. [{title}]({url})" if url else f"{item_index}. {title}")
            if snippet:
                lines.append(f"   - {snippet}")
        lines.append("")
    return lines


def _render_report(
    case: Dict[str, Any],
    *,
    question: str,
    answer: str,
    events: List[Dict[str, Any]],
    issues: List[str],
    base_url: str,
) -> str:
    """把本次运行结果写成报告。"""
    lines: List[str] = []
    lines.append("# Vet MoE HTTP Live Report")
    lines.append("")
    lines.append(f"- 时间: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- case_id: {case['id']}")
    lines.append(f"- title: {case['title']}")
    lines.append(f"- base_url: {base_url}")
    lines.append(f"- user_role: veterinarian")
    lines.append("- legacy_concrete_gate: removed; classification now uses the D1-D8 LLM module")
    lines.append("")
    lines.append("## 请求问题")
    lines.append("")
    lines.append("```text")
    lines.append(question)
    lines.append("```")
    lines.append("")
    opinions = _expert_opinions(events)
    lines.append("## 专家循环摘要")
    lines.append("")
    if opinions:
        lines.append("| expert | rounds | confidence | required | recommended | attempted | pending | final conclusion 摘要 |")
        lines.append("| --- | ---: | ---: | --- | --- | --- | --- | --- |")
        for opinion in opinions:
            conclusion = " ".join(str(opinion.get("conclusion") or "").split())[:180]
            conclusion = conclusion.replace("|", "\\|")
            required = ", ".join(opinion.get("required_tools") or [])
            recommended = ", ".join(opinion.get("recommended_tools") or [])
            attempted = ", ".join(opinion.get("attempted_tools") or [])
            pending = ", ".join(opinion.get("pending_tools") or [])
            lines.append(
                f"| {opinion.get('name_zh') or opinion.get('expert')} | {opinion.get('rounds') or 0} | "
                f"{opinion.get('confidence') or 0} | {required} | {recommended} | {attempted} | {pending} | {conclusion} |"
            )
    else:
        lines.append("_未记录到专家完成事件。_")
    lines.append("")
    lines.extend(_render_web_search_section(_web_search_records(events)))
    lines.append("## SSE 进度摘要")
    lines.append("")
    lines.append("| # | agent_status | detail 摘要 |")
    lines.append("| --- | --- | --- |")
    for i, ev in enumerate(events, 1):
        if ev.get("type") == "done":
            lines.append(f"| {i} | done | |")
            continue
        st = ev.get("agent_status") or ""
        detail = ev.get("agent_detail") or {}
        summary = json.dumps(detail, ensure_ascii=False)
        if len(summary) > 160:
            summary = summary[:160] + "…"
        summary = summary.replace("|", "\\|")
        lines.append(f"| {i} | {st} | {summary} |")
    lines.append("")
    lines.append("## 最终答案")
    lines.append("")
    lines.append(answer or "_（空）_")
    lines.append("")
    lines.append("## 风格验收")
    lines.append("")
    if issues:
        lines.extend(f"- FAIL: {x}" for x in issues)
    else:
        lines.append("- PASS")
    lines.append("")
    lines.append("## 原始 SSE 事件（截断保存）")
    lines.append("")
    lines.append("```json")
    # Drop huge content deltas; keep status events + short streaming markers
    slim: List[Any] = []
    for ev in events:
        if ev.get("type") == "done":
            slim.append(ev)
            continue
        st = ev.get("agent_status")
        if st == "streaming":
            continue
        slim.append({
            "agent_status": st,
            "agent_detail": ev.get("agent_detail"),
            "finish_reason": ((ev.get("choices") or [{}])[0].get("finish_reason")),
        })
    lines.append(json.dumps(slim, ensure_ascii=False, indent=2)[:20000])
    lines.append("```")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    """解析命令行参数并返回配置。"""
    p = argparse.ArgumentParser(description="兽医 MoE：HTTP 请求已启动后端")
    p.add_argument("--base-url", default=os.getenv("AGENT_BASE_URL") or "http://127.0.0.1:8000")
    p.add_argument("--api-key", default=None, help="默认读 agent_api/keys.txt")
    p.add_argument("--case", action="append", help="只跑指定 case id；可多次")
    p.add_argument("--max-tokens", type=int, default=1500)
    p.add_argument("--timeout", type=float, default=300.0)
    p.add_argument("--out-dir", default=None)
    return p.parse_args()


def main() -> None:
    """脚本入口，解析参数并执行主流程。"""
    args = parse_args()
    api_key = args.api_key or _default_api_key()
    base = args.base_url.rstrip("/")

    # health check
    try:
        with httpx.Client(timeout=5.0, trust_env=False) as c:
            r = c.get(base + "/health")
            r.raise_for_status()
    except Exception as exc:  # noqa: BLE001
        print(f"[错误] 无法连接 {base}/health：{exc}")
        print("请先启动：python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000")
        raise SystemExit(2)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) if args.out_dir else (_THIS.parent / "reports" / f"vet_live_{ts}")
    out_dir.mkdir(parents=True, exist_ok=True)

    selected = CASES
    if args.case:
        ids = set(args.case)
        selected = [c for c in CASES if c["id"] in ids]
        if not selected:
            print(f"[错误] 无匹配 case: {args.case}")
            raise SystemExit(2)

    summary: List[str] = []
    all_ok = True
    for case in selected:
        print(f"\n=== RUN {case['id']}: {case['title']} ===")
        q = case["question"]
        try:
            answer, events = _chat_stream(
                base_url=base,
                api_key=api_key,
                question=q,
                max_tokens=args.max_tokens,
                timeout_s=args.timeout,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] 请求异常: {exc}")
            all_ok = False
            summary.append(f"- {case['id']}: ERROR ({exc})")
            continue
        issues = _style_checks(case, answer)
        md = _render_report(case, question=q, answer=answer, events=events, issues=issues, base_url=base)
        out_path = out_dir / f"case_{case['id']}.md"
        out_path.write_text(md, encoding="utf-8")
        (out_dir / f"case_{case['id']}_answer.txt").write_text(answer or "", encoding="utf-8")
        web_records = _web_search_records(events)
        (out_dir / f"case_{case['id']}_websearch.json").write_text(
            json.dumps(web_records, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        ok = not issues
        all_ok = all_ok and ok
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] -> {out_path}")
        for iss in issues:
            print(f"  - {iss}")
        summary.append(f"- {case['id']}: {status} ({out_path.name})")

    (out_dir / "SUMMARY.md").write_text("# Vet MoE Live Summary\n\n" + "\n".join(summary) + "\n", encoding="utf-8")
    print(f"\n报告目录: {out_dir}")
    raise SystemExit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
