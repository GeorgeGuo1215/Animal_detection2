"""Real-API A/B for stable MoE prompts and DeepSeek prompt-cache metrics.

The test isolates the final synthesis stage: both variants receive identical
D1-D8 intent contracts and fixed evidence.  ``legacy`` restores the former
calendar value in the long system prefix; ``optimized`` keeps the system
prefix stable and only appends ``as_of_date`` to current-Web user payloads.
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import sys
import time
from datetime import date, datetime, timedelta
from pathlib import Path
from statistics import mean
from typing import Any


HERE = Path(__file__).resolve().parent
AGENT_API = HERE.parents[1]
ROOT = HERE.parents[2]
for path in (str(AGENT_API), str(ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)


def load_dotenv() -> None:
    path = ROOT / ".env"
    if not path.exists():
        return
    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


load_dotenv()
os.environ.setdefault("HTTPX_TRUST_ENV", "0")

from agent_api.app.integrations.llm.client import OpenAIChatClient, extract_text  # noqa: E402
from agent_api.app.prompts.intent_contracts import get_intent_spec  # noqa: E402
from agent_api.app.services.moe.critic import CriticResult  # noqa: E402
from agent_api.app.services.moe.orchestration.service import MoEOrchestrator, OrchestratorConfig  # noqa: E402
from agent_api.app.services.moe.retrieval_policy import EvidenceTask  # noqa: E402
from agent_api.app.services.moe.router import RouterDecision  # noqa: E402
from agent_api.app.services.moe.task_policy import IntentDecision, TaskPolicyDecision  # noqa: E402
from agent_api.app.services.moe.trace import extract_usage  # noqa: E402


FIXTURE = HERE / "fixtures" / "evidence_architecture_100.json"


def selected_cases() -> list[dict[str, Any]]:
    data = json.loads(FIXTURE.read_text(encoding="utf-8"))
    rows = data.get("items", data) if isinstance(data, dict) else data
    selected: dict[str, dict[str, Any]] = {}
    for row in rows:
        intent = str(row.get("expected_intent") or "")
        if intent and intent not in selected:
            selected[intent] = row
    if set(selected) != {f"D{i}" for i in range(1, 9)}:
        raise RuntimeError("fixture does not cover D1-D8")
    return [selected[f"D{i}"] for i in range(1, 9)]


def decision() -> RouterDecision:
    return RouterDecision(
        scores={"clinical": 8}, raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0}, selected_experts=["clinical"],
        emergency=False, out_of_scope=False, reason="controlled A/B",
    )


def messages_for(case: dict[str, Any], variant: str) -> list[dict[str, str]]:
    intent_id = str(case["expected_intent"])
    output_variant = str(case.get("output_variant") or "default")
    evidence = str(case["evidence_excerpt"])
    task = EvidenceTask(
        capability="current_web" if intent_id == "D6" else "local_knowledge",
        owner="clinical", requirement="required", reason="controlled evidence",
        query=str((case.get("suggested_queries") or [case["question"]])[0]),
    )
    policy = TaskPolicyDecision(
        primary_intent=intent_id, secondary_intents=tuple(case.get("secondary_intents") or ()),
        confidence=0.95, output_variant=output_variant, scores={"clinical": 8.0},
        emergency=intent_id == "D8", emergency_confidence=0.9,
        emergency_evidence=("受控高风险证据",) if intent_id == "D8" else (),
        evidence_tasks=(task,), missing_information=(), reason="controlled A/B",
    )
    orchestrator = MoEOrchestrator(config=OrchestratorConfig(
        user_role="veterinarian", max_tokens=1200,
    ))
    orchestrator._active_task_policy = policy
    orchestrator._active_intent_decision = IntentDecision(
        intent_id=intent_id, name=get_intent_spec(intent_id).name, confidence=0.95,
        output_variant=output_variant, reason="controlled A/B",
    )
    orchestrator._active_evidence_tasks = (task,)
    messages = orchestrator._build_synthesis_messages(
        query=str(case["question"]),
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": evidence, "evidence": [evidence], "risks": ["不得超出所给证据"],
            "confidence": 0.95, "plan_steps": [],
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision(),
    )
    if variant == "legacy":
        today = date.today().isoformat()
        messages[0]["content"] = messages[0]["content"].replace(
            "AI 临床助手（不是兽医同事、也不扮演真人医生）。\n",
            f"AI 临床助手（不是兽医同事、也不扮演真人医生）。今天是 {today}。\n",
            1,
        )
        payload = json.loads(messages[1]["content"])
        payload.pop("as_of_date", None)
        messages[1]["content"] = json.dumps(payload, ensure_ascii=False)
    return messages


async def complete(client: OpenAIChatClient, messages: list[dict[str, str]], max_tokens: int) -> dict[str, Any]:
    started = time.perf_counter()
    response = await client.chat(
        messages=messages, temperature=0.0, max_tokens=max_tokens, thinking=False,
    )
    return {
        "text": extract_text(response),
        "usage": extract_usage(response),
        "latency_ms": round((time.perf_counter() - started) * 1000, 1),
    }


def section_score(answer: str, sections: list[str]) -> float:
    if not sections:
        return 1.0
    return sum(section in answer for section in sections) / len(sections)


async def judge_batch(client: OpenAIChatClient, rows: list[dict[str, Any]]) -> dict[str, dict[str, float]]:
    payload = [{
        "id": row["id"], "question": row["question"], "reference_evidence": row["evidence"],
        "required_sections": row["required_sections"],
        "legacy_answer": row["legacy"]["text"], "optimized_answer": row["optimized"]["text"],
    } for row in rows]
    response = await client.chat(
        messages=[{
            "role": "system",
            "content": (
                "你是盲评兽医回答质量审计员。只依据题目和 reference_evidence，分别给 legacy/optimized "
                "的 accuracy 与 reliability 打0-100分。accuracy衡量证据事实保持且无矛盾；reliability衡量"
                "不编造、区分事实/推断、证据不足时收缩。不得因文风偏好评分。只输出严格JSON对象："
                "{case_id:{legacy:{accuracy:0,reliability:0},optimized:{accuracy:0,reliability:0}}}。"
            ),
        }, {"role": "user", "content": json.dumps(payload, ensure_ascii=False)}],
        temperature=0.0, max_tokens=1200, response_format={"type": "json_object"}, thinking=False,
    )
    text = extract_text(response)
    text = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip())
    return json.loads(text)


def with_legacy_date(system: str, value: str) -> str:
    return system.replace(
        "AI 临床助手（不是兽医同事、也不扮演真人医生）。\n",
        f"AI 临床助手（不是兽医同事、也不扮演真人医生）。今天是 {value}。\n",
        1,
    )


async def cache_probe(client: OpenAIChatClient) -> list[dict[str, Any]]:
    base = messages_for(selected_cases()[5], "optimized")[0]["content"]
    namespace = f"PROMPT_CACHE_AB_{datetime.now().strftime('%Y%m%d%H%M%S')}"
    day1 = (date.today() - timedelta(days=1)).isoformat()
    day2 = date.today().isoformat()
    probes = [
        ("legacy_day1_warm", namespace + "_LEGACY\n" + with_legacy_date(base, day1)),
        ("legacy_day1_repeat", namespace + "_LEGACY\n" + with_legacy_date(base, day1)),
        ("legacy_day2_rollover", namespace + "_LEGACY\n" + with_legacy_date(base, day2)),
        ("optimized_warm", namespace + "_OPTIMIZED\n" + base),
        ("optimized_repeat", namespace + "_OPTIMIZED\n" + base),
    ]
    output = []
    for name, system in probes:
        result = await complete(client, [
            {"role": "system", "content": system},
            {"role": "user", "content": "缓存测试，只回答：收到。"},
        ], 16)
        output.append({"name": name, **result["usage"], "latency_ms": result["latency_ms"]})
        await asyncio.sleep(3)
    return output


async def main() -> None:
    output_dir = HERE / "reports" / "prompt_cache_ab_20260823"
    output_dir.mkdir(parents=True, exist_ok=True)
    client = OpenAIChatClient()
    rows: list[dict[str, Any]] = []
    try:
        for case in selected_cases():
            row = {
                "id": case["id"], "intent": case["expected_intent"],
                "question": case["question"], "evidence": case["evidence_excerpt"],
                "required_sections": case["required_sections"],
            }
            for variant in ("legacy", "optimized"):
                row[variant] = await complete(client, messages_for(case, variant), 1200)
                row[variant]["section_score"] = section_score(
                    row[variant]["text"], row["required_sections"],
                )
            rows.append(row)

        judgments: dict[str, dict[str, Any]] = {}
        for start in range(0, len(rows), 4):
            judgments.update(await judge_batch(client, rows[start:start + 4]))
        for row in rows:
            row["judge"] = judgments[row["id"]]

        probes = await cache_probe(client)
    finally:
        await client.close()

    def avg(variant: str, metric: str) -> float:
        return mean(float(row["judge"][variant][metric]) for row in rows)

    summary = {
        variant: {
            "accuracy": round(avg(variant, "accuracy"), 2),
            "reliability": round(avg(variant, "reliability"), 2),
            "section_score": round(mean(row[variant]["section_score"] for row in rows), 4),
            "mean_latency_ms": round(mean(row[variant]["latency_ms"] for row in rows), 1),
            "prompt_cache_hit_tokens": sum(row[variant]["usage"]["prompt_cache_hit_tokens"] for row in rows),
            "prompt_cache_miss_tokens": sum(row[variant]["usage"]["prompt_cache_miss_tokens"] for row in rows),
        } for variant in ("legacy", "optimized")
    }
    summary["accept"] = bool(
        summary["optimized"]["accuracy"] >= summary["legacy"]["accuracy"] - 2
        and summary["optimized"]["reliability"] >= summary["legacy"]["reliability"] - 2
        and summary["optimized"]["section_score"] >= summary["legacy"]["section_score"]
    )
    result = {"summary": summary, "cache_probe": probes, "cases": rows}
    (output_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# Prompt Cache A/B（真实 DeepSeek API）", "",
        "| 版本 | 准确率 | 可靠性 | 结构覆盖 | 平均耗时(ms) | A/B命中token | A/B未命中token |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for variant in ("legacy", "optimized"):
        item = summary[variant]
        lines.append(
            f"| {variant} | {item['accuracy']} | {item['reliability']} | {item['section_score']:.2%} | "
            f"{item['mean_latency_ms']} | {item['prompt_cache_hit_tokens']} | {item['prompt_cache_miss_tokens']} |"
        )
    lines.extend(["", f"- 接受候选版本：`{summary['accept']}`", "", "## 跨日缓存探针", "",
                  "| 探针 | hit | miss | hit rate | latency(ms) |", "| --- | ---: | ---: | ---: | ---: |"])
    for item in probes:
        total = item["prompt_cache_hit_tokens"] + item["prompt_cache_miss_tokens"]
        rate = item["prompt_cache_hit_tokens"] / total if total else 0
        lines.append(f"| {item['name']} | {item['prompt_cache_hit_tokens']} | {item['prompt_cache_miss_tokens']} | {rate:.2%} | {item['latency_ms']} |")
    lines.extend(["", "## 分题", ""])
    for row in rows:
        lines.append(
            f"- {row['intent']} {row['id']}：legacy A/R={row['judge']['legacy']['accuracy']}/"
            f"{row['judge']['legacy']['reliability']}，optimized A/R={row['judge']['optimized']['accuracy']}/"
            f"{row['judge']['optimized']['reliability']}，结构={row['legacy']['section_score']:.0%}/"
            f"{row['optimized']['section_score']:.0%}"
        )
    (output_dir / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"output_dir": str(output_dir), "summary": summary, "cache_probe": probes}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    asyncio.run(main())
