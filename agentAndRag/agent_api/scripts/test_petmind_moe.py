#!/usr/bin/env python3
"""PetMind 运行与 MoE 路径探测脚本。

对已启动的 PetMind Agent API 做健康检查，并通过 `/v1/chat/completions`（SSE）
判断是否走了 MoE 编排（routing / expert_* / reviewing），可与 plan-and-solve 对照。

参考：`tests/moe/run_moe_live.py`（进程内 MoE）、`tests/api_live/run_api_live.py`（HTTP 全量）。

用法（在 agentAndRag 目录或任意路径）：
    python agent_api/scripts/test_petmind_moe.py
    python agent_api/scripts/test_petmind_moe.py --base http://127.0.0.1:8002 -q "犬细小病毒有哪些典型症状？"
    python agent_api/scripts/test_petmind_moe.py --compare-plan   # 同题对比 MoE vs Plan

环境变量：
    AGENT_API_BASE   默认 http://127.0.0.1:8002
    AGENT_API_KEY    可选（服务端 AGENT_DISABLE_AUTH=1 时可省略）
    OPENAI_API_KEY   聊天需 LLM；也可写在 agentAndRag/.env
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

try:
    import httpx
except ImportError:
    print("请先安装: pip install httpx", file=sys.stderr)
    raise

_THIS = Path(__file__).resolve()
_AGENT_API = _THIS.parents[1]
_AGENTANDRAG = _THIS.parents[2]

MOE_STATUSES = frozenset({"routing", "expert_calling", "expert_complete", "reviewing"})
PLAN_STATUSES = frozenset({"planning", "plan_complete"})
MULTI_STATUSES = frozenset({"thinking", "tool_calling", "tool_complete", "decided_final"})


def _load_dotenv() -> None:
    env_path = _AGENTANDRAG / ".env"
    if not env_path.exists():
        return
    for line in env_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        k, v = k.strip(), v.strip().strip('"').strip("'")
        if k and k not in os.environ:
            os.environ[k] = v


def _default_api_key() -> str:
    if os.getenv("AGENT_API_KEY"):
        return os.getenv("AGENT_API_KEY", "")
    keys_file = _AGENTANDRAG / "agent_api" / "keys.txt"
    if keys_file.is_file():
        for line in keys_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                return line
    return ""


@dataclass
class ChatProbe:
    model: str
    question: str
    answer: str = ""
    statuses: List[str] = field(default_factory=list)
    details: List[Dict[str, Any]] = field(default_factory=list)
    tools: Set[str] = field(default_factory=set)
    experts: List[str] = field(default_factory=list)
    chunks: int = 0
    elapsed_ms: float = 0.0
    http_status: int = 0
    error: str = ""

    @property
    def pipeline(self) -> str:
        if self.error:
            return "error"
        s = set(self.statuses)
        if s & MOE_STATUSES:
            return "moe"
        if s & PLAN_STATUSES:
            return "plan-and-solve"
        if s & MULTI_STATUSES:
            return "multi-turn"
        if "generating" in s and not (s & MOE_STATUSES):
            return "unknown"
        return "unknown"

    def moe_evidence(self) -> Dict[str, Any]:
        routing_details = [d for d in self.details if d.get("selected_experts") or d.get("weights")]
        expert_calls = [d for d in self.details if d.get("expert")]
        critic = [d for d in self.details if d.get("verdict")]
        return {
            "pipeline": self.pipeline,
            "moe_status_hits": sorted(set(self.statuses) & MOE_STATUSES),
            "selected_experts": (routing_details[-1].get("selected_experts") if routing_details else None),
            "expert_weights": (routing_details[-1].get("weights") if routing_details else None),
            "experts_traced": sorted({str(d.get("expert")) for d in expert_calls if d.get("expert")}),
            "tools_used": sorted(self.tools),
            "critic_verdict": (critic[-1].get("verdict") if critic else None),
        }


async def probe_chat(
    client: httpx.AsyncClient,
    *,
    base: str,
    model: str,
    question: str,
    api_key: str,
    user_role: str = "pet_owner",
    max_tokens: int = 500,
    stream_answer: bool = False,
) -> ChatProbe:
    probe = ChatProbe(model=model, question=question)
    payload: Dict[str, Any] = {
        "model": model,
        "messages": [{"role": "user", "content": question}],
        "stream": True,
        "temperature": 0.3,
        "max_tokens": max_tokens,
        "user_role": user_role,
        "debug_timing": False,
    }
    headers: Dict[str, str] = {
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
    }
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    t0 = time.perf_counter()
    try:
        async with client.stream("POST", f"{base.rstrip('/')}/v1/chat/completions", json=payload, headers=headers) as resp:
            probe.http_status = resp.status_code
            if resp.status_code != 200:
                probe.error = (await resp.aread()).decode("utf-8", "replace")[:400]
                return probe
            async for line in resp.aiter_lines():
                if not line or not line.startswith("data: "):
                    continue
                data = line[6:]
                if data == "[DONE]":
                    break
                try:
                    chunk = json.loads(data)
                except json.JSONDecodeError:
                    continue
                probe.chunks += 1
                status = chunk.get("agent_status")
                if status:
                    probe.statuses.append(status)
                detail = chunk.get("agent_detail")
                if isinstance(detail, dict):
                    probe.details.append(detail)
                    if detail.get("tool_name"):
                        probe.tools.add(str(detail["tool_name"]))
                    for tu in detail.get("tools_used") or []:
                        probe.tools.add(str(tu))
                    if detail.get("selected_experts"):
                        probe.experts = list(detail["selected_experts"])
                try:
                    delta = chunk.get("choices", [{}])[0].get("delta", {})
                    piece = delta.get("content") or ""
                    if piece and status == "streaming":
                        probe.answer += piece
                        if stream_answer:
                            print(piece, end="", flush=True)
                except Exception:  # noqa: BLE001
                    pass
    except Exception as exc:  # noqa: BLE001
        probe.error = f"{type(exc).__name__}: {exc}"
    probe.elapsed_ms = round((time.perf_counter() - t0) * 1000, 1)
    return probe


def _print_probe(title: str, probe: ChatProbe) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")
    print(f"  model        : {probe.model}")
    print(f"  http         : {probe.http_status}")
    print(f"  elapsed      : {probe.elapsed_ms} ms")
    print(f"  chunks       : {probe.chunks}")
    if probe.error:
        print(f"  ERROR        : {probe.error}")
        return
    ev = probe.moe_evidence()
    print(f"  pipeline     : {ev['pipeline']}")
    print(f"  MoE statuses : {ev['moe_status_hits'] or '(none)'}")
    print(f"  all statuses : {probe.statuses}")
    if ev["selected_experts"]:
        print(f"  experts      : {ev['selected_experts']}")
        print(f"  weights      : {ev['expert_weights']}")
    if ev["tools_used"]:
        print(f"  tools        : {ev['tools_used']}")
    if ev["critic_verdict"]:
        print(f"  critic       : {ev['critic_verdict']}")
    excerpt = (probe.answer or "").strip().replace("\n", " ")
    print(f"  answer       : {excerpt[:280]}{'…' if len(excerpt) > 280 else ''}")


async def _check_infra(client: httpx.AsyncClient, base: str) -> bool:
    ok = True
    for path in ("/health", "/ready"):
        try:
            r = await client.get(f"{base}{path}", timeout=10.0)
            print(f"[infra] GET {path} -> {r.status_code} {r.text[:120]}")
            if path == "/health" and r.status_code == 200 and not r.json().get("ok"):
                ok = False
        except Exception as exc:  # noqa: BLE001
            print(f"[infra] GET {path} FAILED: {exc}")
            ok = False

    try:
        r = await client.get(f"{base}/v1/models", timeout=10.0)
        print(f"[infra] GET /v1/models -> {r.status_code}")
        if r.status_code == 200:
            ids = [m.get("id") for m in r.json().get("data", [])]
            print(f"        models: {ids}")
            if "agent-moe" not in ids:
                print("        [warn] 未列出 agent-moe，但默认 model 仍可能走 MoE")
    except Exception as exc:  # noqa: BLE001
        print(f"[infra] GET /v1/models FAILED: {exc}")
        ok = False
    return ok


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PetMind HTTP 探活 + MoE 是否调用检测")
    p.add_argument("--base", default=os.getenv("AGENT_API_BASE", "http://127.0.0.1:8002"))
    p.add_argument("--api-key", default=None, help="默认读 AGENT_API_KEY 或 keys.txt")
    p.add_argument(
        "--question", "-q",
        default="我家成年犬最近软便、精神略差，可能是什么原因？何时必须就医？",
    )
    p.add_argument("--model", default="agent-moe", help="显式 MoE；也可用任意非 plan/multi-turn 名称")
    p.add_argument("--default-model", default="petmind-default", help="测试「未识别 model 走默认 MoE」")
    p.add_argument("--compare-plan", action="store_true", help="同题再跑 agent-plan-solve 作对照")
    p.add_argument("--max-tokens", type=int, default=500)
    p.add_argument("--stream-answer", action="store_true", help="流式打印最终回答")
    p.add_argument("--skip-chat", action="store_true", help="仅探活，不调用 LLM")
    return p.parse_args()


async def amain() -> int:
    _load_dotenv()
    args = parse_args()
    base = args.base.rstrip("/")
    api_key = args.api_key if args.api_key is not None else _default_api_key()

    if not (os.getenv("OPENAI_API_KEY") or os.getenv("DEEPSEEK_API_KEY")) and not args.skip_chat:
        print("[warn] 未设置 OPENAI_API_KEY / DEEPSEEK_API_KEY，聊天可能 500；可用 --skip-chat 仅测探活")

    print(f"[config] base={base}")
    print(f"[config] question={args.question!r}")

    async with httpx.AsyncClient(timeout=httpx.Timeout(180.0, connect=15.0)) as client:
        if not await _check_infra(client, base):
            print("\n[FAIL] 基础设施检查未通过，请确认 PetMind 已启动（如 bash start_agent.sh）")
            return 1

        if args.skip_chat:
            print("\n[OK] 探活完成（--skip-chat）")
            return 0

        # 1) 显式 agent-moe
        if args.stream_answer:
            print("\n--- 流式回答 (agent-moe) ---")
        moe_probe = await probe_chat(
            client, base=base, model=args.model, question=args.question,
            api_key=api_key, max_tokens=args.max_tokens, stream_answer=args.stream_answer,
        )
        if args.stream_answer:
            print()
        _print_probe(f"MoE 路径 · model={args.model}", moe_probe)

        # 2) 默认 model（服务端应对未识别名走 MoE）
        default_probe = await probe_chat(
            client, base=base, model=args.default_model, question=args.question,
            api_key=api_key, max_tokens=min(300, args.max_tokens),
        )
        _print_probe(f"默认 MoE · model={args.default_model}", default_probe)

        plan_probe: Optional[ChatProbe] = None
        if args.compare_plan:
            plan_probe = await probe_chat(
                client, base=base, model="agent-plan-solve", question=args.question,
                api_key=api_key, max_tokens=min(400, args.max_tokens),
            )
            _print_probe("对照 · agent-plan-solve", plan_probe)

    # 判定
    print(f"\n{'=' * 60}")
    print("  结论")
    print(f"{'=' * 60}")
    exit_code = 0
    if moe_probe.error or default_probe.error:
        print("  [FAIL] 聊天请求失败")
        exit_code = 1
    elif moe_probe.pipeline != "moe":
        print(f"  [FAIL] model={args.model} 未观测到 MoE 状态 {sorted(MOE_STATUSES)}")
        print(f"         实际 pipeline={moe_probe.pipeline}, statuses={moe_probe.statuses}")
        exit_code = 1
    else:
        print(f"  [PASS] MoE 已调用（statuses 含 {moe_probe.moe_evidence()['moe_status_hits']}）")

    if not default_probe.error and default_probe.pipeline == "moe":
        print("  [PASS] 未识别 model 名称仍走 MoE 默认管线")
    elif not default_probe.error:
        print(f"  [WARN] model={args.default_model} pipeline={default_probe.pipeline}（期望 moe）")

    if plan_probe and not plan_probe.error:
        if plan_probe.pipeline == "plan-and-solve" and moe_probe.pipeline == "moe":
            print("  [PASS] compare-plan：MoE 与 plan-and-solve 路径已区分")
        else:
            print(f"  [WARN] compare-plan：plan={plan_probe.pipeline}, moe={moe_probe.pipeline}")

    return exit_code


def main() -> None:
    raise SystemExit(asyncio.run(amain()))


if __name__ == "__main__":
    main()
