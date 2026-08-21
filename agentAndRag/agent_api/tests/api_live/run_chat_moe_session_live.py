"""Live regression for browser-only sessions and stateless production requests."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, List

import httpx


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DB = ROOT / "agent_api_logs" / "petmind_sessions.db"

FIRST_MESSAGE = (
    "8岁犬正在服用卡洛芬治疗关节炎，另一位医生因皮肤病拟开泼尼松。"
    "请调用RAG和网络搜索核对两药联用风险，并给出在存在禁忌时的替代方向。"
)
FOLLOWUP_MESSAGE = "这个联用禁忌的机制、证据依据和替代方案分别是什么？请继续结合前面的病例说明。"
UNRELATED_MESSAGE = "现在明确换个话题：请用Python写一个快速排序算法。"


def _api_key() -> str:
    """读取本次运行使用的 API 密钥。"""
    value = os.getenv("AGENT_API_KEY", "").strip()
    if value:
        return value
    key_file = ROOT / "agent_api" / "keys.txt"
    if key_file.exists():
        for line in key_file.read_text(encoding="utf-8").splitlines():
            value = line.strip()
            if value and not value.startswith("#"):
                return value
    return ""


async def _read_sse(response: httpx.Response, started: float) -> Dict[str, Any]:
    """读取并解析 SSE 事件流。"""
    events: List[Dict[str, Any]] = []
    answer: List[str] = []
    finish_reason = None
    async for line in response.aiter_lines():
        if not line.startswith("data: "):
            continue
        payload = line[6:]
        if payload == "[DONE]":
            break
        event = json.loads(payload)
        event["elapsed_ms"] = round((time.perf_counter() - started) * 1000, 1)
        events.append(event)
        choice = (event.get("choices") or [{}])[0]
        if choice.get("finish_reason"):
            finish_reason = choice["finish_reason"]
        if event.get("agent_status") == "streaming":
            content = (choice.get("delta") or {}).get("content") or ""
            answer.append(content)
    return {
        "answer": "".join(answer),
        "finish_reason": finish_reason,
        "elapsed_ms": round((time.perf_counter() - started) * 1000, 1),
        "events": events,
    }


async def _chat_moe(
    client: httpx.AsyncClient,
    base_url: str,
    session_id: str,
    message: str,
) -> Dict[str, Any]:
    """调用 MoE 聊天接口并收集事件。"""
    started = time.perf_counter()
    async with client.stream(
        "POST",
        f"{base_url}/chat-moe/completions",
        json={
            "session_id": session_id,
            "message": message,
            "user_role": "veterinarian",
            "response_lang": "zh",
            "temperature": 0.2,
            "max_tokens": 1200,
        },
    ) as response:
        response.raise_for_status()
        return await _read_sse(response, started)


async def _production_chat(
    client: httpx.AsyncClient,
    base_url: str,
    api_key: str,
) -> Dict[str, Any]:
    """按生产路径发起一次聊天请求。"""
    started = time.perf_counter()
    async with client.stream(
        "POST",
        f"{base_url}/v1/chat/completions",
        headers={"Authorization": f"Bearer {api_key}"},
        json={
            "model": "agent-moe",
            "messages": [{"role": "user", "content": UNRELATED_MESSAGE}],
            "stream": True,
            "tools": [],
            "temperature": 0.1,
            "max_tokens": 300,
        },
    ) as response:
        response.raise_for_status()
        return await _read_sse(response, started)


def _context_event(result: Dict[str, Any]) -> Dict[str, Any]:
    """从事件列表里取出上下文事件。"""
    for event in result["events"]:
        if event.get("agent_status") == "session_context_loaded":
            return dict(event.get("agent_detail") or {})
    raise AssertionError("missing session_context_loaded event")


def _out_of_scope(result: Dict[str, Any]) -> bool:
    """判断事件中是否出现超出范围标记。"""
    return any(
        event.get("agent_status") == "routing"
        and bool((event.get("agent_detail") or {}).get("out_of_scope"))
        for event in result["events"]
    )


def _error_events(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """筛选出错误类事件。"""
    return [
        event
        for event in result["events"]
        if event.get("agent_status") == "error"
        or (event.get("choices") or [{}])[0].get("finish_reason") == "error"
    ]


def _session_snapshot(db_path: Path, session_id: str) -> Dict[str, Any]:
    """读取当前会话快照。"""
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        row = conn.execute(
            "SELECT * FROM agent_sessions WHERE session_id = ?", (session_id,)
        ).fetchone()
    finally:
        conn.close()
    if row is None:
        raise AssertionError(f"session missing from SQLite: {session_id}")
    return {
        "session_id": session_id,
        "messages": json.loads(row["messages"]),
        "expert_contexts": json.loads(row["expert_contexts"]),
        "tool_results": json.loads(row["tool_results"]),
        "created_at": row["created_at"],
        "last_active": row["last_active"],
    }


def _tool_names(snapshot: Dict[str, Any]) -> List[str]:
    """从事件中收集用到的工具名。"""
    names = set()
    for context in snapshot["expert_contexts"]:
        for expert in context.get("experts") or []:
            names.update(str(name) for name in (expert.get("tools_used") or []) if name)
    return sorted(names)


async def _seed(args: argparse.Namespace) -> None:
    """写入本用例所需的种子数据。"""
    timeout = httpx.Timeout(connect=20, read=args.timeout, write=30, pool=30)
    async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
        response: httpx.Response | None = None
        last_error: Exception | None = None
        for attempt in range(1, 6):
            try:
                response = await client.post(f"{args.base_url}/chat-moe/sessions")
                break
            except httpx.ConnectError as exc:
                last_error = exc
                if attempt == 5:
                    raise
                await asyncio.sleep(float(attempt))
        if response is None:
            raise RuntimeError("session creation failed") from last_error
        response.raise_for_status()
        session_id = response.json()["session_id"]
        first = await _chat_moe(client, args.base_url, session_id, FIRST_MESSAGE)

    snapshot = _session_snapshot(args.db_path, session_id)
    state = {
        "session_id": session_id,
        "first_message": FIRST_MESSAGE,
        "first_result": first,
        "seed_snapshot": snapshot,
        "error_events": _error_events(first),
    }
    args.state.parent.mkdir(parents=True, exist_ok=True)
    args.state.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")

    if state["error_events"]:
        print(json.dumps({
            "phase": "seed_error",
            "session_id": session_id,
            "error_events": state["error_events"],
            "state": str(args.state),
        }, ensure_ascii=False, indent=2))

    assert _context_event(first)["complete_turns"] == 0
    assert not _out_of_scope(first)
    assert first["answer"], f"empty answer; error_events={state['error_events']!r}"
    assert len(snapshot["messages"]) == 2
    assert len(snapshot["expert_contexts"]) == 1
    tool_names = set(_tool_names(snapshot))
    assert {"rag.search", "mcp.web_search.web_search"} <= tool_names, (
        f"first expert turn did not record both required searches: {sorted(tool_names)}"
    )
    print(json.dumps({
        "phase": "seed",
        "session_id": session_id,
        "elapsed_ms": first["elapsed_ms"],
        "tools": _tool_names(snapshot),
        "messages": len(snapshot["messages"]),
    }, ensure_ascii=False))


async def _resume(args: argparse.Namespace) -> None:
    """按已有会话继续聊一轮。"""
    state = json.loads(args.state.read_text(encoding="utf-8"))
    session_id = state["session_id"]
    before = _session_snapshot(args.db_path, session_id)
    api_key = _api_key()
    if not api_key:
        raise RuntimeError("missing API key for stateless production endpoint check")

    timeout = httpx.Timeout(connect=20, read=args.timeout, write=30, pool=30)
    async with httpx.AsyncClient(timeout=timeout, trust_env=False) as client:
        followup = await _chat_moe(client, args.base_url, session_id, FOLLOWUP_MESSAGE)
        unrelated = await _chat_moe(client, args.base_url, session_id, UNRELATED_MESSAGE)
        after_test_turns = _session_snapshot(args.db_path, session_id)
        production = await _production_chat(client, args.base_url, api_key)
        after_production = _session_snapshot(args.db_path, session_id)

    followup_context = _context_event(followup)
    unrelated_context = _context_event(unrelated)
    assert len(before["messages"]) == 2, "seed state was not restored after restart"
    assert followup_context["complete_turns"] == 1
    assert followup_context["expert_context_turns"] == 1
    assert followup_context["prior_experts"]
    assert followup_context["prior_tools"]
    assert followup_context["prior_searches"] >= 2
    assert followup_context["prior_evidence_items"] > 0
    assert not _out_of_scope(followup), "veterinary contextual follow-up was rejected"
    assert unrelated_context["complete_turns"] == 2
    assert _out_of_scope(unrelated), "explicit unrelated topic shift was not rejected"
    assert len(after_test_turns["messages"]) == 6
    assert after_production["messages"] == after_test_turns["messages"]
    assert after_production["expert_contexts"] == after_test_turns["expert_contexts"]
    assert after_production["tool_results"] == after_test_turns["tool_results"]

    report = {
        **state,
        "followup_message": FOLLOWUP_MESSAGE,
        "followup_result": followup,
        "unrelated_message": UNRELATED_MESSAGE,
        "unrelated_result": unrelated,
        "production_stateless_result": production,
        "final_snapshot": after_production,
        "checks": {
            "sqlite_survived_restart": True,
            "complete_turn_history_loaded": True,
            "expert_context_loaded": True,
            "tool_history_loaded": True,
            "search_and_evidence_history_loaded": True,
            "veterinary_followup_allowed": True,
            "unrelated_topic_rejected": True,
            "production_endpoint_did_not_mutate_test_session": True,
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({
        "phase": "resume",
        "session_id": session_id,
        "followup_context": followup_context,
        "unrelated_rejected": True,
        "production_session_unchanged": True,
        "report": str(args.report),
    }, ensure_ascii=False))


def _args() -> argparse.Namespace:
    """解析本脚本的命令行参数。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("seed", "resume"), required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--db-path", type=Path, default=DEFAULT_DB)
    parser.add_argument(
        "--state", type=Path,
        default=ROOT / "agent_api_logs" / "chat_moe_session_live_state.json",
    )
    parser.add_argument(
        "--report", type=Path,
        default=ROOT / "agent_api_logs" / "chat_moe_session_live_report.json",
    )
    parser.add_argument("--timeout", type=float, default=900.0)
    return parser.parse_args()


if __name__ == "__main__":
    parsed = _args()
    asyncio.run(_seed(parsed) if parsed.phase == "seed" else _resume(parsed))
