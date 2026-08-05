"""Live user-memory regression across production and Chat-MoE API boundaries."""
from __future__ import annotations

import argparse
import asyncio
import json
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List

import httpx


async def _chat_moe_turn(
    client: httpx.AsyncClient,
    base_url: str,
    *,
    session_id: str,
    message: str,
) -> Dict[str, Any]:
    events: List[Dict[str, Any]] = []
    answer: List[str] = []
    async with client.stream(
        "POST",
        f"{base_url}/chat-moe/completions",
        json={"session_id": session_id, "message": message, "user_role": "pet_owner"},
    ) as response:
        response.raise_for_status()
        async for line in response.aiter_lines():
            if not line.startswith("data:"):
                continue
            payload = line[5:].strip()
            if not payload or payload == "[DONE]":
                continue
            event = json.loads(payload)
            events.append(event)
            if event.get("agent_status") == "streaming":
                answer.append(str(event.get("choices", [{}])[0].get("delta", {}).get("content") or ""))
    return {"answer": "".join(answer), "events": events}


def _event_detail(turn: Dict[str, Any], status: str) -> Dict[str, Any]:
    event = next(item for item in turn["events"] if item.get("agent_status") == status)
    return dict(event.get("agent_detail") or {})


async def run(args: argparse.Namespace) -> Dict[str, Any]:
    suffix = uuid.uuid4().hex[:8]
    v1_user = f"live-v1-{suffix}"
    v1_other = f"live-v1-other-{suffix}"
    v1_pet_name = f"星尘{suffix[:4]}"
    chat_username = f"live tester {suffix}"
    chat_other = f"other tester {suffix}"
    chat_pet_name = f"量子{suffix[:4]}"

    async with httpx.AsyncClient(timeout=args.timeout, trust_env=False) as client:
        ready = (await client.get(f"{args.base_url}/ready")).json()
        memory_health = (await client.get(f"{args.memory_url}/health")).json()

        first_v1 = (
            await client.post(
                f"{args.base_url}/v1/chat/completions",
                json={
                    "model": args.v1_model,
                    "stream": False,
                    "tools": [],
                    "user_id": v1_user,
                    "memory_session_id": f"v1-session-a-{suffix}",
                    "memory_turn_id": f"v1-turn-a-{suffix}",
                    "messages": [{
                        "role": "user",
                        "content": (
                            f"我的猫目前精神和食欲正常，请记住它叫{v1_pet_name}，"
                            "以后健康随访用这个名字；请简短确认当前没有明显异常信号。"
                        ),
                    }],
                },
            )
        )
        first_v1.raise_for_status()
        first_v1_json = first_v1.json()

        second_v1 = (
            await client.post(
                f"{args.base_url}/v1/chat/completions",
                json={
                    "model": args.v1_model,
                    "stream": False,
                    "tools": [],
                    "user_id": v1_user,
                    "memory_session_id": f"v1-session-b-{suffix}",
                    "memory_turn_id": f"v1-turn-b-{suffix}",
                    "messages": [{
                        "role": "user",
                        "content": (
                            "新会话继续健康随访：请在开头称呼我的猫的名字，"
                            "并依据它上次精神和食欲正常的记录给出两项日常观察点。"
                        ),
                    }],
                },
            )
        )
        second_v1.raise_for_status()
        second_v1_json = second_v1.json()
        second_v1_answer = second_v1_json["choices"][0]["message"]["content"]

        isolated_context = (
            await client.post(
                f"{args.memory_url}/v1/memory/context",
                json={"user_id": v1_other, "query": "宠物名字", "include_text": True},
            )
        ).json()

        session_a = (
            await client.post(f"{args.base_url}/chat-moe/sessions", json={"username": chat_username})
        ).json()
        first_chat = await _chat_moe_turn(
            client,
            args.base_url,
            session_id=session_a["session_id"],
            message=(
                f"我的狗目前饮食和精神正常，请记住它叫{chat_pet_name}。"
                "以后回答健康问题时使用它的名字，并简短确认。"
            ),
        )
        session_b = (
            await client.post(f"{args.base_url}/chat-moe/sessions", json={"username": chat_username.upper()})
        ).json()
        second_chat = await _chat_moe_turn(
            client,
            args.base_url,
            session_id=session_b["session_id"],
            message=(
                "新会话继续健康随访：请在开头称呼我的狗的名字，"
                "并依据它上次饮食和精神正常的记录给出两项日常观察点。"
            ),
        )
        session_other = (
            await client.post(f"{args.base_url}/chat-moe/sessions", json={"username": chat_other})
        ).json()
        isolated_chat = await _chat_moe_turn(
            client,
            args.base_url,
            session_id=session_other["session_id"],
            message="为了建立健康档案，我以前说过宠物名字吗？没有资料就明确说不知道。",
        )

        v1_stats = (await client.get(f"{args.memory_url}/v1/memory/stats/{v1_user}")).json()
        chat_stats = (
            await client.get(
                f"{args.memory_url}/v1/memory/stats/{session_a['memory_user_id']}"
            )
        ).json()

    assertions = {
        "stack_ready": ready.get("ready") is True and ready.get("memory", {}).get("status") == "ok",
        "memory_database_ok": memory_health.get("database") == "ok",
        "v1_first_stored": first_v1_json.get("memory", {}).get("write", {}).get("stored") is True,
        "v1_cross_session_loaded": second_v1_json.get("memory", {}).get("load", {}).get("recent_turns", 0) >= 1,
        "v1_answer_recalled_name": v1_pet_name in second_v1_answer,
        "v1_other_user_isolated": len(isolated_context.get("recent_dialogue") or []) == 0,
        "chat_subject_stable": session_a["memory_user_id"] == session_b["memory_user_id"],
        "chat_first_stored": _event_detail(first_chat, "memory_stored").get("stored") is True,
        "chat_cross_session_loaded": _event_detail(second_chat, "memory_context_loaded").get("recent_turns", 0) >= 1,
        "chat_answer_recalled_name": chat_pet_name in second_chat["answer"],
        "chat_other_user_isolated": _event_detail(isolated_chat, "memory_context_loaded").get("recent_turns", 0) == 0,
    }
    return {
        "run_id": suffix,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "assertions": assertions,
        "passed": all(assertions.values()),
        "v1": {
            "user_id": v1_user,
            "first_answer": first_v1_json["choices"][0]["message"]["content"],
            "second_answer": second_v1_answer,
            "second_memory": second_v1_json.get("memory"),
            "stats": v1_stats,
        },
        "chat_moe": {
            "username": chat_username,
            "memory_user_id": session_a["memory_user_id"],
            "first_session": session_a["session_id"],
            "second_session": session_b["session_id"],
            "first_answer": first_chat["answer"],
            "second_answer": second_chat["answer"],
            "isolated_answer": isolated_chat["answer"],
            "second_memory": _event_detail(second_chat, "memory_context_loaded"),
            "stats": chat_stats,
        },
    }


def write_report(result: Dict[str, Any], out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"memory_live_{result['run_id']}"
    json_path = out_dir / f"{stem}.json"
    md_path = out_dir / f"{stem}.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    checks = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — `{name}`"
        for name, passed in result["assertions"].items()
    )
    md_path.write_text(
        "\n".join([
            "# Agent 用户记忆真实联调报告",
            "",
            f"- 时间：{result['timestamp']}",
            f"- 总结：{'PASS' if result['passed'] else 'FAIL'}",
            "",
            "## 验证项",
            "",
            checks,
            "",
            "## /v1/chat/completions 真实 LLM 回复",
            "",
            result["v1"]["second_answer"],
            "",
            "## /chat-moe 跨 session 真实 LLM 回复",
            "",
            result["chat_moe"]["second_answer"],
            "",
            "## 隔离用户回复",
            "",
            result["chat_moe"]["isolated_answer"],
        ]),
        encoding="utf-8",
    )
    return json_path, md_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--memory-url", default="http://127.0.0.1:8300")
    parser.add_argument("--v1-model", default="agent-moe")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--out-dir", type=Path, default=Path("memory_service/reports"))
    args = parser.parse_args()
    result = asyncio.run(run(args))
    json_path, md_path = write_report(result, args.out_dir)
    print(json.dumps({"passed": result["passed"], "json": str(json_path), "markdown": str(md_path)}, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
