"""Real DeepSeek, multi-conversation memory lifecycle acceptance.

This intentionally creates enough turns to cross short-term capacity, form
mid-term segments and trigger long-term knowledge/profile extraction.  It then
exports a full user snapshot, deletes one source conversation, and restores the
snapshot through the SUPER_ADMIN API.
"""
from __future__ import annotations

import argparse
import gzip
import json
import time
import uuid
from datetime import datetime
from pathlib import Path
from urllib.parse import quote

import httpx


PROMPT_GROUPS = [
    [
        "病例建档：4岁已绝育英短母猫团团，长期反复瘙痒。请先记录基本情况并说明还要问什么。",
        "团团每年春秋耳后和颈部瘙痒加重，瘙痒评分约7/10，请结合上一轮信息继续分析。",
        "稳定事实：团团吃鸡肉配方后更容易出现红斑和抓挠，停用后约两周缓解。请记住这一点。",
        "团团没有呼吸困难或面部肿胀，主要是皮肤红斑、抓挠和偶发外耳炎。下一步怎么排查？",
        "既往皮肤刮片未见螨虫，真菌培养阴性，但跳蚤预防有时不规律。如何解释？",
        "请总结本会话里团团的皮肤问题时间线、稳定事实和待验证假设。",
    ],
    [
        "继续团团的过敏病例：最近严格使用异噁唑啉类驱虫后仍有颈部瘙痒，请结合既往信息。",
        "我们做了8周水解蛋白排除饮食，瘙痒从7/10降到3/10，这对鉴别有什么意义？",
        "随后少量复食鸡肉，三天内瘙痒升到6/10，但没有呕吐腹泻。请记录并评估证据强度。",
        "团团对鱼肉配方耐受尚可。日常给药很困难，主人偏好步骤少、可以书面打勾的方案。",
        "如果需要控制继发感染，应先做哪些检查，不能仅凭哪些表现用药？",
        "请把本会话新增事实和上一会话信息合并，区分已知事实与仍需核实的推断。",
    ],
    [
        "团团复诊：耳道细胞学见较多马拉色菌，球菌少量，未见杆菌。请结合长期过敏背景分析。",
        "耳镜显示鼓膜完整，左耳分泌物多于右耳；团团目前精神食欲正常。治疗优先级是什么？",
        "主人再次强调团团口服药依从性差，更容易完成外用和每周记录。方案怎样适配？",
        "两周后外耳分泌物减少，瘙痒约3/10，但颈部仍有少量丘疹。复查重点是什么？",
        "请不要把鸡肉相关性直接写成确诊食物过敏，说明还缺什么标准化验证。",
        "总结目前团团的个人特点、主人沟通偏好、可能诱因和复诊观察指标。",
    ],
    [
        "跨会话复核：团团之前对哪种蛋白复食后瘙痒加重？请先调用已有记忆再回答。",
        "如果主人忘记之前的水解蛋白试验结果，请根据既往记录复述变化幅度和证据边界。",
        "团团本周瘙痒又到5/10，但跳蚤预防按时完成。请基于历史信息列复发原因优先级。",
        "主人仍希望采用少步骤、书面可勾选的计划。请给一个符合其沟通偏好的复诊清单。",
        "本轮没有新的检查结果。请明确哪些结论来自历史记忆，哪些是当前推断。",
        "最终汇总团团的长期病例画像，并列出下次复诊必须带来的三类数据。",
    ],
]


def _headers() -> dict[str, str]:
    """构造带鉴权的请求头。"""
    return {"Idempotency-Key": str(uuid.uuid4())}


def _run_turn(client: httpx.Client, base_url: str, conversation_id: str, prompt: str) -> dict:
    """跑一轮对话并返回结果。"""
    client_message_id = str(uuid.uuid4())
    started = time.perf_counter()
    events, answer, run_id = [], "", ""
    with client.stream(
        "POST",
        f"{base_url}/api/v1/conversations/{conversation_id}/runs",
        headers={"Idempotency-Key": client_message_id},
        json={
            "message": prompt,
            "client_message_id": client_message_id,
            "delivery": "sse",
            "user_role": "veterinarian",
            "max_tokens": 1800,
        },
    ) as response:
        response.raise_for_status()
        run_id = response.headers.get("X-PetMind-Run-Id", "")
        for line in response.iter_lines():
            if not line.startswith("data:"):
                continue
            raw = line[5:].strip()
            if not raw or raw == "[DONE]":
                continue
            event = json.loads(raw)
            events.append(event)
            answer += str(event.get("content") or "")
    if not run_id:
        raise RuntimeError("run id missing from SSE response")
    run = client.get(f"{base_url}/api/v1/runs/{run_id}").json()
    if run.get("status") != "completed" or not run.get("response"):
        raise RuntimeError(f"run did not complete with a final answer: {run}")
    return {
        "run_id": run_id,
        "latency_seconds": round(time.perf_counter() - started, 3),
        "answer": run["response"],
        "event_count": len(events),
    }


def main() -> int:
    """脚本入口，解析参数并执行主流程。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8002")
    parser.add_argument("--memory-url", default="http://127.0.0.1:8300")
    parser.add_argument("--email", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--out", type=Path, default=Path("agent_api/tests/api_live/reports/platform_memory_lifecycle.json"))
    parser.add_argument("--poll-seconds", type=int, default=600)
    parser.add_argument(
        "--resume-existing",
        action="store_true",
        help="reuse exact-title acceptance conversations and continue after their completed assistant turns",
    )
    args = parser.parse_args()
    base_url = args.base_url.rstrip("/")
    with httpx.Client(timeout=httpx.Timeout(900.0), trust_env=False) as client:
        login = client.post(f"{base_url}/api/v1/auth/login", json={"email": args.email, "password": args.password})
        login.raise_for_status()
        auth = login.json()
        client.headers["Authorization"] = f"Bearer {auth['access_token']}"
        user_id = auth["user"]["id"]
        if auth["user"].get("role") != "SUPER_ADMIN":
            raise RuntimeError("live lifecycle test requires a SUPER_ADMIN test account")

        existing_by_title = {}
        if args.resume_existing:
            listed = client.get(f"{base_url}/api/v1/conversations", params={"limit": 100})
            listed.raise_for_status()
            existing_by_title = {item["title"]: item for item in listed.json().get("items", [])}

        conversations, turns, session_counts, transcripts = [], [], [], []
        for session_index, prompts in enumerate(PROMPT_GROUPS, start=1):
            title = f"团团过敏长期记忆验收 {session_index}"
            conversation = existing_by_title.get(title)
            if conversation is None:
                response = client.post(
                    f"{base_url}/api/v1/conversations",
                    headers=_headers(),
                    json={"title": title},
                )
                response.raise_for_status()
                conversation = response.json()
            conversations.append(conversation["id"])
            completed_turns = 0
            if args.resume_existing and title in existing_by_title:
                message_response = client.get(
                    f"{base_url}/api/v1/conversations/{conversation['id']}/messages"
                )
                message_response.raise_for_status()
                completed_turns = sum(
                    1 for item in message_response.json().get("items", [])
                    if item.get("role") == "assistant" and item.get("status") == "complete"
                )
                completed_turns = min(completed_turns, len(prompts))
                print(f"session={session_index} resume_completed={completed_turns}")
            for turn_index, prompt in enumerate(prompts[completed_turns:], start=completed_turns + 1):
                result = _run_turn(client, base_url, conversation["id"], prompt)
                result.update({"session": session_index, "turn": turn_index, "prompt": prompt})
                turns.append(result)
                print(f"session={session_index} turn={turn_index} run={result['run_id']} latency={result['latency_seconds']}s")
            final_messages = client.get(
                f"{base_url}/api/v1/conversations/{conversation['id']}/messages"
            ).json().get("items", [])
            session_counts.append({
                "session": session_index,
                "conversation_id": conversation["id"],
                "user_messages": sum(item.get("role") == "user" for item in final_messages),
                "assistant_messages": sum(item.get("role") == "assistant" for item in final_messages),
            })
            transcripts.append({
                "session": session_index,
                "conversation_id": conversation["id"],
                "messages": [
                    {
                        "role": item.get("role"),
                        "content": item.get("content"),
                        "status": item.get("status"),
                        "run_id": item.get("run_id"),
                    }
                    for item in final_messages
                ],
            })

        deadline = time.time() + args.poll_seconds
        stats = {}
        while time.time() < deadline:
            response = client.get(f"{args.memory_url.rstrip('/')}/v1/memory/stats/{user_id}")
            response.raise_for_status()
            stats = response.json()
            if int(stats.get("knowledge", 0)) > 0:
                break
            time.sleep(3)
        profile = client.get(f"{args.memory_url.rstrip('/')}/v1/memory/profile/{user_id}").json()
        memories = client.get(f"{base_url}/api/v1/me/memories").json()
        if int(stats.get("knowledge", 0)) < 1 or int(profile.get("version", 0)) < 1:
            raise RuntimeError(f"long-term memory did not form: stats={stats}, profile={profile}")
        if not memories.get("items") or any(not item.get("generation_tags") for item in memories["items"]):
            raise RuntimeError(f"memory management output lacks provenance labels: {memories}")

        snapshot_response = client.get(
            f"{base_url}/api/v1/admin/users/{user_id}/data-snapshot"
        )
        snapshot_response.raise_for_status()
        snapshot = snapshot_response.json()
        before_conversations = len(client.get(f"{base_url}/api/v1/conversations").json()["items"])
        deleted = client.delete(f"{base_url}/api/v1/conversations/{conversations[0]}")
        deleted.raise_for_status()
        visible_conversations = len(client.get(f"{base_url}/api/v1/conversations").json()["items"])
        post_hide_stats = client.get(
            f"{args.memory_url.rstrip('/')}/v1/memory/stats/{user_id}"
        ).json()
        count_keys = ("short_term", "segments", "knowledge")
        if visible_conversations != before_conversations - 1 or any(
            post_hide_stats.get(key) != stats.get(key) for key in count_keys
        ):
            raise RuntimeError(
                "conversation hiding unexpectedly changed consolidated memory: "
                f"conversations {before_conversations}->{visible_conversations}, memory {stats}->{post_hide_stats}"
            )
        compressed_snapshot = gzip.compress(
            json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            compresslevel=9,
        )
        restore = client.post(
            f"{base_url}/api/v1/admin/users/{user_id}/data-snapshot/restore-file",
            headers={
                **_headers(),
                "Content-Type": "application/gzip",
                "X-Restore-Confirmation": "OVERWRITE_USER_DATA",
            },
            content=compressed_snapshot,
        )
        restore.raise_for_status()
        after_conversations = len(client.get(f"{base_url}/api/v1/conversations").json()["items"])
        restored_stats = client.get(
            f"{args.memory_url.rstrip('/')}/v1/memory/stats/{user_id}"
        ).json()
        if after_conversations != before_conversations or any(
            restored_stats.get(key) != stats.get(key) for key in count_keys
        ):
            raise RuntimeError(
                f"snapshot restore mismatch: conversations {before_conversations}->{after_conversations}, memory {stats}->{restored_stats}"
            )

        long_item = next(
            (item for item in memories["items"] if item.get("type") == "knowledge"),
            None,
        )
        if long_item is None:
            raise RuntimeError("no long-term knowledge item available for deletion acceptance")
        delete_long = client.delete(
            f"{base_url}/api/v1/me/memories/{quote(long_item['id'], safe='')}"
        )
        delete_long.raise_for_status()
        after_long_delete = client.get(
            f"{args.memory_url.rstrip('/')}/v1/memory/stats/{user_id}"
        ).json()
        if (
            after_long_delete.get("segments") != restored_stats.get("segments")
            or after_long_delete.get("pages") != restored_stats.get("pages")
            or int(after_long_delete.get("knowledge", 0))
            != int(restored_stats.get("knowledge", 0)) - 1
        ):
            raise RuntimeError(
                f"long-term deletion changed the wrong memory layers: {restored_stats}->{after_long_delete}"
            )
        restore_after_long_delete = client.post(
            f"{base_url}/api/v1/admin/users/{user_id}/data-snapshot/restore-file",
            headers={
                **_headers(),
                "Content-Type": "application/gzip",
                "X-Restore-Confirmation": "OVERWRITE_USER_DATA",
            },
            content=compressed_snapshot,
        )
        restore_after_long_delete.raise_for_status()
        final_stats = client.get(
            f"{args.memory_url.rstrip('/')}/v1/memory/stats/{user_id}"
        ).json()
        if any(final_stats.get(key) != stats.get(key) for key in count_keys):
            raise RuntimeError(
                f"memory item restore mismatch: expected={stats}, actual={final_stats}"
            )

    report = {
        "generated_at": datetime.now().isoformat(),
        "user_id": user_id,
        "conversations": conversations,
        "turns": turns,
        "session_counts": session_counts,
        "transcripts": transcripts,
        "memory_stats": stats,
        "memory_stats_after_conversation_hide": post_hide_stats,
        "deleted_long_term_item": long_item["id"],
        "memory_stats_after_long_term_delete": after_long_delete,
        "memory_stats_after_final_restore": final_stats,
        "profile": profile,
        "managed_memories": memories,
        "snapshot_checksum": snapshot["checksum"],
        "snapshot_compressed_bytes": len(compressed_snapshot),
        "restore": restore.json(),
        "restore_after_long_term_delete": restore_after_long_delete.json(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"PASS report={args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
