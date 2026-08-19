from __future__ import annotations

import asyncio
import json
import logging
import math
from datetime import timezone
from typing import Any, AsyncIterator

from sqlalchemy import func, select

from ..memory import load_user_memory, write_user_memory
from ..services.agent_execution import build_moe_orchestrator, public_moe_allowed_tools
from ..services.moe import MoETrace
from ..tools.tool_registry import get_registry
from .config import get_platform_settings
from .database import platform_session
from .expert_consultations import persist_expert_consultation
from .models import AgentRun, Conversation, Message, RunEvent, UsageRecord, UserPreference, utcnow
from .services import settle_credits


logger = logging.getLogger(__name__)

TERMINAL_RUN_STATES = {"completed", "failed", "cancelled"}
PUBLIC_PHASES = {
    "intent_classifying": ("understanding", "正在理解临床问题"),
    "intent_classified": ("understanding", "已识别任务类型"),
    "routing": ("routing", "正在组织会诊路径"),
    "expert_calling": ("consulting", "专家会诊中"),
    "expert_complete": ("consulting", "专家意见已返回"),
    "tool_complete": ("consulting", "临床资料核对完成"),
    "reviewing": ("reviewing", "正在进行安全复核"),
    "generating": ("generating", "正在整理答复"),
}


def _public_tool_summary(record: dict[str, Any]) -> dict[str, Any]:
    """Return a compact, prompt-free tool transcript safe for the web UI."""
    tool_name = str(record.get("tool_name") or "")
    result = record.get("result") if isinstance(record.get("result"), dict) else {}
    summary: dict[str, Any] = {
        "kind": "tool",
        "tool_name": tool_name,
        "ok": bool(record.get("ok")),
        "latency_ms": float(record.get("latency_ms") or 0.0),
    }
    if tool_name == "rag.search":
        hits = result.get("hits") if isinstance(result.get("hits"), list) else []
        summary["result"] = {
            "hits": len(hits),
            "sources": [
                str(hit.get("source_path") or hit.get("source") or "")[-160:]
                for hit in hits[:3]
                if isinstance(hit, dict)
            ],
        }
    elif "web_search" in tool_name:
        rows = result.get("results") if isinstance(result.get("results"), list) else []
        summary["result"] = {
            "results": len(rows),
            "titles": [str(row.get("title") or "")[:160] for row in rows[:3] if isinstance(row, dict)],
        }
    else:
        summary["result"] = {
            key: result.get(key)
            for key in ("code", "status", "alert_level")
            if result.get(key) is not None
        }
    if record.get("error"):
        summary["error"] = str(record["error"])[:300]
    return summary


def _public_expert_trace(opinion: dict[str, Any]) -> dict[str, Any]:
    """Expose expert work products, never the system prompt or hidden model trace."""
    return {
        "expert": str(opinion.get("expert") or ""),
        "name": str(opinion.get("name_zh") or "专家"),
        "status": "completed",
        "task": str(opinion.get("retrieval_reason") or "根据统一任务策略形成专业意见"),
        "required_tools": list(opinion.get("required_tools") or []),
        "recommended_tools": list(opinion.get("recommended_tools") or []),
        "tools": [_public_tool_summary(row) for row in opinion.get("tool_results") or [] if isinstance(row, dict)],
        "opinion": {
            "conclusion": str(opinion.get("conclusion") or ""),
            "evidence": [str(item) for item in opinion.get("evidence") or []],
            "risks": [str(item) for item in opinion.get("risks") or []],
            "confidence": float(opinion.get("confidence") or 0.0),
        },
        "execution": "single_pass",
    }


async def append_run_event(run_id: str, event_type: str, payload: dict[str, Any]) -> int:
    async with platform_session() as session:
        last = await session.scalar(select(func.max(RunEvent.sequence)).where(RunEvent.run_id == run_id))
        sequence = int(last or 0) + 1
        session.add(RunEvent(run_id=run_id, sequence=sequence, event_type=event_type, payload=payload))
        await session.commit()
        return sequence


async def _run_cancel_requested(run_id: str) -> bool:
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        return run is None or run.cancel_requested


async def enqueue_run(run_id: str) -> None:
    settings = get_platform_settings()
    if settings.redis_url:
        try:
            from redis.asyncio import Redis

            redis = Redis.from_url(settings.redis_url, decode_responses=True)
            await redis.rpush("petmind:platform:runs", run_id)
            await redis.aclose()
            return
        except Exception:
            if settings.production:
                raise
    asyncio.create_task(execute_run(run_id), name=f"platform-run-{run_id}")


async def execute_run(run_id: str) -> None:
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        if run is None or run.status not in {"queued", "retry"}:
            return
        run.status = "running"
        run.started_at = utcnow()
        conversation = await session.get(Conversation, run.conversation_id)
        history_rows = list((await session.scalars(
            select(Message).where(
                Message.conversation_id == run.conversation_id,
                Message.status == "complete",
            ).order_by(Message.created_at.asc()).limit(48)
        )).all())
        await session.commit()
        query = run.query
        user_id = run.user_id
        conversation_id = run.conversation_id
        parameters = dict(run.parameters or {})
        preference = await session.get(UserPreference, run.user_id)
        memory_recall_enabled = preference.memory_recall_enabled if preference else True
        memory_write_enabled = preference.memory_write_enabled if preference else True
        user_role = str(parameters.get("user_role") or "veterinarian")
        if user_role not in {"pet_owner", "veterinarian"}:
            user_role = "veterinarian"
        conversation_history = [
            {"role": item.role, "content": item.content}
            for item in history_rows
            if item.id != run.user_message_id and item.role in {"user", "assistant"}
        ]

    await append_run_event(run_id, "status", {"phase": "queued", "message": "任务已进入会诊队列"})
    answer_parts: list[str] = []
    expert_calls = 0
    tool_calls = 0
    current_phase = "queued"
    try:
        memory_injection = ""
        if memory_recall_enabled:
            memory_injection, _ = await load_user_memory(user_id=user_id, query=query, pet_id=None)
        registry = get_registry()
        allowed_tools = public_moe_allowed_tools(tool.name for tool in registry.list_tools())
        orchestrator = build_moe_orchestrator(
            registry=registry,
            temperature=float(parameters.get("temperature", 0.3)),
            max_tokens=int(parameters.get("max_tokens", 2500)),
            user_role=user_role,
            allowed_tools=allowed_tools,
        )
        trace = MoETrace(question=query, user_role=user_role)
        async for event in orchestrator.stream(
            query=query,
            system_context=(
                "你正在为已认证兽医提供临床决策支持。"
                if user_role == "veterinarian"
                else "你正在用宠物主容易理解的方式提供健康信息、风险分级与就医建议。"
            ),
            conversation_history=conversation_history,
            user_memory=memory_injection,
            recorder=trace,
        ):
            if await _run_cancel_requested(run_id):
                raise asyncio.CancelledError
            status = str(event.get("status") or "")
            content = str(event.get("content") or "")
            if status == "streaming" and content:
                answer_parts.append(content)
                await append_run_event(run_id, "delta", {"content": content})
            elif status == "answer_reset":
                answer_parts.clear()
                detail = event.get("detail") if isinstance(event.get("detail"), dict) else {}
                await append_run_event(run_id, "reset", {
                    "reason": str(detail.get("reason") or "answer_replaced"),
                    "message": "终答连接中断，已自动重新生成完整答复",
                })
            elif status in PUBLIC_PHASES:
                phase, message = PUBLIC_PHASES[status]
                current_phase = phase
                if status == "expert_calling":
                    expert_calls += 1
                if status == "tool_complete":
                    tool_calls += 1
                payload: dict[str, Any] = {"phase": phase, "message": message, "agent_status": status}
                detail = event.get("detail") if isinstance(event.get("detail"), dict) else {}
                if status == "expert_calling":
                    payload["expert"] = {
                        "expert": str(detail.get("expert") or ""),
                        "name": str(detail.get("name_zh") or "专家"),
                        "status": "running",
                    }
                elif status == "expert_complete" and isinstance(detail.get("opinion"), dict):
                    tool_calls += len(detail["opinion"].get("tool_results") or [])
                    payload["expert"] = _public_expert_trace(detail["opinion"])
                    await persist_expert_consultation(run_id, payload["expert"])
                await append_run_event(run_id, "status", payload)

        answer = "".join(answer_parts).strip()
        if not answer:
            raise RuntimeError("Agent completed without a final answer")
        credits = max(1, math.ceil((len(query) + len(answer)) / 1000) + expert_calls * 2 + tool_calls)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is None:
                return
            assistant = Message(
                conversation_id=conversation_id,
                run_id=run_id,
                role="assistant",
                content=answer,
                status="complete",
            )
            session.add(assistant)
            await session.flush()
            run.assistant_message_id = assistant.id
            run.response = answer
            run.status = "completed"
            run.finished_at = utcnow()
            conversation = await session.get(Conversation, conversation_id)
            if conversation is not None:
                conversation.last_active_at = utcnow()
                if conversation.title == "新会诊":
                    conversation.title = query[:60]
            billed_credits = await settle_credits(session, run_id=run_id, actual_amount=credits)
            run.actual_credits = billed_credits
            session.add(UsageRecord(
                run_id=run_id,
                user_id=user_id,
                expert_calls=expert_calls,
                tool_calls=tool_calls,
                credits=billed_credits,
                details={"finish_reason": orchestrator.last_finish_reason, "calculated_credits": credits},
            ))
            await session.commit()
        memory_synced = True
        try:
            if memory_write_enabled:
                await write_user_memory(
                    user_id=user_id,
                    query=query,
                    answer=answer,
                    pet_id=None,
                    session_id=conversation_id,
                    turn_id=run_id,
                )
        except Exception as exc:  # noqa: BLE001
            # The answer is already durable. Memory synchronization must not
            # retroactively turn a successful consultation into a failed run.
            memory_synced = False
            logger.exception("memory sync failed after completed run run_id=%s", run_id)
            await append_run_event(run_id, "warning", {
                "code": "memory_sync_failed",
                "message": "回答已完成，但长期记忆同步失败，系统将稍后重试",
            })
        await append_run_event(run_id, "completed", {
            "finish_reason": orchestrator.last_finish_reason,
            "credits": billed_credits,
            "memory_synced": memory_synced,
        })
    except asyncio.CancelledError:
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is not None:
                run.status = "cancelled"
                run.finished_at = utcnow()
                await settle_credits(session, run_id=run_id, actual_amount=0)
                await session.commit()
        await append_run_event(run_id, "cancelled", {"message": "任务已取消"})
    except Exception as exc:  # noqa: BLE001
        logger.exception("platform run failed run_id=%s phase=%s", run_id, current_phase)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is not None:
                run.status = "failed"
                run.error_code = "agent_run_failed"
                run.error_message = str(exc)[:1000]
                run.finished_at = utcnow()
                await settle_credits(session, run_id=run_id, actual_amount=0)
                await session.commit()
        await append_run_event(run_id, "failed", {
            "code": "agent_run_failed",
            "message": "终答生成失败，请重试" if current_phase == "generating" else "会诊任务执行失败",
            "phase": current_phase,
            "retryable": current_phase == "generating",
        })


async def run_event_stream(run_id: str, *, after_sequence: int = 0) -> AsyncIterator[str]:
    sequence = max(0, after_sequence)
    while True:
        async with platform_session() as session:
            rows = list((await session.scalars(select(RunEvent).where(
                RunEvent.run_id == run_id,
                RunEvent.sequence > sequence,
            ).order_by(RunEvent.sequence.asc()))).all())
            run = await session.get(AgentRun, run_id)
        for row in rows:
            sequence = row.sequence
            data = json.dumps(row.payload, ensure_ascii=False, default=str)
            yield f"id: {row.sequence}\nevent: {row.event_type}\ndata: {data}\n\n"
        if run is None or (run.status in TERMINAL_RUN_STATES and not rows):
            break
        yield ": keep-alive\n\n"
        await asyncio.sleep(0.5)


async def wait_for_run(run_id: str, timeout_seconds: int) -> AgentRun | None:
    deadline = asyncio.get_running_loop().time() + timeout_seconds
    while asyncio.get_running_loop().time() < deadline:
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is None or run.status in TERMINAL_RUN_STATES:
                return run
        await asyncio.sleep(0.4)
    return None


async def worker_forever() -> None:
    settings = get_platform_settings()
    if not settings.redis_url:
        raise RuntimeError("AGENT_PLATFORM_REDIS_URL is required for the production worker")
    from redis.asyncio import Redis

    # BLPOP intentionally waits while the queue is empty. Explicitly disable
    # socket read timeouts so redis-py 8 does not treat a normal empty poll as
    # a worker failure.
    redis = Redis.from_url(settings.redis_url, decode_responses=True, socket_timeout=None, health_check_interval=30)
    try:
        # Recover persisted work after API/Redis/worker restarts. Duplicate IDs
        # are harmless because execute_run claims only queued/retry states.
        async with platform_session() as session:
            recoverable = list((await session.scalars(select(AgentRun).where(
                AgentRun.status.in_(["queued", "running", "retry"])
            ))).all())
            for run in recoverable:
                if run.status == "running":
                    run.status = "retry"
                await redis.rpush("petmind:platform:runs", run.id)
            await session.commit()
        while True:
            item = await redis.blpop("petmind:platform:runs", timeout=5)
            if item:
                run_id = item[1]
                await execute_run(run_id)
                async with platform_session() as session:
                    completed = await session.get(AgentRun, run_id)
                    if completed is not None and completed.status == "failed":
                        await redis.rpush("petmind:platform:runs:dead", run_id)
    finally:
        await redis.aclose()
