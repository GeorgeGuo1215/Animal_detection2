from __future__ import annotations

import asyncio
import json
import logging
import math
from datetime import timezone
from typing import Any, AsyncIterator

from sqlalchemy import func, select

from ...memory import load_user_memory, write_user_memory
from ...services.agent_execution import build_moe_orchestrator, public_moe_allowed_tools
from ...services.moe import MoETrace
from ...tools.tool_registry import get_registry
from ..config import get_platform_settings
from ..database import platform_session
from ..expert_consultations import persist_expert_consultation
from ..models import AgentRun, Conversation, Message, RunEvent, UsageRecord, UserPreference, utcnow
from ..services import settle_credits
from .public_trace import PUBLIC_PHASES, _public_expert_trace, _trace_node, _trace_nodes_for_agent_event


logger = logging.getLogger(__name__)

TERMINAL_RUN_STATES = {"completed", "failed", "cancelled"}


async def append_run_event(run_id: str, event_type: str, payload: dict[str, Any]) -> int:
    """追加一条 Run 事件并返回新的序号。"""
    async with platform_session() as session:
        last = await session.scalar(select(func.max(RunEvent.sequence)).where(RunEvent.run_id == run_id))
        sequence = int(last or 0) + 1
        session.add(RunEvent(run_id=run_id, sequence=sequence, event_type=event_type, payload=payload))
        await session.commit()
        return sequence


async def _run_cancel_requested(run_id: str) -> bool:
    """查询该 Run 是否已被请求取消；记录不存在时视为已取消。"""
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        return run is None or run.cancel_requested


async def enqueue_run(run_id: str) -> None:
    """将 Run 推入 Redis 队列；无 Redis 时在进程内创建后台任务执行。

    生产环境 Redis 失败会向上抛出，开发环境则回退到本地 ``execute_run``。
    """
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
    """执行一次排队中的 Agent Run：流式会诊、落库答复、结算积分并写记忆。

    仅认领 ``queued`` / ``retry`` 状态。取消、失败或终答生成异常会更新 Run 状态并释放预留积分。
    """
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
                reset_reason = str(detail.get("reason") or "answer_replaced")
                await append_run_event(run_id, "reset", {
                    "reason": reset_reason,
                    "message": (
                        "证据复核后已收缩未核实的具体结论"
                        if reset_reason == "evidence_safety_repair"
                        else "终答连接中断，已自动重新生成完整答复"
                    ),
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
                public_expert: dict[str, Any] | None = None
                if status == "expert_calling":
                    payload["expert"] = {
                        "expert": str(detail.get("expert") or ""),
                        "name": str(detail.get("name_zh") or "专家"),
                        "status": "running",
                    }
                elif status == "expert_complete" and isinstance(detail.get("opinion"), dict):
                    tool_calls += len(detail["opinion"].get("tool_results") or [])
                    public_expert = _public_expert_trace(detail["opinion"])
                    payload["expert"] = public_expert
                    await persist_expert_consultation(run_id, payload["expert"])
                await append_run_event(run_id, "status", payload)
                for trace_node in _trace_nodes_for_agent_event(
                    status,
                    detail,
                    public_expert=public_expert,
                ):
                    await append_run_event(run_id, "trace", trace_node)

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
                input_tokens=trace.total_prompt_tokens(),
                output_tokens=trace.total_completion_tokens(),
                expert_calls=expert_calls,
                tool_calls=tool_calls,
                credits=billed_credits,
                details={
                    "finish_reason": orchestrator.last_finish_reason,
                    "calculated_credits": credits,
                    "prompt_cache_hit_tokens": trace.total_prompt_cache_hit_tokens(),
                    "prompt_cache_miss_tokens": trace.total_prompt_cache_miss_tokens(),
                    "prompt_cache_hit_rate": round(trace.prompt_cache_hit_rate(), 6),
                },
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
            # 回答已持久化。记忆同步失败不得把成功会诊事后改成失败。
            memory_synced = False
            logger.exception("memory sync failed after completed run run_id=%s", run_id)
            await append_run_event(run_id, "warning", {
                "code": "memory_sync_failed",
                "message": "回答已完成，但长期记忆同步失败，系统将稍后重试",
            })
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "completed",
            details={"finish_reason": orchestrator.last_finish_reason},
        ))
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
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "cancelled", details={"message": "任务已取消"},
        ))
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
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "failed",
            details={"message": "终答生成失败" if current_phase == "generating" else "会诊执行失败"},
        ))
        await append_run_event(run_id, "failed", {
            "code": "agent_run_failed",
            "message": "终答生成失败，请重试" if current_phase == "generating" else "会诊任务执行失败",
            "phase": current_phase,
            "retryable": current_phase == "generating",
        })


async def run_event_stream(run_id: str, *, after_sequence: int = 0) -> AsyncIterator[str]:
    """以 SSE 文本帧推送指定序号之后的 Run 事件，直到进入终态。"""
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
    """轮询直到 Run 进入终态或超时；超时返回 ``None``，记录不存在则立即返回。"""
    deadline = asyncio.get_running_loop().time() + timeout_seconds
    while asyncio.get_running_loop().time() < deadline:
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is None or run.status in TERMINAL_RUN_STATES:
                return run
        await asyncio.sleep(0.4)
    return None


async def worker_forever() -> None:
    """生产 Worker：从 Redis 阻塞弹出 Run 并执行，启动时回收未完成任务。

    需要配置 ``AGENT_PLATFORM_REDIS_URL``。失败的 Run 会写入死信队列。
    """
    settings = get_platform_settings()
    if not settings.redis_url:
        raise RuntimeError("AGENT_PLATFORM_REDIS_URL is required for the production worker")
    from redis.asyncio import Redis

    # BLPOP 在空队列时会一直等待。显式关闭 socket 读超时，
    # 避免 redis-py 8 把正常的空轮询当成 Worker 失败。
    redis = Redis.from_url(settings.redis_url, decode_responses=True, socket_timeout=None, health_check_interval=30)
    try:
        # 在 API/Redis/Worker 重启后回收已持久化的任务。
        # 重复 ID 无害，因为 execute_run 只会认领 queued/retry 状态。
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
