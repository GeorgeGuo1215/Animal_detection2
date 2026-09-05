from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import uuid
from datetime import timedelta
import time
from typing import Any, AsyncIterator, Optional

from sqlalchemy import or_, select, update

from ...memory import load_user_memory, write_user_memory
from ...services.agent_execution import build_moe_orchestrator, public_moe_allowed_tools
from ...services.moe import MoETrace
from ...tools.tool_registry import get_registry
from ..config import get_platform_settings
from ..database import platform_session
from ..expert_consultations import persist_expert_consultation
from ..models import AgentRun, Conversation, Message, RunEvent, UsageRecord, UserPreference, utcnow
from ..services import settle_credits
from .events import DeltaBuffer, append_events, append_in_session, commit_run, event_hub
from .ownership import ExecutionLease, LeaseLost, current_lease, database_now, db_clock, lock_run, ownership_predicates
from .memory_outbox import process_memory_outbox, schedule_memory
from .public_trace import PUBLIC_PHASES, _public_expert_trace, _trace_node, _trace_nodes_for_agent_event


logger = logging.getLogger(__name__)

TERMINAL_RUN_STATES = {"completed", "failed", "cancelled"}
TERMINAL_EVENT_TYPES = {"completed", "failed", "cancelled"}
CLAIMABLE_RUN_STATES = ("queued", "retry")

RUN_QUEUE_KEY = "petmind:platform:runs"
RUN_DEAD_QUEUE_KEY = "petmind:platform:runs:dead"

# 取消检查按时间节流：每个 delta 都查库会把单次会诊放大成上千次事务。
_CANCEL_CHECK_INTERVAL_S = 0.5
# Run 已进入终态但终态事件尚未落库时，SSE 最多再等待这么久。
_TERMINAL_EVENT_GRACE_S = 3.0
_STREAM_POLL_INTERVAL_S = 0.5
_WORKER_ERROR_BACKOFF_S = 2.0
_LEASE_SECONDS = max(30, int(os.getenv("AGENT_RUN_LEASE_SECONDS", "60")))
_HEARTBEAT_SECONDS = min(_LEASE_SECONDS / 3, max(1, int(os.getenv("AGENT_RUN_HEARTBEAT_SECONDS", "15"))))
_RECOVERY_SECONDS = max(1, int(os.getenv("AGENT_RUN_RECOVERY_SECONDS", "10")))
_MAX_ATTEMPTS = max(1, int(os.getenv("AGENT_RUN_MAX_ATTEMPTS", "3")))
_WORKER_ID = uuid.uuid4().hex


class RunCancelled(Exception):
    """用户显式请求取消当前 Run；与 Worker 停机触发的 ``asyncio.CancelledError`` 区分。"""


# --------------------------------------------------------------------------- redis

_QUEUE_REDIS: Any = None
_QUEUE_REDIS_URL: Optional[str] = None
_LOCAL_RUN_TASKS: set[asyncio.Task[None]] = set()


async def get_run_queue_redis() -> Any:
    """返回进程级共享的队列 Redis 客户端；配置变化时重建。"""
    global _QUEUE_REDIS, _QUEUE_REDIS_URL  # noqa: PLW0603
    settings = get_platform_settings()
    if not settings.redis_url:
        raise RuntimeError("AGENT_PLATFORM_REDIS_URL is not configured")
    if _QUEUE_REDIS is None or _QUEUE_REDIS_URL != settings.redis_url:
        from redis.asyncio import Redis

        if _QUEUE_REDIS is not None:
            await _QUEUE_REDIS.aclose()
        _QUEUE_REDIS = Redis.from_url(settings.redis_url, decode_responses=True)
        _QUEUE_REDIS_URL = settings.redis_url
    return _QUEUE_REDIS


async def close_run_queue_redis() -> None:
    """关闭共享队列客户端，并回收无 Redis 模式下的本地执行任务。"""
    global _QUEUE_REDIS, _QUEUE_REDIS_URL  # noqa: PLW0603
    client = _QUEUE_REDIS
    _QUEUE_REDIS = None
    _QUEUE_REDIS_URL = None
    if client is not None:
        await client.aclose()
    pending = [task for task in _LOCAL_RUN_TASKS if not task.done()]
    for task in pending:
        task.cancel()
    if pending:
        await asyncio.gather(*pending, return_exceptions=True)
    _LOCAL_RUN_TASKS.clear()
    await event_hub().close()


def _track_local_task(task: asyncio.Task[None]) -> None:
    """持有本地执行任务引用，避免被 GC 且能在关闭时统一取消。"""
    _LOCAL_RUN_TASKS.add(task)

    def _done(finished: asyncio.Task[None]) -> None:
        _LOCAL_RUN_TASKS.discard(finished)
        if finished.cancelled():
            return
        exc = finished.exception()
        if exc is not None:
            logger.error("local platform run task crashed: %r", exc)

    task.add_done_callback(_done)


# --------------------------------------------------------------------------- events

async def append_run_event(run_id: str, event_type: str, payload: dict[str, Any]) -> int:
    return await append_events(run_id, [(event_type, payload)])


class _CancelWatcher:
    """按时间节流地查询 Run 是否被请求取消。"""

    def __init__(self, run_id: str, interval_s: float | None = None) -> None:
        self.run_id = run_id
        self.interval_s = _CANCEL_CHECK_INTERVAL_S if interval_s is None else interval_s
        self._last_check = 0.0

    async def requested(self, *, force: bool = False) -> bool:
        now = time.monotonic()
        if not force and now - self._last_check < self.interval_s:
            return False
        self._last_check = now
        async with platform_session() as session:
            run = await session.get(AgentRun, self.run_id)
            return run is None or run.cancel_requested


# --------------------------------------------------------------------------- queue

async def enqueue_run(run_id: str) -> None:
    """将 Run 推入 Redis 队列；无 Redis 时在进程内创建后台任务执行。

    生产环境 Redis 失败会向上抛出，开发环境则回退到本地 ``execute_run``。
    """
    settings = get_platform_settings()
    if settings.redis_url:
        try:
            redis = await get_run_queue_redis()
            await redis.rpush(RUN_QUEUE_KEY, run_id)
            return
        except Exception:
            if settings.production:
                raise
            logger.warning("redis enqueue failed; falling back to in-process execution run_id=%s", run_id)
    _track_local_task(asyncio.create_task(execute_run(run_id), name=f"platform-run-{run_id}"))


async def claim_run(run_id: str) -> bool:
    async with platform_session() as session:
        now = await database_now(session)
        result = await session.execute(update(AgentRun).where(
            AgentRun.id == run_id, AgentRun.status.in_(CLAIMABLE_RUN_STATES),
            AgentRun.cancel_requested.is_(False), AgentRun.execution_epoch < _MAX_ATTEMPTS,
        ).values(status="running", started_at=now, claimed_by=_WORKER_ID,
                 lease_until=now + timedelta(seconds=_LEASE_SECONDS),
                 execution_epoch=AgentRun.execution_epoch + 1))
        await session.commit()
        return bool(result.rowcount)


async def _mark_run_retry(run_id: str) -> None:
    async with platform_session() as session:
        run = await lock_run(session, run_id)
        if run.status in TERMINAL_RUN_STATES:
            return
        if run.cancel_requested:
            await _finish_cancelled_in_session(session, run)
        else:
            await append_in_session(session, run_id, [("status", {
                "phase": "queued", "message": "会诊服务正在重启，任务将自动恢复",
            })], locked_run=run)
            run.status = "retry"
            run.claimed_by = None
            run.lease_until = None
        await commit_run(session, run_id)


async def _finish_cancelled_in_session(session, run: AgentRun) -> None:
    await append_in_session(session, run.id, [
        ("trace", _trace_node("answer", "answer", "cancelled", details={"message": "任务已取消"})),
        ("cancelled", {"message": "任务已取消"}),
    ], locked_run=run)
    run.status = "cancelled"
    run.finished_at = utcnow()
    run.claimed_by = None
    run.lease_until = None
    await settle_credits(session, run_id=run.id, actual_amount=0)


async def _finalize_cancelled(run_id: str) -> None:
    async with platform_session() as session:
        run = await lock_run(session, run_id)
        if run.status in TERMINAL_RUN_STATES:
            return
        await _finish_cancelled_in_session(session, run)
        await commit_run(session, run_id)


async def _finalize_failed(run_id: str, exc: Exception, *, code: str = "agent_run_failed") -> None:
    async with platform_session() as session:
        run = await lock_run(session, run_id)
        if run.status in TERMINAL_RUN_STATES:
            return
        if run.cancel_requested:
            await _finish_cancelled_in_session(session, run)
        else:
            await append_in_session(session, run_id, [("failed", {
                "code": code, "message": "会诊任务执行失败，请重试", "retryable": True,
            })], locked_run=run)
            run.status = "failed"
            run.error_code = code
            run.error_message = str(exc)[:1000]
            run.finished_at = utcnow()
            run.claimed_by = None
            run.lease_until = None
            await settle_credits(session, run_id=run_id, actual_amount=0)
        await commit_run(session, run_id)


async def _heartbeat(lease: ExecutionLease, task: asyncio.Task, lost: asyncio.Event) -> None:
    while True:
        await asyncio.sleep(_HEARTBEAT_SECONDS)
        try:
            async with platform_session() as session:
                now = await database_now(session)
                result = await session.execute(update(AgentRun).where(*ownership_predicates(session, lease)).values(
                    lease_until=now + timedelta(seconds=_LEASE_SECONDS)))
                await session.commit()
                if result.rowcount:
                    continue
        except Exception:
            logger.exception("run heartbeat failed; stopping this attempt run_id=%s", lease.run_id)
        lost.set()
        task.cancel()
        return


# --------------------------------------------------------------------------- execute

async def execute_run(run_id: str) -> None:
    if not await claim_run(run_id):
        return
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        if run is None:
            return
        lease = ExecutionLease(run_id, str(run.claimed_by), run.execution_epoch)
    token = current_lease.set(lease)
    buffer = DeltaBuffer(run_id)
    lost = asyncio.Event()
    task = asyncio.current_task()
    assert task is not None
    heartbeat = asyncio.create_task(_heartbeat(lease, task, lost), name=f"run-heartbeat:{run_id}")
    try:
        if lease.epoch > 1:
            await append_run_event(run_id, "reset", {"reason": "worker_retry", "message": "正在重新生成会诊答复"})
        await _execute_claimed(run_id, buffer)
    except RunCancelled:
        await _finalize_cancelled(run_id)
    except LeaseLost:
        logger.info("run attempt fenced out run_id=%s epoch=%s", run_id, lease.epoch)
    except asyncio.CancelledError:
        if lost.is_set():
            return
        try:
            await asyncio.shield(_mark_run_retry(run_id))
        except LeaseLost:
            pass
        except Exception:
            logger.exception("shutdown recovery deferred to lease expiry run_id=%s", run_id)
        raise
    except Exception as exc:
        logger.exception("platform run failed run_id=%s", run_id)
        try:
            await _finalize_failed(run_id, exc)
        except LeaseLost:
            pass
    finally:
        heartbeat.cancel()
        await asyncio.gather(heartbeat, return_exceptions=True)
        await buffer.close()
        current_lease.reset(token)


async def _execute_claimed(run_id: str, buffer: DeltaBuffer) -> None:
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        if run is None:
            return
        history_rows = list((await session.scalars(
            select(Message).where(
                Message.conversation_id == run.conversation_id,
                Message.status == "complete",
            ).order_by(Message.created_at.desc(), Message.id.desc()).limit(48)
        )).all())
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
            for item in reversed(history_rows)
            if item.id != run.user_message_id and item.role in {"user", "assistant"}
        ]

    await append_run_event(run_id, "status", {"phase": "queued", "message": "任务已进入会诊队列"})
    cancel_watcher = _CancelWatcher(run_id)
    answer_parts: list[str] = []
    expert_calls = 0
    tool_calls = 0
    current_phase = "queued"
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
        if await cancel_watcher.requested():
            raise RunCancelled
        status = str(event.get("status") or "")
        content = str(event.get("content") or "")
        if status == "streaming" and content:
            answer_parts.append(content)
            await buffer.add(content)
        elif status == "answer_reset":
            await buffer.flush()
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
            await buffer.flush()
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

    await buffer.flush()
    # 终答落库前做一次强制检查，避免用户在最后一个 delta 之后取消却仍被计费。
    if await cancel_watcher.requested(force=True):
        raise RunCancelled

    answer = "".join(answer_parts).strip()
    if not answer:
        raise RuntimeError("Agent completed without a final answer")
    credits = max(1, math.ceil((len(query) + len(answer)) / 1000) + expert_calls * 2 + tool_calls)
    async with platform_session() as session:
        run = await lock_run(session, run_id)
        if run.cancel_requested:
            raise RunCancelled
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
        if memory_write_enabled:
            schedule_memory(session, run)
        else:
            run.memory_status = "disabled"
        await append_in_session(session, run_id, [
            ("trace", _trace_node("answer", "answer", "completed", details={"finish_reason": orchestrator.last_finish_reason})),
            ("completed", {"finish_reason": orchestrator.last_finish_reason, "credits": billed_credits,
                           "memory_synced": False, "memory_status": run.memory_status}),
        ], locked_run=run)
        run.status = "completed"
        run.claimed_by = None
        run.lease_until = None
        await commit_run(session, run_id)
    # Maintenance handles production retries; development also makes immediate progress.
    if memory_write_enabled and not get_platform_settings().redis_url:
        _track_local_task(asyncio.create_task(process_memory_outbox(run_id=run_id, writer=write_user_memory),
                                           name=f"run-memory:{run_id}"))


# --------------------------------------------------------------------------- stream / wait

async def run_event_stream(run_id: str, *, after_sequence: int = 0) -> AsyncIterator[str]:
    """以 SSE 文本帧推送指定序号之后的 Run 事件，直到出现终态事件。

    新 Run 的终态与事件原子提交；仍以终态事件为退出条件，并为旧版本遗留的
    缺失终态事件提供有限宽限窗口。
    """
    async with event_hub().subscribe(run_id) as wake:
        sequence = max(0, after_sequence)
        terminal_seen_at: Optional[float] = None
        while True:
            wake.clear()
            async with platform_session() as session:
                rows = list((await session.scalars(select(RunEvent).where(
                    RunEvent.run_id == run_id,
                    RunEvent.sequence > sequence,
                ).order_by(RunEvent.sequence.asc()))).all())
                run = await session.get(AgentRun, run_id)
            saw_terminal_event = False
            for row in rows:
                sequence = row.sequence
                data = json.dumps(row.payload, ensure_ascii=False, default=str)
                yield f"id: {row.sequence}\nevent: {row.event_type}\ndata: {data}\n\n"
                if row.event_type in TERMINAL_EVENT_TYPES:
                    saw_terminal_event = True
            if run is None or saw_terminal_event:
                break
            if run.status in TERMINAL_RUN_STATES:
                if terminal_seen_at is None:
                    terminal_seen_at = time.monotonic()
                elif time.monotonic() - terminal_seen_at >= _TERMINAL_EVENT_GRACE_S:
                    break
            yield ": keep-alive\n\n"
            timeout = _STREAM_POLL_INTERVAL_S if not get_platform_settings().database_url.startswith("postgresql") else 5.0
            if terminal_seen_at is not None:
                timeout = min(timeout, max(.001, _TERMINAL_EVENT_GRACE_S - (time.monotonic() - terminal_seen_at)))
            try:
                await asyncio.wait_for(wake.wait(), timeout=timeout)
            except TimeoutError:
                await event_hub().connect()


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


# --------------------------------------------------------------------------- worker

async def _recover_unfinished_runs(redis: Any) -> None:
    async with platform_session() as session:
        candidates = list((await session.scalars(select(AgentRun.id).where(
            AgentRun.status.in_((*CLAIMABLE_RUN_STATES, "running", "cancel_requested")),
            or_(AgentRun.lease_until.is_(None), AgentRun.lease_until <= db_clock(session)),
        ).order_by(AgentRun.updated_at).limit(200))).all())
    for run_id in candidates:
        requeue = False
        async with platform_session() as session:
            run = await lock_run(session, run_id, fenced=False)
            now = await database_now(session)
            expiry = run.lease_until
            if expiry and expiry.tzinfo is None:
                expiry = expiry.replace(tzinfo=now.tzinfo)
            if run.status in TERMINAL_RUN_STATES or (expiry and expiry > now):
                continue
            if run.cancel_requested or run.status == "cancel_requested":
                await _finish_cancelled_in_session(session, run)
            elif run.execution_epoch >= _MAX_ATTEMPTS:
                await append_in_session(session, run_id, [("failed", {
                    "code": "run_retry_exhausted", "message": "会诊恢复次数已达上限，请重新发送",
                })], locked_run=run)
                run.status = "failed"
                run.error_code = "run_retry_exhausted"
                run.finished_at = now
                run.claimed_by = None
                run.lease_until = None
                await settle_credits(session, run_id=run_id, actual_amount=0)
            else:
                if run.status == "running":
                    run.status = "retry"
                run.claimed_by = None
                run.lease_until = None
                requeue = True
            await commit_run(session, run_id)
        if requeue:
            # Avoid growing Redis on each reconciliation pass; CAS still guards delivery races.
            await redis.eval("if not redis.call('LPOS', KEYS[1], ARGV[1]) then return redis.call('RPUSH', KEYS[1], ARGV[1]) end return 0", 1, RUN_QUEUE_KEY, run_id)


async def _maintenance(redis: Any) -> None:
    while True:
        try:
            await _recover_unfinished_runs(redis)
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("platform maintenance iteration failed")
        await asyncio.sleep(_RECOVERY_SECONDS)


async def _memory_maintenance() -> None:
    while True:
        try:
            await process_memory_outbox()
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("memory outbox iteration failed")
        await asyncio.sleep(1)


async def worker_forever() -> None:
    settings = get_platform_settings()
    if not settings.redis_url:
        raise RuntimeError("AGENT_PLATFORM_REDIS_URL is required for the production worker")
    from redis.asyncio import Redis
    redis = Redis.from_url(settings.redis_url, decode_responses=True, socket_timeout=None, health_check_interval=30)
    maintenance = asyncio.create_task(_maintenance(redis), name="platform-run-maintenance")
    memory_task = asyncio.create_task(_memory_maintenance(), name="platform-memory-outbox")
    try:
        while True:
            try:
                item = await redis.blpop(RUN_QUEUE_KEY, timeout=5)
                if not item:
                    continue
                run_id = item[1]
                await execute_run(run_id)
                async with platform_session() as session:
                    completed = await session.get(AgentRun, run_id)
                    if completed is not None and completed.status == "failed":
                        await redis.rpush(RUN_DEAD_QUEUE_KEY, run_id)
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("platform worker iteration failed; retrying after backoff")
                await asyncio.sleep(_WORKER_ERROR_BACKOFF_S)
    finally:
        maintenance.cancel()
        memory_task.cancel()
        await asyncio.gather(maintenance, memory_task, return_exceptions=True)
        await redis.aclose()
