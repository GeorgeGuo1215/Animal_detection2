"""平台 Run 生命周期：原子认领、取消竞态、事件序号、SSE 终态与 Worker 停机语义。"""
from __future__ import annotations

import asyncio
import os
import uuid
from typing import Any, AsyncIterator

import pytest
from sqlalchemy import func, select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import AgentRun, Conversation, CreditAccount, Message, PlatformUser, RunEvent
from agent_api.app.platform.runs import service as run_service
from agent_api.app.platform.services import adjust_credits, reserve_credits


async def _setup(tmp_path, monkeypatch, name: str) -> tuple[str, str]:
    """初始化独立 SQLite 平台库，返回 (user_id, run_id)；Run 已预占 10 积分。"""
    await close_platform_database()
    database_url = os.getenv("PETMIND_TEST_POSTGRES_URL")
    if database_url:
        name = f"{name}-{uuid.uuid4().hex[:8]}"
    monkeypatch.setenv("AGENT_PLATFORM_DB_URL", database_url or f"sqlite+aiosqlite:///{(tmp_path / f'{name}.db').as_posix()}")
    monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
    monkeypatch.delenv("AGENT_PLATFORM_REDIS_URL", raising=False)
    reset_platform_settings_cache()
    await init_platform_database()
    async with platform_session() as session:
        user = PlatformUser(email=f"{name}@example.com", display_name=name, password_hash="x", role="VET", status="active")
        session.add(user); await session.flush()
        conversation = Conversation(user_id=user.id, title="新会诊")
        session.add(conversation); await session.flush()
        message = Message(conversation_id=conversation.id, client_message_id=f"{name}-msg", role="user", content="猫呕吐")
        session.add(message); await session.flush()
        run = AgentRun(
            user_id=user.id, conversation_id=conversation.id, user_message_id=message.id,
            idempotency_key=f"{name}-run", query="猫呕吐", status="queued", reserved_credits=10,
            parameters={"temperature": 0.3, "max_tokens": 500, "user_role": "veterinarian"},
        )
        session.add(run); await session.flush()
        await adjust_credits(session, user_id=user.id, amount=100, reason="grant", reference_type="test", reference_id=None, idempotency_key=f"{name}-grant")
        await reserve_credits(session, user_id=user.id, run_id=run.id, amount=10)
        await session.commit()
        return user.id, run.id


async def _teardown() -> None:
    await run_service.close_run_queue_redis()
    await close_platform_database()
    reset_platform_settings_cache()


class _FakeOrchestrator:
    """按预设事件序列产出的编排器替身。"""

    def __init__(self, events: list[dict[str, Any]], *, gate: asyncio.Event | None = None) -> None:
        self._events = events
        self._gate = gate
        self.last_finish_reason = "stop"

    async def stream(self, **_: Any) -> AsyncIterator[dict[str, Any]]:
        for event in self._events:
            if self._gate is not None:
                await self._gate.wait()
            yield event
            await asyncio.sleep(0)


def _install_fake_agent(monkeypatch, orchestrator: _FakeOrchestrator, *, memory_calls: list[str] | None = None) -> None:
    async def _load(**_: Any):
        return "", {}

    async def _write(**kwargs: Any):
        if memory_calls is not None:
            memory_calls.append(kwargs["turn_id"])
        return {"stored": True}

    monkeypatch.setattr(run_service, "load_user_memory", _load)
    monkeypatch.setattr(run_service, "write_user_memory", _write)
    monkeypatch.setattr(run_service, "build_moe_orchestrator", lambda **_: orchestrator)
    monkeypatch.setattr(run_service, "public_moe_allowed_tools", lambda names: list(names))


async def _events(run_id: str) -> list[RunEvent]:
    async with platform_session() as session:
        return list((await session.scalars(
            select(RunEvent).where(RunEvent.run_id == run_id).order_by(RunEvent.sequence.asc())
        )).all())


def test_claim_run_is_atomic_and_rejects_cancelled(tmp_path, monkeypatch):
    """同一 Run 只能被认领一次；已请求取消的 Run 不可认领。"""
    async def scenario() -> None:
        _, run_id = await _setup(tmp_path, monkeypatch, "claim")
        first, second = await asyncio.gather(run_service.claim_run(run_id), run_service.claim_run(run_id))
        assert sorted([first, second]) == [False, True]
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "running"
            run.status = "retry"
            run.cancel_requested = True
            await session.commit()
        assert await run_service.claim_run(run_id) is False
        await _teardown()
    asyncio.run(scenario())


def test_execute_run_completes_and_streams_terminal_event(tmp_path, monkeypatch):
    """正常完成：落库答复、结算积分、写记忆，SSE 以 completed 事件收尾。"""
    async def scenario() -> None:
        user_id, run_id = await _setup(tmp_path, monkeypatch, "complete")
        memory_calls: list[str] = []
        _install_fake_agent(monkeypatch, _FakeOrchestrator([
            {"status": "router_selected", "detail": {}},
            {"status": "streaming", "content": "建议先"},
            {"status": "streaming", "content": "禁食观察。"},
        ]), memory_calls=memory_calls)
        await run_service.execute_run(run_id)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "completed"
            assert run.response == "建议先禁食观察。"
            account = await session.get(CreditAccount, user_id)
            assert account.reserved == 0
            assert account.balance < 100
        frames = [frame async for frame in run_service.run_event_stream(run_id)]
        assert frames[-1].startswith("id: ") and "event: completed" in frames[-1]
        assert not any("keep-alive" in frame for frame in frames)
        # Completion is visible before the independently scheduled memory write finishes.
        await asyncio.gather(*list(run_service._LOCAL_RUN_TASKS))
        assert memory_calls == [run_id]
        # 二次执行不会重复认领
        await run_service.execute_run(run_id)
        assert len(memory_calls) == 1
        await _teardown()
    asyncio.run(scenario())


def test_user_cancel_during_stream_marks_cancelled_and_refunds(tmp_path, monkeypatch):
    """执行中用户取消：Run 置为 cancelled、退回预占，并写入 cancelled 事件。"""
    async def scenario() -> None:
        user_id, run_id = await _setup(tmp_path, monkeypatch, "cancel")
        gate = asyncio.Event()
        _install_fake_agent(monkeypatch, _FakeOrchestrator([
            {"status": "streaming", "content": "第一段"},
            {"status": "streaming", "content": "第二段"},
        ], gate=gate))
        monkeypatch.setattr(run_service, "_CANCEL_CHECK_INTERVAL_S", 0.0)
        task = asyncio.create_task(run_service.execute_run(run_id))
        await asyncio.sleep(0.05)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "running"
            run.cancel_requested = True
            run.status = "cancel_requested"
            await session.commit()
        gate.set()
        await task
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "cancelled"
            account = await session.get(CreditAccount, user_id)
            assert account.balance == 100 and account.reserved == 0
        assert [event.event_type for event in await _events(run_id)][-1] == "cancelled"
        await _teardown()
    asyncio.run(scenario())


def test_worker_shutdown_returns_run_to_retry_instead_of_cancelling(tmp_path, monkeypatch):
    """Worker 停机导致的任务取消不能伪装成用户取消：Run 回到 retry 且不退款。"""
    async def scenario() -> None:
        user_id, run_id = await _setup(tmp_path, monkeypatch, "shutdown")
        gate = asyncio.Event()
        _install_fake_agent(monkeypatch, _FakeOrchestrator([
            {"status": "streaming", "content": "部分答复"},
            {"status": "streaming", "content": "永远不会到达"},
        ], gate=gate))
        task = asyncio.create_task(run_service.execute_run(run_id))
        await asyncio.sleep(0.05)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "retry"
            account = await session.get(CreditAccount, user_id)
            assert account.reserved == 10
        assert await run_service.claim_run(run_id) is True
        await _teardown()
    asyncio.run(scenario())


def test_append_run_event_retries_on_sequence_conflict(tmp_path, monkeypatch):
    """并发追加事件时序号冲突应自动重试，最终序号连续且唯一。"""
    async def scenario() -> None:
        _, run_id = await _setup(tmp_path, monkeypatch, "events")
        await asyncio.gather(*(
            run_service.append_run_event(run_id, "delta", {"content": str(index)})
            for index in range(12)
        ))
        async with platform_session() as session:
            sequences = list((await session.scalars(
                select(RunEvent.sequence).where(RunEvent.run_id == run_id).order_by(RunEvent.sequence.asc())
            )).all())
            total = await session.scalar(select(func.count()).select_from(RunEvent).where(RunEvent.run_id == run_id))
        assert total == 12
        assert sequences == list(range(1, 13))
        await _teardown()
    asyncio.run(scenario())


def test_event_stream_waits_for_terminal_event_after_terminal_status(tmp_path, monkeypatch):
    """Run 状态先于终态事件提交时，SSE 不能提前断流。"""
    async def scenario() -> None:
        _, run_id = await _setup(tmp_path, monkeypatch, "stream")
        monkeypatch.setattr(run_service, "_STREAM_POLL_INTERVAL_S", 0.01)
        monkeypatch.setattr(run_service, "_TERMINAL_EVENT_GRACE_S", 5.0)
        await run_service.append_run_event(run_id, "delta", {"content": "a"})
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            run.status = "completed"
            await session.commit()

        async def _late_terminal() -> None:
            await asyncio.sleep(0.1)
            await run_service.append_run_event(run_id, "completed", {"finish_reason": "stop"})

        writer = asyncio.create_task(_late_terminal())
        frames = [frame async for frame in run_service.run_event_stream(run_id)]
        await writer
        assert any("event: delta" in frame for frame in frames)
        assert "event: completed" in frames[-1]
        await _teardown()
    asyncio.run(scenario())


def test_event_stream_gives_up_after_grace_when_no_terminal_event(tmp_path, monkeypatch):
    """历史脏数据（终态却没有终态事件）在宽限期后退出，不会永久挂起。"""
    async def scenario() -> None:
        _, run_id = await _setup(tmp_path, monkeypatch, "grace")
        monkeypatch.setattr(run_service, "_STREAM_POLL_INTERVAL_S", 0.01)
        monkeypatch.setattr(run_service, "_TERMINAL_EVENT_GRACE_S", 0.05)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            run.status = "failed"
            await session.commit()
        frames = await asyncio.wait_for(
            _collect(run_service.run_event_stream(run_id)), timeout=2.0,
        )
        assert all("keep-alive" in frame for frame in frames)
        await _teardown()
    asyncio.run(scenario())


async def _collect(stream: AsyncIterator[str]) -> list[str]:
    return [frame async for frame in stream]
