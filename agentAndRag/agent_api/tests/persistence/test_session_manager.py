from __future__ import annotations

import asyncio
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.persistence.session_manager import SessionManager, _default_db_path, select_complete_turn_context
from app.lifecycle_tasks.session_cleanup import SessionCleanupTask


def test_complete_turn_context_never_returns_orphan_messages():
    """验证完整轮次上下文不会返回孤儿消息。"""
    messages = [
        {"role": "user", "content": "u1"},
        {"role": "assistant", "content": "a1"},
        {"role": "user", "content": "u2"},
        {"role": "assistant", "content": "a2"},
        {"role": "user", "content": "unfinished"},
    ]

    selected = select_complete_turn_context(messages, max_turns=1, max_chars=1)

    assert selected == [
        {"role": "user", "content": "u2"},
        {"role": "assistant", "content": "a2"},
    ]


def test_default_session_db_is_inside_agent_and_rag_workspace():
    """验证默认会话库落在 agentAndRag 工作区内。"""
    assert _default_db_path() == (
        Path(__file__).resolve().parents[3] / "agent_api_logs" / "petmind_sessions.db"
    )


def test_session_persists_across_manager_instances(tmp_path):
    """验证会话能在不同管理器实例之间持久化。"""
    async def scenario():
        """本用例的异步执行体。"""
        db_path = tmp_path / "sessions.db"
        first = SessionManager(db_path=db_path, context_max_turns=2, context_max_chars=1000)
        session = await first.create({"channel": "test"})
        await first.commit_turn(
            session.session_id,
            user_message="第一轮",
            assistant_message="回答一",
            expert_context={"experts": [{"conclusion": "仅为推断"}]},
            tool_results=[{"tool_name": "rag.search", "ok": True}],
        )

        second = SessionManager(db_path=db_path, context_max_turns=2, context_max_chars=1000)
        messages, experts = await second.context(session.session_id)
        restored = await second.get(session.session_id, touch=False)

        assert [item["content"] for item in messages] == ["第一轮", "回答一"]
        assert experts[0]["turn_index"] == 1
        assert restored is not None
        assert restored.tool_results[0]["results"][0]["tool_name"] == "rag.search"

    asyncio.run(scenario())


def test_session_ttl_expires_persisted_row(tmp_path):
    """验证会话 TTL 到期后会让持久化行失效。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "ttl.db", ttl_seconds=0.01)
        session = await manager.create()
        session.last_active = time.time() - 1
        await asyncio.to_thread(manager._save_sync, session)
        manager._sessions.clear()

        assert await manager.get(session.session_id) is None
        assert await asyncio.to_thread(manager._load_sync, session.session_id) is None

    asyncio.run(scenario())


def test_same_session_lock_serializes_requests(tmp_path):
    """验证同一会话锁会把请求串行化。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "locks.db")
        session = await manager.create()
        active = 0
        max_active = 0

        async def worker():
            """线程或协程里的工作函数。"""
            nonlocal active, max_active
            async with manager.session_lock(session.session_id):
                active += 1
                max_active = max(max_active, active)
                await asyncio.sleep(0.02)
                active -= 1

        await asyncio.gather(worker(), worker(), worker())
        assert max_active == 1

    asyncio.run(scenario())


def test_manual_cleanup_removes_expired_persisted_row(tmp_path):
    """验证手动清理会删掉已过期的持久化行。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "cleanup.db", ttl_seconds=0.01)
        session = await manager.create()
        session.last_active = time.time() - 1
        await asyncio.to_thread(manager._save_sync, session)

        removed = await manager.cleanup()

        assert removed == [session.session_id]
        assert await asyncio.to_thread(manager._load_sync, session.session_id) is None

    asyncio.run(scenario())


def test_cleanup_skips_active_and_waiting_session_locks(tmp_path):
    """验证清理会跳过仍在活跃或等待锁的会话。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "active.db", ttl_seconds=0.01)
        session = await manager.create()
        session.last_active = time.time() - 1
        await asyncio.to_thread(manager._save_sync, session)
        first_entered = asyncio.Event()
        release_first = asyncio.Event()

        async def holder():
            """占住资源直到释放。"""
            async with manager.session_lock(session.session_id):
                first_entered.set()
                await release_first.wait()

        async def waiter():
            """等待限流名额的协程。"""
            async with manager.session_lock(session.session_id):
                return

        holder_task = asyncio.create_task(holder())
        await first_entered.wait()
        waiter_task = asyncio.create_task(waiter())
        await asyncio.sleep(0)

        assert await manager.cleanup() == []
        assert await asyncio.to_thread(manager._load_sync, session.session_id) is not None

        release_first.set()
        await asyncio.gather(holder_task, waiter_task)
        assert await manager.cleanup() == [session.session_id]

    asyncio.run(scenario())


def test_active_session_can_outlive_ttl_and_commit(tmp_path):
    """验证活跃会话可以活过 TTL 并仍能成功提交。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "long_request.db", ttl_seconds=0.01)
        session = await manager.create()

        async with manager.session_lock(session.session_id):
            session.last_active = time.time() - 1
            await asyncio.to_thread(manager._save_sync, session)
            messages, experts = await manager.context(session.session_id)
            committed = await manager.commit_turn(
                session.session_id,
                user_message="长请求",
                assistant_message="完成回答",
            )

        assert messages == []
        assert experts == []
        assert committed is not None
        restored = await manager.get(session.session_id, touch=False)
        assert restored is not None
        assert restored.messages[-1]["content"] == "完成回答"

    asyncio.run(scenario())


def test_capacity_cleanup_defers_active_session_eviction(tmp_path):
    """验证容量清理会推迟驱逐仍在活跃的会话。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "capacity.db", max_sessions=1)
        active = await manager.create()

        async with manager.session_lock(active.session_id):
            newer = await manager.create()
            assert await asyncio.to_thread(manager._load_sync, active.session_id) is not None
            assert await asyncio.to_thread(manager._load_sync, newer.session_id) is not None

        removed = await manager.cleanup()
        assert removed == [active.session_id]
        assert await asyncio.to_thread(manager._load_sync, newer.session_id) is not None

    asyncio.run(scenario())


def test_periodic_cleanup_task_starts_runs_and_stops(tmp_path):
    """验证周期清理任务能启动、运行并停止。"""
    async def scenario():
        """本用例的异步执行体。"""
        manager = SessionManager(db_path=tmp_path / "periodic.db", ttl_seconds=0.01)
        session = await manager.create()
        session.last_active = time.time() - 1
        await asyncio.to_thread(manager._save_sync, session)
        task = SessionCleanupTask(manager, interval_seconds=0.01)

        await task.start()
        assert task.running
        await asyncio.sleep(0.03)
        assert await asyncio.to_thread(manager._load_sync, session.session_id) is None
        await task.stop()
        assert not task.running

    asyncio.run(scenario())
