import asyncio
from datetime import timedelta

import pytest

from agent_api.app.platform.database import platform_session
from agent_api.app.platform.models import AgentRun, CreditAccount, utcnow
from agent_api.app.platform.runs import service
from agent_api.app.platform.runs.ownership import ExecutionLease, LeaseLost, current_lease
from agent_api.tests.platform.test_run_lifecycle import _setup, _teardown, _events, _FakeOrchestrator, _install_fake_agent


class Queue:
    def __init__(self):
        self.ids = []

    async def eval(self, script, numkeys, key, run_id):
        self.ids.append(run_id)


def test_recovery_does_not_steal_live_lease_and_fences_old_attempt(tmp_path, monkeypatch):
    async def scenario():
        _, run_id = await _setup(tmp_path, monkeypatch, "lease")
        assert await service.claim_run(run_id)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            old = ExecutionLease(run_id, run.claimed_by, run.execution_epoch)
        queue = Queue()
        await service._recover_unfinished_runs(queue)
        assert run_id not in queue.ids
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            run.lease_until = utcnow() - timedelta(seconds=1)
            await session.commit()
        await service._recover_unfinished_runs(queue)
        assert queue.ids.count(run_id) == 1
        assert await service.claim_run(run_id)
        token = current_lease.set(old)
        try:
            with pytest.raises(LeaseLost):
                await service.append_run_event(run_id, "delta", {"content": "stale"})
            with pytest.raises(LeaseLost):
                await service._finalize_failed(run_id, RuntimeError("stale failure"))
        finally:
            current_lease.reset(token)
        async with platform_session() as session:
            assert (await session.get(AgentRun, run_id)).status == "running"
        await _teardown()
    asyncio.run(scenario())


def test_terminal_event_is_durable_while_memory_is_blocked(tmp_path, monkeypatch):
    async def scenario():
        _, run_id = await _setup(tmp_path, monkeypatch, "slowmemory")
        gate = asyncio.Event()
        _install_fake_agent(monkeypatch, _FakeOrchestrator([{"status": "streaming", "content": "answer"}]))
        async def memory(**kwargs):
            await gate.wait()
            return {"stored": True}
        monkeypatch.setattr(service, "write_user_memory", memory)
        await asyncio.wait_for(service.execute_run(run_id), 3)
        frames = [frame async for frame in service.run_event_stream(run_id)]
        assert "event: completed" in frames[-1]
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.memory_status == "pending" and run.status == "completed"
        before = len(await _events(run_id))
        await service._finalize_cancelled(run_id)
        await service._finalize_failed(run_id, RuntimeError("late failure"))
        assert len(await _events(run_id)) == before
        gate.set()
        await asyncio.gather(*list(service._LOCAL_RUN_TASKS))
        async with platform_session() as session:
            assert (await session.get(AgentRun, run_id)).memory_status == "synced"
        await _teardown()
    asyncio.run(scenario())


def test_cancel_in_gap_after_last_check_wins_before_completion(tmp_path, monkeypatch):
    async def scenario():
        user_id, run_id = await _setup(tmp_path, monkeypatch, "lastcancel")
        _install_fake_agent(monkeypatch, _FakeOrchestrator([{"status": "streaming", "content": "answer"}]))
        async def checked(self, *, force=False):
            if force:
                async with platform_session() as session:
                    run = await session.get(AgentRun, run_id)
                    run.cancel_requested = True
                    run.status = "cancel_requested"
                    await session.commit()
            return False
        monkeypatch.setattr(service._CancelWatcher, "requested", checked)
        await service.execute_run(run_id)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            assert run.status == "cancelled" and run.assistant_message_id is None
            account = await session.get(CreditAccount, user_id)
            assert account.balance == 100 and account.reserved == 0
        assert [e.event_type for e in await _events(run_id)][-1] == "cancelled"
        await _teardown()
    asyncio.run(scenario())
