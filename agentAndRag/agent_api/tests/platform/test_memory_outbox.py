import asyncio
from datetime import timedelta

from agent_api.app.platform.database import platform_session
from agent_api.app.platform.models import AgentRun, OutboxEvent, UserPreference, utcnow
from agent_api.app.platform.runs.memory_outbox import process_memory_outbox, schedule_memory
from agent_api.tests.platform.test_run_lifecycle import _setup, _teardown


def test_memory_retry_is_idempotent_and_respects_changed_preference(tmp_path, monkeypatch):
    async def scenario():
        user_id, run_id = await _setup(tmp_path, monkeypatch, 'memoryretry')
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            run.status, run.response = 'completed', 'test answer'
            schedule_memory(session, run)
            await session.commit()
        calls = []
        async def unavailable(**kwargs):
            calls.append(kwargs['turn_id'])
            return {'stored': False, 'reason': 'memory_unavailable'}
        await process_memory_outbox(run_id=run_id, writer=unavailable)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            event = await session.get(OutboxEvent, run_id)
            assert run.status == 'completed' and run.memory_status == 'pending'
            assert event.status == 'pending' and event.attempts == 1
            event.available_at = utcnow() - timedelta(seconds=1)
            session.add(UserPreference(user_id=user_id, memory_write_enabled=False))
            await session.commit()
        await process_memory_outbox(run_id=run_id, writer=unavailable)
        await process_memory_outbox(run_id=run_id, writer=unavailable)
        async with platform_session() as session:
            assert (await session.get(AgentRun, run_id)).memory_status == 'disabled'
            assert (await session.get(OutboxEvent, run_id)).status == 'done'
        assert calls == [run_id]
        await _teardown()
    asyncio.run(scenario())
