"""Optional real PostgreSQL acceptance: use a migrated, dedicated test database."""
import asyncio
import os
import time

import pytest
from sqlalchemy import select, func, text

from agent_api.app.platform.database import platform_session
from agent_api.app.platform.models import CreditAccount, CreditLedger
from agent_api.app.platform.runs.events import DeltaBuffer, EventHub, append_events
from agent_api.app.platform.services import adjust_credits
from agent_api.tests.platform.test_run_lifecycle import _setup, _teardown, _events

pytestmark = pytest.mark.skipif(not os.getenv('PETMIND_TEST_POSTGRES_URL'), reason='dedicated PostgreSQL URL required')


def test_concurrent_ledger_idempotency_and_contiguous_event_batches(tmp_path, monkeypatch):
    async def scenario():
        user, run = await _setup(tmp_path, monkeypatch, 'pg-concurrency')
        async def credit():
            async with platform_session() as session:
                await adjust_credits(session, user_id=user, amount=7, reason='concurrency', reference_type='test', reference_id=None, idempotency_key=f'{run}-same')
                await session.commit()
        await asyncio.gather(*(credit() for _ in range(16)))
        async with platform_session() as session:
            assert (await session.get(CreditAccount, user)).balance == 107
            assert await session.scalar(select(func.count()).select_from(CreditLedger).where(CreditLedger.idempotency_key == f'{run}-same')) == 1
        await asyncio.gather(*(append_events(run, [('status', {'phase': str(i)})]) for i in range(24)))
        buffer = DeltaBuffer(run)
        for _ in range(100):
            await buffer.add('x')
        await buffer.flush(); await buffer.close()
        rows = await _events(run)
        assert [r.sequence for r in rows] == list(range(1, 125))
        assert sum(r.event_type == 'delta' for r in rows) == 100
        await _teardown()
    asyncio.run(scenario())


def test_notifications_are_commit_only_and_shared(tmp_path, monkeypatch):
    async def scenario():
        _, run = await _setup(tmp_path, monkeypatch, 'pg-notify')
        hub = EventHub()
        async with hub.subscribe(run) as first, hub.subscribe(run) as second:
            connection = hub.connection
            async with platform_session() as session:
                await session.execute(text('select pg_notify(:channel, :run)'), {'channel': 'petmind_run_events', 'run': run})
                await session.rollback()
            await asyncio.sleep(.1)
            assert not first.is_set() and not second.is_set()
            started = time.perf_counter()
            await append_events(run, [('status', {'phase': 'ready'})])
            await asyncio.wait_for(asyncio.gather(first.wait(), second.wait()), 2)
            assert time.perf_counter() - started < 2
            assert hub.connection is connection
        assert not hub.listeners
        await hub.close(); await _teardown()
    asyncio.run(scenario())
