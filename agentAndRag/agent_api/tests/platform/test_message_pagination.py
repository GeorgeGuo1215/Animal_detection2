import asyncio
from datetime import timedelta
from types import SimpleNamespace

from agent_api.app.platform.database import platform_session
from agent_api.app.platform.models import AgentRun, Message, utcnow
from agent_api.app.routers.routes_platform_core import list_messages
from agent_api.tests.platform.test_run_lifecycle import _setup, _teardown


def test_latest_page_and_tied_timestamp_cursors_cover_history_once(tmp_path, monkeypatch):
    async def scenario():
        user, run_id = await _setup(tmp_path, monkeypatch, 'pages')
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            conversation = run.conversation_id
            same_time = utcnow() + timedelta(seconds=1)
            for i in range(130):
                session.add(Message(id=f'{i:032d}', conversation_id=conversation, role='assistant', content=f'evidence {i}', created_at=same_time, status='complete'))
            await session.commit()
        ids, cursor = [], None
        for page_number in range(3):
            async with platform_session() as session:
                page = await list_messages(conversation, limit=48, before=cursor, principal=SimpleNamespace(user_id=user), session=session)
            if page_number == 0:
                assert page['items'][-1]['content'] == 'evidence 129'
            ids.extend(item['id'] for item in page['items'])
            cursor = page['next_cursor']
        assert cursor is None
        assert len(ids) == len(set(ids)) == 131
        await _teardown()
    asyncio.run(scenario())
