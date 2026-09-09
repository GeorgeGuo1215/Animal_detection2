"""Opt-in acceptance against two running API processes and their real test DB.

PETMIND_TEST_API_URLS is a comma-separated pair of loopback URLs. Processes must
share the DB, Redis namespace, feedback 30/minute burst 2, read 1/minute burst 2.
"""
import asyncio
import os
import uuid
from urllib.parse import urlsplit

import httpx
import pytest
from sqlalchemy import delete, select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, platform_session
from agent_api.app.platform.models import ApiKey, AuditLog, Conversation, Message, PlatformUser
from agent_api.app.platform.security import hash_password, hash_secret


URLS = os.getenv('PETMIND_TEST_API_URLS', '').split(',')
DB = os.getenv('PETMIND_TEST_PROCESS_DB_URL', '')
pytestmark = pytest.mark.skipif(len(URLS) != 2 or not DB, reason='two explicit API URLs and test DB required')


def test_two_processes_share_feedback_and_read_identity(monkeypatch):
    assert urlsplit(DB).hostname in ('127.0.0.1', 'localhost')
    assert urlsplit(DB).path.lstrip('/').startswith('feedback_test')
    assert all(urlsplit(url).hostname in ('127.0.0.1', 'localhost') for url in URLS)
    monkeypatch.setenv('AGENT_PLATFORM_DB_URL', DB)
    reset_platform_settings_cache()

    async def scenario():
        await close_platform_database()
        password = uuid.uuid4().hex
        async with platform_session() as session:
            user = PlatformUser(email=uuid.uuid4().hex+'@example.invalid', password_hash=hash_password(password))
            session.add(user); await session.flush()
            conversation = Conversation(user_id=user.id, title='two-process test')
            session.add(conversation); await session.flush()
            messages = [Message(conversation_id=conversation.id, role='assistant', content='Synthetic') for _ in range(3)]
            session.add_all(messages)
            keys = ['pm_live_'+uuid.uuid4().hex for _ in range(2)]
            session.add_all([ApiKey(user_id=user.id, name='test', key_prefix=k[8:18], key_hash=hash_secret(k), scopes=['models:read']) for k in keys])
            await session.commit()
            user_id, email = user.id, user.email
            ids = [m.id for m in messages]
        try:
            async with httpx.AsyncClient(base_url=URLS[0]) as first, httpx.AsyncClient(base_url=URLS[1]) as second:
                login = await first.post('/api/v1/auth/login', json={'email':email, 'password':password})
                assert login.status_code == 200
                token = login.json()['access_token']
                headers = {'Authorization':'Bearer '+token, 'X-Forwarded-For':'127.10.0.1'}
                other_headers = {**headers, 'X-Forwarded-For':'127.10.0.2'}
                path = f'/api/v1/messages/{ids[0]}/feedback'
                pair = await asyncio.gather(first.put(path,headers=headers,json={'rating':'up'}), second.put(path,headers=other_headers,json={'rating':'down'}))
                assert [r.status_code for r in pair] == [200,200]
                async with platform_session() as session:
                    logs = (await session.scalars(select(AuditLog).where(AuditLog.resource_id==ids[0]).order_by(AuditLog.created_at))).all()
                    row = await session.get(Message,ids[0])
                    assert len(logs)==2
                    assert logs[0].detail['previous_is_good'] is None
                    assert logs[1].detail['previous_is_good'] is logs[0].detail['is_good']
                    assert row.feedback_is_good is logs[-1].detail['is_good']
                    assert row.feedback_updated_at <= logs[-1].created_at
                # A different message/IP/process still uses the same feedback quota.
                limited = await second.put(f'/api/v1/messages/{ids[1]}/feedback',headers=other_headers,json={'rating':'up'})
                assert limited.status_code==429 and int(limited.headers['Retry-After'])>=1
                assert limited.headers['X-RateLimit-Remaining']=='0'
                # Distinct API keys aggregate to the same stable user read quota.
                reached_limit = False
                for i in range(80):
                    response = await first.get('/v1/models',headers={'Authorization':'Bearer '+keys[0], 'X-Forwarded-For':'127.10.0.3'})
                    if response.status_code==429:
                        reached_limit=True;break
                    assert response.status_code==200
                assert reached_limit
                assert (await second.get('/v1/models',headers={'Authorization':'Bearer '+keys[1], 'X-Forwarded-For':'127.10.0.4'})).status_code==429
                assert (await second.get('/api/v1/conversations',headers=other_headers)).status_code==429
                await asyncio.sleep(4.1)
                # A newly issued login JWT does not create a different user bucket.
                new_login = await second.post('/api/v1/auth/login',json={'email':email,'password':password})
                renewed={'Authorization':'Bearer '+new_login.json()['access_token']}
                assert renewed['Authorization']!=headers['Authorization']
                pair=await asyncio.gather(first.put(path,headers=headers,json={'rating':'up'}),second.put(path,headers=renewed,json={'rating':'up'}))
                assert all(r.status_code==200 for r in pair)
                assert pair[0].json()['updated_at']==pair[1].json()['updated_at']
                assert (await first.put(f'/api/v1/messages/{ids[2]}/feedback',headers=renewed,json={'rating':'up'})).status_code==429
        finally:
            async with platform_session() as session:
                await session.execute(delete(AuditLog).where(AuditLog.actor_user_id==user_id))
                await session.execute(delete(PlatformUser).where(PlatformUser.id==user_id))
                await session.commit()
            await close_platform_database()
    try:
        asyncio.run(scenario())
    finally:
        reset_platform_settings_cache()
