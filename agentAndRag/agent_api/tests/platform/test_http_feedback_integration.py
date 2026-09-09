"""Real PostgreSQL/Redis contracts. Only explicitly configured loopback test DBs."""
import asyncio
import copy
import os
import uuid
from contextlib import AsyncExitStack, ExitStack
from urllib.parse import urlsplit

from alembic.migration import MigrationContext
from alembic.operations import Operations
from fastapi import FastAPI
import httpx
import pytest
from redis.asyncio import Redis
from sqlalchemy import select, text
from sqlalchemy.ext.asyncio import create_async_engine

from agent_api.app.http_limits import HttpLimits, HttpRateLimitMiddleware, install_http_limits
from agent_api.app.http_limits.backend import RedisBuckets, Bucket
from agent_api.app.http_limits.policies import DEFAULTS
from agent_api.app.middleware.auth import APIKeyAuthMiddleware
from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import AgentRun, ApiKey, AuditLog, Conversation, Message, PlatformUser
from agent_api.app.platform.security import create_access_token, hash_password, hash_secret
from agent_api.app.platform import user_backup
from agent_api.app.routers.routes_platform_conversation_actions import router as actions_router
from agent_api.app.routers.routes_platform_core import router as core_router
from agent_api.tests.platform.test_boolean_feedback import feedback_migration

PG = os.getenv('PETMIND_TEST_POSTGRES_URL', '')
REDIS = os.getenv('PETMIND_TEST_REDIS_URL', '')
pytestmark = pytest.mark.skipif(not PG or not REDIS, reason='explicit PostgreSQL/Redis test URLs required')


def validate_test_url():
    parsed = urlsplit(PG)
    assert parsed.hostname in ('localhost', '127.0.0.1')
    assert parsed.path.lstrip('/').startswith('feedback_test')
    assert urlsplit(REDIS).hostname in ('localhost', '127.0.0.1')


def test_postgres_feedback_migration_and_invalid_history():
    validate_test_url()
    async def scenario():
        engine = create_async_engine(PG)
        schema = 'migration_' + uuid.uuid4().hex
        async with engine.begin() as conn:
            await conn.execute(text(f'CREATE SCHEMA {schema}'))
            await conn.execute(text(f'SET LOCAL search_path TO {schema}'))
            await conn.execute(text('CREATE TABLE platform_messages (id int primary key, feedback_rating varchar(8), feedback_updated_at timestamptz)'))
            await conn.execute(text("INSERT INTO platform_messages VALUES (1,'up','2026-09-09T00:00:00Z'),(2,'down','2026-09-08T00:00:00Z'),(3,NULL,NULL)"))
            def run(sync):
                module = feedback_migration()
                module.op = Operations(MigrationContext.configure(sync))
                module.upgrade()
                assert sync.execute(text('SELECT feedback_is_good FROM platform_messages ORDER BY id')).scalars().all() == [True, False, None]
                assert sync.scalar(text("SELECT data_type FROM information_schema.columns WHERE table_schema=:schema AND table_name='platform_messages' AND column_name='feedback_is_good'"), {'schema': schema}) == 'boolean'
                module.downgrade()
                assert sync.execute(text('SELECT feedback_rating FROM platform_messages ORDER BY id')).scalars().all() == ['up','down',None]
                sync.execute(text("UPDATE platform_messages SET feedback_rating='invalid' WHERE id=1"))
                with pytest.raises(ValueError, match='historical'):
                    module.upgrade()
                assert sync.scalar(text('SELECT feedback_rating FROM platform_messages WHERE id=1')) == 'invalid'
            await conn.run_sync(run)
            await conn.execute(text(f'DROP SCHEMA {schema} CASCADE'))
        await engine.dispose()
    asyncio.run(scenario())


def test_real_feedback_concurrency_identity_limits_and_snapshot(monkeypatch):
    validate_test_url()
    monkeypatch.setenv('AGENT_PLATFORM_DB_URL', PG)
    monkeypatch.setenv('AGENT_PLATFORM_REDIS_URL', REDIS)
    monkeypatch.setenv('AGENT_PLATFORM_ENV', 'development')
    monkeypatch.setenv('AGENT_PLATFORM_AUTO_CREATE_SCHEMA', '1')
    monkeypatch.setenv('AGENT_PLATFORM_JWT_SECRET', 'feedback-integration-secret-with-thirty-two-bytes')
    monkeypatch.setenv('AGENT_DISABLE_AUTH', '0')
    reset_platform_settings_cache()

    async def scenario():
        await close_platform_database()
        await init_platform_database()
        prefix = 'feedback-integration:' + uuid.uuid4().hex
        async with platform_session() as session:
            owner = PlatformUser(email=uuid.uuid4().hex+'@example.invalid', display_name='Test', password_hash=hash_password('test-password-123'), role='VET')
            other = PlatformUser(email=uuid.uuid4().hex+'@example.invalid', display_name='Other', password_hash=owner.password_hash, role='VET')
            session.add_all([owner, other]); await session.flush()
            conversation = Conversation(user_id=owner.id, title='Synthetic integration')
            session.add(conversation); await session.flush()
            messages = [Message(conversation_id=conversation.id, role='assistant', content=f'Synthetic {i}') for i in range(4)]
            messages.append(Message(conversation_id=conversation.id, role='assistant', content='unfinished', status='streaming'))
            session.add_all(messages)
            raw_keys = ['pm_live_'+uuid.uuid4().hex+'_'+uuid.uuid4().hex for _ in range(2)]
            session.add_all([ApiKey(user_id=owner.id, name='integration', key_prefix=k[8:16], key_hash=hash_secret(k), scopes=['models:read','chat:write']) for k in raw_keys])
            await session.commit()
            owner_id, other_id, conversation_id = owner.id, other.id, conversation.id
            token_version = owner.token_version
            ids = [m.id for m in messages]
            token = create_access_token(user_id=owner.id, role=owner.role, token_version=owner.token_version)
            other_token = create_access_token(user_id=other.id, role=other.role, token_version=other.token_version)

        def make_app():
            app = FastAPI()
            app.include_router(actions_router); app.include_router(core_router)
            @app.get('/v1/models')
            async def models():
                return {'data': []}
            app.add_middleware(APIKeyAuthMiddleware)
            limits = dict(DEFAULTS, write=(120, 100), feedback=(.001, 50), read=(.001, 2))
            service = HttpLimits(backend=RedisBuckets(REDIS, production=True), limits=limits, prefix=prefix)
            install_http_limits(app, service, add_middleware=False)
            app.add_middleware(HttpRateLimitMiddleware, service=service)
            return app, service
        app1, service1 = make_app(); app2, service2 = make_app()
        headers = {'Authorization': 'Bearer '+token}
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app1, client=('127.0.0.2',1)), base_url='http://test') as first, httpx.AsyncClient(transport=httpx.ASGITransport(app=app2, client=('127.0.0.3',1)), base_url='http://test') as second:
            path = f'/api/v1/messages/{ids[0]}/feedback'
            # Different app instances and IPs write one locked row.
            results = await asyncio.gather(*(c.put(path, headers=headers, json={'rating':'up'}) for c in [first,second]*4))
            assert all(r.status_code == 200 for r in results)
            assert len({r.json()['updated_at'] for r in results}) == 1
            async with platform_session() as session:
                assert (await session.get(Message, ids[0])).feedback_is_good is True
                logs = (await session.scalars(select(AuditLog).where(AuditLog.resource_id==ids[0],AuditLog.action=='message.feedback.updated'))).all()
                assert len(logs)==1
            assert (await second.put(path, headers=headers, json={'rating':'down'})).json()['rating']=='down'
            async with platform_session() as session:
                assert (await session.get(Message,ids[0])).feedback_is_good is False
                snapshot = await user_backup.export_records(session,user_id=owner_id)
                legacy = copy.deepcopy(snapshot); legacy['schema_version']=1
                for row in legacy['records']['platform_messages']:
                    value=row.pop('feedback_is_good'); row['feedback_rating']=None if value is None else 'up' if value else 'down'
                legacy['checksum']=user_backup.checksum({k:v for k,v in legacy.items() if k!='checksum'})
                await user_backup.restore_records(session,user_id=owner_id,snapshot=legacy); await session.commit()
                assert (await session.get(Message,ids[0])).feedback_is_good is False
            assert (await first.put(path,headers={'Authorization':'Bearer '+other_token},json={'rating':'up'})).status_code==404
            assert (await first.put(f'/api/v1/messages/{ids[-1]}/feedback',headers=headers,json={'rating':'up'})).status_code==400
            listed=await first.get(f'/api/v1/conversations/{conversation_id}/messages',headers=headers)
            assert next(m for m in listed.json()['items'] if m['id']==ids[0])['feedback_rating']=='down'
            # The read bucket already has one debit; two different API keys share it.
            assert (await first.get('/v1/models',headers={'Authorization':'Bearer '+raw_keys[0]})).status_code==200
            assert (await second.get('/v1/models',headers={'Authorization':'Bearer '+raw_keys[1]})).status_code==429
            assert (await second.get(f'/api/v1/conversations/{conversation_id}/messages',headers=headers)).status_code==429
            # Feedback remains independent of exhausted reads/generation.
            service1.limits['feedback']=(.001,2); service2.limits['feedback']=(.001,2)
            redis = Redis.from_url(REDIS,decode_responses=True)
            await redis.delete(service1.key('user:'+owner_id,'feedback'))
            await redis.hset(service1.key('user:'+owner_id,'generation'),mapping={'tokens':0,'updated':(await redis.time())[0]})
            async with platform_session() as session:
                user_message=Message(conversation_id=conversation_id,role='user',content='synthetic cancellation')
                session.add(user_message); await session.flush()
                run=AgentRun(user_id=owner_id,conversation_id=conversation_id,user_message_id=user_message.id,
                             query='synthetic cancellation',idempotency_key=uuid.uuid4().hex,status='running')
                session.add(run); await session.commit(); run_id=run.id
            from agent_api.app.concurrency import get_resource_limits
            from agent_api.app.platform.runs import service as run_service
            def forbidden(*args,**kwargs):
                raise AssertionError('feedback/cancel must not invoke agent resources')
            monkeypatch.setattr(run_service,'build_moe_orchestrator',forbidden)
            resources=get_resource_limits()
            async with AsyncExitStack() as slots:
                with ExitStack() as sync_slots:
                    for _ in range(resources.llm.limit):
                        await slots.enter_async_context(resources.llm.slot(timeout_s=.01))
                    for _ in range(resources.mcp.limit):
                        await slots.enter_async_context(resources.mcp.slot(timeout_s=.01))
                    for _ in range(resources.rag.limit):
                        sync_slots.enter_context(resources.rag.slot(timeout_s=.01))
                    before=resources.snapshot()
                    assert (await first.put(path,headers=headers,json={'rating':None})).status_code==200
                    cancelled=await second.delete(f'/api/v1/runs/{run_id}',headers=headers)
                    assert cancelled.status_code==202 and cancelled.json()['status']=='cancel_requested'
                    assert resources.snapshot()==before
            # Refresh token version of the same user does not reset the quota.
            refreshed=create_access_token(user_id=owner_id,role='VET',token_version=token_version)
            assert (await second.put(f'/api/v1/messages/{ids[1]}/feedback',headers={'Authorization':'Bearer '+refreshed},json={'rating':'down'})).status_code==200
            limited=await first.put(f'/api/v1/messages/{ids[2]}/feedback',headers=headers,json={'rating':'up'})
            assert limited.status_code==429 and int(limited.headers['Retry-After'])>0
            async with platform_session() as session:
                assert (await session.get(Message,ids[0])).feedback_is_good is None
                assert (await session.get(Message,ids[2])).feedback_is_good is None
                user=await session.get(PlatformUser,owner_id)
                user.status='disabled'; await session.commit()
            # Authentication validity is checked anew despite earlier successful requests.
            assert (await first.get(f'/api/v1/runs/{run_id}',headers=headers)).status_code==401
            # Redis keys expire and every shared budget remains bounded.
            key=service1.key('expiry-test','fixed')
            await service1.backend.check([Bucket(key,0,1,1)])
            assert 0 <= await redis.ttl(key) <= 1
            await asyncio.sleep(1.1)
            assert not await redis.exists(key)
            client_id=await service1.backend.client.client_id()
            await service1.close(); await service2.close()
            assert all(int(client['id'])!=client_id for client in await redis.client_list())
            keys=[k async for k in redis.scan_iter(match=prefix+':*')]
            if keys: await redis.delete(*keys)
            await redis.aclose()
        async with platform_session() as session:
            from sqlalchemy import delete
            await session.execute(delete(AuditLog).where(AuditLog.actor_user_id.in_([owner_id,other_id])))
            await session.execute(delete(PlatformUser).where(PlatformUser.id.in_([owner_id,other_id])))
            await session.commit()
        await close_platform_database()
    try:
        asyncio.run(scenario())
    finally:
        reset_platform_settings_cache()
