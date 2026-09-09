"""Opt-in ASGI/real PostgreSQL+Redis latency and SQL-count benchmark.

Set AGENT_PLATFORM_DB_URL to a disposable feedback_test* database and
AGENT_PLATFORM_REDIS_URL to loopback Redis. Pass a package prefix, e.g.
agent_api.app or baseline_app (an unmodified copy of the earlier app package).
No model calls. Removes its own records and quota namespace on completion.
"""
import argparse
import asyncio
import importlib
import json
import os
from pathlib import Path
import statistics
import time
from urllib.parse import urlsplit
import uuid

import httpx
from redis.asyncio import Redis
from sqlalchemy import delete, event


async def measure(package: str):
    database_url = os.environ['AGENT_PLATFORM_DB_URL']
    redis_url = os.environ['AGENT_PLATFORM_REDIS_URL']
    assert urlsplit(database_url).hostname in ('127.0.0.1', 'localhost')
    assert urlsplit(database_url).path.lstrip('/').startswith('feedback_test')
    assert urlsplit(redis_url).hostname in ('127.0.0.1', 'localhost')
    prefix = 'feedback-benchmark:' + uuid.uuid4().hex
    os.environ.update(AGENT_PLATFORM_AUTO_CREATE_SCHEMA='1', AGENT_PLATFORM_RATE_LIMIT='10000',
                      AGENT_PLATFORM_RATE_BURST='10000', AGENT_HTTP_RATE_PREFIX=prefix)
    for group in ('IP', 'READ', 'WRITE', 'FEEDBACK', 'GENERATION', 'CONTROL'):
        os.environ[f'AGENT_HTTP_{group}_RATE'] = '10000'
        os.environ[f'AGENT_HTTP_{group}_BURST'] = '10000'
    def module(name):
        return importlib.import_module(package + '.' + name)
    models, db, security = module('platform.models'), module('platform.database'), module('platform.security')
    app = module('main').app
    await db.init_platform_database()
    async with db.platform_session() as session:
        user = models.PlatformUser(email=uuid.uuid4().hex+'@example.invalid', password_hash='unused', role='VET')
        session.add(user); await session.flush()
        conversation = models.Conversation(user_id=user.id, title='benchmark')
        session.add(conversation); await session.flush()
        messages = [models.Message(conversation_id=conversation.id, role='assistant', content='Synthetic benchmark answer') for _ in range(48)]
        session.add_all(messages); await session.commit()
        token = security.create_access_token(user_id=user.id, role=user.role, token_version=user.token_version)
    count = [0]
    def count_sql(*args):
        count[0] += 1
    engine = db.get_platform_engine().sync_engine
    event.listen(engine, 'before_cursor_execute', count_sql)
    result = {'package': package, 'samples_per_case': 400, 'concurrency': 4, 'cases': {}}
    try:
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test',
                                    headers={'Authorization':'Bearer '+token}) as client:
            for case in ('read', 'feedback'):
                async def request(i):
                    start = time.perf_counter()
                    response = await client.get(f'/api/v1/conversations/{conversation.id}/messages') if case == 'read' else await client.put(
                        f'/api/v1/messages/{messages[i % 4].id}/feedback', json={'rating':'up' if (i // 4) % 2 else 'down'})
                    assert response.status_code == 200, response.status_code
                    return (time.perf_counter() - start) * 1000
                for i in range(24):
                    await request(i)
                count[0] = 0
                times = []
                for start in range(0, 400, 4):
                    times.extend(await asyncio.gather(*(request(i) for i in range(start, start + 4))))
                result['cases'][case] = {'p50_ms':round(statistics.median(times),3), 'p95_ms':round(sorted(times)[379],3),
                                         'queries_per_request': count[0] / 400}
        return result
    finally:
        event.remove(engine, 'before_cursor_execute', count_sql)
        async with db.platform_session() as session:
            await session.execute(delete(models.AuditLog).where(models.AuditLog.actor_user_id == user.id))
            await session.execute(delete(models.PlatformUser).where(models.PlatformUser.id == user.id))
            await session.commit()
        limiter = getattr(app.state, 'http_limits', None)
        if limiter:
            await limiter.close()
        await db.close_platform_database()
        async with Redis.from_url(redis_url) as redis:
            keys = [key async for key in redis.scan_iter(match=prefix+':*')]
            if keys:
                await redis.delete(*keys)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', default='agent_api.app')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = asyncio.run(measure(args.package))
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result))
