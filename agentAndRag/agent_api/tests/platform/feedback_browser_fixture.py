"""Manual browser fixture; never import into a deployed application.

Use an explicitly configured loopback feedback_test* PostgreSQL database.
seed requires PETMIND_TEST_BROWSER_PASSWORD and prints only synthetic record IDs.
worker replaces only the model adapter; Run persistence, queue, SSE and billing
remain the real implementation. Queries containing 取消 stream for 40 seconds.
"""
import argparse
import asyncio
from datetime import timedelta
import json
import os
from urllib.parse import urlsplit
import uuid

from agent_api.app.platform.database import close_platform_database, platform_session
from agent_api.app.platform.models import Conversation, CreditAccount, Message, PlatformUser, Subscription, UserPreference, utcnow
from agent_api.app.platform.security import hash_password
from agent_api.app.platform.services import seed_platform_plans, seed_platform_rbac


def validate_environment():
    parsed = urlsplit(os.environ['AGENT_PLATFORM_DB_URL'])
    assert parsed.hostname in ('localhost', '127.0.0.1')
    assert parsed.path.lstrip('/').startswith('feedback_test')
    assert urlsplit(os.environ['AGENT_PLATFORM_REDIS_URL']).hostname in ('localhost', '127.0.0.1')


async def seed():
    password = os.environ['PETMIND_TEST_BROWSER_PASSWORD']
    suffix = uuid.uuid4().hex[:8]
    result = []
    async with platform_session() as session:
        await seed_platform_plans(session)
        await seed_platform_rbac(session)
        for index in (1, 2):
            user = PlatformUser(email=f'feedback{index}-{suffix}@example.invalid', display_name=f'验收用户{index}',
                                password_hash=hash_password(password), role='VET')
            session.add(user); await session.flush()
            session.add(CreditAccount(user_id=user.id, balance=100000))
            session.add(Subscription(user_id=user.id, plan_code='trial'))
            session.add(UserPreference(user_id=user.id, memory_recall_enabled=False, memory_write_enabled=False))
            conversation = Conversation(user_id=user.id, title=f'回答评价验收 · 账号{index}')
            session.add(conversation); await session.flush()
            for i in range(144 if index == 1 else 2):
                session.add(Message(conversation_id=conversation.id, role='user' if i % 2 == 0 else 'assistant',
                                    content=f'合成病例记录 {i+1}：软件验收内容，仅供测试。',
                                    created_at=utcnow()-timedelta(minutes=150-i)))
            result.append({'email':user.email, 'user_id':user.id, 'conversation_id':conversation.id})
        await session.commit()
    print(json.dumps(result, ensure_ascii=False))


async def worker():
    from agent_api.app.platform.runs import service
    class Synthetic:
        last_finish_reason = 'stop'
        async def stream(self, **kwargs):
            for content in ['合成回答：', '本次测试验证', '流式展示、', '重写与取消。'] * (20 if '取消' in kwargs['query'] else 4):
                await asyncio.sleep(.5)
                yield {'status':'streaming', 'content':content}
    service.build_moe_orchestrator = lambda **kwargs: Synthetic()
    try:
        await service.worker_forever()
    finally:
        await service.close_run_queue_redis()


async def main(command):
    validate_environment()
    try:
        await (seed() if command == 'seed' else worker())
    finally:
        await close_platform_database()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['seed', 'worker'])
    asyncio.run(main(parser.parse_args().command))
