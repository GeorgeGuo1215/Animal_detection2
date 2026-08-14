from __future__ import annotations

import asyncio

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import AgentRun, Conversation, CreditAccount, Message, PlatformUser
from agent_api.app.platform.services import adjust_credits, reserve_credits, settle_credits


def test_reservation_protects_concurrent_balance_and_settles_actual_usage(tmp_path, monkeypatch):
    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv("AGENT_PLATFORM_DB_URL", f"sqlite+aiosqlite:///{(tmp_path / 'credits.db').as_posix()}")
        reset_platform_settings_cache(); await init_platform_database()
        async with platform_session() as session:
            user = PlatformUser(email="credits@example.com", display_name="Credits", password_hash="not-used", role="VET", status="active")
            session.add(user); await session.flush()
            conversation = Conversation(user_id=user.id, title="credits")
            session.add(conversation); await session.flush()
            message = Message(conversation_id=conversation.id, client_message_id="credit-message", role="user", content="test")
            session.add(message); await session.flush()
            run = AgentRun(user_id=user.id, conversation_id=conversation.id, user_message_id=message.id, idempotency_key="credit-run", query="test", reserved_credits=10)
            session.add(run); await session.flush()
            await adjust_credits(session, user_id=user.id, amount=100, reason="test", reference_type="test", reference_id=None, idempotency_key="grant-100")
            await reserve_credits(session, user_id=user.id, run_id=run.id, amount=10)
            charged = await settle_credits(session, run_id=run.id, actual_amount=25)
            assert charged == 25
            account = await session.get(CreditAccount, user.id)
            assert account.balance == 75
            assert account.reserved == 0
            assert await settle_credits(session, run_id=run.id, actual_amount=25) == 0
            await session.commit()
        await close_platform_database(); reset_platform_settings_cache()
    asyncio.run(scenario())
