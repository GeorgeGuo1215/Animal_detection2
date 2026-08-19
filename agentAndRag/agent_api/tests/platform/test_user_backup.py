from __future__ import annotations

import asyncio

from sqlalchemy import select

from agent_api.app.platform import user_backup
from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import (
    close_platform_database,
    init_platform_database,
    platform_session,
)
from agent_api.app.platform.models import (
    AgentRun,
    Conversation,
    ExpertConsultation,
    Message,
    PlatformUser,
    RunEvent,
)
from agent_api.app.platform.security import hash_password


def test_conversation_snapshot_overwrite_restore(tmp_path, monkeypatch):
    async def scenario():
        await close_platform_database()
        monkeypatch.setenv(
            "AGENT_PLATFORM_DB_URL",
            f"sqlite+aiosqlite:///{(tmp_path / 'backup.db').as_posix()}",
        )
        monkeypatch.setenv(
            "AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes"
        )
        reset_platform_settings_cache()
        await init_platform_database()
        async with platform_session() as session:
            user = PlatformUser(
                email="backup@example.com",
                display_name="Backup Vet",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
            )
            session.add(user)
            await session.flush()
            conversation = Conversation(user_id=user.id, title="原始会诊")
            session.add(conversation)
            await session.flush()
            question = Message(
                conversation_id=conversation.id, role="user", content="猫频繁蹲盆"
            )
            session.add(question)
            await session.flush()
            run = AgentRun(
                user_id=user.id,
                conversation_id=conversation.id,
                user_message_id=question.id,
                idempotency_key="backup-run-idem",
                status="completed",
                query=question.content,
                response="优先排查尿闭",
            )
            session.add(run)
            await session.flush()
            answer = Message(
                conversation_id=conversation.id,
                run_id=run.id,
                role="assistant",
                content=run.response,
            )
            session.add(answer)
            await session.flush()
            run.assistant_message_id = answer.id
            session.add(RunEvent(run_id=run.id, sequence=1, event_type="done", payload={}))
            session.add(ExpertConsultation(
                run_id=run.id,
                conversation_id=conversation.id,
                expert_key="clinical",
                expert_name="兽医临床专家",
                conclusion="排查尿闭",
            ))
            await session.commit()
            user_id = user.id

        async with platform_session() as session:
            snapshot = await user_backup.export_records(session, user_id=user_id)
            assert snapshot["checksum"]
            conversation = await session.scalar(select(Conversation).where(
                Conversation.user_id == user_id
            ))
            conversation.title = "被修改"
            await session.commit()

        async with platform_session() as session:
            counts = await user_backup.restore_records(
                session, user_id=user_id, snapshot=snapshot
            )
            await session.commit()
            assert counts["platform_conversations"] == 1
            restored = await session.scalar(select(Conversation).where(
                Conversation.user_id == user_id
            ))
            assert restored.title == "原始会诊"
            assert len(list((await session.scalars(select(ExpertConsultation))).all())) == 1
            assert len(list((await session.scalars(select(RunEvent))).all())) == 1

        broken = dict(snapshot)
        broken["checksum"] = "f" * 64
        async with platform_session() as session:
            try:
                await user_backup.restore_records(session, user_id=user_id, snapshot=broken)
            except ValueError as exc:
                assert "checksum" in str(exc)
            else:
                raise AssertionError("corrupted platform snapshot must be rejected")
        await close_platform_database()
        reset_platform_settings_cache()

    asyncio.run(scenario())
