from __future__ import annotations

import asyncio
from datetime import timedelta

import httpx
from fastapi import FastAPI
from sqlalchemy import select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import (
    close_platform_database,
    init_platform_database,
    platform_session,
)
from agent_api.app.platform.models import (
    AgentRun,
    AuditLog,
    Conversation,
    ExpertConsultation,
    Message,
    PlatformUser,
    RunEvent,
    UsageRecord,
    utcnow,
)
from agent_api.app.platform.security import create_access_token, hash_password
from agent_api.app.platform.services import grant_plan, seed_platform_plans, seed_platform_rbac
from agent_api.app.routers import routes_platform_core


def test_rewrite_truncates_later_history_and_creates_replacement_message(tmp_path, monkeypatch):
    """编辑用户消息会原子截断可见历史、保留审计数据并创建新的持久化任务。"""

    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv(
            "AGENT_PLATFORM_DB_URL",
            f"sqlite+aiosqlite:///{(tmp_path / 'message-rewrite.db').as_posix()}",
        )
        monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
        monkeypatch.setenv("AGENT_PLATFORM_ENV", "development")
        reset_platform_settings_cache()
        await init_platform_database()

        async def enqueue_noop(_run_id: str) -> None:
            return None

        monkeypatch.setattr(routes_platform_core, "enqueue_run", enqueue_noop)

        async with platform_session() as session:
            await seed_platform_plans(session)
            await seed_platform_rbac(session)
            owner = PlatformUser(
                email="rewrite-owner@example.com",
                display_name="Owner",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
                email_verified_at=utcnow(),
            )
            other = PlatformUser(
                email="rewrite-other@example.com",
                display_name="Other",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
                email_verified_at=utcnow(),
            )
            session.add_all([owner, other])
            await session.flush()
            trial = await session.get(
                __import__("agent_api.app.platform.models", fromlist=["Plan"]).Plan,
                "trial",
            )
            await grant_plan(
                session,
                user_id=owner.id,
                plan=trial,
                reference_type="test",
                reference_id="rewrite-owner",
            )
            await grant_plan(
                session,
                user_id=other.id,
                plan=trial,
                reference_type="test",
                reference_id="rewrite-other",
            )

            conversation = Conversation(user_id=owner.id, title="旧标题")
            other_conversation = Conversation(user_id=other.id, title="其他用户")
            session.add_all([conversation, other_conversation])
            await session.flush()
            created_at = utcnow()
            first_user = Message(
                conversation_id=conversation.id,
                client_message_id="original-user-message-0001",
                role="user",
                content="猫咪频繁蹲盆。",
                status="complete",
                created_at=created_at,
            )
            first_answer = Message(
                conversation_id=conversation.id,
                role="assistant",
                content="先排除尿道梗阻。",
                status="complete",
                created_at=created_at + timedelta(seconds=1),
            )
            follow_up = Message(
                conversation_id=conversation.id,
                client_message_id="original-user-message-0002",
                role="user",
                content="仍能排少量尿。",
                status="complete",
                created_at=created_at + timedelta(seconds=2),
            )
            second_answer = Message(
                conversation_id=conversation.id,
                role="assistant",
                content="仍不能完全排除梗阻。",
                status="complete",
                created_at=created_at + timedelta(seconds=3),
            )
            foreign_message = Message(
                conversation_id=other_conversation.id,
                client_message_id="foreign-user-message-0001",
                role="user",
                content="其他用户的问题",
                status="complete",
            )
            session.add_all([first_user, first_answer, follow_up, second_answer, foreign_message])
            await session.flush()

            first_run = AgentRun(
                user_id=owner.id,
                conversation_id=conversation.id,
                user_message_id=first_user.id,
                assistant_message_id=first_answer.id,
                idempotency_key="old-run-0001",
                status="completed",
                query=first_user.content,
            )
            second_run = AgentRun(
                user_id=owner.id,
                conversation_id=conversation.id,
                user_message_id=follow_up.id,
                assistant_message_id=second_answer.id,
                idempotency_key="old-run-0002",
                status="completed",
                query=follow_up.content,
            )
            session.add_all([first_run, second_run])
            await session.flush()
            first_answer.run_id = first_run.id
            second_answer.run_id = second_run.id
            session.add_all([
                RunEvent(run_id=first_run.id, sequence=1, event_type="completed", payload={}),
                ExpertConsultation(
                    run_id=first_run.id,
                    conversation_id=conversation.id,
                    expert_key="clinical",
                    expert_name="兽医临床专家",
                ),
                UsageRecord(run_id=first_run.id, user_id=owner.id, input_tokens=100),
            ])
            await session.commit()

            token = create_access_token(
                user_id=owner.id,
                role=owner.role,
                token_version=owner.token_version,
            )
            other_token = create_access_token(
                user_id=other.id,
                role=other.role,
                token_version=other.token_version,
            )
            conversation_id = conversation.id
            first_user_id = first_user.id
            foreign_message_id = foreign_message.id
            old_run_ids = {first_run.id, second_run.id}

        app = FastAPI()
        app.include_router(routes_platform_core.router)
        headers = {
            "Authorization": f"Bearer {token}",
            "Idempotency-Key": "rewrite-run-0001",
        }
        body = {
            "message": "猫频繁蹲盆且只能排出几滴尿，如何排急症？",
            "client_message_id": "rewritten-user-message-0001",
            "delivery": "async",
            "rewrite_message_id": first_user_id,
        }
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            rewritten = await client.post(
                f"/api/v1/conversations/{conversation_id}/runs",
                headers=headers,
                json=body,
            )
            assert rewritten.status_code == 202, rewritten.text
            new_run_id = rewritten.json()["run"]["id"]
            visible = await client.get(
                f"/api/v1/conversations/{conversation_id}/messages",
                headers={"Authorization": f"Bearer {token}"},
            )
            assert visible.status_code == 200
            visible_items = visible.json()["items"]
            assert [item["content"] for item in visible_items] == [body["message"]]
            replacement_message_id = visible_items[0]["id"]
            assert replacement_message_id != first_user_id

            replay = await client.post(
                f"/api/v1/conversations/{conversation_id}/runs",
                headers=headers,
                json=body,
            )
            assert replay.status_code == 202
            assert replay.json()["run"]["id"] == new_run_id

            active_conflict = await client.post(
                f"/api/v1/conversations/{conversation_id}/runs",
                headers={
                    "Authorization": f"Bearer {token}",
                    "Idempotency-Key": "rewrite-run-0002",
                },
                json={
                    **body,
                    "client_message_id": "rewritten-user-message-0002",
                    "rewrite_message_id": replacement_message_id,
                },
            )
            assert active_conflict.status_code == 409

            foreign_target = await client.post(
                f"/api/v1/conversations/{conversation_id}/runs",
                headers={
                    "Authorization": f"Bearer {token}",
                    "Idempotency-Key": "rewrite-run-foreign-target",
                },
                json={**body, "rewrite_message_id": foreign_message_id},
            )
            assert foreign_target.status_code == 404
            assert (
                await client.post(
                    f"/api/v1/conversations/{conversation_id}/runs",
                    headers={
                        "Authorization": f"Bearer {other_token}",
                        "Idempotency-Key": "rewrite-run-cross-user",
                    },
                    json=body,
                )
            ).status_code == 404

        async with platform_session() as session:
            messages = list((await session.scalars(
                select(Message)
                .where(Message.conversation_id == conversation_id)
                .order_by(Message.created_at.asc(), Message.id.asc())
            )).all())
            visible_messages = [item for item in messages if item.status != "superseded"]
            assert [(item.id, item.content) for item in visible_messages] == [
                (replacement_message_id, body["message"]),
            ]
            assert visible_messages[0].client_message_id == body["client_message_id"]
            assert len([item for item in messages if item.status == "superseded"]) == 4
            conversation = await session.get(Conversation, conversation_id)
            assert conversation.title == body["message"][:60]

            runs = list((await session.scalars(
                select(AgentRun).where(AgentRun.conversation_id == conversation_id)
            )).all())
            assert len(runs) == 3
            new_run = next(item for item in runs if item.id == new_run_id)
            assert new_run.user_message_id == replacement_message_id
            assert new_run.query == body["message"]
            assert old_run_ids.issubset({item.id for item in runs})
            assert list((await session.scalars(
                select(RunEvent).where(RunEvent.run_id.in_(old_run_ids))
            )).all())
            assert list((await session.scalars(
                select(ExpertConsultation).where(ExpertConsultation.run_id.in_(old_run_ids))
            )).all())
            assert list((await session.scalars(
                select(UsageRecord).where(UsageRecord.run_id.in_(old_run_ids))
            )).all())
            audit_row = await session.scalar(select(AuditLog).where(
                AuditLog.action == "message.rewritten",
                AuditLog.resource_id == first_user_id,
            ))
            assert audit_row is not None
            assert audit_row.detail["superseded_messages"] == 4
            assert audit_row.detail["preserved_runs"] == 2
            assert audit_row.detail["replacement_message_id"] == replacement_message_id

        await close_platform_database()
        reset_platform_settings_cache()

    asyncio.run(scenario())
