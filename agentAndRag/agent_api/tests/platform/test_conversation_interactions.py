from __future__ import annotations

import asyncio
from datetime import timedelta

import httpx
from fastapi import FastAPI
from sqlalchemy import select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import AuditLog, Conversation, Message, PlatformUser, utcnow
from agent_api.app.platform.security import create_access_token, hash_password
from agent_api.app.routers.routes_platform_conversation_actions import router as actions_router
from agent_api.app.routers.routes_platform_core import router as core_router


def test_feedback_fork_delete_and_cross_user_isolation(tmp_path, monkeypatch):
    """赞踩、分支和删除即时落库，并始终限制在当前用户数据内。"""

    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv(
            "AGENT_PLATFORM_DB_URL",
            f"sqlite+aiosqlite:///{(tmp_path / 'conversation-actions.db').as_posix()}",
        )
        monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
        monkeypatch.setenv("AGENT_PLATFORM_ENV", "development")
        reset_platform_settings_cache()
        await init_platform_database()

        async with platform_session() as session:
            owner = PlatformUser(
                email="owner@example.com",
                display_name="Owner",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
                email_verified_at=utcnow(),
            )
            other = PlatformUser(
                email="other@example.com",
                display_name="Other",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
                email_verified_at=utcnow(),
            )
            session.add_all([owner, other])
            await session.flush()
            conversation = Conversation(user_id=owner.id, title="猫泌尿会诊")
            session.add(conversation)
            await session.flush()
            message_time = utcnow()
            user_message = Message(
                conversation_id=conversation.id,
                role="user",
                content="猫频繁进出猫砂盆",
                status="complete",
                created_at=message_time,
            )
            assistant_message = Message(
                conversation_id=conversation.id,
                run_id="run-source-001",
                role="assistant",
                content="先排除尿道梗阻。",
                status="complete",
                created_at=message_time + timedelta(seconds=1),
            )
            later_message = Message(
                conversation_id=conversation.id,
                role="user",
                content="目前还能排少量尿。",
                status="complete",
                created_at=message_time + timedelta(seconds=2),
            )
            session.add_all([user_message, assistant_message, later_message])
            await session.commit()
            owner_token = create_access_token(
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
            user_message_id = user_message.id
            assistant_message_id = assistant_message.id

        app = FastAPI()
        app.include_router(core_router)
        app.include_router(actions_router)
        owner_headers = {"Authorization": f"Bearer {owner_token}"}
        other_headers = {"Authorization": f"Bearer {other_token}"}
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            liked = await client.put(
                f"/api/v1/messages/{assistant_message_id}/feedback",
                headers=owner_headers,
                json={"rating": "up"},
            )
            assert liked.status_code == 200
            assert liked.json()["rating"] == "up"

            switched = await client.put(
                f"/api/v1/messages/{assistant_message_id}/feedback",
                headers=owner_headers,
                json={"rating": "down"},
            )
            assert switched.status_code == 200
            assert switched.json()["rating"] == "down"
            listed = await client.get(
                f"/api/v1/conversations/{conversation_id}/messages",
                headers=owner_headers,
            )
            assistant_payload = next(
                item for item in listed.json()["items"] if item["id"] == assistant_message_id
            )
            assert assistant_payload["feedback_rating"] == "down"
            assert assistant_payload["feedback_updated_at"] is not None

            cleared = await client.put(
                f"/api/v1/messages/{assistant_message_id}/feedback",
                headers=owner_headers,
                json={"rating": None},
            )
            assert cleared.status_code == 200
            assert cleared.json()["rating"] is None
            invalid_user_rating = await client.put(
                f"/api/v1/messages/{user_message_id}/feedback",
                headers=owner_headers,
                json={"rating": "up"},
            )
            assert invalid_user_rating.status_code == 400
            assert (
                await client.put(
                    f"/api/v1/messages/{assistant_message_id}/feedback",
                    headers=other_headers,
                    json={"rating": "up"},
                )
            ).status_code == 404

            fork_headers = {**owner_headers, "Idempotency-Key": "fork-action-0001"}
            forked = await client.post(
                f"/api/v1/conversations/{conversation_id}/forks",
                headers=fork_headers,
                json={"message_id": assistant_message_id},
            )
            assert forked.status_code == 201
            fork_payload = forked.json()
            assert fork_payload["source_conversation_id"] == conversation_id
            assert fork_payload["forked_from_message_id"] == assistant_message_id
            assert fork_payload["copied_messages"] == 2
            fork_id = fork_payload["id"]
            replay = await client.post(
                f"/api/v1/conversations/{conversation_id}/forks",
                headers=fork_headers,
                json={"message_id": assistant_message_id},
            )
            assert replay.status_code == 201
            assert replay.json()["id"] == fork_id
            fork_messages = await client.get(
                f"/api/v1/conversations/{fork_id}/messages",
                headers=owner_headers,
            )
            assert [item["content"] for item in fork_messages.json()["items"]] == [
                "猫频繁进出猫砂盆",
                "先排除尿道梗阻。",
            ]
            assert (
                await client.post(
                    f"/api/v1/conversations/{conversation_id}/forks",
                    headers={**other_headers, "Idempotency-Key": "fork-action-0002"},
                    json={"message_id": assistant_message_id},
                )
            ).status_code == 404

            deleted = await client.delete(
                f"/api/v1/conversations/{fork_id}",
                headers=owner_headers,
            )
            assert deleted.status_code == 204
            assert (
                await client.get(f"/api/v1/conversations/{fork_id}", headers=owner_headers)
            ).status_code == 404

        async with platform_session() as session:
            stored_message = await session.get(Message, assistant_message_id)
            stored_fork = await session.get(Conversation, fork_id)
            actions = list(
                (
                    await session.scalars(
                        select(AuditLog.action).where(
                            AuditLog.actor_user_id == stored_fork.user_id
                        )
                    )
                ).all()
            )
            assert stored_message.feedback_rating is None
            assert stored_message.feedback_updated_at is None
            assert stored_fork.deleted_at is not None
            assert "message.feedback.updated" in actions
            assert "conversation.forked" in actions
            assert "conversation.deleted" in actions

        await close_platform_database()
        reset_platform_settings_cache()

    asyncio.run(scenario())
