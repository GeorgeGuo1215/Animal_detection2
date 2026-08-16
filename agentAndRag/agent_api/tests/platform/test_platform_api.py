from __future__ import annotations

import asyncio

import httpx
from fastapi import FastAPI
from sqlalchemy import select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.expert_consultations import persist_expert_consultation
from agent_api.app.platform.models import AgentRun, Conversation, ExpertConsultation, Message, PlatformUser, RunEvent, utcnow
from agent_api.app.platform.security import hash_password
from agent_api.app.platform.services import grant_plan, seed_platform_plans, seed_platform_rbac
from agent_api.app.middleware.auth import APIKeyAuthMiddleware
from agent_api.app.routers.routes_platform_admin import router as admin_router
from agent_api.app.routers.routes_platform_auth import router as auth_router
from agent_api.app.routers.routes_platform_core import router as core_router


def test_auth_conversation_isolation_and_api_key(tmp_path, monkeypatch):
    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv("AGENT_PLATFORM_DB_URL", f"sqlite+aiosqlite:///{(tmp_path / 'platform.db').as_posix()}")
        monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
        monkeypatch.setenv("AGENT_PLATFORM_EXPOSE_DEV_TOKENS", "1")
        reset_platform_settings_cache()
        await init_platform_database()
        async with platform_session() as session:
            await seed_platform_plans(session)
            await seed_platform_rbac(session)
            trial = await session.get(__import__("agent_api.app.platform.models", fromlist=["Plan"]).Plan, "trial")
            for index in (1, 2):
                user = PlatformUser(
                    email=f"vet{index}@example.com",
                    display_name=f"Vet {index}",
                    password_hash=hash_password("correct-horse-123"),
                    role="VET",
                    status="active",
                    email_verified_at=utcnow(),
                )
                session.add(user)
                await session.flush()
                await grant_plan(session, user_id=user.id, plan=trial, reference_type="test", reference_id=f"user-{index}")
            await session.commit()

        app = FastAPI()
        app.include_router(auth_router)
        app.include_router(core_router)
        app.include_router(admin_router)
        @app.get("/v1/models")
        async def models():
            return {"data": []}
        @app.get("/tools")
        async def tools():
            return {"data": []}
        app.add_middleware(APIKeyAuthMiddleware)
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            login = await client.post("/api/v1/auth/login", json={"email": "vet1@example.com", "password": "correct-horse-123"})
            assert login.status_code == 200
            token = login.json()["access_token"]
            headers = {"Authorization": f"Bearer {token}"}
            conversation_headers = {**headers, "Idempotency-Key": "conversation-create-001"}
            created = await client.post("/api/v1/conversations", headers=conversation_headers, json={"title": "猫下泌尿道病例"})
            assert created.status_code == 201
            conversation_id = created.json()["id"]
            replayed = await client.post("/api/v1/conversations", headers=conversation_headers, json={"title": "不会重复创建"})
            assert replayed.json()["id"] == conversation_id

            key_response = await client.post(
                "/api/v1/me/api-keys",
                headers={**headers, "Idempotency-Key": "api-key-create-001"},
                json={"name": "clinic", "scopes": ["chat:write", "runs:read", "models:read"]},
            )
            assert key_response.status_code == 201
            raw_key = key_response.json()["key"]
            key_id = key_response.json()["id"]
            assert raw_key.startswith("pm_live_")
            replayed_key = await client.post(
                "/api/v1/me/api-keys",
                headers={**headers, "Idempotency-Key": "api-key-create-001"},
                json={"name": "clinic", "scopes": ["models:read"]},
            )
            assert replayed_key.status_code == 409
            key_headers = {"Authorization": f"Bearer {raw_key}"}
            assert (await client.get("/v1/models", headers=key_headers)).status_code == 200
            assert (await client.get("/tools", headers=key_headers)).status_code == 403
            listed = await client.get("/api/v1/me/api-keys", headers=headers)
            assert raw_key not in listed.text

            revoked = await client.delete(f"/api/v1/me/api-keys/{key_id}", headers=headers)
            assert revoked.status_code == 204
            listed_after_revoke = await client.get("/api/v1/me/api-keys", headers=headers)
            assert listed_after_revoke.json()["items"][0]["revoked_at"] is not None
            assert (await client.get("/v1/models", headers=key_headers)).status_code == 401
            assert (await client.delete(f"/api/v1/me/api-keys/{key_id}", headers=headers)).status_code == 204

            second_login = await client.post("/api/v1/auth/login", json={"email": "vet2@example.com", "password": "correct-horse-123"})
            second_headers = {"Authorization": f"Bearer {second_login.json()['access_token']}"}
            forbidden = await client.get(f"/api/v1/conversations/{conversation_id}", headers=second_headers)
            assert forbidden.status_code == 404

            extra = await client.post("/api/v1/conversations", headers=headers, json={"title": "x", "unexpected": True})
            assert extra.status_code == 422

            logout = await client.post("/api/v1/auth/logout")
            assert logout.status_code == 204
            assert "petmind_refresh=" in logout.headers.get("set-cookie", "")
        await close_platform_database()
        reset_platform_settings_cache()

    asyncio.run(scenario())


def test_refresh_rotation_rejects_replay(tmp_path, monkeypatch):
    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv("AGENT_PLATFORM_DB_URL", f"sqlite+aiosqlite:///{(tmp_path / 'refresh.db').as_posix()}")
        monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
        reset_platform_settings_cache()
        await init_platform_database()
        async with platform_session() as session:
            user = PlatformUser(email="rotate@example.com", display_name="Rotate", password_hash=hash_password("correct-horse-123"), role="VET", status="active")
            session.add(user)
            await session.commit()
        app = FastAPI(); app.include_router(auth_router)
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            login = await client.post("/api/v1/auth/login", json={"email": "rotate@example.com", "password": "correct-horse-123"})
            old_cookie = login.cookies.get("petmind_refresh")
            refreshed = await client.post("/api/v1/auth/refresh")
            assert refreshed.status_code == 200
            replay = await client.post("/api/v1/auth/refresh", headers={"Cookie": f"petmind_refresh={old_cookie}"})
            assert replay.status_code == 401
        await close_platform_database(); reset_platform_settings_cache()
    asyncio.run(scenario())


def test_expert_consultations_are_persisted_and_attached_to_the_assistant_message(tmp_path, monkeypatch):
    async def scenario() -> None:
        await close_platform_database()
        monkeypatch.setenv("AGENT_PLATFORM_DB_URL", f"sqlite+aiosqlite:///{(tmp_path / 'experts.db').as_posix()}")
        monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes")
        reset_platform_settings_cache()
        await init_platform_database()
        async with platform_session() as session:
            user = PlatformUser(
                email="expert-history@example.com",
                display_name="Expert History",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
            )
            session.add(user)
            await session.flush()
            conversation = Conversation(user_id=user.id, title="尿路会诊")
            session.add(conversation)
            await session.flush()
            question = Message(conversation_id=conversation.id, role="user", content="猫频繁蹲盆")
            session.add(question)
            await session.flush()
            run = AgentRun(
                user_id=user.id,
                conversation_id=conversation.id,
                user_message_id=question.id,
                idempotency_key="expert-history-run",
                status="completed",
                query=question.content,
                response="优先排查尿道梗阻",
            )
            session.add(run)
            await session.flush()
            answer = Message(
                conversation_id=conversation.id,
                run_id=run.id,
                role="assistant",
                content=run.response,
                status="complete",
            )
            session.add(answer)
            await session.flush()
            run.assistant_message_id = answer.id
            await session.commit()
            conversation_id, run_id = conversation.id, run.id

        trace = {
            "expert": "clinical",
            "name": "兽医临床专家",
            "status": "completed",
            "task": "核对泌尿急症",
            "required_tools": ["rag.search"],
            "recommended_tools": [],
            "tools": [{"kind": "tool", "tool_name": "rag.search", "ok": True, "latency_ms": 12.0}],
            "opinion": {
                "conclusion": "先确认是否完全尿闭",
                "evidence": ["频繁蹲盆且尿量少"],
                "risks": ["完全梗阻属于急症"],
                "confidence": 0.91,
            },
            "execution": "single_pass",
        }
        await persist_expert_consultation(run_id, trace)
        trace["opinion"]["conclusion"] = "先确认尿道是否完全梗阻"
        await persist_expert_consultation(run_id, trace)

        async with platform_session() as session:
            assert len(list((await session.scalars(select(ExpertConsultation))).all())) == 1

        app = FastAPI(); app.include_router(auth_router); app.include_router(core_router)
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            login = await client.post("/api/v1/auth/login", json={
                "email": "expert-history@example.com",
                "password": "correct-horse-123",
            })
            headers = {"Authorization": f"Bearer {login.json()['access_token']}"}
            messages = await client.get(f"/api/v1/conversations/{conversation_id}/messages", headers=headers)
            assert messages.status_code == 200
            assistant = next(item for item in messages.json()["items"] if item["role"] == "assistant")
            assert assistant["run_id"] == run_id
            assert assistant["expert_consultations"][0]["opinion"]["conclusion"] == "先确认尿道是否完全梗阻"
            experts = await client.get(f"/api/v1/runs/{run_id}/experts", headers=headers)
            assert experts.status_code == 200
            assert experts.json()["items"][0]["required_tools"] == ["rag.search"]

            async with platform_session() as session:
                persisted = await session.scalar(select(ExpertConsultation).where(
                    ExpertConsultation.run_id == run_id,
                ))
                assert persisted is not None
                await session.delete(persisted)
                session.add(RunEvent(
                    run_id=run_id,
                    sequence=1,
                    event_type="status",
                    payload={"phase": "consulting", "expert": trace},
                ))
                await session.commit()
            legacy_messages = await client.get(
                f"/api/v1/conversations/{conversation_id}/messages",
                headers=headers,
            )
            legacy_assistant = next(
                item for item in legacy_messages.json()["items"] if item["role"] == "assistant"
            )
            assert legacy_assistant["expert_consultations"][0]["expert"] == "clinical"

        await close_platform_database(); reset_platform_settings_cache()

    asyncio.run(scenario())
