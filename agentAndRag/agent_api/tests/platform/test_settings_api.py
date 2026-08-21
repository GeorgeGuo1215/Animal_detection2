from __future__ import annotations

import asyncio

import httpx
from fastapi import FastAPI
from sqlalchemy import func, select

from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.platform.database import (
    close_platform_database,
    init_platform_database,
    platform_session,
)
from agent_api.app.platform.legal_documents import PRIVACY_VERSION, TERMS_VERSION
from agent_api.app.platform.models import (
    ActivationCode,
    ActivationRedemption,
    CommonPhrase,
    CreditAccount,
    PlatformUser,
    UserFeedback,
)
from agent_api.app.platform.security import hash_password, hash_secret
from agent_api.app.platform.services import seed_platform_plans
from agent_api.app.routers.routes_platform_auth import router as auth_router
from agent_api.app.routers.routes_platform_settings import router as settings_router
from agent_api.app.routers import routes_platform_settings


def test_settings_feedback_legal_and_activation(tmp_path, monkeypatch):
    """验证设置、反馈、法律文档与激活流程。"""
    async def scenario():
        """本用例的异步执行体。"""
        await close_platform_database()
        monkeypatch.setenv(
            "AGENT_PLATFORM_DB_URL",
            f"sqlite+aiosqlite:///{(tmp_path / 'settings.db').as_posix()}",
        )
        monkeypatch.setenv(
            "AGENT_PLATFORM_JWT_SECRET", "test-secret-with-at-least-thirty-two-bytes"
        )
        reset_platform_settings_cache()
        await init_platform_database()
        raw_code = "PETMIND-2026-TEST"
        async with platform_session() as session:
            await seed_platform_plans(session)
            user = PlatformUser(
                email="settings@example.com",
                display_name="Settings Vet",
                password_hash=hash_password("correct-horse-123"),
                role="VET",
                status="active",
            )
            session.add(user)
            session.add(ActivationCode(
                code_prefix="PETMIND",
                code_hash=hash_secret(raw_code.replace("-", "").upper()),
                plan_code="trial",
                extra_credits=37,
                max_redemptions=1,
            ))
            await session.commit()
            user_id = user.id

        app = FastAPI()
        app.include_router(auth_router)
        app.include_router(settings_router)
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            login = await client.post("/api/v1/auth/login", json={
                "email": "settings@example.com", "password": "correct-horse-123"
            })
            headers = {"Authorization": f"Bearer {login.json()['access_token']}"}
            terms = await client.get("/api/v1/legal/terms")
            privacy = await client.get("/api/v1/legal/privacy")
            assert terms.json()["version"] == TERMS_VERSION
            assert privacy.json()["version"] == PRIVACY_VERSION

            profile = await client.patch(
                "/api/v1/me", headers=headers, json={"display_name": "林医生"}
            )
            assert profile.json()["display_name"] == "林医生"
            prefs = await client.patch("/api/v1/me/preferences", headers=headers, json={
                "theme": "dark",
                "default_expand_experts": False,
                "memory_recall_enabled": False,
                "memory_write_enabled": True,
            })
            assert prefs.json()["theme"] == "dark"
            assert prefs.json()["memory_recall_enabled"] is False

            cleared_scopes = []

            class FakeMemoryClient:
                async def manage_clear(self, *, user_id, scope):
                    """测试替身：清空管理状态。"""
                    cleared_scopes.append((user_id, scope))
                    return {"scope": scope, "deleted": {"items": 2}}

            monkeypatch.setattr(
                routes_platform_settings, "get_memory_client", lambda: FakeMemoryClient()
            )
            cleared = await client.request(
                "DELETE", "/api/v1/me/memories", headers=headers,
                json={"scope": "knowledge"},
            )
            assert cleared.status_code == 200
            assert cleared.json()["scope"] == "knowledge"
            assert cleared_scopes == [(user_id, "knowledge")]
            invalid_clear = await client.request(
                "DELETE", "/api/v1/me/memories", headers=headers,
                json={"scope": "all"},
            )
            assert invalid_clear.status_code == 422

            phrase = await client.post("/api/v1/me/common-phrases", headers=headers, json={
                "title": "复诊模板", "content": "请按时间线整理复诊变化", "sort_order": 2
            })
            phrase_id = phrase.json()["id"]
            edited = await client.patch(
                f"/api/v1/me/common-phrases/{phrase_id}", headers=headers,
                json={"content": "请按时间线整理复诊变化和用药"},
            )
            assert "用药" in edited.json()["content"]
            assert len((await client.get("/api/v1/me/common-phrases", headers=headers)).json()["items"]) == 1

            feedback = await client.post("/api/v1/feedback", headers=headers, json={
                "category": "answer", "content": "希望引用位置更加清晰", "contact": "vet@example.com"
            })
            assert feedback.status_code == 201

            redeemed = await client.post(
                "/api/v1/activation-codes/redeem",
                headers={**headers, "Idempotency-Key": "redeem-settings-test"},
                json={"code": raw_code},
            )
            assert redeemed.status_code == 200
            assert redeemed.json()["credits_granted"] > 37
            duplicate = await client.post(
                "/api/v1/activation-codes/redeem", headers=headers, json={"code": raw_code}
            )
            assert duplicate.status_code == 400
            deleted = await client.delete(f"/api/v1/me/common-phrases/{phrase_id}", headers=headers)
            assert deleted.status_code == 204

        async with platform_session() as session:
            assert await session.scalar(select(func.count()).select_from(UserFeedback)) == 1
            assert await session.scalar(select(func.count()).select_from(CommonPhrase)) == 0
            assert await session.scalar(select(func.count()).select_from(ActivationRedemption)) == 1
            account = await session.get(CreditAccount, user_id)
            assert account and account.balance > 37
        await close_platform_database()
        reset_platform_settings_cache()

    asyncio.run(scenario())
