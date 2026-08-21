from __future__ import annotations

import asyncio

import httpx
from fastapi import FastAPI, Request

from agent_api.app.middleware.platform_http import PlatformRequestMiddleware


def test_snapshot_upload_has_a_separate_bounded_body_limit() -> None:
    """验证快照上传有单独的请求体大小上限。"""
    async def scenario() -> None:
        """本用例的异步执行体。"""
        app = FastAPI()

        @app.post("/api/v1/ordinary")
        async def ordinary(request: Request):
            """对照用的普通实现。"""
            return {"bytes": len(await request.body())}

        @app.post("/api/v1/admin/users/{user_id}/data-snapshot/restore-file")
        async def snapshot(user_id: str, request: Request):
            """返回或构造一份快照。"""
            return {"user_id": user_id, "bytes": len(await request.body())}

        app.add_middleware(PlatformRequestMiddleware, max_body_bytes=1024)
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            payload = b"x" * 2048
            rejected = await client.post("/api/v1/ordinary", content=payload)
            accepted = await client.post(
                "/api/v1/admin/users/user-1/data-snapshot/restore-file",
                content=payload,
            )
        assert rejected.status_code == 413
        assert rejected.json()["details"]["max_bytes"] == 1024
        assert accepted.status_code == 200
        assert accepted.json() == {"user_id": "user-1", "bytes": 2048}

    asyncio.run(scenario())
