from __future__ import annotations

import asyncio
import json

from fastapi import Request
from fastapi.responses import JSONResponse
import httpx

from agent_api.app.routers import routes_chat_ui, routes_openai
from agent_api.app.schemas.chat_moe import ChatMoeCompletionRequest
from agent_api.app.schemas.openai_schemas import ChatCompletionRequest
from agent_api.app.platform.config import reset_platform_settings_cache
from agent_api.app.worker_proxy import should_delegate_agent_execution
from agent_api.app import worker_proxy


def _request(path: str, headers: list[tuple[bytes, bytes]] | None = None) -> Request:
    """构造测试用的 HTTP 请求对象。"""
    return Request({
        "type": "http",
        "http_version": "1.1",
        "method": "POST",
        "scheme": "http",
        "path": path,
        "raw_path": path.encode(),
        "query_string": b"",
        "headers": headers or [],
        "client": ("127.0.0.1", 12345),
        "server": ("127.0.0.1", 8002),
    })


def test_only_production_gateway_delegates(monkeypatch):
    """验证只有生产网关才会把请求委托给 Worker。"""
    monkeypatch.setenv("AGENT_PLATFORM_ENV", "production")
    monkeypatch.setenv("AGENT_PLATFORM_DB_URL", "postgresql+asyncpg://user:pass@127.0.0.1/db")
    monkeypatch.setenv("AGENT_PLATFORM_REDIS_URL", "redis://127.0.0.1:6379/0")
    monkeypatch.setenv("AGENT_PLATFORM_AUTO_CREATE_SCHEMA", "0")
    monkeypatch.setenv("AGENT_PLATFORM_EXPOSE_DEV_TOKENS", "0")
    monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "x" * 32)
    monkeypatch.setenv("AGENT_PLATFORM_COOKIE_SECURE", "1")
    monkeypatch.setenv("AGENT_PLATFORM_FRONTEND_ORIGIN", "https://agent.example.com")
    monkeypatch.setenv("AGENT_WORKER_TOKEN", "w" * 32)
    monkeypatch.setenv("MEMORY_MANAGEMENT_TOKEN", "m" * 32)
    monkeypatch.setenv("AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET", "p" * 32)
    monkeypatch.setenv("AGENT_EXECUTION_ROLE", "gateway")
    reset_platform_settings_cache()
    try:
        assert should_delegate_agent_execution() is True
        monkeypatch.setenv("AGENT_EXECUTION_ROLE", "worker")
        assert should_delegate_agent_execution() is False
    finally:
        reset_platform_settings_cache()


def test_openai_completion_delegates_before_local_execution(monkeypatch):
    """验证 OpenAI 补全会在本地执行前先委托。"""
    async def scenario():
        """本用例的异步执行体。"""
        captured = {}

        async def fake_proxy(request, *, path, payload, stream):
            """测试替身：假装 Worker 代理。"""
            captured.update(path=path, payload=payload, stream=stream)
            return JSONResponse({"delegated": True})

        monkeypatch.setattr(routes_openai, "should_delegate_agent_execution", lambda: True)
        monkeypatch.setattr(routes_openai, "proxy_json_to_worker", fake_proxy)
        request = _request("/v1/chat/completions")
        response = await routes_openai.chat_completions(
            ChatCompletionRequest(
                model="agent-moe",
                messages=[{"role": "user", "content": "猫频繁进出猫砂盆"}],
                stream=True,
            ),
            request,
        )

        assert response.status_code == 200
        assert captured["path"] == "/v1/chat/completions"
        assert captured["stream"] is True
        assert captured["payload"]["model"] == "agent-moe"

    asyncio.run(scenario())


def test_chat_moe_completion_delegates_to_worker(monkeypatch):
    """验证 MoE 聊天补全会委托给 Worker。"""
    async def scenario():
        """本用例的异步执行体。"""
        captured = {}

        async def fake_proxy(request, *, path, payload, stream):
            """测试替身：假装 Worker 代理。"""
            captured.update(path=path, payload=payload, stream=stream)
            return JSONResponse({"delegated": True})

        monkeypatch.setattr(routes_chat_ui, "should_delegate_agent_execution", lambda: True)
        monkeypatch.setattr(routes_chat_ui, "proxy_json_to_worker", fake_proxy)
        response = await routes_chat_ui.chat_moe_public_completions(
            ChatMoeCompletionRequest(session_id="session-1", message="猫尿闭怎么排急症"),
            _request("/chat-moe/completions"),
        )

        assert response.status_code == 200
        assert captured == {
            "path": "/chat-moe/completions",
            "payload": {
                "session_id": "session-1",
                "message": "猫尿闭怎么排急症",
                "user_role": "pet_owner",
                "response_lang": "zh",
                "temperature": 0.3,
            },
            "stream": True,
        }

    asyncio.run(scenario())


def test_worker_proxy_forwards_identity_and_stream_bytes(monkeypatch):
    """验证 Worker 代理会转发身份和流式字节。"""
    async def scenario():
        """本用例的异步执行体。"""
        seen = {}

        class EventStream(httpx.AsyncByteStream):
            async def __aiter__(self):
                """返回异步迭代器自身。"""
                yield b"data: {\"ok\":true}\n\n"

        def handler(upstream: httpx.Request) -> httpx.Response:
            """测试用 HTTP 处理函数，返回预置响应。"""
            seen["path"] = upstream.url.path
            seen["token"] = upstream.headers.get("x-petmind-worker-token")
            seen["user"] = upstream.headers.get("x-user-id")
            seen["payload"] = json.loads(upstream.content)
            return httpx.Response(
                200,
                headers={"content-type": "text/event-stream", "x-accel-buffering": "no"},
                stream=EventStream(),
            )

        transport = httpx.MockTransport(handler)
        monkeypatch.setenv("AGENT_WORKER_URL", "http://worker.internal:8102")
        monkeypatch.setenv("AGENT_WORKER_TOKEN", "internal-secret")
        monkeypatch.setattr(
            worker_proxy,
            "_new_worker_client",
            lambda timeout: httpx.AsyncClient(transport=transport, timeout=timeout),
        )
        request = _request("/v1/chat/completions")
        request.state.platform_user_id = "user-42"
        response = await worker_proxy.proxy_json_to_worker(
            request,
            path="/v1/chat/completions",
            payload={"stream": True, "messages": []},
            stream=True,
        )
        body = b"".join([chunk async for chunk in response.body_iterator])

        assert response.status_code == 200
        assert response.headers["x-accel-buffering"] == "no"
        assert body == b"data: {\"ok\":true}\n\n"
        assert seen == {
            "path": "/v1/chat/completions",
            "token": "internal-secret",
            "user": "user-42",
            "payload": {"stream": True, "messages": []},
        }

    asyncio.run(scenario())


def test_caller_cannot_spoof_memory_identity_or_forwarded_user_header(monkeypatch):
    """验证调用方无法伪造记忆身份或转发用户头。"""
    async def scenario():
        """本用例的异步执行体。"""
        seen = {}

        def handler(upstream: httpx.Request) -> httpx.Response:
            """测试用 HTTP 处理函数，返回预置响应。"""
            seen["user"] = upstream.headers.get("x-user-id")
            return httpx.Response(200, json={"ok": True})

        monkeypatch.setenv("AGENT_WORKER_URL", "http://worker.internal:8102")
        monkeypatch.setenv("AGENT_WORKER_TOKEN", "internal-secret")
        monkeypatch.setattr(
            worker_proxy,
            "_new_worker_client",
            lambda timeout: httpx.AsyncClient(
                transport=httpx.MockTransport(handler), timeout=timeout
            ),
        )
        request = _request(
            "/v1/chat/completions",
            headers=[(b"x-user-id", b"victim-user")],
        )
        req = ChatCompletionRequest(
            model="agent-moe",
            messages=[{"role": "user", "content": "test"}],
            user="victim-openai-user",
            user_id="victim-body-user",
        )
        assert routes_openai._memory_user_id(req, request) is None
        response = await worker_proxy.proxy_json_to_worker(
            request,
            path="/v1/chat/completions",
            payload=req.model_dump(mode="json", exclude_none=True),
            stream=False,
        )
        assert response.status_code == 200
        assert seen["user"] is None

    asyncio.run(scenario())
