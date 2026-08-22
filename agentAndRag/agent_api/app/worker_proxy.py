from __future__ import annotations

import json
import os
from typing import Any

import httpx
from fastapi import HTTPException, Request
from fastapi.responses import Response, StreamingResponse

from .platform.config import get_platform_settings


_FORWARDED_REQUEST_HEADERS = (
    "x-animal-id",
    "x-request-id",
    "last-event-id",
)
_FORWARDED_RESPONSE_HEADERS = (
    "cache-control",
    "content-type",
    "retry-after",
    "x-accel-buffering",
    "x-petmind-run-id",
    "x-request-id",
)


def execution_role() -> str:
    """读取 AGENT_EXECUTION_ROLE，默认 gateway。"""
    return (os.getenv("AGENT_EXECUTION_ROLE") or "gateway").strip().lower()


def should_delegate_agent_execution() -> bool:
    """生产环境 Gateway 不在本地执行 Agent，应转发给 Worker。"""
    return get_platform_settings().production and execution_role() != "worker"


def worker_base_url() -> str:
    """内部 Agent Worker 的基础 URL。"""
    return (os.getenv("AGENT_WORKER_URL") or "http://127.0.0.1:8102").rstrip("/")


def worker_token() -> str:
    """内部 Worker 鉴权 Token；优先 AGENT_WORKER_TOKEN，否则回退 JWT secret 以兼容旧部署。"""
    return os.getenv("AGENT_WORKER_TOKEN") or get_platform_settings().jwt_secret


def _env_float(name: str, default: float) -> float:
    """读取正浮点运行参数。"""
    try:
        return max(0.1, float(os.getenv(name, "") or default))
    except (TypeError, ValueError):
        return default


def _env_int(name: str, default: int) -> int:
    """读取正整数运行参数。"""
    try:
        return max(1, int(os.getenv(name, "") or default))
    except (TypeError, ValueError):
        return default


_WORKER_CLIENT: httpx.AsyncClient | None = None


def get_worker_client() -> httpx.AsyncClient:
    """返回 Gateway 到 Worker 的共享连接池，避免每次请求重新握手。"""
    global _WORKER_CLIENT  # noqa: PLW0603
    if _WORKER_CLIENT is None:
        max_connections = _env_int("AGENT_WORKER_MAX_CONNECTIONS", 100)
        _WORKER_CLIENT = httpx.AsyncClient(
            timeout=httpx.Timeout(
                timeout=None,
                connect=_env_float("AGENT_WORKER_CONNECT_TIMEOUT_SEC", 5.0),
            ),
            limits=httpx.Limits(
                max_connections=max_connections,
                max_keepalive_connections=min(
                    max_connections,
                    _env_int("AGENT_WORKER_MAX_KEEPALIVE_CONNECTIONS", 20),
                ),
            ),
            trust_env=False,
        )
    return _WORKER_CLIENT


async def close_worker_client() -> None:
    """关闭并清空 Gateway 到 Worker 的共享连接池。"""
    global _WORKER_CLIENT  # noqa: PLW0603
    client = _WORKER_CLIENT
    _WORKER_CLIENT = None
    if client is not None:
        await client.aclose()


async def worker_readiness() -> tuple[bool, dict[str, Any]]:
    """探测 Worker /ready，返回 (是否就绪, 响应体)。"""
    try:
        response = await get_worker_client().get(
            f"{worker_base_url()}/ready",
            timeout=_env_float("AGENT_WORKER_READY_TIMEOUT_SEC", 2.0),
        )
        payload = response.json() if response.content else {}
        return response.status_code == 200 and bool(payload.get("ready")), payload
    except (httpx.HTTPError, ValueError) as exc:
        return False, {"ready": False, "error": str(exc)}


async def proxy_json_to_worker(
    request: Request,
    *,
    path: str,
    payload: dict[str, Any],
    stream: bool,
) -> Response:
    """将 JSON POST 转发到内部 Worker；stream=True 时透传原始字节流。"""
    headers = {
        "content-type": "application/json",
        "x-petmind-worker-token": worker_token(),
    }
    for name in _FORWARDED_REQUEST_HEADERS:
        value = request.headers.get(name)
        if value:
            headers[name] = value
    platform_user_id = str(getattr(request.state, "platform_user_id", "") or "").strip()
    if platform_user_id:
        headers["x-user-id"] = platform_user_id

    client = get_worker_client()
    try:
        upstream_request = client.build_request(
            "POST",
            f"{worker_base_url()}{path}",
            headers=headers,
            content=json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )
        upstream = await client.send(upstream_request, stream=stream)
    except httpx.HTTPError as exc:
        raise HTTPException(
            status_code=503,
            detail={
                "error": {
                    "message": "Agent worker is unavailable",
                    "type": "worker_unavailable",
                    "code": "worker_unavailable",
                }
            },
            headers={"Retry-After": "3"},
        ) from exc

    response_headers = {
        name: value
        for name, value in upstream.headers.items()
        if name.lower() in _FORWARDED_RESPONSE_HEADERS
    }
    if not stream:
        try:
            content = await upstream.aread()
            return Response(
                content=content,
                status_code=upstream.status_code,
                headers=response_headers,
            )
        finally:
            await upstream.aclose()

    async def body_iterator():
        """逐块转发上游响应并在结束时关闭连接。"""
        try:
            async for chunk in upstream.aiter_raw():
                yield chunk
        finally:
            await upstream.aclose()

    return StreamingResponse(
        body_iterator(),
        status_code=upstream.status_code,
        headers=response_headers,
        media_type=None,
    )
