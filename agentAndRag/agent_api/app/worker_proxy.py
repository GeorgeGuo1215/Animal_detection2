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
    "x-user-id",
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
    return (os.getenv("AGENT_EXECUTION_ROLE") or "gateway").strip().lower()


def should_delegate_agent_execution() -> bool:
    """Production gateways never execute an Agent locally."""
    return get_platform_settings().production and execution_role() != "worker"


def worker_base_url() -> str:
    return (os.getenv("AGENT_WORKER_URL") or "http://127.0.0.1:8102").rstrip("/")


def worker_token() -> str:
    # The dedicated value is preferred. JWT secret fallback keeps existing
    # deployments bootable while still authenticating the loopback hop.
    return os.getenv("AGENT_WORKER_TOKEN") or get_platform_settings().jwt_secret


def _new_worker_client(timeout: httpx.Timeout | float) -> httpx.AsyncClient:
    return httpx.AsyncClient(timeout=timeout, trust_env=False)


async def worker_readiness() -> tuple[bool, dict[str, Any]]:
    try:
        async with _new_worker_client(2.0) as client:
            response = await client.get(f"{worker_base_url()}/ready")
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

    client = _new_worker_client(httpx.Timeout(timeout=None, connect=5.0))
    try:
        upstream_request = client.build_request(
            "POST",
            f"{worker_base_url()}{path}",
            headers=headers,
            content=json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )
        upstream = await client.send(upstream_request, stream=stream)
    except httpx.HTTPError as exc:
        await client.aclose()
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
            await client.aclose()

    async def body_iterator():
        try:
            async for chunk in upstream.aiter_raw():
                yield chunk
        finally:
            await upstream.aclose()
            await client.aclose()

    return StreamingResponse(
        body_iterator(),
        status_code=upstream.status_code,
        headers=response_headers,
        media_type=None,
    )
