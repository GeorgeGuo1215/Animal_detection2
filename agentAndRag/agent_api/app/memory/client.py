"""Fail-open async client for the PostgreSQL-backed memory service."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import httpx


def _flag(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _number(name: str, default: float) -> float:
    try:
        return max(0.1, float(os.getenv(name, "") or default))
    except ValueError:
        return default


@dataclass(frozen=True)
class AgentMemoryConfig:
    enabled: bool
    required: bool
    base_url: str
    timeout_s: float
    max_context_chars: int


def load_memory_config() -> AgentMemoryConfig:
    return AgentMemoryConfig(
        enabled=_flag("AGENT_MEMORY_ENABLED", False),
        required=_flag("AGENT_MEMORY_REQUIRED", False),
        base_url=(os.getenv("AGENT_MEMORY_URL") or "http://127.0.0.1:8300").rstrip("/"),
        timeout_s=_number("AGENT_MEMORY_TIMEOUT", 3.0),
        max_context_chars=max(1000, int(os.getenv("AGENT_MEMORY_MAX_CONTEXT_CHARS") or 12000)),
    )


class MemoryUnavailable(RuntimeError):
    pass


class MemoryClient:
    def __init__(
        self,
        config: AgentMemoryConfig,
        *,
        transport: Optional[httpx.AsyncBaseTransport] = None,
    ) -> None:
        self.config = config
        self._http = httpx.AsyncClient(
            base_url=config.base_url,
            timeout=config.timeout_s,
            trust_env=False,
            transport=transport,
        )

    async def close(self) -> None:
        await self._http.aclose()

    async def health(self) -> Dict[str, Any]:
        response = await self._http.get("/health")
        response.raise_for_status()
        return dict(response.json())

    async def ensure_subject(
        self,
        *,
        user_id: str,
        display_name: Optional[str] = None,
        source: str = "agent-api",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        response = await self._http.post(
            "/v1/memory/subjects/ensure",
            json={
                "user_id": user_id,
                "display_name": display_name,
                "source": source,
                "metadata": metadata or {},
            },
        )
        response.raise_for_status()
        return dict(response.json())

    async def context(
        self,
        *,
        user_id: str,
        query: str,
        pet_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        response = await self._http.post(
            "/v1/memory/context",
            json={
                "user_id": user_id,
                "query": query,
                "pet_id": pet_id,
                "include_text": True,
            },
        )
        response.raise_for_status()
        payload = dict(response.json())
        text = str(payload.get("text") or "")
        payload["text"] = text[: self.config.max_context_chars]
        return payload

    async def write_turn(
        self,
        *,
        user_id: str,
        user_input: str,
        agent_response: str,
        pet_id: Optional[str] = None,
        session_id: Optional[str] = None,
        turn_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        response = await self._http.post(
            "/v1/memory/messages",
            json={
                "user_id": user_id,
                "user_input": user_input,
                "agent_response": agent_response,
                "pet_id": pet_id,
                "session_id": session_id,
                "turn_id": turn_id,
            },
        )
        response.raise_for_status()
        return dict(response.json())


_client: Optional[MemoryClient] = None
_status: Dict[str, Any] = {"enabled": False, "status": "disabled"}


async def start_memory_client() -> Dict[str, Any]:
    global _client, _status
    config = load_memory_config()
    if not config.enabled:
        _status = {"enabled": False, "status": "disabled"}
        return dict(_status)
    if _client is None:
        _client = MemoryClient(config)
    try:
        health = await _client.health()
        healthy = health.get("status") == "ok" and health.get("database") == "ok"
        _status = {
            "enabled": True,
            "status": "ok" if healthy else "degraded",
            "required": config.required,
            "url": config.base_url,
            "health": health,
        }
        if config.required and not healthy:
            raise MemoryUnavailable(f"required memory service is degraded: {health}")
    except Exception as exc:
        _status = {
            "enabled": True,
            "status": "unavailable",
            "required": config.required,
            "url": config.base_url,
            "error": str(exc),
        }
        if config.required:
            raise MemoryUnavailable(str(exc)) from exc
    return dict(_status)


async def close_memory_client() -> None:
    global _client
    if _client is not None:
        await _client.close()
        _client = None


def get_memory_client() -> Optional[MemoryClient]:
    return _client


def memory_status() -> Dict[str, Any]:
    return dict(_status)
