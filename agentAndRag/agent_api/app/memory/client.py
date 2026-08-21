"""对接 PostgreSQL 记忆服务的失败开放异步客户端。"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import httpx


def _flag(name: str, default: bool = False) -> bool:
    """读取布尔环境变量；未设置则返回 default。"""
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _number(name: str, default: float) -> float:
    """读取正浮点环境变量，最小 0.1；非法则回退 default。"""
    try:
        return max(0.1, float(os.getenv(name, "") or default))
    except ValueError:
        return default


@dataclass(frozen=True)
class AgentMemoryConfig:
    """记忆服务开关、超时与管理 Token 等配置。"""

    enabled: bool
    required: bool
    base_url: str
    timeout_s: float
    max_context_chars: int
    management_token: str = ""


def load_memory_config() -> AgentMemoryConfig:
    """从环境变量加载记忆服务配置。"""
    return AgentMemoryConfig(
        enabled=_flag("AGENT_MEMORY_ENABLED", False),
        required=_flag("AGENT_MEMORY_REQUIRED", False),
        base_url=(os.getenv("AGENT_MEMORY_URL") or "http://127.0.0.1:8300").rstrip("/"),
        timeout_s=_number("AGENT_MEMORY_TIMEOUT", 3.0),
        max_context_chars=max(1000, int(os.getenv("AGENT_MEMORY_MAX_CONTEXT_CHARS") or 12000)),
        management_token=os.getenv("MEMORY_MANAGEMENT_TOKEN", ""),
    )


class MemoryUnavailable(RuntimeError):
    """记忆服务不可用（且配置为 required）时抛出。"""


class MemoryClient:
    """记忆服务 HTTP 客户端：主体、上下文、写入与管理接口。"""

    def __init__(
        self,
        config: AgentMemoryConfig,
        *,
        transport: Optional[httpx.AsyncBaseTransport] = None,
    ) -> None:
        """按配置创建 httpx 异步客户端。"""
        self.config = config
        self._http = httpx.AsyncClient(
            base_url=config.base_url,
            timeout=config.timeout_s,
            trust_env=False,
            transport=transport,
        )

    async def close(self) -> None:
        """关闭 HTTP 客户端。"""
        await self._http.aclose()

    async def health(self) -> Dict[str, Any]:
        """探测记忆服务健康状态。"""
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
        """确保记忆主体存在。"""
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
        """拉取用户记忆上下文，并截断到 max_context_chars。"""
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
        """写入一轮用户输入与 Agent 回复。"""
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

    def _management_headers(self) -> Dict[str, str]:
        """管理接口的 Bearer 头；未配置 Token 则返回空字典。"""
        return {"Authorization": f"Bearer {self.config.management_token}"} if self.config.management_token else {}

    async def manage_list(self, *, user_id: str, limit: int = 100) -> Dict[str, Any]:
        """列出用户记忆条目。"""
        response = await self._http.get(
            f"/v1/memory/manage/{user_id}",
            params={"limit": limit},
            headers=self._management_headers(),
        )
        response.raise_for_status()
        return dict(response.json())

    async def manage_delete(self, *, user_id: str, item_id: str) -> Dict[str, Any]:
        """删除指定记忆条目。"""
        response = await self._http.delete(
            f"/v1/memory/manage/{user_id}/items/{item_id}",
            headers=self._management_headers(),
        )
        response.raise_for_status()
        return dict(response.json())

    async def manage_clear(self, *, user_id: str, scope: str) -> Dict[str, Any]:
        """按 scope 清空用户记忆。"""
        response = await self._http.request(
            "DELETE",
            f"/v1/memory/manage/{user_id}",
            json={"scope": scope},
            headers=self._management_headers(),
        )
        response.raise_for_status()
        return dict(response.json())

    async def manage_export_snapshot(self, *, user_id: str) -> Dict[str, Any]:
        """导出用户记忆快照。"""
        response = await self._http.get(
            f"/v1/memory/manage/{user_id}/snapshot",
            headers=self._management_headers(),
        )
        response.raise_for_status()
        return dict(response.json())

    async def manage_restore_snapshot(
        self, *, user_id: str, snapshot: Dict[str, Any]
    ) -> Dict[str, Any]:
        """用快照覆盖恢复用户记忆。"""
        response = await self._http.post(
            f"/v1/memory/manage/{user_id}/snapshot/restore",
            json={"confirmation": "覆盖恢复用户数据", "snapshot": snapshot},
            headers=self._management_headers(),
        )
        response.raise_for_status()
        return dict(response.json())


_client: Optional[MemoryClient] = None
_status: Dict[str, Any] = {"enabled": False, "status": "disabled"}


async def start_memory_client() -> Dict[str, Any]:
    """启动全局记忆客户端并探测健康状态。"""
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
    """关闭并清空全局记忆客户端。"""
    global _client
    if _client is not None:
        await _client.close()
        _client = None


def get_memory_client() -> Optional[MemoryClient]:
    """返回全局记忆客户端；未启用则为 None。"""
    return _client


def memory_status() -> Dict[str, Any]:
    """返回记忆服务当前状态快照。"""
    return dict(_status)
