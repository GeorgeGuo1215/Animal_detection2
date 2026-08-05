"""
In-memory session manager with TTL expiration.

Stores per-session conversation history and tool results so that multi-turn
conversations can persist across HTTP requests without relying on the client
to re-send the full history each time.

For production use this can be swapped to Redis with the same interface.
"""
from __future__ import annotations

import asyncio
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Session:
    session_id: str
    messages: List[Dict[str, str]] = field(default_factory=list)
    tool_results: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    created_at: float = field(default_factory=time.time)
    last_active: float = field(default_factory=time.time)

    def touch(self) -> None:
        self.last_active = time.time()


class SessionManager:
    """Thread-safe, TTL-aware session store."""

    def __init__(self, ttl_seconds: float = 3600, max_sessions: int = 10_000) -> None:
        self._sessions: Dict[str, Session] = {}
        self._ttl = ttl_seconds
        self._max = max_sessions
        self._lock = asyncio.Lock()

    async def create(self, metadata: Optional[Dict[str, Any]] = None) -> Session:
        async with self._lock:
            self._evict_expired()
            sid = uuid.uuid4().hex
            sess = Session(session_id=sid, metadata=metadata or {})
            self._sessions[sid] = sess
            return sess

    async def get(self, session_id: str) -> Optional[Session]:
        async with self._lock:
            sess = self._sessions.get(session_id)
            if sess is None:
                return None
            if time.time() - sess.last_active > self._ttl:
                del self._sessions[session_id]
                return None
            sess.touch()
            return sess

    async def get_or_create(self, session_id: Optional[str], metadata: Optional[Dict[str, Any]] = None) -> Session:
        if session_id:
            sess = await self.get(session_id)
            if sess:
                return sess
        return await self.create(metadata)

    async def delete(self, session_id: str) -> bool:
        async with self._lock:
            return self._sessions.pop(session_id, None) is not None

    def _evict_expired(self) -> None:
        now = time.time()
        expired = [k for k, v in self._sessions.items() if now - v.last_active > self._ttl]
        for k in expired:
            del self._sessions[k]
        if len(self._sessions) > self._max:
            by_age = sorted(self._sessions.items(), key=lambda kv: kv[1].last_active)
            for k, _ in by_age[: len(self._sessions) - self._max]:
                del self._sessions[k]


_MANAGER: Optional[SessionManager] = None


def get_session_manager() -> SessionManager:
    global _MANAGER  # noqa: PLW0603
    if _MANAGER is None:
        _MANAGER = SessionManager()
    return _MANAGER
