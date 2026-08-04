from __future__ import annotations

import asyncio
import os
import threading
import time
from contextlib import asynccontextmanager, contextmanager
from dataclasses import dataclass
from typing import Any, AsyncIterator, Dict, Iterator, Optional


class ResourceBusyError(RuntimeError):
    """Raised when a shared resource slot cannot be acquired in time."""

    code = "RESOURCE_BUSY"

    def __init__(self, resource: str, timeout_s: float) -> None:
        self.resource = resource
        self.timeout_s = float(timeout_s)
        super().__init__(f"Resource '{resource}' is busy after waiting {timeout_s:g}s")

    def as_dict(self) -> Dict[str, Any]:
        return {
            "code": self.code,
            "message": str(self),
            "detail": {"resource": self.resource, "timeout_seconds": self.timeout_s},
        }


class AsyncResourceLimiter:
    def __init__(self, name: str, limit: int, *, enabled: bool = True) -> None:
        if limit < 1:
            raise ValueError(f"{name} concurrency limit must be >= 1")
        self.name = name
        self.limit = int(limit)
        self.enabled = bool(enabled)
        # A threading semaphore is deliberately used as the backing counter.
        # It remains process-global across the Uvicorn loop and legacy sync
        # wrappers that create short-lived event loops.
        self._semaphore = threading.BoundedSemaphore(self.limit)
        self._metrics_lock = threading.Lock()
        self._active = 0
        self._waiting = 0
        self._rejected = 0
        self._acquired = 0
        self._wait_seconds = 0.0

    @asynccontextmanager
    async def slot(self, *, timeout_s: float) -> AsyncIterator[None]:
        if not self.enabled:
            yield
            return

        acquired = False
        started = time.monotonic()
        with self._metrics_lock:
            self._waiting += 1
        try:
            deadline = started + float(timeout_s)
            while not acquired:
                acquired = self._semaphore.acquire(blocking=False)
                if acquired:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    with self._metrics_lock:
                        self._rejected += 1
                    raise ResourceBusyError(self.name, timeout_s)
                await asyncio.sleep(min(0.01, remaining))

            with self._metrics_lock:
                self._waiting -= 1
                self._active += 1
                self._acquired += 1
                self._wait_seconds += time.monotonic() - started
            yield
        finally:
            if not acquired:
                with self._metrics_lock:
                    self._waiting -= 1
            if acquired:
                with self._metrics_lock:
                    self._active -= 1
                self._semaphore.release()

    @contextmanager
    def sync_slot(self, *, timeout_s: float) -> Iterator[None]:
        """Acquire the same process-wide counter from synchronous code."""
        if not self.enabled:
            yield
            return

        acquired = False
        started = time.monotonic()
        with self._metrics_lock:
            self._waiting += 1
        try:
            acquired = self._semaphore.acquire(timeout=float(timeout_s))
            if not acquired:
                with self._metrics_lock:
                    self._rejected += 1
                raise ResourceBusyError(self.name, timeout_s)
            with self._metrics_lock:
                self._waiting -= 1
                self._active += 1
                self._acquired += 1
                self._wait_seconds += time.monotonic() - started
            yield
        finally:
            if acquired:
                with self._metrics_lock:
                    self._active -= 1
                self._semaphore.release()
            else:
                with self._metrics_lock:
                    self._waiting -= 1

    def snapshot(self) -> Dict[str, Any]:
        with self._metrics_lock:
            return {
                "limit": self.limit,
                "enabled": self.enabled,
                "active": self._active,
                "waiting": self._waiting,
                "acquired": self._acquired,
                "rejected": self._rejected,
                "wait_seconds_total": round(self._wait_seconds, 6),
            }


class SyncResourceLimiter:
    def __init__(self, name: str, limit: int, *, enabled: bool = True) -> None:
        if limit < 1:
            raise ValueError(f"{name} concurrency limit must be >= 1")
        self.name = name
        self.limit = int(limit)
        self.enabled = bool(enabled)
        self._semaphore = threading.BoundedSemaphore(self.limit)
        self._metrics_lock = threading.Lock()
        self._active = 0
        self._waiting = 0
        self._rejected = 0
        self._acquired = 0
        self._wait_seconds = 0.0

    @contextmanager
    def slot(self, *, timeout_s: float) -> Iterator[None]:
        if not self.enabled:
            yield
            return

        acquired = False
        started = time.monotonic()
        with self._metrics_lock:
            self._waiting += 1
        try:
            acquired = self._semaphore.acquire(timeout=float(timeout_s))
            if not acquired:
                with self._metrics_lock:
                    self._rejected += 1
                raise ResourceBusyError(self.name, timeout_s)
            with self._metrics_lock:
                self._waiting -= 1
                self._active += 1
                self._acquired += 1
                self._wait_seconds += time.monotonic() - started
            yield
        finally:
            if acquired:
                with self._metrics_lock:
                    self._active -= 1
                self._semaphore.release()
            else:
                with self._metrics_lock:
                    self._waiting -= 1

    def snapshot(self) -> Dict[str, Any]:
        with self._metrics_lock:
            return {
                "limit": self.limit,
                "enabled": self.enabled,
                "active": self._active,
                "waiting": self._waiting,
                "acquired": self._acquired,
                "rejected": self._rejected,
                "wait_seconds_total": round(self._wait_seconds, 6),
            }


@dataclass(frozen=True)
class ResourceLimits:
    llm: AsyncResourceLimiter
    rag: SyncResourceLimiter
    mcp: AsyncResourceLimiter
    acquire_timeout_s: float

    def snapshot(self) -> Dict[str, Any]:
        return {
            "acquire_timeout_seconds": self.acquire_timeout_s,
            "resources": {
                "llm": self.llm.snapshot(),
                "rag": self.rag.snapshot(),
                "mcp": self.mcp.snapshot(),
            },
        }


_RESOURCE_LIMITS: Optional[ResourceLimits] = None
_CONFIG_LOCK = threading.Lock()


def _positive_int_env(name: str, default: int) -> int:
    raw = (os.getenv(name) or str(default)).strip()
    try:
        value = int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive integer, got {raw!r}") from exc
    if value < 1:
        raise ValueError(f"{name} must be >= 1, got {value}")
    return value


def _positive_float_env(name: str, default: float) -> float:
    raw = (os.getenv(name) or str(default)).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive number, got {raw!r}") from exc
    if value <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")
    return value


def _enabled_env(name: str, default: bool = True) -> bool:
    raw = (os.getenv(name) or ("1" if default else "0")).strip().lower()
    return raw not in ("0", "false", "no", "off")


def configure_resource_limits() -> ResourceLimits:
    global _RESOURCE_LIMITS  # noqa: PLW0603
    with _CONFIG_LOCK:
        if _RESOURCE_LIMITS is not None:
            return _RESOURCE_LIMITS
        enabled = _enabled_env("AGENT_RESOURCE_LIMITS_ENABLED", True)
        _RESOURCE_LIMITS = ResourceLimits(
            llm=AsyncResourceLimiter(
                "llm",
                _positive_int_env("AGENT_LLM_MAX_CONCURRENCY", 4),
                enabled=enabled,
            ),
            rag=SyncResourceLimiter(
                "rag",
                _positive_int_env("AGENT_RAG_MAX_CONCURRENCY", 1),
                enabled=enabled,
            ),
            mcp=AsyncResourceLimiter(
                "mcp",
                _positive_int_env("AGENT_MCP_MAX_CONCURRENCY", 4),
                enabled=enabled,
            ),
            acquire_timeout_s=_positive_float_env("AGENT_RESOURCE_ACQUIRE_TIMEOUT_SEC", 30.0),
        )
        return _RESOURCE_LIMITS


def get_resource_limits() -> ResourceLimits:
    return configure_resource_limits()
