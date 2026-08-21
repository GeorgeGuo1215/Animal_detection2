from __future__ import annotations

import asyncio
import os
import threading
import time
from contextlib import asynccontextmanager, contextmanager
from dataclasses import dataclass
from typing import Any, AsyncIterator, Dict, Iterator, Optional


class ResourceBusyError(RuntimeError):
    """共享资源槽位在超时时间内未能获取时抛出。"""

    code = "RESOURCE_BUSY"

    def __init__(self, resource: str, timeout_s: float) -> None:
        """记录资源名与等待超时秒数。"""
        self.resource = resource
        self.timeout_s = float(timeout_s)
        super().__init__(f"Resource '{resource}' is busy after waiting {timeout_s:g}s")

    def as_dict(self) -> Dict[str, Any]:
        """转为可序列化的错误字典（含 code、message、detail）。"""
        return {
            "code": self.code,
            "message": str(self),
            "detail": {"resource": self.resource, "timeout_seconds": self.timeout_s},
        }


class AsyncResourceLimiter:
    """异步场景下的进程级并发槽位限流器。"""

    def __init__(self, name: str, limit: int, *, enabled: bool = True) -> None:
        """按名称与上限初始化限流器；底层用线程信号量以便跨事件循环共享。"""
        if limit < 1:
            raise ValueError(f"{name} concurrency limit must be >= 1")
        self.name = name
        self.limit = int(limit)
        self.enabled = bool(enabled)
        # 有意使用 threading 信号量作为计数器，以便在 Uvicorn 事件循环
        # 以及会创建短生命周期事件循环的同步包装之间保持进程级共享。
        self._semaphore = threading.BoundedSemaphore(self.limit)
        self._metrics_lock = threading.Lock()
        self._active = 0
        self._waiting = 0
        self._rejected = 0
        self._acquired = 0
        self._wait_seconds = 0.0

    @asynccontextmanager
    async def slot(self, *, timeout_s: float) -> AsyncIterator[None]:
        """异步获取一个槽位；超时则抛出 ResourceBusyError。"""
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
        """从同步代码获取同一进程级计数器槽位。"""
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
        """返回当前限流指标快照。"""
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
    """同步场景下的进程级并发槽位限流器。"""

    def __init__(self, name: str, limit: int, *, enabled: bool = True) -> None:
        """按名称与上限初始化同步限流器。"""
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
        """同步获取一个槽位；超时则抛出 ResourceBusyError。"""
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
        """返回当前限流指标快照。"""
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
    """LLM / RAG / MCP 三类资源限流器及获取超时的聚合配置。"""

    llm: AsyncResourceLimiter
    rag: SyncResourceLimiter
    mcp: AsyncResourceLimiter
    acquire_timeout_s: float

    def snapshot(self) -> Dict[str, Any]:
        """汇总三类资源的限流快照。"""
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
    """读取环境变量为正整数；非法则抛出 ValueError。"""
    raw = (os.getenv(name) or str(default)).strip()
    try:
        value = int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive integer, got {raw!r}") from exc
    if value < 1:
        raise ValueError(f"{name} must be >= 1, got {value}")
    return value


def _positive_float_env(name: str, default: float) -> float:
    """读取环境变量为正浮点数；非法则抛出 ValueError。"""
    raw = (os.getenv(name) or str(default)).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive number, got {raw!r}") from exc
    if value <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")
    return value


def _enabled_env(name: str, default: bool = True) -> bool:
    """读取开关类环境变量；0/false/no/off 视为关闭。"""
    raw = (os.getenv(name) or ("1" if default else "0")).strip().lower()
    return raw not in ("0", "false", "no", "off")


def configure_resource_limits() -> ResourceLimits:
    """按环境变量初始化并缓存全局 ResourceLimits（幂等）。"""
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
    """获取全局资源限流配置，尚未初始化时先完成配置。"""
    return configure_resource_limits()
