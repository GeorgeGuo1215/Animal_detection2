"""周期性清理持久化的 `/chat-moe` 浏览器测试会话。"""
from __future__ import annotations

import asyncio
import os
from contextlib import suppress
from typing import Optional

from .session_store import SessionManager, get_session_manager


def _cleanup_interval_seconds() -> float:
    """读取会话清理间隔秒数，非法值回退为 300。"""
    try:
        return max(1.0, float(os.getenv("AGENT_SESSION_CLEANUP_INTERVAL_SEC", "300")))
    except (TypeError, ValueError):
        return 300.0


class SessionCleanupTask:
    """在 FastAPI 进程生命周期内周期性执行会话清理。"""

    def __init__(
        self,
        manager: SessionManager,
        interval_seconds: Optional[float] = None,
    ) -> None:
        """绑定 SessionManager 并确定清理间隔。"""
        self._manager = manager
        self._interval = (
            max(0.01, float(interval_seconds))
            if interval_seconds is not None
            else _cleanup_interval_seconds()
        )
        self._task: Optional[asyncio.Task[None]] = None

    @property
    def running(self) -> bool:
        """后台清理任务是否正在运行。"""
        return self._task is not None and not self._task.done()

    async def start(self) -> None:
        """立即清理一次并启动周期性后台任务（已运行则跳过）。"""
        if self.running:
            return
        await self._manager.cleanup()
        self._task = asyncio.create_task(self._run(), name="session-cleanup")

    async def stop(self) -> None:
        """取消后台清理任务并等待其退出。"""
        task = self._task
        self._task = None
        if task is None:
            return
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task

    async def _run(self) -> None:
        """循环等待间隔后调用 SessionManager.cleanup。"""
        while True:
            await asyncio.sleep(self._interval)
            try:
                await self._manager.cleanup()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                print(f"[session-cleanup] cleanup failed: {exc}")


_SESSION_CLEANUP_TASK: Optional[SessionCleanupTask] = None


async def start_session_cleanup_task() -> SessionCleanupTask:
    """启动全局会话清理任务（单例）。"""
    global _SESSION_CLEANUP_TASK  # noqa: PLW0603
    if _SESSION_CLEANUP_TASK is None:
        _SESSION_CLEANUP_TASK = SessionCleanupTask(get_session_manager())
    await _SESSION_CLEANUP_TASK.start()
    return _SESSION_CLEANUP_TASK


async def stop_session_cleanup_task() -> None:
    """停止并清空全局会话清理任务。"""
    global _SESSION_CLEANUP_TASK  # noqa: PLW0603
    task = _SESSION_CLEANUP_TASK
    _SESSION_CLEANUP_TASK = None
    if task is not None:
        await task.stop()
