"""Periodic cleanup for persisted `/chat-moe` browser-test sessions."""
from __future__ import annotations

import asyncio
import os
from contextlib import suppress
from typing import Optional

from ..persistence.session_manager import SessionManager, get_session_manager


def _cleanup_interval_seconds() -> float:
    try:
        return max(1.0, float(os.getenv("AGENT_SESSION_CLEANUP_INTERVAL_SEC", "300")))
    except (TypeError, ValueError):
        return 300.0


class SessionCleanupTask:
    """Run session cleanup periodically within the FastAPI process lifecycle."""

    def __init__(
        self,
        manager: SessionManager,
        interval_seconds: Optional[float] = None,
    ) -> None:
        self._manager = manager
        self._interval = (
            max(0.01, float(interval_seconds))
            if interval_seconds is not None
            else _cleanup_interval_seconds()
        )
        self._task: Optional[asyncio.Task[None]] = None

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    async def start(self) -> None:
        if self.running:
            return
        await self._manager.cleanup()
        self._task = asyncio.create_task(self._run(), name="session-cleanup")

    async def stop(self) -> None:
        task = self._task
        self._task = None
        if task is None:
            return
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task

    async def _run(self) -> None:
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
    global _SESSION_CLEANUP_TASK  # noqa: PLW0603
    if _SESSION_CLEANUP_TASK is None:
        _SESSION_CLEANUP_TASK = SessionCleanupTask(get_session_manager())
    await _SESSION_CLEANUP_TASK.start()
    return _SESSION_CLEANUP_TASK


async def stop_session_cleanup_task() -> None:
    global _SESSION_CLEANUP_TASK  # noqa: PLW0603
    task = _SESSION_CLEANUP_TASK
    _SESSION_CLEANUP_TASK = None
    if task is not None:
        await task.stop()
