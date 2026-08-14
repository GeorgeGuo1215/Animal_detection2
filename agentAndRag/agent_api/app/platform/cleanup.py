from __future__ import annotations

import asyncio
from datetime import timedelta

from sqlalchemy import delete, update

from .config import get_platform_settings
from .database import platform_session
from .models import Conversation, utcnow

_task: asyncio.Task | None = None
_stop: asyncio.Event | None = None


async def cleanup_expired_conversations() -> dict[str, int]:
    settings = get_platform_settings()
    now = utcnow()
    inactive_before = now - timedelta(days=settings.conversation_retention_days)
    purge_before = now - timedelta(days=settings.conversation_delete_grace_days)
    async with platform_session() as session:
        soft = await session.execute(
            update(Conversation)
            .where(Conversation.deleted_at.is_(None), Conversation.last_active_at < inactive_before)
            .values(status="deleted", deleted_at=now, updated_at=now)
        )
        hard = await session.execute(
            delete(Conversation).where(
                Conversation.deleted_at.is_not(None),
                Conversation.deleted_at < purge_before,
            )
        )
        await session.commit()
    return {"soft_deleted": int(soft.rowcount or 0), "purged": int(hard.rowcount or 0)}


async def _loop() -> None:
    assert _stop is not None
    while not _stop.is_set():
        try:
            await cleanup_expired_conversations()
        except Exception as exc:  # noqa: BLE001
            print(f"[platform-cleanup] {exc}")
        try:
            await asyncio.wait_for(_stop.wait(), timeout=24 * 3600)
        except TimeoutError:
            pass


async def start_platform_cleanup_task() -> None:
    global _task, _stop
    if _task is not None and not _task.done():
        return
    _stop = asyncio.Event()
    _task = asyncio.create_task(_loop(), name="platform-conversation-cleanup")


async def stop_platform_cleanup_task() -> None:
    global _task, _stop
    if _stop is not None:
        _stop.set()
    if _task is not None:
        await _task
    _task = None
    _stop = None
