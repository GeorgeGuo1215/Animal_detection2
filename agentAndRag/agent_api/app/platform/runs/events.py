"""Durable ordered event batches and a shared PostgreSQL notification listener."""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from typing import Any

from sqlalchemy import select, func
from sqlalchemy.ext.asyncio import AsyncSession

from ..config import get_platform_settings
from ..database import platform_session
from ..models import RunEvent
from .ownership import lock_run

logger = logging.getLogger(__name__)
_CHANNEL = "petmind_run_events"


class EventHub:
    def __init__(self) -> None:
        self.listeners: dict[str, set[asyncio.Event]] = {}
        self.connection: Any = None
        self.connect_lock = asyncio.Lock()

    def wake(self, run_id: str) -> None:
        for event in tuple(self.listeners.get(run_id, ())):
            event.set()

    async def connect(self) -> None:
        url = get_platform_settings().database_url
        if not url.startswith("postgresql"):
            return
        async with self.connect_lock:
            if self.connection is not None and not self.connection.is_closed():
                return
            import asyncpg
            try:
                self.connection = await asyncpg.connect(
                    url.replace("postgresql+asyncpg://", "postgresql://"), timeout=2,
                )
                await self.connection.add_listener(_CHANNEL, lambda connection, pid, channel, payload: self.wake(payload))
            except Exception:
                logger.warning("run notifications unavailable; durable polling remains active", exc_info=True)
                await self.close()

    @asynccontextmanager
    async def subscribe(self, run_id: str):
        event = asyncio.Event()
        self.listeners.setdefault(run_id, set()).add(event)
        try:
            await self.connect()
            yield event
        finally:
            self.listeners[run_id].discard(event)
            if not self.listeners[run_id]:
                del self.listeners[run_id]

    async def close(self) -> None:
        connection, self.connection = self.connection, None
        if connection is not None:
            await connection.close()


_hub: EventHub | None = None
_hub_loop: asyncio.AbstractEventLoop | None = None


def event_hub() -> EventHub:
    global _hub, _hub_loop
    loop = asyncio.get_running_loop()
    if _hub is None or _hub_loop is not loop:
        _hub, _hub_loop = EventHub(), loop
    return _hub


async def append_in_session(session: AsyncSession, run_id: str, events: list[tuple[str, dict]], *, locked_run=None) -> int:
    """Allocate IDs under the Run lock, in the caller's transaction."""
    run = locked_run if locked_run is not None else await lock_run(session, run_id)
    for event_type, payload in events:
        run.event_sequence += 1
        session.add(RunEvent(run_id=run_id, sequence=run.event_sequence, event_type=event_type, payload=payload))
    await session.flush()
    return run.event_sequence


async def commit_run(session: AsyncSession, run_id: str) -> None:
    if session.get_bind().dialect.name == "postgresql":
        # PostgreSQL delivers only after commit. No answer text is sent on this channel.
        await session.execute(select(func.pg_notify(_CHANNEL, run_id)))
    await session.commit()
    event_hub().wake(run_id)


async def append_events(run_id: str, events: list[tuple[str, dict]]) -> int:
    async with platform_session() as session:
        sequence = await append_in_session(session, run_id, events)
        await commit_run(session, run_id)
        return sequence


class DeltaBuffer:
    """First delta is immediate; subsequent deltas commit in bounded batches."""
    def __init__(self, run_id: str, *, interval: float = 0.1, max_events: int = 32):
        self.run_id = run_id
        self.interval = interval
        self.max_events = max_events
        self.pending: list[tuple[str, dict]] = []
        self.first = True
        self.lock = asyncio.Lock()
        self.timer: asyncio.Task | None = None
        self.error: Exception | None = None

    async def add(self, content: str) -> None:
        if self.error:
            raise self.error
        self.pending.append(("delta", {"content": content}))
        if self.first or len(self.pending) >= self.max_events:
            self.first = False
            await self.flush()
        elif self.timer is None or self.timer.done():
            self.timer = asyncio.create_task(self._later(), name=f"run-delta:{self.run_id}")

    async def _later(self) -> None:
        try:
            await asyncio.sleep(self.interval)
            await self.flush()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.error = exc

    async def flush(self) -> None:
        async with self.lock:
            if self.error:
                raise self.error
            if self.pending:
                batch, self.pending = self.pending, []
                await append_events(self.run_id, batch)

    async def close(self) -> None:
        if self.timer is not None and not self.timer.done():
            self.timer.cancel()
            await asyncio.gather(self.timer, return_exceptions=True)
        self.pending.clear()
