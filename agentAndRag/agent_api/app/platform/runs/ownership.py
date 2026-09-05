"""Execution fencing shared by events, expert results and terminal transactions."""
from contextvars import ContextVar
from dataclasses import dataclass
from datetime import datetime, timezone

from sqlalchemy import func, select, update
from sqlalchemy.ext.asyncio import AsyncSession

from ..models import AgentRun


@dataclass(frozen=True)
class ExecutionLease:
    run_id: str
    owner: str
    epoch: int


class LeaseLost(Exception):
    """The current attempt no longer has permission to persist results."""


current_lease: ContextVar[ExecutionLease | None] = ContextVar("platform_run_lease", default=None)


def db_clock(session: AsyncSession):
    return func.clock_timestamp() if session.get_bind().dialect.name == "postgresql" else func.current_timestamp()


async def database_now(session: AsyncSession) -> datetime:
    value = await session.scalar(select(db_clock(session)))
    return value.replace(tzinfo=timezone.utc) if value.tzinfo is None else value


def ownership_predicates(session: AsyncSession, lease: ExecutionLease):
    return (
        AgentRun.id == lease.run_id,
        AgentRun.claimed_by == lease.owner,
        AgentRun.execution_epoch == lease.epoch,
        AgentRun.lease_until > db_clock(session),
        AgentRun.status.in_(("running", "cancel_requested")),
    )


async def lock_run(session: AsyncSession, run_id: str, *, fenced: bool = True) -> AgentRun:
    """Obtain the row write lock on PostgreSQL and SQLite, then refresh ORM state.

    The no-op UPDATE provides actual serialization on SQLite too, unlike FOR UPDATE.
    Locks are held only for short persistence transactions, never model calls.
    """
    lease = current_lease.get() if fenced else None
    conditions = [AgentRun.id == run_id]
    if lease is not None:
        if lease.run_id != run_id:
            raise LeaseLost("attempted to write another run")
        conditions.extend(ownership_predicates(session, lease))
    result = await session.execute(update(AgentRun).where(*conditions)
                                   .values(event_sequence=AgentRun.event_sequence).returning(AgentRun.id))
    if result.scalar_one_or_none() is None:
        raise LeaseLost("run missing or execution lease expired")
    run = await session.get(AgentRun, run_id, populate_existing=True)
    assert run is not None
    return run
