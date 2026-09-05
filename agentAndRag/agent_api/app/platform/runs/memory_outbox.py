"""Retry completed-run memory writes without delaying answer delivery."""
import asyncio
import logging
from datetime import timedelta

from sqlalchemy import func, select, update

from ...memory import write_user_memory
from ..database import platform_session
from ..models import AgentRun, OutboxEvent, UserPreference
from .ownership import database_now

logger = logging.getLogger(__name__)
TOPIC = "run.memory_sync"


def schedule_memory(session, run: AgentRun) -> None:
    # One terminal transaction per Run makes this deterministic ID idempotent.
    session.add(OutboxEvent(id=run.id, topic=TOPIC, payload={"run_id": run.id}, available_at=func.current_timestamp()))
    run.memory_status = "pending"


async def process_memory_outbox(*, run_id: str | None = None, writer=None) -> int:
    writer = writer or write_user_memory
    async with platform_session() as session:
        now = await database_now(session)
        query = select(OutboxEvent.id).where(
            OutboxEvent.topic == TOPIC,
            OutboxEvent.status.in_(("pending", "processing")),
            OutboxEvent.available_at <= now,
        ).order_by(OutboxEvent.available_at).limit(8)
        if run_id:
            query = query.where(OutboxEvent.id == run_id)
        ids = list((await session.scalars(query)).all())
    processed = 0
    for event_id in ids:
        async with platform_session() as session:
            now = await database_now(session)
            claimed = await session.execute(update(OutboxEvent).where(
                OutboxEvent.id == event_id,
                OutboxEvent.status.in_(("pending", "processing")),
                OutboxEvent.available_at <= now,
            ).values(status="processing", available_at=now + timedelta(seconds=60),
                     attempts=OutboxEvent.attempts + 1).returning(OutboxEvent.attempts))
            attempt = claimed.scalar_one_or_none()
            if attempt is None:
                continue
            run = await session.get(AgentRun, event_id)
            if run is None or run.status != "completed":
                await session.rollback()
                continue
            preference = await session.get(UserPreference, run.user_id)
            if preference is not None and not preference.memory_write_enabled:
                await session.execute(update(OutboxEvent).where(OutboxEvent.id == event_id).values(status="done", processed_at=now))
                run.memory_status = "disabled"
                await session.commit()
                processed += 1
                continue
            payload = dict(user_id=run.user_id, query=run.query, answer=run.response,
                           pet_id=None, session_id=run.conversation_id, turn_id=run.id)
            await session.commit()
        error = None
        disabled = False
        try:
            async with asyncio.timeout(30):
                result = await writer(**payload)
                disabled = isinstance(result, dict) and result.get("reason") == "disabled_or_incomplete"
                if not disabled and (not isinstance(result, dict) or not result.get("stored")):
                    raise RuntimeError("memory service rejected write")
        except asyncio.CancelledError:
            raise  # The processing lease allows another maintenance iteration to retry.
        except Exception as exc:
            error = exc
            logger.warning("memory outbox attempt failed run_id=%s attempt=%s type=%s", event_id, attempt, type(exc).__name__)
        async with platform_session() as session:
            now = await database_now(session)
            values = ({"status": "done", "processed_at": now} if error is None else {
                "status": "failed" if attempt >= 8 else "pending",
                "available_at": now + timedelta(seconds=min(300, 2 ** attempt)),
            })
            updated = await session.execute(update(OutboxEvent).where(
                OutboxEvent.id == event_id, OutboxEvent.attempts == attempt,
                OutboxEvent.status == "processing",
            ).values(**values))
            if updated.rowcount:
                await session.execute(update(AgentRun).where(AgentRun.id == event_id).values(
                    memory_status="disabled" if disabled else "synced" if error is None else ("failed" if attempt >= 8 else "pending")))
            await session.commit()
        processed += 1
    return processed
