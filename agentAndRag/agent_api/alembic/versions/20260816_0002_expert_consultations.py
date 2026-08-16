"""Persist public-safe MoE expert consultations per Agent Run.

Revision ID: 20260816_0002
Revises: 20260814_0001
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone

import sqlalchemy as sa
from alembic import op


revision = "20260816_0002"
down_revision = "20260814_0001"
branch_labels = None
depends_on = None

TABLE_NAME = "platform_expert_consultations"


def _as_payload(value):
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            return {}
    return {}


def _create_table_if_missing(bind) -> None:
    inspector = sa.inspect(bind)
    if TABLE_NAME not in inspector.get_table_names():
        op.create_table(
            TABLE_NAME,
            sa.Column("id", sa.String(length=32), primary_key=True),
            sa.Column("run_id", sa.String(length=32), sa.ForeignKey("platform_agent_runs.id", ondelete="CASCADE"), nullable=False),
            sa.Column("conversation_id", sa.String(length=32), sa.ForeignKey("platform_conversations.id", ondelete="CASCADE"), nullable=False),
            sa.Column("expert_key", sa.String(length=40), nullable=False),
            sa.Column("expert_name", sa.String(length=100), nullable=False),
            sa.Column("status", sa.String(length=24), nullable=False),
            sa.Column("task", sa.Text(), nullable=False),
            sa.Column("required_tools", sa.JSON(), nullable=False),
            sa.Column("recommended_tools", sa.JSON(), nullable=False),
            sa.Column("tool_summaries", sa.JSON(), nullable=False),
            sa.Column("conclusion", sa.Text(), nullable=False),
            sa.Column("evidence", sa.JSON(), nullable=False),
            sa.Column("risks", sa.JSON(), nullable=False),
            sa.Column("confidence", sa.Float(), nullable=False),
            sa.Column("execution", sa.String(length=32), nullable=False),
            sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
            sa.UniqueConstraint("run_id", "expert_key", name="uq_platform_expert_consultation_run_expert"),
        )
        op.create_index("ix_platform_expert_consultations_run_id", TABLE_NAME, ["run_id"])
        op.create_index("ix_platform_expert_consultations_conversation_id", TABLE_NAME, ["conversation_id"])
        op.create_index(
            "ix_platform_expert_consultations_conversation_created",
            TABLE_NAME,
            ["conversation_id", "created_at"],
        )


def _backfill_from_run_events(bind) -> None:
    metadata = sa.MetaData()
    consultations = sa.Table(TABLE_NAME, metadata, autoload_with=bind)
    events = sa.Table("platform_run_events", metadata, autoload_with=bind)
    runs = sa.Table("platform_agent_runs", metadata, autoload_with=bind)
    existing = set(bind.execute(sa.select(consultations.c.run_id, consultations.c.expert_key)).all())
    rows = bind.execute(
        sa.select(events.c.run_id, events.c.payload, events.c.created_at, runs.c.conversation_id)
        .join(runs, runs.c.id == events.c.run_id)
        .where(events.c.event_type == "status")
        .order_by(events.c.sequence.asc())
    ).all()
    inserts = []
    for run_id, raw_payload, created_at, conversation_id in rows:
        expert = _as_payload(raw_payload).get("expert")
        if not isinstance(expert, dict) or expert.get("status") != "completed":
            continue
        expert_key = str(expert.get("expert") or "")[:40]
        if not expert_key or (run_id, expert_key) in existing:
            continue
        opinion = expert.get("opinion") if isinstance(expert.get("opinion"), dict) else {}
        timestamp = created_at or datetime.now(timezone.utc)
        inserts.append({
            "id": uuid.uuid4().hex,
            "run_id": run_id,
            "conversation_id": conversation_id,
            "expert_key": expert_key,
            "expert_name": str(expert.get("name") or "专家")[:100],
            "status": "completed",
            "task": str(expert.get("task") or ""),
            "required_tools": list(expert.get("required_tools") or []),
            "recommended_tools": list(expert.get("recommended_tools") or []),
            "tool_summaries": list(expert.get("tools") or []),
            "conclusion": str(opinion.get("conclusion") or ""),
            "evidence": list(opinion.get("evidence") or []),
            "risks": list(opinion.get("risks") or []),
            "confidence": float(opinion.get("confidence") or 0.0),
            "execution": str(expert.get("execution") or "single_pass")[:32],
            "created_at": timestamp,
            "updated_at": timestamp,
        })
        existing.add((run_id, expert_key))
    if inserts:
        bind.execute(consultations.insert(), inserts)


def upgrade() -> None:
    bind = op.get_bind()
    _create_table_if_missing(bind)
    _backfill_from_run_events(bind)


def downgrade() -> None:
    bind = op.get_bind()
    if TABLE_NAME not in sa.inspect(bind).get_table_names():
        return
    op.drop_index("ix_platform_expert_consultations_conversation_created", table_name=TABLE_NAME)
    op.drop_index("ix_platform_expert_consultations_conversation_id", table_name=TABLE_NAME)
    op.drop_index("ix_platform_expert_consultations_run_id", table_name=TABLE_NAME)
    op.drop_table(TABLE_NAME)
