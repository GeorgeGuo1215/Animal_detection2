"""Fence Run execution and allocate durable event sequences.

Revision ID: 20260905_0005
Revises: 20260823_0004
"""
import sqlalchemy as sa
from alembic import op

revision = "20260905_0005"
down_revision = "20260823_0004"
branch_labels = None
depends_on = None


def upgrade() -> None:
    columns = {c["name"] for c in sa.inspect(op.get_bind()).get_columns("platform_agent_runs")}
    additions = [
        sa.Column("claimed_by", sa.String(64), nullable=True),
        sa.Column("lease_until", sa.DateTime(timezone=True), nullable=True),
        sa.Column("execution_epoch", sa.Integer(), nullable=False, server_default="0"),
        sa.Column("event_sequence", sa.Integer(), nullable=False, server_default="0"),
        sa.Column("memory_status", sa.String(20), nullable=False, server_default="disabled"),
    ]
    for column in additions:
        if column.name not in columns:
            op.add_column("platform_agent_runs", column)
    # Existing event IDs remain valid for Last-Event-ID clients.
    op.execute(sa.text("UPDATE platform_agent_runs SET event_sequence = "
                       "(SELECT COALESCE(MAX(sequence), 0) FROM platform_run_events "
                       "WHERE run_id = platform_agent_runs.id)"))
    indexes = {i["name"] for i in sa.inspect(op.get_bind()).get_indexes("platform_agent_runs")}
    if "ix_platform_agent_runs_lease_until" not in indexes:
        op.create_index("ix_platform_agent_runs_lease_until", "platform_agent_runs", ["lease_until"])


def downgrade() -> None:
    op.drop_index("ix_platform_agent_runs_lease_until", table_name="platform_agent_runs")
    for name in ("memory_status", "event_sequence", "execution_epoch", "lease_until", "claimed_by"):
        op.drop_column("platform_agent_runs", name)
