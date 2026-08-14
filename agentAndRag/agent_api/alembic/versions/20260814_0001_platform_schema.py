"""Create the isolated PetMind Agent platform schema.

Revision ID: 20260814_0001
Revises: None
"""
from __future__ import annotations

from alembic import op

from agent_api.app.platform.models import Base

revision = "20260814_0001"
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    bind = op.get_bind()
    # Metadata contains only platform_* objects. checkfirst makes a repeated
    # migration safe against a database provisioned during local prototyping.
    Base.metadata.create_all(bind=bind, checkfirst=True)
    if bind.dialect.name == "postgresql":
        op.execute("CREATE EXTENSION IF NOT EXISTS pg_trgm")
        op.execute(
            "CREATE INDEX IF NOT EXISTS ix_platform_conversations_title_trgm "
            "ON platform_conversations USING gin (title gin_trgm_ops)"
        )
        op.execute(
            "CREATE INDEX IF NOT EXISTS ix_platform_messages_content_trgm "
            "ON platform_messages USING gin (content gin_trgm_ops)"
        )


def downgrade() -> None:
    bind = op.get_bind()
    if bind.dialect.name == "postgresql":
        op.execute("DROP INDEX IF EXISTS ix_platform_messages_content_trgm")
        op.execute("DROP INDEX IF EXISTS ix_platform_conversations_title_trgm")
    # Only tables represented by the isolated platform metadata are removed.
    Base.metadata.drop_all(bind=bind, checkfirst=True)
