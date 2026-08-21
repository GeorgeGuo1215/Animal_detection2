"""创建独立的 PetMind Agent 平台库表结构。

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
    """按平台 metadata 建表；PostgreSQL 额外启用 pg_trgm 并创建标题/内容模糊检索索引。"""
    bind = op.get_bind()
    # metadata 仅含 platform_* 对象；checkfirst 使本地已预建库时重复执行仍安全。
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
    """撤销 PostgreSQL 模糊检索索引，并仅删除平台 metadata 中的表。"""
    bind = op.get_bind()
    if bind.dialect.name == "postgresql":
        op.execute("DROP INDEX IF EXISTS ix_platform_messages_content_trgm")
        op.execute("DROP INDEX IF EXISTS ix_platform_conversations_title_trgm")
    # 仅删除独立平台 metadata 所表示的表。
    Base.metadata.drop_all(bind=bind, checkfirst=True)
