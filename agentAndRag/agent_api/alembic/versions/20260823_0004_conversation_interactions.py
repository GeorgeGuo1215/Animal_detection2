"""新增消息反馈与会话分支来源字段。

Revision ID: 20260823_0004
Revises: 20260820_0003
"""
from __future__ import annotations

import sqlalchemy as sa
from alembic import op


revision = "20260823_0004"
down_revision = "20260820_0003"
branch_labels = None
depends_on = None


def _columns(table_name: str) -> set[str]:
    return {column["name"] for column in sa.inspect(op.get_bind()).get_columns(table_name)}


def _indexes(table_name: str) -> set[str]:
    return {index["name"] for index in sa.inspect(op.get_bind()).get_indexes(table_name)}


def upgrade() -> None:
    """幂等增加分支来源、赞踩状态及必要索引。"""
    conversation_columns = _columns("platform_conversations")
    if "source_conversation_id" not in conversation_columns:
        op.add_column(
            "platform_conversations",
            sa.Column("source_conversation_id", sa.String(32), nullable=True),
        )
    if "forked_from_message_id" not in conversation_columns:
        op.add_column(
            "platform_conversations",
            sa.Column("forked_from_message_id", sa.String(32), nullable=True),
        )
    if "ix_platform_conversations_source_conversation_id" not in _indexes("platform_conversations"):
        op.create_index(
            "ix_platform_conversations_source_conversation_id",
            "platform_conversations",
            ["source_conversation_id"],
        )

    message_columns = _columns("platform_messages")
    if "feedback_rating" not in message_columns:
        op.add_column("platform_messages", sa.Column("feedback_rating", sa.String(8), nullable=True))
    if "feedback_updated_at" not in message_columns:
        op.add_column(
            "platform_messages",
            sa.Column("feedback_updated_at", sa.DateTime(timezone=True), nullable=True),
        )


def downgrade() -> None:
    """移除本版本增加的索引与字段。"""
    if "feedback_updated_at" in _columns("platform_messages"):
        op.drop_column("platform_messages", "feedback_updated_at")
    if "feedback_rating" in _columns("platform_messages"):
        op.drop_column("platform_messages", "feedback_rating")
    indexes = _indexes("platform_conversations")
    if "ix_platform_conversations_source_conversation_id" in indexes:
        op.drop_index(
            "ix_platform_conversations_source_conversation_id",
            table_name="platform_conversations",
        )
    conversation_columns = _columns("platform_conversations")
    if "forked_from_message_id" in conversation_columns:
        op.drop_column("platform_conversations", "forked_from_message_id")
    if "source_conversation_id" in conversation_columns:
        op.drop_column("platform_conversations", "source_conversation_id")
