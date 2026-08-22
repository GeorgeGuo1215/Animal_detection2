"""新增：用户设置、反馈、激活码兑换与法律文档同意记录。

Revision ID: 20260820_0003
Revises: 20260816_0002
"""
from __future__ import annotations

import sqlalchemy as sa
from alembic import op

revision = "20260820_0003"
down_revision = "20260816_0002"
branch_labels = None
depends_on = None


def upgrade() -> None:
    """创建偏好、常用语、反馈、激活码及其兑换、法律同意等表（表已存在则跳过）。"""
    bind = op.get_bind()
    existing = set(sa.inspect(bind).get_table_names())
    if "platform_user_preferences" not in existing:
        op.create_table(
            "platform_user_preferences",
            sa.Column("user_id", sa.String(32), sa.ForeignKey("platform_users.id", ondelete="CASCADE"), primary_key=True),
            sa.Column("theme", sa.String(16), nullable=False, server_default="system"),
            sa.Column("locale", sa.String(16), nullable=False, server_default="zh-CN"),
            sa.Column("default_expand_experts", sa.Boolean(), nullable=False, server_default=sa.true()),
            sa.Column("memory_recall_enabled", sa.Boolean(), nullable=False, server_default=sa.true()),
            sa.Column("memory_write_enabled", sa.Boolean(), nullable=False, server_default=sa.true()),
            sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        )
    if "platform_common_phrases" not in existing:
        op.create_table(
            "platform_common_phrases",
            sa.Column("id", sa.String(32), primary_key=True),
            sa.Column("user_id", sa.String(32), sa.ForeignKey("platform_users.id", ondelete="CASCADE"), nullable=False),
            sa.Column("title", sa.String(100), nullable=False, server_default=""),
            sa.Column("content", sa.Text(), nullable=False),
            sa.Column("sort_order", sa.Integer(), nullable=False, server_default="0"),
            sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        )
        op.create_index("ix_platform_common_phrases_user_id", "platform_common_phrases", ["user_id"])
    if "platform_feedback" not in existing:
        op.create_table(
            "platform_feedback",
            sa.Column("id", sa.String(32), primary_key=True),
            sa.Column("user_id", sa.String(32), sa.ForeignKey("platform_users.id", ondelete="CASCADE"), nullable=False),
            sa.Column("email_snapshot", sa.String(320), nullable=False),
            sa.Column("display_name_snapshot", sa.String(100), nullable=False, server_default=""),
            sa.Column("category", sa.String(40), nullable=False, server_default="product"),
            sa.Column("content", sa.Text(), nullable=False),
            sa.Column("contact", sa.String(320)),
            sa.Column("page_path", sa.String(500)),
            sa.Column("status", sa.String(24), nullable=False, server_default="submitted"),
            sa.Column("admin_note", sa.Text(), nullable=False, server_default=""),
            sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        )
        op.create_index("ix_platform_feedback_user_id", "platform_feedback", ["user_id"])
        op.create_index("ix_platform_feedback_status", "platform_feedback", ["status"])
    if "platform_activation_codes" not in existing:
        op.create_table(
            "platform_activation_codes",
            sa.Column("id", sa.String(32), primary_key=True),
            sa.Column("code_prefix", sa.String(16), nullable=False),
            sa.Column("code_hash", sa.String(64), nullable=False, unique=True),
            sa.Column("plan_code", sa.String(40), sa.ForeignKey("platform_plans.code")),
            sa.Column("extra_credits", sa.Integer(), nullable=False, server_default="0"),
            sa.Column("max_redemptions", sa.Integer(), nullable=False, server_default="1"),
            sa.Column("redemption_count", sa.Integer(), nullable=False, server_default="0"),
            sa.Column("per_user_limit", sa.Integer(), nullable=False, server_default="1"),
            sa.Column("starts_at", sa.DateTime(timezone=True)),
            sa.Column("expires_at", sa.DateTime(timezone=True)),
            sa.Column("status", sa.String(24), nullable=False, server_default="active"),
            sa.Column("created_by", sa.String(32)),
            sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("updated_at", sa.DateTime(timezone=True), nullable=False),
        )
        op.create_index("ix_platform_activation_codes_prefix", "platform_activation_codes", ["code_prefix"])
        op.create_index("ix_platform_activation_codes_hash", "platform_activation_codes", ["code_hash"], unique=True)
    if "platform_activation_redemptions" not in existing:
        op.create_table(
            "platform_activation_redemptions",
            sa.Column("id", sa.String(32), primary_key=True),
            sa.Column("code_id", sa.String(32), sa.ForeignKey("platform_activation_codes.id", ondelete="CASCADE"), nullable=False),
            sa.Column("user_id", sa.String(32), sa.ForeignKey("platform_users.id", ondelete="CASCADE"), nullable=False),
            sa.Column("plan_code", sa.String(40)),
            sa.Column("credits_granted", sa.Integer(), nullable=False, server_default="0"),
            sa.Column("redeemed_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("idempotency_key", sa.String(100)),
            sa.UniqueConstraint("code_id", "user_id", name="uq_platform_activation_redemption_user"),
        )
        op.create_index("ix_platform_activation_redemptions_code_id", "platform_activation_redemptions", ["code_id"])
        op.create_index("ix_platform_activation_redemptions_user_id", "platform_activation_redemptions", ["user_id"])
    if "platform_legal_acceptances" not in existing:
        op.create_table(
            "platform_legal_acceptances",
            sa.Column("id", sa.String(32), primary_key=True),
            sa.Column("user_id", sa.String(32), sa.ForeignKey("platform_users.id", ondelete="CASCADE"), nullable=False),
            sa.Column("document_type", sa.String(20), nullable=False),
            sa.Column("version", sa.String(32), nullable=False),
            sa.Column("accepted_at", sa.DateTime(timezone=True), nullable=False),
            sa.Column("ip_address", sa.String(64)),
            sa.Column("user_agent", sa.String(500)),
            sa.UniqueConstraint("user_id", "document_type", "version", name="uq_platform_legal_acceptance"),
        )
        op.create_index("ix_platform_legal_acceptances_user_id", "platform_legal_acceptances", ["user_id"])


def downgrade() -> None:
    """按依赖逆序删除本迁移创建的表（表不存在则跳过）。"""
    bind = op.get_bind()
    existing = set(sa.inspect(bind).get_table_names())
    for table in (
        "platform_legal_acceptances",
        "platform_activation_redemptions",
        "platform_activation_codes",
        "platform_feedback",
        "platform_common_phrases",
        "platform_user_preferences",
    ):
        if table in existing:
            op.drop_table(table)
