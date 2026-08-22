from __future__ import annotations

import uuid
from datetime import datetime, timezone

from sqlalchemy import (
    JSON,
    Boolean,
    DateTime,
    ForeignKey,
    Float,
    Index,
    Integer,
    Numeric,
    String,
    Text,
    UniqueConstraint,
)
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


def new_id() -> str:
    """生成 32 位十六进制主键 ID。"""
    return uuid.uuid4().hex


def utcnow() -> datetime:
    """返回当前 UTC 时间（带时区）。"""
    return datetime.now(timezone.utc)


class Base(DeclarativeBase):
    pass


class TimestampMixin:
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, nullable=False)
    updated_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, onupdate=utcnow, nullable=False)


class PlatformUser(Base, TimestampMixin):
    __tablename__ = "platform_users"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    email: Mapped[str] = mapped_column(String(320), unique=True, index=True)
    display_name: Mapped[str] = mapped_column(String(100), default="")
    password_hash: Mapped[str] = mapped_column(Text)
    role: Mapped[str] = mapped_column(String(32), default="VET", index=True)
    status: Mapped[str] = mapped_column(String(24), default="active", index=True)
    token_version: Mapped[int] = mapped_column(Integer, default=1)
    email_verified_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    last_login_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class Role(Base):
    __tablename__ = "platform_roles"
    code: Mapped[str] = mapped_column(String(32), primary_key=True)
    name: Mapped[str] = mapped_column(String(80))
    description: Mapped[str] = mapped_column(Text, default="")


class Permission(Base):
    __tablename__ = "platform_permissions"
    code: Mapped[str] = mapped_column(String(80), primary_key=True)
    description: Mapped[str] = mapped_column(Text, default="")


class RolePermission(Base):
    __tablename__ = "platform_role_permissions"
    role_code: Mapped[str] = mapped_column(ForeignKey("platform_roles.code", ondelete="CASCADE"), primary_key=True)
    permission_code: Mapped[str] = mapped_column(ForeignKey("platform_permissions.code", ondelete="CASCADE"), primary_key=True)


class Invitation(Base, TimestampMixin):
    __tablename__ = "platform_invitations"
    __table_args__ = (UniqueConstraint("created_by", "idempotency_key", name="uq_platform_invitation_idem"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    email: Mapped[str] = mapped_column(String(320), index=True)
    token_hash: Mapped[str] = mapped_column(String(64), unique=True)
    role: Mapped[str] = mapped_column(String(32), default="VET")
    initial_plan_code: Mapped[str | None] = mapped_column(String(40), nullable=True)
    expires_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), index=True)
    accepted_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    revoked_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    created_by: Mapped[str] = mapped_column(ForeignKey("platform_users.id"), index=True)
    idempotency_key: Mapped[str | None] = mapped_column(String(100), nullable=True)


class RefreshToken(Base):
    __tablename__ = "platform_refresh_tokens"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    family_id: Mapped[str] = mapped_column(String(32), index=True)
    token_hash: Mapped[str] = mapped_column(String(64), unique=True)
    expires_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), index=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    revoked_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    replaced_by_id: Mapped[str | None] = mapped_column(String(32), nullable=True)


class PasswordResetToken(Base):
    __tablename__ = "platform_password_reset_tokens"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    token_hash: Mapped[str] = mapped_column(String(64), unique=True)
    expires_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), index=True)
    used_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)


class ApiKey(Base, TimestampMixin):
    __tablename__ = "platform_api_keys"
    __table_args__ = (UniqueConstraint("user_id", "idempotency_key", name="uq_platform_api_key_idem"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    name: Mapped[str] = mapped_column(String(80))
    key_prefix: Mapped[str] = mapped_column(String(20), unique=True, index=True)
    key_hash: Mapped[str] = mapped_column(String(64), unique=True)
    scopes: Mapped[list] = mapped_column(JSON, default=list)
    expires_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    revoked_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    last_used_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    idempotency_key: Mapped[str | None] = mapped_column(String(100), nullable=True)


class UserPreference(Base, TimestampMixin):
    __tablename__ = "platform_user_preferences"
    user_id: Mapped[str] = mapped_column(
        ForeignKey("platform_users.id", ondelete="CASCADE"), primary_key=True
    )
    theme: Mapped[str] = mapped_column(String(16), default="system")
    locale: Mapped[str] = mapped_column(String(16), default="zh-CN")
    default_expand_experts: Mapped[bool] = mapped_column(Boolean, default=True)
    memory_recall_enabled: Mapped[bool] = mapped_column(Boolean, default=True)
    memory_write_enabled: Mapped[bool] = mapped_column(Boolean, default=True)


class CommonPhrase(Base, TimestampMixin):
    __tablename__ = "platform_common_phrases"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(
        ForeignKey("platform_users.id", ondelete="CASCADE"), index=True
    )
    title: Mapped[str] = mapped_column(String(100), default="")
    content: Mapped[str] = mapped_column(Text)
    sort_order: Mapped[int] = mapped_column(Integer, default=0)


class UserFeedback(Base, TimestampMixin):
    __tablename__ = "platform_feedback"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(
        ForeignKey("platform_users.id", ondelete="CASCADE"), index=True
    )
    email_snapshot: Mapped[str] = mapped_column(String(320))
    display_name_snapshot: Mapped[str] = mapped_column(String(100), default="")
    category: Mapped[str] = mapped_column(String(40), default="product")
    content: Mapped[str] = mapped_column(Text)
    contact: Mapped[str | None] = mapped_column(String(320), nullable=True)
    page_path: Mapped[str | None] = mapped_column(String(500), nullable=True)
    status: Mapped[str] = mapped_column(String(24), default="submitted", index=True)
    admin_note: Mapped[str] = mapped_column(Text, default="")


class ActivationCode(Base, TimestampMixin):
    __tablename__ = "platform_activation_codes"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    code_prefix: Mapped[str] = mapped_column(String(16), index=True)
    code_hash: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    plan_code: Mapped[str | None] = mapped_column(
        ForeignKey("platform_plans.code"), nullable=True
    )
    extra_credits: Mapped[int] = mapped_column(Integer, default=0)
    max_redemptions: Mapped[int] = mapped_column(Integer, default=1)
    redemption_count: Mapped[int] = mapped_column(Integer, default=0)
    per_user_limit: Mapped[int] = mapped_column(Integer, default=1)
    starts_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    expires_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True, index=True)
    status: Mapped[str] = mapped_column(String(24), default="active", index=True)
    created_by: Mapped[str | None] = mapped_column(String(32), nullable=True)


class ActivationRedemption(Base):
    __tablename__ = "platform_activation_redemptions"
    __table_args__ = (
        UniqueConstraint("code_id", "user_id", name="uq_platform_activation_redemption_user"),
    )
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    code_id: Mapped[str] = mapped_column(
        ForeignKey("platform_activation_codes.id", ondelete="CASCADE"), index=True
    )
    user_id: Mapped[str] = mapped_column(
        ForeignKey("platform_users.id", ondelete="CASCADE"), index=True
    )
    plan_code: Mapped[str | None] = mapped_column(String(40), nullable=True)
    credits_granted: Mapped[int] = mapped_column(Integer, default=0)
    redeemed_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    idempotency_key: Mapped[str | None] = mapped_column(String(100), nullable=True)


class LegalAcceptance(Base):
    __tablename__ = "platform_legal_acceptances"
    __table_args__ = (
        UniqueConstraint("user_id", "document_type", "version", name="uq_platform_legal_acceptance"),
    )
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(
        ForeignKey("platform_users.id", ondelete="CASCADE"), index=True
    )
    document_type: Mapped[str] = mapped_column(String(20))
    version: Mapped[str] = mapped_column(String(32))
    accepted_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    ip_address: Mapped[str | None] = mapped_column(String(64), nullable=True)
    user_agent: Mapped[str | None] = mapped_column(String(500), nullable=True)


class Conversation(Base, TimestampMixin):
    __tablename__ = "platform_conversations"
    __table_args__ = (UniqueConstraint("user_id", "idempotency_key", name="uq_platform_conversation_idem"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    title: Mapped[str] = mapped_column(String(160), default="新会诊")
    status: Mapped[str] = mapped_column(String(24), default="active", index=True)
    last_active_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, index=True)
    deleted_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True, index=True)
    idempotency_key: Mapped[str | None] = mapped_column(String(100), nullable=True)
    source_conversation_id: Mapped[str | None] = mapped_column(String(32), nullable=True, index=True)
    forked_from_message_id: Mapped[str | None] = mapped_column(String(32), nullable=True)


class Message(Base):
    __tablename__ = "platform_messages"
    __table_args__ = (UniqueConstraint("conversation_id", "client_message_id", name="uq_platform_message_client"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    conversation_id: Mapped[str] = mapped_column(ForeignKey("platform_conversations.id", ondelete="CASCADE"), index=True)
    run_id: Mapped[str | None] = mapped_column(String(32), index=True, nullable=True)
    client_message_id: Mapped[str | None] = mapped_column(String(100), nullable=True)
    role: Mapped[str] = mapped_column(String(20))
    content: Mapped[str] = mapped_column(Text)
    status: Mapped[str] = mapped_column(String(20), default="complete")
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, index=True)
    feedback_rating: Mapped[str | None] = mapped_column(String(8), nullable=True)
    feedback_updated_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class AgentRun(Base, TimestampMixin):
    __tablename__ = "platform_agent_runs"
    __table_args__ = (UniqueConstraint("user_id", "idempotency_key", name="uq_platform_run_idempotency"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    conversation_id: Mapped[str] = mapped_column(ForeignKey("platform_conversations.id", ondelete="CASCADE"), index=True)
    user_message_id: Mapped[str] = mapped_column(ForeignKey("platform_messages.id"))
    assistant_message_id: Mapped[str | None] = mapped_column(ForeignKey("platform_messages.id"), nullable=True)
    idempotency_key: Mapped[str] = mapped_column(String(100))
    delivery: Mapped[str] = mapped_column(String(16), default="sse")
    status: Mapped[str] = mapped_column(String(24), default="queued", index=True)
    query: Mapped[str] = mapped_column(Text)
    parameters: Mapped[dict] = mapped_column(JSON, default=dict)
    response: Mapped[str] = mapped_column(Text, default="")
    error_code: Mapped[str | None] = mapped_column(String(80), nullable=True)
    error_message: Mapped[str | None] = mapped_column(Text, nullable=True)
    reserved_credits: Mapped[int] = mapped_column(Integer, default=0)
    actual_credits: Mapped[int] = mapped_column(Integer, default=0)
    cancel_requested: Mapped[bool] = mapped_column(Boolean, default=False)
    started_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    finished_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class RunEvent(Base):
    __tablename__ = "platform_run_events"
    __table_args__ = (UniqueConstraint("run_id", "sequence", name="uq_platform_run_event_sequence"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    run_id: Mapped[str] = mapped_column(ForeignKey("platform_agent_runs.id", ondelete="CASCADE"), index=True)
    sequence: Mapped[int] = mapped_column(Integer)
    event_type: Mapped[str] = mapped_column(String(50))
    payload: Mapped[dict] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)


class ExpertConsultation(Base, TimestampMixin):
    """一次 MoE 专家会诊的持久化、可对用户公开的工作产物。"""

    __tablename__ = "platform_expert_consultations"
    __table_args__ = (
        UniqueConstraint("run_id", "expert_key", name="uq_platform_expert_consultation_run_expert"),
    )
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    run_id: Mapped[str] = mapped_column(
        ForeignKey("platform_agent_runs.id", ondelete="CASCADE"), index=True
    )
    conversation_id: Mapped[str] = mapped_column(
        ForeignKey("platform_conversations.id", ondelete="CASCADE"), index=True
    )
    expert_key: Mapped[str] = mapped_column(String(40))
    expert_name: Mapped[str] = mapped_column(String(100), default="专家")
    status: Mapped[str] = mapped_column(String(24), default="completed")
    task: Mapped[str] = mapped_column(Text, default="")
    required_tools: Mapped[list] = mapped_column(JSON, default=list)
    recommended_tools: Mapped[list] = mapped_column(JSON, default=list)
    tool_summaries: Mapped[list] = mapped_column(JSON, default=list)
    conclusion: Mapped[str] = mapped_column(Text, default="")
    evidence: Mapped[list] = mapped_column(JSON, default=list)
    risks: Mapped[list] = mapped_column(JSON, default=list)
    confidence: Mapped[float] = mapped_column(Float, default=0.0)
    execution: Mapped[str] = mapped_column(String(32), default="single_pass")


class Plan(Base, TimestampMixin):
    __tablename__ = "platform_plans"
    code: Mapped[str] = mapped_column(String(40), primary_key=True)
    name: Mapped[str] = mapped_column(String(100))
    description: Mapped[str] = mapped_column(Text, default="")
    billing_period: Mapped[str] = mapped_column(String(20))
    price_cents: Mapped[int] = mapped_column(Integer, default=0)
    currency: Mapped[str] = mapped_column(String(8), default="CNY")
    credit_grant: Mapped[int] = mapped_column(Integer, default=0)
    duration_days: Mapped[int] = mapped_column(Integer, default=0)
    active: Mapped[bool] = mapped_column(Boolean, default=False, index=True)
    features: Mapped[dict] = mapped_column(JSON, default=dict)


class Subscription(Base, TimestampMixin):
    __tablename__ = "platform_subscriptions"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    plan_code: Mapped[str] = mapped_column(ForeignKey("platform_plans.code"), index=True)
    status: Mapped[str] = mapped_column(String(24), default="active", index=True)
    starts_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    expires_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True, index=True)
    order_id: Mapped[str | None] = mapped_column(String(32), nullable=True)


class Order(Base, TimestampMixin):
    __tablename__ = "platform_orders"
    __table_args__ = (UniqueConstraint("user_id", "idempotency_key", name="uq_platform_order_idem"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    plan_code: Mapped[str] = mapped_column(ForeignKey("platform_plans.code"))
    status: Mapped[str] = mapped_column(String(30), default="pending_payment", index=True)
    amount_cents: Mapped[int] = mapped_column(Integer)
    currency: Mapped[str] = mapped_column(String(8), default="CNY")
    external_reference: Mapped[str | None] = mapped_column(String(120), nullable=True, index=True)
    paid_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    fulfilled_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    idempotency_key: Mapped[str | None] = mapped_column(String(100), nullable=True)


class PaymentEvent(Base):
    __tablename__ = "platform_payment_events"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    external_event_id: Mapped[str] = mapped_column(String(120), unique=True, index=True)
    order_id: Mapped[str] = mapped_column(ForeignKey("platform_orders.id"), index=True)
    event_type: Mapped[str] = mapped_column(String(60))
    payload: Mapped[dict] = mapped_column(JSON, default=dict)
    processed_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)


class CreditAccount(Base):
    __tablename__ = "platform_credit_accounts"
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), primary_key=True)
    balance: Mapped[int] = mapped_column(Integer, default=0)
    reserved: Mapped[int] = mapped_column(Integer, default=0)
    updated_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, onupdate=utcnow)


class CreditLedger(Base):
    __tablename__ = "platform_credit_ledger"
    __table_args__ = (UniqueConstraint("user_id", "idempotency_key", name="uq_platform_credit_ledger_idem"),)
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    amount: Mapped[int] = mapped_column(Integer)
    balance_after: Mapped[int] = mapped_column(Integer)
    reason: Mapped[str] = mapped_column(String(60))
    reference_type: Mapped[str] = mapped_column(String(40))
    reference_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    idempotency_key: Mapped[str] = mapped_column(String(140))
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, index=True)


class CreditReservation(Base):
    __tablename__ = "platform_credit_reservations"
    run_id: Mapped[str] = mapped_column(ForeignKey("platform_agent_runs.id", ondelete="CASCADE"), primary_key=True)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    amount: Mapped[int] = mapped_column(Integer)
    status: Mapped[str] = mapped_column(String(20), default="active")
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    settled_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class UsageRecord(Base):
    __tablename__ = "platform_usage_records"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    run_id: Mapped[str] = mapped_column(ForeignKey("platform_agent_runs.id", ondelete="CASCADE"), unique=True)
    user_id: Mapped[str] = mapped_column(ForeignKey("platform_users.id", ondelete="CASCADE"), index=True)
    input_tokens: Mapped[int] = mapped_column(Integer, default=0)
    output_tokens: Mapped[int] = mapped_column(Integer, default=0)
    tool_calls: Mapped[int] = mapped_column(Integer, default=0)
    expert_calls: Mapped[int] = mapped_column(Integer, default=0)
    credits: Mapped[int] = mapped_column(Integer, default=0)
    details: Mapped[dict] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)


class AuditLog(Base):
    __tablename__ = "platform_audit_logs"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    actor_user_id: Mapped[str | None] = mapped_column(String(32), nullable=True, index=True)
    action: Mapped[str] = mapped_column(String(100), index=True)
    resource_type: Mapped[str] = mapped_column(String(60))
    resource_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    request_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    ip_address: Mapped[str | None] = mapped_column(String(64), nullable=True)
    detail: Mapped[dict] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow, index=True)


class OutboxEvent(Base):
    __tablename__ = "platform_outbox_events"
    id: Mapped[str] = mapped_column(String(32), primary_key=True, default=new_id)
    topic: Mapped[str] = mapped_column(String(100), index=True)
    payload: Mapped[dict] = mapped_column(JSON, default=dict)
    status: Mapped[str] = mapped_column(String(20), default="pending", index=True)
    available_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), default=utcnow)
    processed_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    attempts: Mapped[int] = mapped_column(Integer, default=0)


Index("ix_platform_messages_search", Message.conversation_id, Message.created_at)
Index("ix_platform_runs_owner_status", AgentRun.user_id, AgentRun.status)
Index("ix_platform_expert_consultations_conversation_created", ExpertConsultation.conversation_id, ExpertConsultation.created_at)
Index("ix_platform_orders_owner_status", Order.user_id, Order.status)
