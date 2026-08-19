from __future__ import annotations

from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, str_strip_whitespace=True)


class InvitationAcceptRequest(StrictModel):
    token: str = Field(min_length=32, max_length=300)
    password: str = Field(min_length=10, max_length=200)
    display_name: str = Field(min_length=1, max_length=100)
    terms_version: str = Field(min_length=1, max_length=32)
    privacy_version: str = Field(min_length=1, max_length=32)
    accept_terms: bool
    accept_privacy: bool


class LoginRequest(StrictModel):
    email: str = Field(min_length=3, max_length=320)
    password: str = Field(min_length=1, max_length=200)


class ForgotPasswordRequest(StrictModel):
    email: str = Field(min_length=3, max_length=320)


class ResetPasswordRequest(StrictModel):
    token: str = Field(min_length=32, max_length=300)
    password: str = Field(min_length=10, max_length=200)


class TokenResponse(StrictModel):
    access_token: str
    token_type: str = "bearer"
    expires_in: int
    user: dict[str, Any]


class ApiKeyCreateRequest(StrictModel):
    name: str = Field(min_length=1, max_length=80)
    scopes: list[str] = Field(default_factory=lambda: ["chat:write", "models:read"], max_length=20)
    expires_at: datetime | None = None

    @field_validator("scopes")
    @classmethod
    def validate_scopes(cls, value: list[str]) -> list[str]:
        allowed = {"chat:write", "models:read", "runs:read"}
        normalized = sorted(set(value))
        if not normalized or any(scope not in allowed for scope in normalized):
            raise ValueError("unsupported API key scope")
        return normalized


class ProfileUpdateRequest(StrictModel):
    display_name: str = Field(min_length=1, max_length=100)


class PreferenceUpdateRequest(StrictModel):
    theme: Literal["light", "dark", "system"] | None = None
    default_expand_experts: bool | None = None
    memory_recall_enabled: bool | None = None
    memory_write_enabled: bool | None = None


class CommonPhraseCreateRequest(StrictModel):
    title: str = Field(default="", max_length=100)
    content: str = Field(min_length=1, max_length=2000)
    sort_order: int = Field(default=0, ge=0, le=10000)


class CommonPhraseUpdateRequest(StrictModel):
    title: str | None = Field(default=None, max_length=100)
    content: str | None = Field(default=None, min_length=1, max_length=2000)
    sort_order: int | None = Field(default=None, ge=0, le=10000)


class FeedbackCreateRequest(StrictModel):
    category: Literal["product", "answer", "billing", "security", "other"] = "product"
    content: str = Field(min_length=5, max_length=4000)
    contact: str | None = Field(default=None, max_length=320)
    page_path: str | None = Field(default=None, max_length=500)


class ActivationCodeRedeemRequest(StrictModel):
    code: str = Field(min_length=8, max_length=64, pattern=r"^[A-Za-z0-9-]+$")


class LegalAcceptRequest(StrictModel):
    terms_version: str = Field(min_length=1, max_length=32)
    privacy_version: str = Field(min_length=1, max_length=32)
    accept_terms: bool
    accept_privacy: bool


class MemoryClearRequest(StrictModel):
    scope: Literal["short_term", "knowledge", "profile"]


class ConversationCreateRequest(StrictModel):
    title: str = Field(default="新会诊", min_length=1, max_length=160)


class ConversationUpdateRequest(StrictModel):
    title: str | None = Field(default=None, min_length=1, max_length=160)
    status: Literal["active", "archived"] | None = None


class RunCreateRequest(StrictModel):
    message: str = Field(min_length=1, max_length=30_000)
    client_message_id: str = Field(min_length=8, max_length=100)
    delivery: Literal["sse", "sync", "async"] = "sse"
    user_role: Literal["pet_owner", "veterinarian"] = "veterinarian"
    temperature: float = Field(default=0.3, ge=0.0, le=1.0)
    max_tokens: int = Field(default=2500, ge=64, le=4000)


class OrderCreateRequest(StrictModel):
    plan_code: Literal["pro_monthly", "pro_yearly"]


class TestPaymentWebhook(StrictModel):
    event_id: str = Field(min_length=8, max_length=120)
    order_id: str = Field(min_length=16, max_length=64)
    event_type: Literal["payment.succeeded", "payment.refunded"]


class AdminInvitationCreateRequest(StrictModel):
    email: str = Field(min_length=3, max_length=320)
    role: Literal["VET", "SUPPORT_ADMIN", "BILLING_ADMIN"] = "VET"
    initial_plan_code: str | None = Field(default="trial", max_length=40)


class AdminUserUpdateRequest(StrictModel):
    status: Literal["active", "suspended", "deleted"] | None = None
    role: Literal["VET", "SUPPORT_ADMIN", "BILLING_ADMIN", "SUPER_ADMIN"] | None = None


class AdminPlanUpdateRequest(StrictModel):
    name: str | None = Field(default=None, min_length=1, max_length=100)
    description: str | None = Field(default=None, max_length=2000)
    price_cents: int | None = Field(default=None, ge=0, le=100_000_000)
    credit_grant: int | None = Field(default=None, ge=0, le=100_000_000)
    duration_days: int | None = Field(default=None, ge=0, le=3660)
    active: bool | None = None
    features: dict[str, Any] | None = None


class CreditAdjustRequest(StrictModel):
    amount: int = Field(ge=-100_000_000, le=100_000_000)
    reason: str = Field(min_length=3, max_length=120)
    idempotency_key: str = Field(min_length=8, max_length=140)


class AdminSubscriptionUpdateRequest(StrictModel):
    status: Literal["active", "expired", "suspended", "cancelled"] | None = None
    expires_at: datetime | None = None


class AdminOrderUpdateRequest(StrictModel):
    status: Literal["cancelled", "expired", "refunded"]


class AdminUserDataRestoreRequest(StrictModel):
    confirmation: Literal["覆盖恢复用户数据"]
    snapshot: dict[str, Any]
