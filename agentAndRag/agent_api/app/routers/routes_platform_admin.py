from __future__ import annotations

from datetime import timedelta

from fastapi import APIRouter, Depends, Header, HTTPException, Query
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, require_roles
from ..platform.models import (
    AgentRun,
    ApiKey,
    AuditLog,
    Invitation,
    Order,
    OutboxEvent,
    Plan,
    PlatformUser,
    Subscription,
    utcnow,
)
from ..platform.schemas import (
    AdminInvitationCreateRequest,
    AdminOrderUpdateRequest,
    AdminPlanUpdateRequest,
    AdminSubscriptionUpdateRequest,
    AdminUserUpdateRequest,
    CreditAdjustRequest,
)
from ..platform.security import hash_secret, normalize_email, secure_token
from ..platform.services import adjust_credits, audit, fulfill_order, user_payload

router = APIRouter(prefix="/api/v1/admin", tags=["platform-admin"])
ADMIN_ROLES = ("SUPPORT_ADMIN", "BILLING_ADMIN", "SUPER_ADMIN")


@router.get("/overview")
async def overview(
    principal: Principal = Depends(require_roles(*ADMIN_ROLES)),
    session: AsyncSession = Depends(get_platform_session),
):
    users = len(list((await session.scalars(select(PlatformUser.id))).all()))
    pending_orders = len(list((await session.scalars(select(Order.id).where(Order.status == "pending_payment"))).all()))
    active_runs = len(list((await session.scalars(select(AgentRun.id).where(AgentRun.status.in_(["queued", "running", "cancel_requested"])))).all()))
    return {"users": users, "pending_orders": pending_orders, "active_runs": active_runs}


@router.post("/invitations", status_code=201)
async def create_invitation(
    body: AdminInvitationCreateRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    idem = idempotency_key.strip()[:100] if idempotency_key else None
    if idem:
        existing = await session.scalar(select(Invitation).where(Invitation.created_by == principal.user_id, Invitation.idempotency_key == idem))
        if existing is not None:
            return {"id": existing.id, "email": existing.email, "role": existing.role, "expires_at": existing.expires_at, "replayed": True}
    try:
        email = normalize_email(body.email)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if await session.scalar(select(PlatformUser.id).where(PlatformUser.email == email)):
        raise HTTPException(status_code=409, detail="email is already registered")
    raw = secure_token(40)
    invitation = Invitation(
        email=email,
        token_hash=hash_secret(raw),
        role=body.role,
        initial_plan_code=body.initial_plan_code,
        expires_at=utcnow() + timedelta(seconds=get_platform_settings().invite_ttl_seconds),
        created_by=principal.user_id,
        idempotency_key=idem,
    )
    session.add(invitation)
    session.add(OutboxEvent(topic="auth.invitation", payload={"email": email, "token": raw}))
    await audit(session, action="invitation.created", resource_type="invitation", actor_user_id=principal.user_id, resource_id=invitation.id)
    await session.commit()
    result = {"id": invitation.id, "email": email, "role": invitation.role, "expires_at": invitation.expires_at}
    if get_platform_settings().expose_dev_tokens:
        result["development_token"] = raw
    return result


@router.get("/invitations")
async def list_invitations(
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(Invitation).order_by(Invitation.created_at.desc()).limit(200))).all())
    return {"items": [{
        "id": item.id,
        "email": item.email,
        "role": item.role,
        "initial_plan_code": item.initial_plan_code,
        "expires_at": item.expires_at,
        "accepted_at": item.accepted_at,
        "revoked_at": item.revoked_at,
    } for item in rows]}


@router.delete("/invitations/{invitation_id}")
async def revoke_invitation(
    invitation_id: str,
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    item = await session.get(Invitation, invitation_id)
    if item is None:
        raise HTTPException(status_code=404, detail="invitation not found")
    item.revoked_at = utcnow()
    await session.commit()
    return {"ok": True}


@router.get("/users")
async def list_users(
    q: str = Query(default="", max_length=200),
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    statement = select(PlatformUser)
    if q:
        statement = statement.where(PlatformUser.email.ilike(f"%{q}%"))
    rows = list((await session.scalars(statement.order_by(PlatformUser.created_at.desc()).limit(200))).all())
    return {"items": [user_payload(item) for item in rows]}


@router.patch("/users/{user_id}")
async def update_user(
    user_id: str,
    body: AdminUserUpdateRequest,
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    user = await session.get(PlatformUser, user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="user not found")
    if user.id == principal.user_id and body.status in {"suspended", "deleted"}:
        raise HTTPException(status_code=400, detail="cannot disable current administrator")
    if body.status is not None and body.status != user.status:
        user.status = body.status
        user.token_version += 1
    if body.role is not None:
        user.role = body.role
        user.token_version += 1
    await audit(session, action="user.updated", resource_type="user", actor_user_id=principal.user_id, resource_id=user.id, detail=body.model_dump(exclude_none=True))
    await session.commit()
    return user_payload(user)


@router.get("/plans")
async def admin_list_plans(
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(Plan).order_by(Plan.price_cents.asc()))).all())
    return {"items": [{
        "code": item.code,
        "name": item.name,
        "description": item.description,
        "billing_period": item.billing_period,
        "price_cents": item.price_cents,
        "credit_grant": item.credit_grant,
        "duration_days": item.duration_days,
        "active": item.active,
        "features": item.features,
    } for item in rows]}


@router.patch("/plans/{plan_code}")
async def update_plan(
    plan_code: str,
    body: AdminPlanUpdateRequest,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    plan = await session.get(Plan, plan_code)
    if plan is None:
        raise HTTPException(status_code=404, detail="plan not found")
    for key, value in body.model_dump(exclude_none=True).items():
        setattr(plan, key, value)
    await audit(session, action="plan.updated", resource_type="plan", actor_user_id=principal.user_id, resource_id=plan.code, detail=body.model_dump(exclude_none=True))
    await session.commit()
    return {"ok": True, "code": plan.code}


@router.get("/orders")
async def admin_list_orders(
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(Order).order_by(Order.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "plan_code": item.plan_code, "status": item.status, "amount_cents": item.amount_cents, "created_at": item.created_at} for item in rows]}


@router.post("/orders/{order_id}/confirm")
async def confirm_order(
    order_id: str,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    order = await session.get(Order, order_id)
    if order is None:
        raise HTTPException(status_code=404, detail="order not found")
    try:
        await fulfill_order(session, order)
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    await audit(session, action="order.confirmed", resource_type="order", actor_user_id=principal.user_id, resource_id=order.id)
    await session.commit()
    return {"ok": True, "status": order.status}


@router.patch("/orders/{order_id}")
async def update_order_status(
    order_id: str,
    body: AdminOrderUpdateRequest,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    order = await session.get(Order, order_id)
    if order is None:
        raise HTTPException(status_code=404, detail="order not found")
    if order.status == "fulfilled" and body.status != "refunded":
        raise HTTPException(status_code=409, detail="fulfilled order can only be marked refunded")
    order.status = body.status
    await audit(session, action="order.status_updated", resource_type="order", actor_user_id=principal.user_id, resource_id=order.id, detail={"status": body.status})
    await session.commit()
    return {"ok": True, "status": order.status}


@router.get("/subscriptions")
async def list_subscriptions(
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(Subscription).order_by(Subscription.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "plan_code": item.plan_code, "status": item.status, "starts_at": item.starts_at, "expires_at": item.expires_at} for item in rows]}


@router.patch("/subscriptions/{subscription_id}")
async def update_subscription(
    subscription_id: str,
    body: AdminSubscriptionUpdateRequest,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    item = await session.get(Subscription, subscription_id)
    if item is None:
        raise HTTPException(status_code=404, detail="subscription not found")
    if body.status is not None:
        item.status = body.status
    if body.expires_at is not None:
        item.expires_at = body.expires_at
    await audit(session, action="subscription.updated", resource_type="subscription", actor_user_id=principal.user_id, resource_id=item.id, detail=body.model_dump(exclude_none=True))
    await session.commit()
    return {"ok": True, "id": item.id, "status": item.status, "expires_at": item.expires_at}


@router.get("/api-keys")
async def admin_list_api_keys(
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(ApiKey).order_by(ApiKey.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "name": item.name, "prefix": item.key_prefix, "scopes": item.scopes, "expires_at": item.expires_at, "revoked_at": item.revoked_at, "last_used_at": item.last_used_at} for item in rows]}


@router.delete("/api-keys/{key_id}")
async def admin_revoke_api_key(
    key_id: str,
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    item = await session.get(ApiKey, key_id)
    if item is None:
        raise HTTPException(status_code=404, detail="API key not found")
    item.revoked_at = utcnow()
    await audit(session, action="api_key.admin_revoked", resource_type="api_key", actor_user_id=principal.user_id, resource_id=item.id)
    await session.commit()
    return {"ok": True}


@router.get("/rate-limits")
async def rate_limit_status(
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
):
    settings = get_platform_settings()
    redis_ok = False
    if settings.redis_url:
        try:
            from redis.asyncio import Redis

            redis = Redis.from_url(settings.redis_url)
            redis_ok = bool(await redis.ping())
            await redis.aclose()
        except Exception:  # noqa: BLE001
            redis_ok = False
    return {"default_per_minute": settings.rate_limit_per_minute, "default_burst": settings.rate_limit_burst, "redis_configured": bool(settings.redis_url), "redis_available": redis_ok}


@router.post("/users/{user_id}/credits")
async def admin_adjust_credits(
    user_id: str,
    body: CreditAdjustRequest,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    if await session.get(PlatformUser, user_id) is None:
        raise HTTPException(status_code=404, detail="user not found")
    try:
        ledger = await adjust_credits(
            session,
            user_id=user_id,
            amount=body.amount,
            reason=body.reason,
            reference_type="admin_adjustment",
            reference_id=principal.user_id,
            idempotency_key=body.idempotency_key,
        )
    except ValueError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    await audit(session, action="credits.adjusted", resource_type="user", actor_user_id=principal.user_id, resource_id=user_id, detail={"amount": body.amount, "reason": body.reason})
    await session.commit()
    return {"ok": True, "ledger_id": ledger.id, "balance_after": ledger.balance_after}


@router.get("/runs")
async def admin_list_runs(
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(AgentRun).order_by(AgentRun.created_at.desc()).limit(200))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "status": item.status, "credits": item.actual_credits, "created_at": item.created_at, "error_code": item.error_code} for item in rows]}


@router.get("/audit")
async def admin_audit(
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(AuditLog).order_by(AuditLog.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "actor_user_id": item.actor_user_id, "action": item.action, "resource_type": item.resource_type, "resource_id": item.resource_id, "detail": item.detail, "created_at": item.created_at} for item in rows]}
