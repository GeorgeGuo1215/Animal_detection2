from __future__ import annotations

import gzip
import io
import json
from datetime import timedelta

from fastapi import APIRouter, Depends, Header, HTTPException, Query, Request
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, require_roles
from ..memory import get_memory_client
from ..platform import user_backup
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
    AdminUserDataRestoreRequest,
)
from ..platform.security import hash_secret, normalize_email, secure_token
from ..platform.services import adjust_credits, audit, fulfill_order, user_payload

router = APIRouter(prefix="/api/v1/admin", tags=["platform-admin"])
ADMIN_ROLES = ("SUPPORT_ADMIN", "BILLING_ADMIN", "SUPER_ADMIN")


def _memory_client_or_503():
    """获取记忆客户端；未启用则 503。"""
    client = get_memory_client()
    if client is None:
        raise HTTPException(status_code=503, detail="memory service unavailable")
    return client


@router.get("/users/{user_id}/data-snapshot")
async def export_user_data_snapshot(
    user_id: str,
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """导出用户数据快照。"""
    if await session.get(PlatformUser, user_id) is None:
        raise HTTPException(status_code=404, detail="user not found")
    platform_snapshot = await user_backup.export_records(session, user_id=user_id)
    try:
        memory_snapshot = await _memory_client_or_503().manage_export_snapshot(user_id=user_id)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=503, detail="memory snapshot unavailable") from exc
    payload = {
        "schema_version": 1,
        "user_id": user_id,
        "platform": platform_snapshot,
        "memory": memory_snapshot,
    }
    return {**payload, "checksum": user_backup.checksum(payload)}


@router.post("/users/{user_id}/data-snapshot/restore")
async def restore_user_data_snapshot(
    user_id: str,
    body: AdminUserDataRestoreRequest,
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """仅 SUPER_ADMIN 可用：校验并覆盖恢复指定用户的平台与记忆 JSON 快照。"""
    return await _restore_user_data_snapshot(
        user_id=user_id,
        snapshot=body.snapshot,
        principal=principal,
        session=session,
    )


@router.post("/users/{user_id}/data-snapshot/restore-file")
async def restore_user_data_snapshot_file(
    user_id: str,
    request: Request,
    confirmation: str = Header(default="", alias="X-Restore-Confirmation"),
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """仅 SUPER_ADMIN 可用：从受限大小的 gzip JSON 文件覆盖恢复用户快照。

    请求必须携带显式覆盖确认头和 ``application/gzip``，解压上限为 64 MiB；后续仍会
    校验 schema_version、user_id 与 SHA-256 checksum。
    """
    if confirmation != "OVERWRITE_USER_DATA":
        raise HTTPException(status_code=422, detail="explicit restore confirmation required")
    if request.headers.get("content-type", "").split(";", 1)[0].strip().lower() != "application/gzip":
        raise HTTPException(status_code=415, detail="snapshot restore requires application/gzip")
    compressed = await request.body()
    try:
        with gzip.GzipFile(fileobj=io.BytesIO(compressed), mode="rb") as archive:
            raw = archive.read(64 * 1024 * 1024 + 1)
        if len(raw) > 64 * 1024 * 1024:
            raise HTTPException(status_code=413, detail="uncompressed snapshot exceeds 64 MiB")
        snapshot = json.loads(raw.decode("utf-8"))
    except HTTPException:
        raise
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise HTTPException(status_code=422, detail="invalid gzip snapshot") from exc
    if not isinstance(snapshot, dict):
        raise HTTPException(status_code=422, detail="snapshot root must be an object")
    return await _restore_user_data_snapshot(
        user_id=user_id,
        snapshot=snapshot,
        principal=principal,
        session=session,
    )


async def _restore_user_data_snapshot(
    *,
    user_id: str,
    snapshot: dict,
    principal: Principal,
    session: AsyncSession,
):
    """校验快照归属与完整性后，覆盖平台记录和 Memory 快照并写审计日志。

    平台事务在任一校验或下游恢复失败时回滚；Memory 不可用或恢复异常统一映射为
    503，格式/校验问题映射为 422。
    """
    payload = {key: value for key, value in snapshot.items() if key != "checksum"}
    if snapshot.get("schema_version") != 1 or snapshot.get("user_id") != user_id:
        raise HTTPException(status_code=422, detail="snapshot schema or user does not match")
    if snapshot.get("checksum") != user_backup.checksum(payload):
        raise HTTPException(status_code=422, detail="snapshot checksum mismatch")
    if await session.get(PlatformUser, user_id) is None:
        raise HTTPException(status_code=404, detail="user not found")
    memory = _memory_client_or_503()
    try:
        platform_counts = await user_backup.restore_records(
            session, user_id=user_id, snapshot=dict(snapshot["platform"])
        )
        memory_result = await memory.manage_restore_snapshot(
            user_id=user_id, snapshot=dict(snapshot["memory"])
        )
        await audit(
            session,
            action="user.data_snapshot.restored",
            resource_type="user",
            actor_user_id=principal.user_id,
            resource_id=user_id,
            detail={"platform": platform_counts, "memory": memory_result.get("restored", {})},
        )
        await session.commit()
    except HTTPException:
        await session.rollback()
        raise
    except ValueError as exc:
        await session.rollback()
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except Exception as exc:
        await session.rollback()
        raise HTTPException(status_code=503, detail="snapshot restore failed") from exc
    return {"ok": True, "platform": platform_counts, "memory": memory_result}


@router.get("/overview")
async def overview(
    principal: Principal = Depends(require_roles(*ADMIN_ROLES)),
    session: AsyncSession = Depends(get_platform_session),
):
    """管理后台总览。"""
    users = await session.scalar(select(func.count()).select_from(PlatformUser))
    pending_orders = await session.scalar(select(func.count()).select_from(Order).where(Order.status == "pending_payment"))
    active_runs = await session.scalar(select(func.count()).select_from(AgentRun).where(AgentRun.status.in_(["queued", "retry", "running", "cancel_requested"])))
    return {"users": users, "pending_orders": pending_orders, "active_runs": active_runs}


@router.post("/invitations", status_code=201)
async def create_invitation(
    body: AdminInvitationCreateRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """创建绑定邮箱、角色、初始套餐与有效期的一次性邀请。

    SUPPORT_ADMIN 只能邀请试用兽医；SUPER_ADMIN 可指定允许的角色与套餐。接口支持
    管理员范围内的 Idempotency-Key 复用，明文令牌只写 Outbox，开发模式才回显。
    """
    if principal.role != "SUPER_ADMIN":
        if body.role != "VET" or body.initial_plan_code not in {None, "trial"}:
            raise HTTPException(
                status_code=403,
                detail="support administrators may only invite trial veterinarians",
            )
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
    if body.initial_plan_code is not None:
        plan = await session.get(Plan, body.initial_plan_code)
        if plan is None or not plan.active:
            raise HTTPException(status_code=422, detail="initial plan is unavailable")
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
    await session.flush()
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
    """列出邀请码。"""
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
    """作废邀请码。"""
    item = await session.get(Invitation, invitation_id)
    if item is None:
        raise HTTPException(status_code=404, detail="invitation not found")
    if principal.role != "SUPER_ADMIN" and item.role != "VET":
        raise HTTPException(status_code=403, detail="support administrators may only revoke veterinarian invitations")
    item.revoked_at = utcnow()
    await session.commit()
    return {"ok": True}


@router.get("/users")
async def list_users(
    q: str = Query(default="", max_length=200),
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出用户。"""
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
    """更新用户。"""
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
    """列出套餐。"""
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
    """更新套餐。"""
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
    """列出订单。"""
    rows = list((await session.scalars(select(Order).order_by(Order.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "plan_code": item.plan_code, "status": item.status, "amount_cents": item.amount_cents, "created_at": item.created_at} for item in rows]}


@router.post("/orders/{order_id}/confirm")
async def confirm_order(
    order_id: str,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """确认订单。"""
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
    """更新订单状态。"""
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
    """列出订阅。"""
    rows = list((await session.scalars(select(Subscription).order_by(Subscription.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "plan_code": item.plan_code, "status": item.status, "starts_at": item.starts_at, "expires_at": item.expires_at} for item in rows]}


@router.patch("/subscriptions/{subscription_id}")
async def update_subscription(
    subscription_id: str,
    body: AdminSubscriptionUpdateRequest,
    principal: Principal = Depends(require_roles("BILLING_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """更新订阅。"""
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
    """列出 API key。"""
    rows = list((await session.scalars(select(ApiKey).order_by(ApiKey.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "name": item.name, "prefix": item.key_prefix, "scopes": item.scopes, "expires_at": item.expires_at, "revoked_at": item.revoked_at, "last_used_at": item.last_used_at} for item in rows]}


@router.delete("/api-keys/{key_id}")
async def admin_revoke_api_key(
    key_id: str,
    principal: Principal = Depends(require_roles("SUPPORT_ADMIN", "SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """吊销 API key。"""
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
    """查看限流状态。"""
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
    """调整用户积分。"""
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
    """列出 Agent 运行记录。"""
    rows = list((await session.scalars(select(AgentRun).order_by(AgentRun.created_at.desc()).limit(200))).all())
    return {"items": [{"id": item.id, "user_id": item.user_id, "status": item.status, "credits": item.actual_credits, "created_at": item.created_at, "error_code": item.error_code} for item in rows]}


@router.get("/audit")
async def admin_audit(
    principal: Principal = Depends(require_roles("SUPER_ADMIN")),
    session: AsyncSession = Depends(get_platform_session),
):
    """查询审计日志。"""
    rows = list((await session.scalars(select(AuditLog).order_by(AuditLog.created_at.desc()).limit(300))).all())
    return {"items": [{"id": item.id, "actor_user_id": item.actor_user_id, "action": item.action, "resource_type": item.resource_type, "resource_id": item.resource_id, "detail": item.detail, "created_at": item.created_at} for item in rows]}
