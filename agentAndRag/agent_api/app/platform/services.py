from __future__ import annotations

from datetime import timedelta
from typing import Any

from sqlalchemy import select, update
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from .models import (
    AuditLog,
    CreditAccount,
    CreditLedger,
    CreditReservation,
    Order,
    Plan,
    Permission,
    Role,
    RolePermission,
    Subscription,
    utcnow,
)


async def seed_platform_plans(session: AsyncSession) -> None:
    """幂等写入默认套餐（试用、专业月付、专业年付），已存在的 code 不覆盖。"""
    defaults = (
        {
            "code": "trial",
            "name": "兽医试用",
            "description": "一次性试用积分，用于体验专家会诊。",
            "billing_period": "once",
            "price_cents": 0,
            "credit_grant": 100,
            "duration_days": 7,
            "active": True,
            "features": {"chat": True, "api_keys": False},
        },
        {
            "code": "pro_monthly",
            "name": "专业版（月付）",
            "description": "价格和积分由管理员配置后启用。",
            "billing_period": "month",
            "price_cents": 0,
            "credit_grant": 0,
            "duration_days": 30,
            "active": False,
            "features": {"chat": True, "api_keys": True},
        },
        {
            "code": "pro_yearly",
            "name": "专业版（年付）",
            "description": "价格和积分由管理员配置后启用。",
            "billing_period": "year",
            "price_cents": 0,
            "credit_grant": 0,
            "duration_days": 365,
            "active": False,
            "features": {"chat": True, "api_keys": True},
        },
    )
    changed = False
    for item in defaults:
        if await session.get(Plan, item["code"]) is None:
            session.add(Plan(**item))
            changed = True
    if changed:
        await session.commit()


async def seed_platform_rbac(session: AsyncSession) -> None:
    """幂等写入默认权限、角色及角色-权限关联。"""
    permissions = {
        "chat.use": "创建、查看和取消自己的 Agent 任务",
        "profile.manage": "管理个人资料和 API Key",
        "support.manage": "管理邀请、用户与任务",
        "billing.manage": "管理套餐、订单、订阅与积分",
        "audit.read": "查看完整审计日志",
    }
    role_permissions = {
        "VET": {"chat.use", "profile.manage"},
        "SUPPORT_ADMIN": {"chat.use", "profile.manage", "support.manage"},
        "BILLING_ADMIN": {"chat.use", "profile.manage", "billing.manage"},
        "SUPER_ADMIN": set(permissions),
    }
    changed = False
    for code, description in permissions.items():
        if await session.get(Permission, code) is None:
            session.add(Permission(code=code, description=description))
            changed = True
    for code in role_permissions:
        if await session.get(Role, code) is None:
            session.add(Role(code=code, name=code, description=f"PetMind {code} role"))
            changed = True
    await session.flush()
    for role_code, granted in role_permissions.items():
        for permission_code in granted:
            key = {"role_code": role_code, "permission_code": permission_code}
            if await session.get(RolePermission, key) is None:
                session.add(RolePermission(**key))
                changed = True
    if changed:
        await session.commit()


async def audit(
    session: AsyncSession,
    *,
    action: str,
    resource_type: str,
    actor_user_id: str | None = None,
    resource_id: str | None = None,
    request_id: str | None = None,
    ip_address: str | None = None,
    detail: dict[str, Any] | None = None,
) -> None:
    """向当前会话追加一条审计日志（不主动 commit）。"""
    session.add(AuditLog(
        action=action,
        resource_type=resource_type,
        actor_user_id=actor_user_id,
        resource_id=resource_id,
        request_id=request_id,
        ip_address=ip_address,
        detail=detail or {},
    ))


async def ensure_credit_account(session: AsyncSession, user_id: str, *, lock: bool = False) -> CreditAccount:
    """获取用户积分账户，不存在则创建；``lock=True`` 时使用 ``FOR UPDATE``。"""
    from sqlalchemy.dialects.postgresql import insert as pg_insert
    from sqlalchemy.dialects.sqlite import insert as sqlite_insert
    insert = pg_insert if session.get_bind().dialect.name == "postgresql" else sqlite_insert
    await session.execute(insert(CreditAccount).values(user_id=user_id, balance=0, reserved=0)
                          .on_conflict_do_nothing(index_elements=[CreditAccount.user_id]))
    if lock:
        await session.execute(update(CreditAccount).where(CreditAccount.user_id == user_id)
                              .values(balance=CreditAccount.balance))
    account = await session.get(CreditAccount, user_id, populate_existing=True)
    assert account is not None
    return account


async def adjust_credits(
    session: AsyncSession,
    *,
    user_id: str,
    amount: int,
    reason: str,
    reference_type: str,
    reference_id: str | None,
    idempotency_key: str,
) -> CreditLedger:
    """按幂等键调整积分余额并写入账本。

    同一 ``idempotency_key`` 重复调用直接返回已有账本记录。
    调整后余额不得低于已预留额度，也不得为负。
    """
    statement = select(CreditLedger).where(
        CreditLedger.user_id == user_id,
        CreditLedger.idempotency_key == idempotency_key,
    )
    def checked(existing: CreditLedger) -> CreditLedger:
        if (existing.amount, existing.reason, existing.reference_type, existing.reference_id) != (
            int(amount), reason, reference_type, reference_id,
        ):
            raise ValueError("idempotency key conflicts with a different credit adjustment")
        return existing
    existing = await session.scalar(statement)
    if existing is not None:
        return checked(existing)
    account = await ensure_credit_account(session, user_id, lock=True)
    # The first lookup can precede another transaction's commit while waiting for this lock.
    existing = await session.scalar(statement)
    if existing is not None:
        return checked(existing)
    new_balance = account.balance + int(amount)
    if new_balance < account.reserved or new_balance < 0:
        raise ValueError("insufficient credits")
    ledger = CreditLedger(
        user_id=user_id,
        amount=int(amount),
        balance_after=new_balance,
        reason=reason,
        reference_type=reference_type,
        reference_id=reference_id,
        idempotency_key=idempotency_key,
    )
    try:
        async with session.begin_nested():
            account.balance = new_balance
            account.updated_at = utcnow()
            session.add(ledger)
            await session.flush()
    except IntegrityError:
        existing = await session.scalar(statement)
        if existing is None:
            raise
        return checked(existing)
    return ledger


async def reserve_credits(session: AsyncSession, *, user_id: str, run_id: str, amount: int) -> None:
    """为一次 Agent Run 预留积分；同一 ``run_id`` 已有预留则跳过。"""
    existing = await session.get(CreditReservation, run_id)
    if existing is not None:
        return
    account = await ensure_credit_account(session, user_id, lock=True)
    if account.balance - account.reserved < amount:
        raise ValueError("insufficient credits")
    account.reserved += amount
    account.updated_at = utcnow()
    session.add(CreditReservation(run_id=run_id, user_id=user_id, amount=amount))


async def settle_credits(session: AsyncSession, *, run_id: str, actual_amount: int) -> int:
    """结算一次 Run 的预留积分，按实际用量扣减并释放剩余预留。

    本次 Run 可消耗自身预留，以及当前未被其他并发 Run 预留的余额，
    但不得动用其他 Run 已预留的积分。无有效预留时返回 0。
    """
    reservation = await session.scalar(
        select(CreditReservation).where(CreditReservation.run_id == run_id).with_for_update()
    )
    if reservation is None or reservation.status != "active":
        return 0
    account = await ensure_credit_account(session, reservation.user_id, lock=True)
    spendable = max(0, account.balance - max(0, account.reserved - reservation.amount))
    charge = min(max(0, int(actual_amount)), spendable)
    account.reserved = max(0, account.reserved - reservation.amount)
    if charge:
        account.balance -= charge
        session.add(CreditLedger(
            user_id=reservation.user_id,
            amount=-charge,
            balance_after=account.balance,
            reason="agent_run_settled",
            reference_type="agent_run",
            reference_id=run_id,
            idempotency_key=f"run:{run_id}:settle",
        ))
    reservation.status = "settled"
    reservation.settled_at = utcnow()
    account.updated_at = utcnow()
    return charge


async def grant_plan(
    session: AsyncSession,
    *,
    user_id: str,
    plan: Plan,
    reference_type: str,
    reference_id: str,
) -> Subscription:
    """为用户开通套餐订阅，并按套餐配置发放积分。"""
    now = utcnow()
    subscription = Subscription(
        user_id=user_id,
        plan_code=plan.code,
        status="active",
        starts_at=now,
        expires_at=now + timedelta(days=plan.duration_days) if plan.duration_days else None,
        order_id=reference_id if reference_type == "order" else None,
    )
    session.add(subscription)
    await session.flush()
    if plan.credit_grant:
        await adjust_credits(
            session,
            user_id=user_id,
            amount=plan.credit_grant,
            reason="plan_credit_grant",
            reference_type=reference_type,
            reference_id=reference_id,
            idempotency_key=f"plan:{reference_type}:{reference_id}",
        )
    return subscription


async def fulfill_order(session: AsyncSession, order: Order) -> Order:
    """将已支付/待支付订单履约为 fulfilled，并开通对应套餐。

    已履约订单原样返回；状态不允许履约或套餐已删除时抛出 ``ValueError``。
    """
    if order.status == "fulfilled":
        return order
    if order.status not in {"pending_payment", "paid"}:
        raise ValueError(f"order cannot be fulfilled from status {order.status}")
    plan = await session.get(Plan, order.plan_code)
    if plan is None:
        raise ValueError("order plan no longer exists")
    now = utcnow()
    order.status = "fulfilled"
    order.paid_at = order.paid_at or now
    order.fulfilled_at = now
    await grant_plan(
        session,
        user_id=order.user_id,
        plan=plan,
        reference_type="order",
        reference_id=order.id,
    )
    return order


def user_payload(user: Any) -> dict[str, Any]:
    """将平台用户对象序列化为对外安全的公开字段字典。"""
    return {
        "id": user.id,
        "email": user.email,
        "display_name": user.display_name,
        "role": user.role,
        "status": user.status,
        "created_at": user.created_at,
    }
