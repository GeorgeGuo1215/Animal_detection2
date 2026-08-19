from __future__ import annotations

import argparse
import asyncio
import secrets
from datetime import timedelta

from sqlalchemy import select

from agent_api.app.memory import ensure_memory_subject
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import Plan, PlatformUser, Subscription, utcnow
from agent_api.app.platform.security import hash_password, normalize_email
from agent_api.app.platform.services import adjust_credits, ensure_credit_account, grant_plan, seed_platform_plans, seed_platform_rbac


async def _run(email: str, password: str, display_name: str, role: str, credits: int, subscription_days: int) -> None:
    await init_platform_database()
    async with platform_session() as session:
        await seed_platform_plans(session)
        await seed_platform_rbac(session)
        normalized = normalize_email(email)
        user = await session.scalar(select(PlatformUser).where(PlatformUser.email == normalized))
        if user is None:
            user = PlatformUser(
                email=normalized,
                display_name=display_name,
                password_hash=hash_password(password),
                role=role,
                status="active",
                email_verified_at=utcnow(),
            )
            session.add(user)
            await session.flush()
            await ensure_credit_account(session, user.id)
        else:
            user.role = role
            user.status = "active"
            user.password_hash = hash_password(password)
            user.token_version += 1
        await ensure_credit_account(session, user.id)
        plan = await session.get(Plan, "trial")
        existing_sub = await session.scalar(
            select(Subscription).where(Subscription.user_id == user.id, Subscription.status == "active")
        )
        if plan is not None and existing_sub is None:
            existing_sub = await grant_plan(
                session,
                user_id=user.id,
                plan=plan,
                reference_type="bootstrap",
                reference_id=user.id,
            )
        if existing_sub is not None and subscription_days > 0:
            existing_sub.expires_at = utcnow() + timedelta(days=subscription_days)
        account = await ensure_credit_account(session, user.id)
        if credits > account.balance:
            balance_before = account.balance
            await adjust_credits(
                session,
                user_id=user.id,
                amount=credits - account.balance,
                reason="bootstrap_test_credits",
                reference_type="bootstrap",
                reference_id=user.id,
                idempotency_key=f"bootstrap:credits:{user.id}:{balance_before}:{credits}",
            )
        await session.commit()
        print(
            f"Platform {role} ready: {user.email} ({user.id}); "
            f"credits={max(account.balance, credits)} subscription_days={subscription_days or 'plan-default'}"
        )
    await ensure_memory_subject(user_id=user.id, display_name=user.display_name, source="agent-platform", metadata={"role": user.role})
    await close_platform_database()


def main() -> None:
    parser = argparse.ArgumentParser(description="Create or rotate a PetMind platform account")
    parser.add_argument("--email", required=True)
    parser.add_argument("--password", help="Prefer omitting this option so a password is generated outside shell history")
    parser.add_argument("--display-name", default="PetMind Admin")
    parser.add_argument("--role", choices=["VET", "SUPPORT_ADMIN", "BILLING_ADMIN", "SUPER_ADMIN"], default="SUPER_ADMIN")
    parser.add_argument("--credits", type=int, default=0, help="Ensure at least this many test credits")
    parser.add_argument("--subscription-days", type=int, default=0, help="Extend the active subscription from now")
    args = parser.parse_args()
    password = args.password or f"Pm-{secrets.token_urlsafe(18)}!9a"
    asyncio.run(_run(args.email, password, args.display_name, args.role, max(0, args.credits), max(0, args.subscription_days)))
    if args.password is None:
        print(f"Generated password (shown once): {password}")


if __name__ == "__main__":
    main()
