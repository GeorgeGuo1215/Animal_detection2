from __future__ import annotations

import argparse
import asyncio

from sqlalchemy import select

from agent_api.app.memory import ensure_memory_subject
from agent_api.app.platform.database import close_platform_database, init_platform_database, platform_session
from agent_api.app.platform.models import Plan, PlatformUser, Subscription, utcnow
from agent_api.app.platform.security import hash_password, normalize_email
from agent_api.app.platform.services import ensure_credit_account, grant_plan, seed_platform_plans, seed_platform_rbac


async def _run(email: str, password: str, display_name: str) -> None:
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
                role="SUPER_ADMIN",
                status="active",
                email_verified_at=utcnow(),
            )
            session.add(user)
            await session.flush()
            await ensure_credit_account(session, user.id)
        else:
            user.role = "SUPER_ADMIN"
            user.status = "active"
            user.password_hash = hash_password(password)
            user.token_version += 1
        await ensure_credit_account(session, user.id)
        plan = await session.get(Plan, "trial")
        existing_sub = await session.scalar(
            select(Subscription).where(Subscription.user_id == user.id, Subscription.status == "active")
        )
        if plan is not None and existing_sub is None:
            await grant_plan(
                session,
                user_id=user.id,
                plan=plan,
                reference_type="bootstrap",
                reference_id=user.id,
            )
        await session.commit()
        print(f"Platform SUPER_ADMIN ready: {user.email} ({user.id})")
    await ensure_memory_subject(user_id=user.id, display_name=user.display_name, source="agent-platform", metadata={"role": user.role})
    await close_platform_database()


def main() -> None:
    parser = argparse.ArgumentParser(description="Create or rotate the PetMind platform super administrator")
    parser.add_argument("--email", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--display-name", default="PetMind Admin")
    args = parser.parse_args()
    asyncio.run(_run(args.email, args.password, args.display_name))


if __name__ == "__main__":
    main()
