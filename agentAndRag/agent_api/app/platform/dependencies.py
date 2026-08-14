from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

import jwt
from fastapi import Depends, HTTPException, Request, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .database import get_platform_session, platform_session
from .models import ApiKey, PlatformUser, utcnow
from .security import decode_access_token, hash_secret


@dataclass(frozen=True)
class Principal:
    user_id: str
    email: str
    role: str
    scopes: frozenset[str]
    auth_kind: str


def _aware(value: datetime | None) -> datetime | None:
    if value is None:
        return None
    return value if value.tzinfo else value.replace(tzinfo=timezone.utc)


async def authenticate_platform_api_key(raw_key: str) -> Principal | None:
    if not raw_key.startswith("pm_live_"):
        return None
    digest = hash_secret(raw_key)
    async with platform_session() as session:
        key = await session.scalar(select(ApiKey).where(ApiKey.key_hash == digest))
        if key is None or key.revoked_at is not None:
            return None
        expires_at = _aware(key.expires_at)
        if expires_at is not None and expires_at <= utcnow():
            return None
        user = await session.get(PlatformUser, key.user_id)
        if user is None or user.status != "active":
            return None
        key.last_used_at = utcnow()
        await session.commit()
        return Principal(
            user_id=user.id,
            email=user.email,
            role=user.role,
            scopes=frozenset(str(scope) for scope in (key.scopes or [])),
            auth_kind="api_key",
        )


async def get_current_principal(
    request: Request,
    session: AsyncSession = Depends(get_platform_session),
) -> Principal:
    cached = getattr(request.state, "platform_principal", None)
    if isinstance(cached, Principal):
        return cached

    authorization = request.headers.get("authorization", "")
    if not authorization.lower().startswith("bearer "):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="missing bearer token")
    token = authorization[7:].strip()
    if token.startswith("pm_live_"):
        principal = await authenticate_platform_api_key(token)
        if principal is None:
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid API key")
        request.state.platform_principal = principal
        request.state.platform_user_id = principal.user_id
        return principal

    try:
        payload = decode_access_token(token)
    except jwt.PyJWTError as exc:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid or expired access token") from exc
    user = await session.get(PlatformUser, str(payload["sub"]))
    if user is None or user.status != "active" or int(payload.get("ver", 0)) != user.token_version:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="account is unavailable")
    principal = Principal(
        user_id=user.id,
        email=user.email,
        role=user.role,
        scopes=frozenset({"chat:write", "models:read", "runs:read", "profile:write"}),
        auth_kind="jwt",
    )
    request.state.platform_principal = principal
    request.state.platform_user_id = principal.user_id
    return principal


def require_roles(*roles: str) -> Callable:
    allowed = set(roles)

    async def dependency(principal: Principal = Depends(get_current_principal)) -> Principal:
        if principal.role not in allowed:
            raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="insufficient role")
        return principal

    return dependency


def require_scope(scope: str) -> Callable:
    async def dependency(principal: Principal = Depends(get_current_principal)) -> Principal:
        if scope not in principal.scopes and principal.role != "SUPER_ADMIN":
            raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="missing API scope")
        return principal

    return dependency
