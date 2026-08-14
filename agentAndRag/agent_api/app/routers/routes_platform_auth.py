from __future__ import annotations

from datetime import timedelta, timezone

from fastapi import APIRouter, Depends, Header, HTTPException, Request, Response, status
from sqlalchemy import select, update
from sqlalchemy.ext.asyncio import AsyncSession

from ..memory import ensure_memory_subject
from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, get_current_principal
from ..platform.models import (
    ApiKey,
    Invitation,
    OutboxEvent,
    PasswordResetToken,
    Plan,
    PlatformUser,
    RefreshToken,
    new_id,
    utcnow,
)
from ..platform.schemas import (
    ApiKeyCreateRequest,
    ForgotPasswordRequest,
    InvitationAcceptRequest,
    LoginRequest,
    ResetPasswordRequest,
)
from ..platform.security import (
    create_access_token,
    create_api_key,
    hash_password,
    hash_secret,
    normalize_email,
    secure_token,
    verify_password,
)
from ..platform.services import audit, grant_plan, user_payload

router = APIRouter(prefix="/api/v1", tags=["platform-auth"])


def _aware(value):
    return value if value is None or value.tzinfo else value.replace(tzinfo=timezone.utc)


def _set_refresh_cookie(response: Response, token: str) -> None:
    settings = get_platform_settings()
    response.set_cookie(
        "petmind_refresh",
        token,
        max_age=settings.refresh_ttl_seconds,
        httponly=True,
        secure=settings.cookie_secure,
        samesite="strict",
        path="/api/v1/auth",
    )


def _clear_refresh_cookie(response: Response) -> None:
    response.delete_cookie("petmind_refresh", path="/api/v1/auth")


async def _issue_session(session: AsyncSession, user: PlatformUser, response: Response, *, family_id: str | None = None):
    settings = get_platform_settings()
    raw_refresh = secure_token(48)
    record = RefreshToken(
        user_id=user.id,
        family_id=family_id or new_id(),
        token_hash=hash_secret(raw_refresh),
        expires_at=utcnow() + timedelta(seconds=settings.refresh_ttl_seconds),
    )
    session.add(record)
    await session.flush()
    access = create_access_token(user_id=user.id, role=user.role, token_version=user.token_version)
    _set_refresh_cookie(response, raw_refresh)
    return {
        "access_token": access,
        "token_type": "bearer",
        "expires_in": settings.access_ttl_seconds,
        "user": user_payload(user),
    }, record


@router.post("/auth/invitations/accept", status_code=201)
async def accept_invitation(
    body: InvitationAcceptRequest,
    response: Response,
    request: Request,
    session: AsyncSession = Depends(get_platform_session),
):
    invitation = await session.scalar(
        select(Invitation).where(Invitation.token_hash == hash_secret(body.token)).with_for_update()
    )
    if (
        invitation is None
        or invitation.accepted_at is not None
        or invitation.revoked_at is not None
        or _aware(invitation.expires_at) <= utcnow()
    ):
        raise HTTPException(status_code=400, detail="invitation is invalid or expired")
    if await session.scalar(select(PlatformUser).where(PlatformUser.email == invitation.email)):
        raise HTTPException(status_code=409, detail="email is already registered")
    try:
        password_hash = hash_password(body.password)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    user = PlatformUser(
        email=invitation.email,
        display_name=body.display_name,
        password_hash=password_hash,
        role=invitation.role,
        status="active",
        email_verified_at=utcnow(),
    )
    session.add(user)
    invitation.accepted_at = utcnow()
    await session.flush()
    if invitation.initial_plan_code:
        plan = await session.get(Plan, invitation.initial_plan_code)
        if plan is not None:
            await grant_plan(
                session,
                user_id=user.id,
                plan=plan,
                reference_type="invitation",
                reference_id=invitation.id,
            )
    payload, _ = await _issue_session(session, user, response)
    await audit(
        session,
        action="auth.invitation.accepted",
        resource_type="user",
        actor_user_id=user.id,
        resource_id=user.id,
        ip_address=request.client.host if request.client else None,
    )
    await session.commit()
    await ensure_memory_subject(
        user_id=user.id,
        display_name=user.display_name,
        source="agent-platform",
        metadata={"role": user.role},
    )
    return payload


@router.post("/auth/login")
async def login(
    body: LoginRequest,
    response: Response,
    request: Request,
    session: AsyncSession = Depends(get_platform_session),
):
    try:
        email = normalize_email(body.email)
    except ValueError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid credentials")
    user = await session.scalar(select(PlatformUser).where(PlatformUser.email == email))
    if user is None or user.status != "active" or not verify_password(user.password_hash, body.password):
        await audit(
            session,
            action="auth.login.failed",
            resource_type="user",
            detail={"email_hash": hash_secret(email)},
            ip_address=request.client.host if request.client else None,
        )
        await session.commit()
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="invalid credentials")
    user.last_login_at = utcnow()
    payload, _ = await _issue_session(session, user, response)
    await audit(session, action="auth.login.success", resource_type="user", actor_user_id=user.id, resource_id=user.id)
    await session.commit()
    return payload


@router.post("/auth/refresh")
async def refresh_session(
    request: Request,
    response: Response,
    session: AsyncSession = Depends(get_platform_session),
):
    raw = request.cookies.get("petmind_refresh", "")
    if not raw:
        raise HTTPException(status_code=401, detail="missing refresh token")
    record = await session.scalar(
        select(RefreshToken).where(RefreshToken.token_hash == hash_secret(raw)).with_for_update()
    )
    if record is None or _aware(record.expires_at) <= utcnow():
        _clear_refresh_cookie(response)
        raise HTTPException(status_code=401, detail="invalid refresh token")
    if record.revoked_at is not None:
        await session.execute(update(RefreshToken).where(
            RefreshToken.family_id == record.family_id,
            RefreshToken.revoked_at.is_(None),
        ).values(revoked_at=utcnow()))
        await session.commit()
        _clear_refresh_cookie(response)
        raise HTTPException(status_code=401, detail="refresh token reuse detected")
    user = await session.get(PlatformUser, record.user_id)
    if user is None or user.status != "active":
        _clear_refresh_cookie(response)
        raise HTTPException(status_code=401, detail="account is unavailable")
    record.revoked_at = utcnow()
    payload, replacement = await _issue_session(session, user, response, family_id=record.family_id)
    record.replaced_by_id = replacement.id
    await session.commit()
    return payload


@router.post("/auth/logout", status_code=204)
async def logout(
    request: Request,
    response: Response,
    session: AsyncSession = Depends(get_platform_session),
):
    raw = request.cookies.get("petmind_refresh", "")
    if raw:
        record = await session.scalar(select(RefreshToken).where(RefreshToken.token_hash == hash_secret(raw)))
        if record is not None and record.revoked_at is None:
            record.revoked_at = utcnow()
            await session.commit()
    _clear_refresh_cookie(response)
    response.status_code = status.HTTP_204_NO_CONTENT
    return response


@router.post("/auth/password/forgot")
async def forgot_password(
    body: ForgotPasswordRequest,
    session: AsyncSession = Depends(get_platform_session),
):
    settings = get_platform_settings()
    try:
        email = normalize_email(body.email)
    except ValueError:
        email = ""
    user = await session.scalar(select(PlatformUser).where(PlatformUser.email == email)) if email else None
    dev_token = None
    if user is not None and user.status == "active":
        raw = secure_token(40)
        session.add(PasswordResetToken(
            user_id=user.id,
            token_hash=hash_secret(raw),
            expires_at=utcnow() + timedelta(seconds=settings.reset_ttl_seconds),
        ))
        session.add(OutboxEvent(topic="auth.password_reset", payload={"email": user.email, "token": raw}))
        dev_token = raw if settings.expose_dev_tokens else None
        await session.commit()
    result = {"ok": True}
    if dev_token:
        result["development_token"] = dev_token
    return result


@router.post("/auth/password/reset")
async def reset_password(
    body: ResetPasswordRequest,
    session: AsyncSession = Depends(get_platform_session),
):
    record = await session.scalar(select(PasswordResetToken).where(
        PasswordResetToken.token_hash == hash_secret(body.token)
    ).with_for_update())
    if record is None or record.used_at is not None or _aware(record.expires_at) <= utcnow():
        raise HTTPException(status_code=400, detail="reset token is invalid or expired")
    user = await session.get(PlatformUser, record.user_id)
    if user is None:
        raise HTTPException(status_code=400, detail="reset token is invalid")
    try:
        user.password_hash = hash_password(body.password)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    user.token_version += 1
    record.used_at = utcnow()
    await session.execute(update(RefreshToken).where(
        RefreshToken.user_id == user.id,
        RefreshToken.revoked_at.is_(None),
    ).values(revoked_at=utcnow()))
    await session.commit()
    return {"ok": True}


@router.get("/me")
async def me(
    principal: Principal = Depends(get_current_principal),
    session: AsyncSession = Depends(get_platform_session),
):
    user = await session.get(PlatformUser, principal.user_id)
    return user_payload(user)


@router.get("/me/api-keys")
async def list_api_keys(
    principal: Principal = Depends(get_current_principal),
    session: AsyncSession = Depends(get_platform_session),
):
    rows = list((await session.scalars(select(ApiKey).where(
        ApiKey.user_id == principal.user_id
    ).order_by(ApiKey.created_at.desc()))).all())
    return {"items": [{
        "id": item.id,
        "name": item.name,
        "prefix": item.key_prefix,
        "scopes": item.scopes,
        "expires_at": item.expires_at,
        "revoked_at": item.revoked_at,
        "last_used_at": item.last_used_at,
        "created_at": item.created_at,
    } for item in rows]}


@router.post("/me/api-keys", status_code=201)
async def create_user_api_key(
    body: ApiKeyCreateRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(get_current_principal),
    session: AsyncSession = Depends(get_platform_session),
):
    idem = idempotency_key.strip()[:100] if idempotency_key else None
    if idem:
        existing = await session.scalar(select(ApiKey).where(ApiKey.user_id == principal.user_id, ApiKey.idempotency_key == idem))
        if existing is not None:
            raise HTTPException(status_code=409, detail="API key creation was already processed; the secret cannot be displayed again")
    raw, prefix, digest = create_api_key()
    key = ApiKey(
        user_id=principal.user_id,
        name=body.name,
        key_prefix=prefix,
        key_hash=digest,
        scopes=body.scopes,
        expires_at=body.expires_at,
        idempotency_key=idem,
    )
    session.add(key)
    await audit(session, action="api_key.created", resource_type="api_key", actor_user_id=principal.user_id, resource_id=key.id)
    await session.commit()
    return {"id": key.id, "name": key.name, "key": raw, "prefix": prefix, "scopes": key.scopes}


@router.delete("/me/api-keys/{key_id}", status_code=204)
async def revoke_user_api_key(
    key_id: str,
    principal: Principal = Depends(get_current_principal),
    session: AsyncSession = Depends(get_platform_session),
):
    key = await session.scalar(select(ApiKey).where(ApiKey.id == key_id, ApiKey.user_id == principal.user_id))
    if key is None:
        raise HTTPException(status_code=404, detail="API key not found")
    if key.revoked_at is None:
        key.revoked_at = utcnow()
        await audit(
            session,
            action="api_key.revoked",
            resource_type="api_key",
            actor_user_id=principal.user_id,
            resource_id=key.id,
        )
    await session.commit()
    return Response(status_code=204)
