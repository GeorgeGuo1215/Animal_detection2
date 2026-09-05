from __future__ import annotations

from datetime import timedelta, timezone

from fastapi import APIRouter, Depends, Header, HTTPException, Request, Response, status
from sqlalchemy import select, update
from sqlalchemy.ext.asyncio import AsyncSession

from ..memory import ensure_memory_subject
from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, require_user_session
from ..platform.models import (
    ApiKey,
    Invitation,
    LegalAcceptance,
    OutboxEvent,
    PasswordResetToken,
    Plan,
    PlatformUser,
    RefreshToken,
    new_id,
    utcnow,
)
from ..platform.legal_documents import PRIVACY_VERSION, TERMS_VERSION
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
    """将 naive datetime 标为 UTC 感知。"""
    return value if value is None or value.tzinfo else value.replace(tzinfo=timezone.utc)


def _set_refresh_cookie(response: Response, token: str) -> None:
    """设置 refresh cookie。"""
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
    """清除 refresh cookie。"""
    settings = get_platform_settings()
    response.delete_cookie(
        "petmind_refresh",
        path="/api/v1/auth",
        httponly=True,
        secure=settings.cookie_secure,
        samesite="strict",
    )


def _invalid_refresh(detail: str) -> HTTPException:
    """抛出 refresh token 无效错误。"""
    response = Response()
    _clear_refresh_cookie(response)
    return HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail=detail,
        headers={"Set-Cookie": response.headers["set-cookie"]},
    )


async def _issue_session(session: AsyncSession, user: PlatformUser, response: Response, *, family_id: str | None = None):
    """签发 access/refresh 会话。"""
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
    """消费一次性管理员邀请，创建已验证账号并签发首个登录会话。

    必须接受当前版本协议与隐私政策；邀请在事务中加锁并校验过期、撤销和重复注册。
    成功后记录法律接受、发放初始套餐、设置轮换 Refresh Cookie，并确保记忆主体存在。
    """
    if (
        not body.accept_terms
        or not body.accept_privacy
        or body.terms_version != TERMS_VERSION
        or body.privacy_version != PRIVACY_VERSION
    ):
        raise HTTPException(status_code=422, detail="current legal documents must be accepted")
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
    client_ip = request.client.host if request.client else None
    user_agent = request.headers.get("user-agent", "")[:500]
    session.add_all([
        LegalAcceptance(user_id=user.id, document_type="terms", version=TERMS_VERSION, ip_address=client_ip, user_agent=user_agent),
        LegalAcceptance(user_id=user.id, document_type="privacy", version=PRIVACY_VERSION, ip_address=client_ip, user_agent=user_agent),
    ])
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
    """校验规范化邮箱、Argon2 密码与账号状态后签发 Access/Refresh 会话。

    成功和失败均写审计日志；失败统一返回无差别凭证错误，避免泄露邮箱是否已注册。
    Refresh Token 仅以哈希落库，原文通过 HttpOnly Cookie 返回。
    """
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
    """校验并轮换 HttpOnly Refresh Token，返回新的短期 Access Token。

    服务器允许极短的并发标签页宽限；超过宽限的已撤销 Token 视为重放，撤销整个
    Token 家族并提升用户 token_version，使现有 Access Token 立即失效。
    """
    raw = request.cookies.get("petmind_refresh", "")
    if not raw:
        raise HTTPException(status_code=401, detail="missing refresh token")
    record = await session.scalar(
        select(RefreshToken).where(RefreshToken.token_hash == hash_secret(raw)).with_for_update()
    )
    if record is None or _aware(record.expires_at) <= utcnow():
        raise _invalid_refresh("invalid refresh token")
    if record.revoked_at is not None:
        settings = get_platform_settings()
        replayed_after = (utcnow() - _aware(record.revoked_at)).total_seconds()
        if record.replaced_by_id and replayed_after <= settings.refresh_reuse_grace_seconds:
            # 第二个浏览器标签可能在看到胜出响应前提交刚轮换的 cookie。
            # 在这段由服务器计时的短暂并发窗口内，不要毁掉整个会话族。
            raise HTTPException(status_code=409, detail="refresh token already rotated")
        user = await session.get(PlatformUser, record.user_id)
        await session.execute(update(RefreshToken).where(
            RefreshToken.family_id == record.family_id,
            RefreshToken.revoked_at.is_(None),
        ).values(revoked_at=utcnow()))
        if user is not None:
            # Access JWT validation compares this version on every request, so
            # confirmed refresh-token reuse invalidates all outstanding access
            # tokens immediately instead of waiting for their 15-minute TTL.
            user.token_version += 1
        await audit(
            session,
            action="auth.refresh.reuse_detected",
            resource_type="refresh_token_family",
            actor_user_id=record.user_id,
            resource_id=record.family_id,
            ip_address=request.client.host if request.client else None,
            detail={"replayed_after_seconds": round(replayed_after, 3)},
        )
        await session.commit()
        raise _invalid_refresh("refresh token reuse detected")
    user = await session.get(PlatformUser, record.user_id)
    if user is None or user.status != "active":
        raise _invalid_refresh("account is unavailable")
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
    """登出并清除 cookie。"""
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
    """为有效账号创建限时一次性重置令牌并写入可替换的 Outbox 邮件事件。

    无论邮箱格式、账号存在性或状态如何都返回相同成功结构，防止账号枚举；仅开发
    配置明确开启时才在响应中附带 development_token。
    """
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
    """消费未使用且未过期的重置令牌，更新 Argon2 密码并撤销旧登录态。

    成功后标记令牌已使用、提升 token_version，并撤销该用户所有未撤销 Refresh Token，
    从而阻止旧 Cookie 与 Access Token 继续访问。
    """
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
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """返回当前用户资料。"""
    user = await session.get(PlatformUser, principal.user_id)
    return user_payload(user)


@router.get("/me/api-keys")
async def list_api_keys(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出当前用户的 API key。"""
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
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """为当前用户创建 API key。"""
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
    await session.flush()
    await audit(session, action="api_key.created", resource_type="api_key", actor_user_id=principal.user_id, resource_id=key.id)
    await session.commit()
    return {"id": key.id, "name": key.name, "key": raw, "prefix": prefix, "scopes": key.scopes}


@router.delete("/me/api-keys/{key_id}", status_code=204)
async def revoke_user_api_key(
    key_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """吊销当前用户的 API key。"""
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
