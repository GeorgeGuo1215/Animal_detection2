from __future__ import annotations

from datetime import timezone

from fastapi import APIRouter, Depends, Header, HTTPException, Query, Request, Response
from sqlalchemy import func, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, require_user_session
from ..platform.legal_documents import PRIVACY_VERSION, TERMS_VERSION, current_document
from ..platform.models import (
    ActivationCode,
    ActivationRedemption,
    CommonPhrase,
    LegalAcceptance,
    Plan,
    PlatformUser,
    UserFeedback,
    UserPreference,
    new_id,
    utcnow,
)
from ..platform.schemas import (
    ActivationCodeRedeemRequest,
    CommonPhraseCreateRequest,
    CommonPhraseUpdateRequest,
    FeedbackCreateRequest,
    LegalAcceptRequest,
    MemoryClearRequest,
    PreferenceUpdateRequest,
    ProfileUpdateRequest,
)
from ..platform.security import hash_secret
from ..memory import get_memory_client
from ..platform.services import adjust_credits, audit, grant_plan

router = APIRouter(prefix="/api/v1", tags=["platform-settings"])


def _aware(value):
    """将 naive datetime 标为 UTC 感知。"""
    return value if value is None or value.tzinfo else value.replace(tzinfo=timezone.utc)


def _preferences_payload(item: UserPreference) -> dict:
    """将偏好设置转为 API 载荷。"""
    return {
        "theme": item.theme,
        "locale": item.locale,
        "default_expand_experts": item.default_expand_experts,
        "memory_recall_enabled": item.memory_recall_enabled,
        "memory_write_enabled": item.memory_write_enabled,
        "updated_at": item.updated_at,
    }


async def _preferences(session: AsyncSession, user_id: str) -> UserPreference:
    """读取或创建用户偏好。"""
    item = await session.get(UserPreference, user_id)
    if item is None:
        item = UserPreference(user_id=user_id)
        session.add(item)
        await session.flush()
    return item


@router.patch("/me")
async def update_profile(
    body: ProfileUpdateRequest,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """更新用户资料。"""
    user = await session.get(PlatformUser, principal.user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="user not found")
    user.display_name = body.display_name
    await session.commit()
    return {"id": user.id, "email": user.email, "display_name": user.display_name, "role": user.role, "status": user.status, "created_at": user.created_at}


@router.get("/me/preferences")
async def get_preferences(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取偏好设置。"""
    item = await _preferences(session, principal.user_id)
    await session.commit()
    return _preferences_payload(item)


@router.patch("/me/preferences")
async def update_preferences(
    body: PreferenceUpdateRequest,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """更新偏好设置。"""
    item = await _preferences(session, principal.user_id)
    for field, value in body.model_dump(exclude_none=True).items():
        setattr(item, field, value)
    item.updated_at = utcnow()
    await session.commit()
    return _preferences_payload(item)


def _phrase_payload(item: CommonPhrase) -> dict:
    """将常用语转为 API 载荷。"""
    return {"id": item.id, "title": item.title, "content": item.content, "sort_order": item.sort_order, "created_at": item.created_at, "updated_at": item.updated_at}


@router.get("/me/common-phrases")
async def list_common_phrases(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出常用语。"""
    rows = list((await session.scalars(select(CommonPhrase).where(CommonPhrase.user_id == principal.user_id).order_by(CommonPhrase.sort_order, CommonPhrase.created_at))).all())
    return {"items": [_phrase_payload(item) for item in rows]}


@router.post("/me/common-phrases", status_code=201)
async def create_common_phrase(
    body: CommonPhraseCreateRequest,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """创建常用语。"""
    count = await session.scalar(select(func.count()).select_from(CommonPhrase).where(CommonPhrase.user_id == principal.user_id))
    if int(count or 0) >= 50:
        raise HTTPException(status_code=409, detail="common phrase limit reached")
    item = CommonPhrase(user_id=principal.user_id, **body.model_dump())
    session.add(item)
    await session.commit()
    return _phrase_payload(item)


async def _owned_phrase(session: AsyncSession, user_id: str, phrase_id: str) -> CommonPhrase:
    """校验常用语归属。"""
    item = await session.scalar(select(CommonPhrase).where(CommonPhrase.id == phrase_id, CommonPhrase.user_id == user_id))
    if item is None:
        raise HTTPException(status_code=404, detail="common phrase not found")
    return item


@router.patch("/me/common-phrases/{phrase_id}")
async def update_common_phrase(
    phrase_id: str,
    body: CommonPhraseUpdateRequest,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """更新常用语。"""
    item = await _owned_phrase(session, principal.user_id, phrase_id)
    for field, value in body.model_dump(exclude_none=True).items():
        setattr(item, field, value)
    item.updated_at = utcnow()
    await session.commit()
    return _phrase_payload(item)


@router.delete("/me/common-phrases/{phrase_id}", status_code=204)
async def delete_common_phrase(
    phrase_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """删除常用语。"""
    await session.delete(await _owned_phrase(session, principal.user_id, phrase_id))
    await session.commit()
    return Response(status_code=204)


@router.post("/feedback", status_code=201)
async def create_feedback(
    body: FeedbackCreateRequest,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """提交用户反馈。"""
    user = await session.get(PlatformUser, principal.user_id)
    if user is None:
        raise HTTPException(status_code=404, detail="user not found")
    item = UserFeedback(
        user_id=user.id,
        email_snapshot=user.email,
        display_name_snapshot=user.display_name,
        **body.model_dump(),
    )
    session.add(item)
    await audit(session, action="feedback.submitted", resource_type="feedback", actor_user_id=user.id, resource_id=item.id, ip_address=request.client.host if request.client else None)
    await session.commit()
    return {"id": item.id, "status": item.status, "created_at": item.created_at}


def _memory_client_or_503():
    """获取记忆客户端；未启用则 503。"""
    client = get_memory_client()
    if client is None:
        raise HTTPException(status_code=503, detail="memory service unavailable")
    return client


@router.get("/me/memories")
async def list_memories(
    limit: int = Query(default=100, ge=1, le=200),
    principal: Principal = Depends(require_user_session),
):
    """列出用户记忆。"""
    try:
        return await _memory_client_or_503().manage_list(user_id=principal.user_id, limit=limit)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=503, detail="memory service unavailable") from exc


@router.delete("/me/memories/{item_id:path}")
async def delete_memory(
    item_id: str,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """删除一条记忆。"""
    try:
        result = await _memory_client_or_503().manage_delete(user_id=principal.user_id, item_id=item_id)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=503, detail="memory service unavailable") from exc
    await audit(session, action="memory.item.deleted", resource_type="memory", actor_user_id=principal.user_id, resource_id=hash_secret(item_id)[:16], ip_address=request.client.host if request.client else None)
    await session.commit()
    return result


@router.delete("/me/memories")
async def clear_memories(
    body: MemoryClearRequest,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """按范围清空记忆。"""
    try:
        result = await _memory_client_or_503().manage_clear(
            user_id=principal.user_id, scope=body.scope
        )
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=503, detail="memory service unavailable") from exc
    await audit(session, action=f"memory.{body.scope}.cleared", resource_type="memory", actor_user_id=principal.user_id, resource_id=body.scope, ip_address=request.client.host if request.client else None)
    await session.commit()
    return result


@router.get("/legal/{document_type}")
async def get_legal_document(document_type: str):
    """获取法律文档。"""
    try:
        return current_document(document_type)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="legal document not found") from exc


async def _record_legal_acceptance(session: AsyncSession, *, user_id: str, body: LegalAcceptRequest, request: Request) -> None:
    """记录法律文档接受状态。"""
    if not body.accept_terms or not body.accept_privacy or body.terms_version != TERMS_VERSION or body.privacy_version != PRIVACY_VERSION:
        raise HTTPException(status_code=422, detail="current legal documents must be accepted")
    for kind, version in (("terms", body.terms_version), ("privacy", body.privacy_version)):
        existing = await session.scalar(select(LegalAcceptance).where(LegalAcceptance.user_id == user_id, LegalAcceptance.document_type == kind, LegalAcceptance.version == version))
        if existing is None:
            session.add(LegalAcceptance(user_id=user_id, document_type=kind, version=version, ip_address=request.client.host if request.client else None, user_agent=request.headers.get("user-agent", "")[:500]))


@router.post("/me/legal-acceptances", status_code=204)
async def accept_current_legal_documents(
    body: LegalAcceptRequest,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """接受当前版本法律文档。"""
    await _record_legal_acceptance(session, user_id=principal.user_id, body=body, request=request)
    await session.commit()
    return Response(status_code=204)


@router.post("/activation-codes/redeem")
async def redeem_activation_code(
    body: ActivationCodeRedeemRequest,
    request: Request,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """在行锁保护下为当前用户一次性兑换有效激活码。

    校验启用时间、有效期、总兑换上限和用户重复兑换；随后原子写入兑换记录、开通套餐、
    发放额外积分并更新计数。所有无效或冲突情况返回同一错误，避免被用于枚举撞库。
    """
    normalized = body.code.replace("-", "").upper()
    generic = HTTPException(status_code=400, detail="activation code is invalid or unavailable")
    code = await session.scalar(select(ActivationCode).where(ActivationCode.code_hash == hash_secret(normalized)).with_for_update())
    now = utcnow()
    if code is None or code.status != "active" or (code.starts_at and _aware(code.starts_at) > now) or (code.expires_at and _aware(code.expires_at) <= now) or code.redemption_count >= code.max_redemptions:
        raise generic
    prior = await session.scalar(select(ActivationRedemption).where(ActivationRedemption.code_id == code.id, ActivationRedemption.user_id == principal.user_id))
    if prior is not None:
        raise generic
    plan = await session.get(Plan, code.plan_code) if code.plan_code else None
    if code.plan_code and plan is None:
        raise generic
    redemption = ActivationRedemption(id=new_id(), code_id=code.id, user_id=principal.user_id, plan_code=code.plan_code, credits_granted=(plan.credit_grant if plan else 0) + code.extra_credits, idempotency_key=(idempotency_key or "")[:100] or None)
    session.add(redemption)
    try:
        await session.flush()
        if plan is not None:
            await grant_plan(session, user_id=principal.user_id, plan=plan, reference_type="activation", reference_id=redemption.id)
        if code.extra_credits:
            await adjust_credits(session, user_id=principal.user_id, amount=code.extra_credits, reason="activation_extra_credit", reference_type="activation", reference_id=redemption.id, idempotency_key=f"activation:{redemption.id}:extra")
        code.redemption_count += 1
        await audit(session, action="activation.redeemed", resource_type="activation_code", actor_user_id=principal.user_id, resource_id=code.id, ip_address=request.client.host if request.client else None, detail={"prefix": code.code_prefix})
        await session.commit()
    except IntegrityError as exc:
        await session.rollback()
        raise generic from exc
    return {"ok": True, "plan_code": code.plan_code, "credits_granted": redemption.credits_granted}
