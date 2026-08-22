from __future__ import annotations

import json
from datetime import timezone

from fastapi import APIRouter, Depends, Header, HTTPException, Query, Request, Response, status
from fastapi.responses import StreamingResponse
from sqlalchemy import and_, or_, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session, platform_session
from ..platform.dependencies import Principal, require_user_session, require_scope
from ..platform.expert_consultations import consultations_by_run, trace_nodes_by_run
from ..platform.models import (
    AgentRun,
    Conversation,
    CreditAccount,
    CreditLedger,
    Message,
    Order,
    PaymentEvent,
    Plan,
    Subscription,
    utcnow,
)
from ..platform.runs import enqueue_run, run_event_stream, wait_for_run
from ..platform.schemas import ConversationCreateRequest, ConversationUpdateRequest, OrderCreateRequest, RunCreateRequest, TestPaymentWebhook
from ..platform.security import verify_webhook_signature
from ..platform.services import audit, reserve_credits, settle_credits

router = APIRouter(prefix="/api/v1", tags=["platform"])


def _conversation_payload(item: Conversation) -> dict:
    """将会话对象转为 API 载荷。"""
    return {
        "id": item.id,
        "title": item.title,
        "status": item.status,
        "created_at": item.created_at,
        "updated_at": item.updated_at,
        "last_active_at": item.last_active_at,
        "source_conversation_id": item.source_conversation_id,
        "forked_from_message_id": item.forked_from_message_id,
    }


def _run_payload(item: AgentRun) -> dict:
    """将运行记录转为 API 载荷。"""
    return {
        "id": item.id,
        "conversation_id": item.conversation_id,
        "status": item.status,
        "delivery": item.delivery,
        "response": item.response if item.status == "completed" else "",
        "error": {"code": item.error_code, "message": item.error_message} if item.error_code else None,
        "reserved_credits": item.reserved_credits,
        "actual_credits": item.actual_credits,
        "created_at": item.created_at,
        "started_at": item.started_at,
        "finished_at": item.finished_at,
    }


async def _owned_conversation(
    session: AsyncSession, user_id: str, conversation_id: str, *, lock: bool = False,
) -> Conversation:
    """校验会话归属当前用户。"""
    statement = select(Conversation).where(
        Conversation.id == conversation_id,
        Conversation.user_id == user_id,
        Conversation.deleted_at.is_(None),
    )
    if lock:
        statement = statement.with_for_update()
    conversation = await session.scalar(statement)
    if conversation is None:
        raise HTTPException(status_code=404, detail="conversation not found")
    return conversation


async def _owned_run(session: AsyncSession, user_id: str, run_id: str) -> AgentRun:
    """校验运行记录归属当前用户。"""
    run = await session.scalar(select(AgentRun).where(AgentRun.id == run_id, AgentRun.user_id == user_id))
    if run is None:
        raise HTTPException(status_code=404, detail="run not found")
    return run


@router.post("/conversations", status_code=201)
async def create_conversation(
    body: ConversationCreateRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """创建对话。"""
    idem = idempotency_key.strip()[:100] if idempotency_key else None
    if idem:
        existing = await session.scalar(select(Conversation).where(Conversation.user_id == principal.user_id, Conversation.idempotency_key == idem))
        if existing is not None:
            return _conversation_payload(existing)
    item = Conversation(user_id=principal.user_id, title=body.title, idempotency_key=idem)
    session.add(item)
    await session.commit()
    return _conversation_payload(item)


@router.get("/conversations")
async def list_conversations(
    status_filter: str | None = Query(default=None, alias="status"),
    limit: int = Query(default=50, ge=1, le=100),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出对话。"""
    statement = select(Conversation).where(
        Conversation.user_id == principal.user_id,
        Conversation.deleted_at.is_(None),
    )
    if status_filter in {"active", "archived"}:
        statement = statement.where(Conversation.status == status_filter)
    rows = list((await session.scalars(statement.order_by(Conversation.last_active_at.desc()).limit(limit))).all())
    return {"items": [_conversation_payload(item) for item in rows]}


@router.get("/conversations/search")
async def search_conversations(
    q: str = Query(min_length=1, max_length=200),
    limit: int = Query(default=30, ge=1, le=100),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """在当前用户未删除的对话标题与消息正文中模糊搜索。

    结果按最近活跃时间排序、按 conversation_id 去重，并返回最多一段命中摘要；
    查询始终带用户归属条件，不能检索其他账号的内容。
    """
    pattern = f"%{q}%"
    statement = select(Conversation, Message).join(
        Message,
        and_(
            Message.conversation_id == Conversation.id,
            Message.status != "superseded",
        ),
        isouter=True,
    ).where(
        Conversation.user_id == principal.user_id,
        Conversation.deleted_at.is_(None),
        or_(Conversation.title.ilike(pattern), Message.content.ilike(pattern)),
    ).order_by(Conversation.last_active_at.desc()).limit(limit)
    rows = (await session.execute(statement)).all()
    seen: set[str] = set()
    items = []
    for conversation, message in rows:
        if conversation.id in seen:
            continue
        seen.add(conversation.id)
        items.append({
            **_conversation_payload(conversation),
            "snippet": (message.content[:240] if message is not None else conversation.title),
        })
    return {"items": items}


@router.get("/conversations/{conversation_id}")
async def get_conversation(
    conversation_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取对话详情。"""
    return _conversation_payload(await _owned_conversation(session, principal.user_id, conversation_id))


@router.patch("/conversations/{conversation_id}")
async def update_conversation(
    conversation_id: str,
    body: ConversationUpdateRequest,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """更新对话。"""
    item = await _owned_conversation(session, principal.user_id, conversation_id)
    if body.title is not None:
        item.title = body.title
    if body.status is not None:
        item.status = body.status
    item.updated_at = utcnow()
    await session.commit()
    return _conversation_payload(item)


@router.delete("/conversations/{conversation_id}", status_code=204)
async def delete_conversation(
    conversation_id: str,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """删除对话。"""
    item = await _owned_conversation(session, principal.user_id, conversation_id)
    # User-facing deletion hides the conversation while retaining the complete
    # platform record for later recovery, audit, and analysis. Consolidated
    # memory remains independently managed by /me/memories.
    item.deleted_at = utcnow()
    item.status = "deleted"
    await audit(
        session,
        action="conversation.deleted",
        resource_type="conversation",
        actor_user_id=principal.user_id,
        resource_id=item.id,
        request_id=getattr(request.state, "request_id", None),
        ip_address=request.client.host if request.client else None,
    )
    await session.commit()
    return Response(status_code=204)


@router.get("/conversations/{conversation_id}/messages")
async def list_messages(
    conversation_id: str,
    limit: int = Query(default=200, ge=1, le=500),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出消息。"""
    await _owned_conversation(session, principal.user_id, conversation_id)
    rows = list((await session.scalars(select(Message).where(
        Message.conversation_id == conversation_id,
        Message.status != "superseded",
    ).order_by(Message.created_at.asc()).limit(limit))).all())
    consultations = await consultations_by_run(session, (item.run_id for item in rows))
    trace_nodes = await trace_nodes_by_run(session, (item.run_id for item in rows))
    return {"items": [{
        "id": item.id,
        "run_id": item.run_id,
        "role": item.role,
        "content": item.content,
        "status": item.status,
        "created_at": item.created_at,
        "expert_consultations": consultations.get(item.run_id or "", []),
        "trace_nodes": trace_nodes.get(item.run_id or "", []),
        "feedback_rating": item.feedback_rating,
        "feedback_updated_at": item.feedback_updated_at,
    } for item in rows]}


@router.post("/conversations/{conversation_id}/runs")
async def create_run(
    conversation_id: str,
    body: RunCreateRequest,
    request: Request,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    last_event_id: int = Header(default=0, alias="Last-Event-ID"),
    principal: Principal = Depends(require_scope("chat:write")),
    session: AsyncSession = Depends(get_platform_session),
):
    """为当前用户的指定对话创建或幂等复用一个持久化 Agent Run。

    接口校验会话归属、chat:write scope、有效订阅与可用积分，先落库用户消息并预占
    积分，再投递 Redis/本地队列。``delivery`` 支持 SSE、限时同步和 202 异步；队列
    投递失败会把 Run 置为失败并释放预占积分。
    """
    # Serialize writes within one conversation so a concurrent append cannot
    # slip past a rewrite boundary. Different conversations remain independent.
    conversation = await _owned_conversation(
        session, principal.user_id, conversation_id, lock=True,
    )
    active_subscription = await session.scalar(select(Subscription).where(
        Subscription.user_id == principal.user_id,
        Subscription.status == "active",
        or_(Subscription.expires_at.is_(None), Subscription.expires_at > utcnow()),
    ).limit(1))
    if active_subscription is None and principal.role != "SUPER_ADMIN":
        raise HTTPException(status_code=402, detail="an active subscription is required")
    idem = (idempotency_key or body.client_message_id).strip()[:100]
    existing = await session.scalar(select(AgentRun).where(
        AgentRun.user_id == principal.user_id,
        AgentRun.idempotency_key == idem,
    ))
    if existing is not None:
        run = existing
    else:
        superseded_messages = 0
        preserved_runs = 0
        if body.rewrite_message_id:
            ordered_messages = list((await session.scalars(
                select(Message).where(Message.conversation_id == conversation_id)
                .order_by(Message.created_at.asc(), Message.id.asc())
            )).all())
            target_index = next(
                (index for index, item in enumerate(ordered_messages)
                 if item.id == body.rewrite_message_id),
                -1,
            )
            if target_index < 0:
                raise HTTPException(status_code=404, detail="message not found")
            user_message = ordered_messages[target_index]
            if user_message.role != "user" or user_message.status != "complete":
                raise HTTPException(status_code=400, detail="only complete user messages can be rewritten")
            later_messages = ordered_messages[target_index + 1:]
            affected_messages = [user_message, *later_messages]
            affected_ids = [item.id for item in affected_messages]
            affected_runs = list((await session.scalars(select(AgentRun).where(
                AgentRun.conversation_id == conversation_id,
                or_(
                    AgentRun.user_message_id.in_(affected_ids),
                    AgentRun.assistant_message_id.in_(affected_ids),
                ),
            ))).all())
            if any(item.status in {"queued", "running", "retry"} for item in affected_runs):
                raise HTTPException(status_code=409, detail="conversation has an active run")
            first_user_id = next(
                (item.id for item in ordered_messages
                 if item.role == "user" and item.status == "complete"),
                None,
            )
            # Keep prior runs, usage and evidence immutable for audit/billing. The
            # visible timeline gets a replacement user message while the edited
            # branch is excluded from history, search, forking and future context.
            preserved_runs = len(affected_runs)
            for item in affected_messages:
                item.status = "superseded"
            superseded_messages = len(affected_messages)
            replacement = Message(
                conversation_id=conversation_id,
                client_message_id=body.client_message_id,
                role="user",
                content=body.message,
                status="complete",
                created_at=user_message.created_at,
            )
            session.add(replacement)
            await session.flush()
            if first_user_id == user_message.id:
                conversation.title = body.message[:60]
            await audit(
                session,
                action="message.rewritten",
                resource_type="message",
                actor_user_id=principal.user_id,
                resource_id=user_message.id,
                request_id=getattr(request.state, "request_id", None),
                ip_address=request.client.host if request.client else None,
                detail={
                    "conversation_id": conversation_id,
                    "superseded_messages": superseded_messages,
                    "preserved_runs": preserved_runs,
                    "replacement_message_id": replacement.id,
                },
            )
            user_message = replacement
        else:
            user_message = Message(
                conversation_id=conversation_id,
                client_message_id=body.client_message_id,
                role="user",
                content=body.message,
                status="complete",
            )
            session.add(user_message)
            await session.flush()
        reserved = max(10, 10 + body.max_tokens // 250)
        run = AgentRun(
            user_id=principal.user_id,
            conversation_id=conversation_id,
            user_message_id=user_message.id,
            idempotency_key=idem,
            delivery=body.delivery,
            status="queued",
            query=body.message,
            parameters={"temperature": body.temperature, "max_tokens": body.max_tokens, "user_role": body.user_role},
            reserved_credits=reserved,
        )
        session.add(run)
        await session.flush()
        try:
            await reserve_credits(session, user_id=principal.user_id, run_id=run.id, amount=reserved)
        except ValueError as exc:
            await session.rollback()
            raise HTTPException(status_code=402, detail="insufficient credits") from exc
        conversation.last_active_at = utcnow()
        await session.commit()
        try:
            await enqueue_run(run.id)
        except Exception as exc:
            async with platform_session() as recovery_session:
                failed_run = await recovery_session.get(AgentRun, run.id)
                if failed_run is not None:
                    failed_run.status = "failed"
                    failed_run.error_code = "queue_unavailable"
                    failed_run.error_message = "Agent queue is unavailable"
                    failed_run.finished_at = utcnow()
                    await settle_credits(recovery_session, run_id=run.id, actual_amount=0)
                    await recovery_session.commit()
            raise HTTPException(status_code=503, detail="Agent queue is unavailable") from exc

    if body.delivery == "sse":
        return StreamingResponse(
            run_event_stream(run.id, after_sequence=last_event_id),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no", "X-PetMind-Run-Id": run.id},
        )
    if body.delivery == "sync":
        completed = await wait_for_run(run.id, get_platform_settings().sync_timeout_seconds)
        if completed is None:
            return Response(
                content=json.dumps({"error": {"code": "run_timeout", "message": "Run continues asynchronously", "run_id": run.id}}),
                status_code=504,
                media_type="application/json",
                headers={"X-PetMind-Run-Id": run.id},
            )
        return _run_payload(completed)
    return Response(
        content=json.dumps({"run": _run_payload(run), "status_url": f"/api/v1/runs/{run.id}"}, default=str),
        status_code=status.HTTP_202_ACCEPTED,
        media_type="application/json",
    )


@router.get("/runs/{run_id}")
async def get_run(
    run_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取运行详情。"""
    return _run_payload(await _owned_run(session, principal.user_id, run_id))


@router.get("/runs/{run_id}/experts")
async def get_run_experts(
    run_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取运行中的专家意见。"""
    await _owned_run(session, principal.user_id, run_id)
    consultations = await consultations_by_run(session, [run_id])
    return {"items": consultations.get(run_id, [])}


@router.delete("/runs/{run_id}", status_code=202)
async def cancel_run(
    run_id: str,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """取消运行。"""
    run = await _owned_run(session, principal.user_id, run_id)
    if run.status in {"queued", "retry"}:
        run.cancel_requested = True
        run.status = "cancelled"
        run.finished_at = utcnow()
        await settle_credits(session, run_id=run.id, actual_amount=0)
        await session.commit()
    elif run.status not in {"completed", "failed", "cancelled"}:
        run.cancel_requested = True
        run.status = "cancel_requested"
        await session.commit()
    return _run_payload(run)


@router.get("/runs/{run_id}/events")
async def get_run_events(
    run_id: str,
    last_event_id: int = Header(default=0, alias="Last-Event-ID"),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取运行 SSE 事件。"""
    await _owned_run(session, principal.user_id, run_id)
    return StreamingResponse(
        run_event_stream(run_id, after_sequence=last_event_id),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/plans")
async def list_plans(session: AsyncSession = Depends(get_platform_session)):
    """列出公开套餐。"""
    rows = list((await session.scalars(select(Plan).where(Plan.active.is_(True)).order_by(Plan.price_cents.asc()))).all())
    return {"items": [{
        "code": item.code,
        "name": item.name,
        "description": item.description,
        "billing_period": item.billing_period,
        "price_cents": item.price_cents,
        "currency": item.currency,
        "credit_grant": item.credit_grant,
        "duration_days": item.duration_days,
        "features": item.features,
    } for item in rows]}


@router.post("/orders", status_code=201)
async def create_order(
    body: OrderCreateRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """创建订单。"""
    idem = idempotency_key.strip()[:100] if idempotency_key else None
    if idem:
        existing = await session.scalar(select(Order).where(Order.user_id == principal.user_id, Order.idempotency_key == idem))
        if existing is not None:
            return {"id": existing.id, "plan_code": existing.plan_code, "status": existing.status, "amount_cents": existing.amount_cents, "currency": existing.currency}
    plan = await session.get(Plan, body.plan_code)
    if plan is None or not plan.active or plan.price_cents <= 0:
        raise HTTPException(status_code=400, detail="plan is not available for purchase")
    order = Order(
        user_id=principal.user_id,
        plan_code=plan.code,
        amount_cents=plan.price_cents,
        currency=plan.currency,
        idempotency_key=idem,
    )
    session.add(order)
    await session.commit()
    return {"id": order.id, "plan_code": order.plan_code, "status": order.status, "amount_cents": order.amount_cents, "currency": order.currency}


@router.get("/orders")
async def list_orders(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """列出订单。"""
    rows = list((await session.scalars(select(Order).where(Order.user_id == principal.user_id).order_by(Order.created_at.desc()))).all())
    return {"items": [{"id": item.id, "plan_code": item.plan_code, "status": item.status, "amount_cents": item.amount_cents, "currency": item.currency, "created_at": item.created_at} for item in rows]}


@router.get("/subscription")
async def get_subscription(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取当前订阅。"""
    item = await session.scalar(select(Subscription).where(
        Subscription.user_id == principal.user_id,
        Subscription.status == "active",
    ).order_by(Subscription.expires_at.desc()).limit(1))
    return {"subscription": None if item is None else {"id": item.id, "plan_code": item.plan_code, "status": item.status, "starts_at": item.starts_at, "expires_at": item.expires_at}}


@router.get("/credits")
async def get_credits(
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """获取积分余额。"""
    account = await session.get(CreditAccount, principal.user_id)
    ledger = list((await session.scalars(select(CreditLedger).where(
        CreditLedger.user_id == principal.user_id
    ).order_by(CreditLedger.created_at.desc()).limit(50))).all())
    return {
        "balance": account.balance if account else 0,
        "reserved": account.reserved if account else 0,
        "ledger": [{"id": item.id, "amount": item.amount, "balance_after": item.balance_after, "reason": item.reason, "created_at": item.created_at} for item in ledger],
    }


@router.post("/payments/test-webhook")
async def test_payment_webhook(
    request: Request,
    session: AsyncSession = Depends(get_platform_session),
):
    """测试支付 webhook。"""
    raw = await request.body()
    timestamp = request.headers.get("x-petmind-timestamp", "")
    signature = request.headers.get("x-petmind-signature", "")
    if not verify_webhook_signature(body=raw, timestamp=timestamp, signature=signature):
        raise HTTPException(status_code=401, detail="invalid webhook signature")
    try:
        body = TestPaymentWebhook.model_validate_json(raw)
    except Exception as exc:
        raise HTTPException(status_code=422, detail="invalid webhook body") from exc
    existing = await session.scalar(select(PaymentEvent).where(PaymentEvent.external_event_id == body.event_id))
    if existing is not None:
        return {"ok": True, "duplicate": True}
    order = await session.get(Order, body.order_id)
    if order is None:
        raise HTTPException(status_code=404, detail="order not found")
    session.add(PaymentEvent(external_event_id=body.event_id, order_id=order.id, event_type=body.event_type, payload=body.model_dump()))
    if body.event_type == "payment.succeeded":
        from ..platform.services import fulfill_order

        await fulfill_order(session, order)
    elif order.status == "fulfilled":
        order.status = "refunded"
    await session.commit()
    return {"ok": True, "duplicate": False, "order_status": order.status}
