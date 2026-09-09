from __future__ import annotations

from datetime import timezone

from fastapi import APIRouter, Depends, Header, HTTPException, Request, status
from sqlalchemy import func, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from ..platform.database import get_platform_session
from ..platform.dependencies import Principal, require_user_session
from ..platform.models import Conversation, Message, utcnow
from ..platform.message_feedback import rating_to_good
from ..platform.schemas import ConversationForkRequest, MessageFeedbackRequest
from ..platform.services import audit


router = APIRouter(prefix="/api/v1", tags=["platform-conversation-actions"])


def _feedback_payload(message: Message) -> dict:
    updated = message.feedback_updated_at
    if updated is not None and updated.tzinfo is None:
        updated = updated.replace(tzinfo=timezone.utc)
    return {"message_id": message.id, "rating": message.feedback_rating, "updated_at": updated}


def _conversation_payload(item: Conversation, *, copied_messages: int) -> dict:
    return {
        "id": item.id,
        "title": item.title,
        "status": item.status,
        "created_at": item.created_at,
        "updated_at": item.updated_at,
        "last_active_at": item.last_active_at,
        "source_conversation_id": item.source_conversation_id,
        "forked_from_message_id": item.forked_from_message_id,
        "copied_messages": copied_messages,
    }


async def _owned_conversation(
    session: AsyncSession,
    user_id: str,
    conversation_id: str,
    *,
    lock: bool = False,
) -> Conversation:
    statement = select(Conversation).where(
        Conversation.id == conversation_id,
        Conversation.user_id == user_id,
        Conversation.deleted_at.is_(None),
    )
    if lock:
        statement = statement.with_for_update()
    item = await session.scalar(statement)
    if item is None:
        raise HTTPException(status_code=404, detail="conversation not found")
    return item


async def _owned_message(
    session: AsyncSession,
    user_id: str,
    message_id: str,
    *, lock: bool = False,
) -> Message:
    statement = (
        select(Message)
        .join(Conversation, Conversation.id == Message.conversation_id)
        .where(
            Message.id == message_id,
            Conversation.user_id == user_id,
            Conversation.deleted_at.is_(None),
        )
    )
    if lock:
        statement = statement.with_for_update(of=Message)
    item = await session.scalar(statement.execution_options(populate_existing=True))
    if item is None:
        raise HTTPException(status_code=404, detail="message not found")
    return item


@router.put("/messages/{message_id}/feedback")
async def set_message_feedback(
    message_id: str,
    body: MessageFeedbackRequest,
    request: Request,
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """保存或取消当前用户对一条助手消息的赞踩。"""
    message = await _owned_message(session, principal.user_id, message_id, lock=True)
    if message.role != "assistant" or message.status != "complete":
        raise HTTPException(status_code=400, detail="only completed assistant messages can be rated")
    next_value = rating_to_good(body.rating)
    if message.feedback_is_good is next_value:
        return _feedback_payload(message)
    previous = message.feedback_is_good
    message.feedback_is_good = next_value
    message.feedback_updated_at = utcnow() if body.rating else None
    await audit(
        session,
        action="message.feedback.updated",
        resource_type="message",
        actor_user_id=principal.user_id,
        resource_id=message.id,
        request_id=getattr(request.state, "request_id", None),
        ip_address=request.client.host if request.client else None,
        detail={"rating": body.rating, "previous_is_good": previous, "is_good": next_value},
    )
    await session.commit()
    return _feedback_payload(message)


@router.post("/conversations/{conversation_id}/forks", status_code=status.HTTP_201_CREATED)
async def fork_conversation(
    conversation_id: str,
    body: ConversationForkRequest,
    request: Request,
    idempotency_key: str = Header(min_length=8, max_length=100, alias="Idempotency-Key"),
    principal: Principal = Depends(require_user_session),
    session: AsyncSession = Depends(get_platform_session),
):
    """复制分支点及之前的消息，创建一条归属当前用户的新会话。"""
    stored_key = f"fork:{idempotency_key}"[:100]
    existing = await session.scalar(
        select(Conversation).where(
            Conversation.user_id == principal.user_id,
            Conversation.idempotency_key == stored_key,
        )
    )
    if existing is not None:
        copied = await session.scalar(
            select(func.count()).select_from(Message).where(Message.conversation_id == existing.id)
        )
        return _conversation_payload(existing, copied_messages=int(copied or 0))

    source = await _owned_conversation(session, principal.user_id, conversation_id, lock=True)
    messages = list(
        (
            await session.scalars(
                select(Message)
                .where(Message.conversation_id == source.id, Message.status == "complete")
                .order_by(Message.created_at.asc(), Message.id.asc())
            )
        ).all()
    )
    target_index = next((index for index, item in enumerate(messages) if item.id == body.message_id), None)
    if target_index is None:
        raise HTTPException(status_code=404, detail="message not found")

    title_base = source.title.removesuffix(" · 分支").strip() or "新会诊"
    forked = Conversation(
        user_id=principal.user_id,
        title=f"{title_base[:153]} · 分支",
        status="active",
        last_active_at=utcnow(),
        idempotency_key=stored_key,
        source_conversation_id=source.id,
        forked_from_message_id=body.message_id,
    )
    session.add(forked)
    try:
        await session.flush()
    except IntegrityError:
        await session.rollback()
        replayed = await session.scalar(
            select(Conversation).where(
                Conversation.user_id == principal.user_id,
                Conversation.idempotency_key == stored_key,
            )
        )
        if replayed is None:
            raise
        copied = await session.scalar(
            select(func.count()).select_from(Message).where(Message.conversation_id == replayed.id)
        )
        return _conversation_payload(replayed, copied_messages=int(copied or 0))
    copied_messages = messages[: target_index + 1]
    session.add_all(
        Message(
            conversation_id=forked.id,
            run_id=item.run_id,
            role=item.role,
            content=item.content,
            status="complete",
            created_at=item.created_at,
        )
        for item in copied_messages
    )
    await audit(
        session,
        action="conversation.forked",
        resource_type="conversation",
        actor_user_id=principal.user_id,
        resource_id=forked.id,
        request_id=getattr(request.state, "request_id", None),
        ip_address=request.client.host if request.client else None,
        detail={
            "source_conversation_id": source.id,
            "forked_from_message_id": body.message_id,
            "copied_messages": len(copied_messages),
        },
    )
    await session.commit()
    return _conversation_payload(forked, copied_messages=len(copied_messages))
