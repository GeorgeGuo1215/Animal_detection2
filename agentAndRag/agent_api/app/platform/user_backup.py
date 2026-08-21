"""带版本号与完整性校验的单用户会话/Run 备份。"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime
from typing import Any

from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.sql.sqltypes import DateTime

from .models import (
    AgentRun,
    Conversation,
    CreditReservation,
    ExpertConsultation,
    Message,
    RunEvent,
    UsageRecord,
)

_MODELS = (
    Conversation,
    Message,
    AgentRun,
    RunEvent,
    ExpertConsultation,
    UsageRecord,
    CreditReservation,
)


def _json_value(value: Any) -> Any:
    """将 datetime 转为 ISO 字符串，其余值原样返回以便 JSON 序列化。"""
    if isinstance(value, datetime):
        return value.isoformat()
    return value


def _row(item) -> dict[str, Any]:
    """将 ORM 行转为列名到 JSON 安全值的字典。"""
    return {
        column.name: _json_value(getattr(item, column.name))
        for column in item.__table__.columns
    }


def checksum(payload: dict[str, Any]) -> str:
    """对备份载荷做稳定 JSON 序列化后计算 SHA-256 校验和。"""
    raw = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


async def export_records(session: AsyncSession, *, user_id: str) -> dict[str, Any]:
    """导出指定用户的会话、消息、Run 及相关子表，并附带 checksum。"""
    conversations = list((await session.scalars(select(Conversation).where(
        Conversation.user_id == user_id
    ))).all())
    conversation_ids = [item.id for item in conversations]
    if conversation_ids:
        messages = list((await session.scalars(select(Message).where(
            Message.conversation_id.in_(conversation_ids)
        ))).all())
        runs = list((await session.scalars(select(AgentRun).where(
            AgentRun.user_id == user_id,
            AgentRun.conversation_id.in_(conversation_ids),
        ))).all())
    else:
        messages, runs = [], []
    run_ids = [item.id for item in runs]
    async def by_runs(model, run_column):
        """按 Run ID 列表查询依赖子表；无 Run 时返回空列表。"""
        if not run_ids:
            return []
        return list((await session.scalars(select(model).where(run_column.in_(run_ids)))).all())
    records = {
        Conversation.__tablename__: [_row(item) for item in conversations],
        Message.__tablename__: [_row(item) for item in messages],
        AgentRun.__tablename__: [_row(item) for item in runs],
        RunEvent.__tablename__: [_row(item) for item in await by_runs(RunEvent, RunEvent.run_id)],
        ExpertConsultation.__tablename__: [_row(item) for item in await by_runs(ExpertConsultation, ExpertConsultation.run_id)],
        UsageRecord.__tablename__: [_row(item) for item in await by_runs(UsageRecord, UsageRecord.run_id)],
        CreditReservation.__tablename__: [_row(item) for item in await by_runs(CreditReservation, CreditReservation.run_id)],
    }
    payload = {"schema_version": 1, "user_id": user_id, "records": records}
    return {**payload, "checksum": checksum(payload)}


def _restore_values(model, raw: dict[str, Any]) -> dict[str, Any]:
    """校验快照列集合并还原 datetime 字段，供 ORM 构造使用。"""
    allowed = {column.name: column for column in model.__table__.columns}
    if set(raw) != set(allowed):
        raise ValueError(f"invalid columns for {model.__tablename__}")
    values = dict(raw)
    for name, column in allowed.items():
        if isinstance(column.type, DateTime) and isinstance(values[name], str):
            values[name] = datetime.fromisoformat(values[name])
    return values


async def restore_records(
    session: AsyncSession, *, user_id: str, snapshot: dict[str, Any]
) -> dict[str, int]:
    """用快照覆盖恢复指定用户的会话与 Run 数据。

    先校验 schema、user_id 与 checksum，再删除该用户现有相关行并按表写入。

    Returns:
        各表恢复的行数。
    """
    payload = {key: value for key, value in snapshot.items() if key != "checksum"}
    if snapshot.get("schema_version") != 1 or snapshot.get("user_id") != user_id:
        raise ValueError("snapshot schema or user does not match")
    if snapshot.get("checksum") != checksum(payload):
        raise ValueError("snapshot checksum mismatch")
    records = snapshot.get("records")
    expected = {model.__tablename__ for model in _MODELS}
    if not isinstance(records, dict) or set(records) != expected:
        raise ValueError("snapshot record set is invalid")

    conversations = list(records[Conversation.__tablename__])
    conversation_ids = {row.get("id") for row in conversations}
    if any(row.get("user_id") != user_id for row in conversations):
        raise ValueError("conversation ownership mismatch")
    messages = list(records[Message.__tablename__])
    if any(row.get("conversation_id") not in conversation_ids for row in messages):
        raise ValueError("message ownership mismatch")
    runs = list(records[AgentRun.__tablename__])
    run_ids = {row.get("id") for row in runs}
    if any(row.get("user_id") != user_id or row.get("conversation_id") not in conversation_ids for row in runs):
        raise ValueError("run ownership mismatch")
    for model in (RunEvent, ExpertConsultation, UsageRecord, CreditReservation):
        if any(row.get("run_id") not in run_ids for row in records[model.__tablename__]):
            raise ValueError(f"{model.__tablename__} ownership mismatch")
        if model in (UsageRecord, CreditReservation) and any(
            row.get("user_id") != user_id for row in records[model.__tablename__]
        ):
            raise ValueError(f"{model.__tablename__} user mismatch")

    existing_conversation_ids = list((await session.scalars(select(Conversation.id).where(
        Conversation.user_id == user_id
    ))).all())
    existing_run_ids = list((await session.scalars(select(AgentRun.id).where(
        AgentRun.user_id == user_id
    ))).all())
    if existing_run_ids:
        for model in (RunEvent, ExpertConsultation, UsageRecord, CreditReservation):
            await session.execute(delete(model).where(model.run_id.in_(existing_run_ids)))
        await session.execute(delete(AgentRun).where(AgentRun.id.in_(existing_run_ids)))
    if existing_conversation_ids:
        await session.execute(delete(Message).where(Message.conversation_id.in_(existing_conversation_ids)))
        await session.execute(delete(Conversation).where(Conversation.id.in_(existing_conversation_ids)))
    await session.flush()

    restored: dict[str, int] = {}
    for model in _MODELS:
        rows = list(records[model.__tablename__])
        session.add_all([model(**_restore_values(model, row)) for row in rows])
        await session.flush()
        restored[model.__tablename__] = len(rows)
    return restored
