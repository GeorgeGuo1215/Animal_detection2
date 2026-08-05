"""短期记忆：最近若干轮原始对话，先进先出。

上游用一个内存 deque 加 JSON 文件，多进程下必然互相覆盖。这里换成表加索引，
出队用一条 DELETE ... RETURNING 完成，多个 worker 同时处理同一用户时不会重复取到
同一批对话。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional

import psycopg

from ..db import new_id

_INSERT = """
INSERT INTO memory_short_term
    ("id", "userId", "petId", "sessionId", "turnId", "userInput", "agentResponse", "createdAt")
VALUES (%(id)s, %(user_id)s, %(pet_id)s, %(session_id)s, %(turn_id)s, %(user_input)s,
        %(agent_response)s, COALESCE(%(created_at)s, CURRENT_TIMESTAMP))
RETURNING "id"
"""

_COUNT = 'SELECT count(*) AS n FROM memory_short_term WHERE "userId" = %s'

# 最近 N 条对话，用于拼当前上下文。倒序取再翻转，避免全表排序。
_RECENT = """
SELECT "id", "petId", "sessionId", "userInput", "agentResponse", "createdAt"
FROM memory_short_term
WHERE "userId" = %(user_id)s
ORDER BY "createdAt" DESC, "id" DESC
LIMIT %(limit)s
"""

# 保留最新 keep 条，其余全部取出并删除。
#
# OFFSET 正好表达"超出容量的部分"，配合 DELETE ... RETURNING 让读取与删除处于
# 同一条语句，不存在两个 worker 都读到同一批再各自提升一次的窗口。
_DRAIN = """
DELETE FROM memory_short_term
WHERE "id" IN (
    SELECT "id" FROM memory_short_term
    WHERE "userId" = %(user_id)s
    ORDER BY "createdAt" DESC, "id" DESC
    OFFSET %(keep)s
)
RETURNING "id", "petId", "sessionId", "userInput", "agentResponse", "createdAt"
"""


def append(
    conn: psycopg.Connection,
    *,
    user_id: str,
    user_input: str,
    agent_response: str,
    pet_id: Optional[str] = None,
    session_id: Optional[str] = None,
    turn_id: Optional[str] = None,
    created_at: Optional[datetime] = None,
) -> str:
    message_id, _ = append_once(
        conn,
        user_id=user_id,
        user_input=user_input,
        agent_response=agent_response,
        pet_id=pet_id,
        session_id=session_id,
        turn_id=turn_id,
        created_at=created_at,
    )
    return message_id


def append_once(
    conn: psycopg.Connection,
    *,
    user_id: str,
    user_input: str,
    agent_response: str,
    pet_id: Optional[str] = None,
    session_id: Optional[str] = None,
    turn_id: Optional[str] = None,
    created_at: Optional[datetime] = None,
) -> tuple[str, bool]:
    """Append a turn and return ``(message_id, created)``.

    The receipt survives short-to-mid promotion, so an old request retry is
    idempotent even after its short-term row moved to ``memory_pages``.
    """
    message_id = new_id()
    normalized_turn_id = (turn_id or "").strip() or None
    if normalized_turn_id:
        receipt = conn.execute(
            """
            INSERT INTO memory_ingest_receipts ("userId", "turnId", "messageId", "createdAt")
            VALUES (%s, %s, %s, COALESCE(%s, CURRENT_TIMESTAMP))
            ON CONFLICT ("userId", "turnId") DO NOTHING
            RETURNING "messageId"
            """,
            (user_id, normalized_turn_id, message_id, created_at),
        ).fetchone()
        if receipt is None:
            existing = conn.execute(
                'SELECT "messageId" FROM memory_ingest_receipts '
                'WHERE "userId" = %s AND "turnId" = %s',
                (user_id, normalized_turn_id),
            ).fetchone()
            return str(existing["messageId"]), False

    row = conn.execute(
        _INSERT,
        {
            "id": message_id,
            "user_id": user_id,
            "pet_id": pet_id,
            "session_id": session_id,
            "turn_id": normalized_turn_id,
            "user_input": user_input,
            "agent_response": agent_response,
            "created_at": created_at,
        },
    ).fetchone()
    return row["id"], True


def count(conn: psycopg.Connection, user_id: str) -> int:
    row = conn.execute(_COUNT, (user_id,)).fetchone()
    return int(row["n"]) if row else 0


def recent(conn: psycopg.Connection, user_id: str, limit: int) -> List[Dict[str, Any]]:
    """最近 limit 条对话，按时间正序返回（老的在前，便于直接拼进 prompt）。"""
    rows = conn.execute(_RECENT, {"user_id": user_id, "limit": limit}).fetchall()
    return list(reversed(rows))


def drain_overflow(
    conn: psycopg.Connection, user_id: str, keep: int
) -> List[Dict[str, Any]]:
    """取出并删除超出容量的最旧对话，按时间正序返回。

    keep 即短期队列容量。返回空列表表示还没溢出，调用方无需做提升。
    """
    rows = conn.execute(_DRAIN, {"user_id": user_id, "keep": max(0, keep)}).fetchall()
    return sorted(rows, key=lambda r: (r["createdAt"], r["id"]))
