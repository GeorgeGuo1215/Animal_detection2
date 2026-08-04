"""记忆任务队列。

用 PostgreSQL 当队列，不额外引入 Redis 或消息中间件：记忆写入本来就要落库，
任务和数据在同一个事务里，天然不会出现"任务提交了但数据没落盘"的错位。

出队靠 ``FOR UPDATE SKIP LOCKED``，多个 worker 各取各的，不需要任何应用层协调。
"""

from __future__ import annotations

import logging
from datetime import datetime
from typing import Any, Dict, Optional

import psycopg
from psycopg.types.json import Jsonb

logger = logging.getLogger(__name__)

KIND_CONSOLIDATE = "consolidate"


def enqueue(
    conn: psycopg.Connection,
    *,
    user_id: str,
    kind: str = KIND_CONSOLIDATE,
    payload: Optional[Dict[str, Any]] = None,
    now: Optional[datetime] = None,
) -> Optional[int]:
    """入队一个任务，同用户同类型已有待处理任务时直接合并。

    合并靠 ``(userId, kind) WHERE status='pending'`` 的唯一索引完成：用户连聊二十轮
    也只会留下一个待处理任务，worker 一次把积压的对话全部消化，队列不会被高频
    写入撑爆。返回 None 表示已被合并到既有任务。
    """
    row = conn.execute(
        """
        INSERT INTO memory_tasks ("userId", "kind", "payload", "createdAt", "updatedAt")
        VALUES (%(user_id)s, %(kind)s, %(payload)s,
                COALESCE(%(now)s, CURRENT_TIMESTAMP),
                COALESCE(%(now)s, CURRENT_TIMESTAMP))
        ON CONFLICT DO NOTHING
        RETURNING "id"
        """,
        {
            "user_id": user_id,
            "kind": kind,
            "payload": Jsonb(payload or {}),
            "now": now,
        },
    ).fetchone()
    return int(row["id"]) if row else None


def claim(
    conn: psycopg.Connection, *, now: Optional[datetime] = None
) -> Optional[Dict[str, Any]]:
    """取一个待处理任务并置为 running。

    必须在调用方的事务里执行：SKIP LOCKED 的排他性依赖行锁，事务一提交锁就释放了。
    """
    row = conn.execute(
        """
        UPDATE memory_tasks
        SET "status" = 'running',
            "attempts" = "attempts" + 1,
            "lockedAt" = COALESCE(%(now)s, CURRENT_TIMESTAMP),
            "updatedAt" = COALESCE(%(now)s, CURRENT_TIMESTAMP)
        WHERE "id" = (
            SELECT "id" FROM memory_tasks
            WHERE "status" = 'pending'
            ORDER BY "createdAt", "id"
            FOR UPDATE SKIP LOCKED
            LIMIT 1
        )
        RETURNING "id", "userId", "kind", "payload", "attempts"
        """,
        {"now": now},
    ).fetchone()
    return dict(row) if row else None


def complete(conn: psycopg.Connection, task_id: int) -> None:
    """成功的任务直接删掉。

    队列表只留待处理与失败的记录，长期运行下不会无限膨胀；处理量的统计从
    记忆表本身就能看出来，不需要在这里留流水。
    """
    conn.execute('DELETE FROM memory_tasks WHERE "id" = %s', (task_id,))


def fail(
    conn: psycopg.Connection,
    *,
    task_id: int,
    user_id: str,
    kind: str,
    error: str,
    attempts: int,
    max_attempts: int,
    now: Optional[datetime] = None,
) -> str:
    """记录失败：还有重试机会就放回队列，否则标记为 failed 留待排查。

    attempts 由调用方显式传入并写回。处理失败时那条 claim 所在的事务已经整体回滚，
    数据库里的计数还停在旧值，不写回的话重试次数永远涨不上去，任务会无限重试。

    放回队列时可能撞上"期间又入队了一个同用户同类型任务"的情况——那个新任务会
    覆盖同样的工作，所以直接把当前这条删掉即可。
    """
    truncated = (error or "")[:2000]

    if attempts >= max_attempts:
        conn.execute(
            """
            UPDATE memory_tasks
            SET "status" = 'failed', "lastError" = %(error)s, "attempts" = %(attempts)s,
                "updatedAt" = COALESCE(%(now)s, CURRENT_TIMESTAMP)
            WHERE "id" = %(task_id)s
            """,
            {"error": truncated, "task_id": task_id, "attempts": attempts, "now": now},
        )
        logger.error(
            "memory_service: task %s for user %s gave up after %s attempts: %s",
            task_id, user_id, attempts, truncated,
        )
        return "failed"

    # 必须排除自己：处理失败时那个事务整体回滚了，本任务的状态已经退回 pending，
    # 不排除就会把自己当成重复任务删掉，用户的这批对话再也没人处理。
    duplicate = conn.execute(
        """
        SELECT 1 FROM memory_tasks
        WHERE "userId" = %s AND "kind" = %s AND "status" = 'pending' AND "id" <> %s
        LIMIT 1
        """,
        (user_id, kind, task_id),
    ).fetchone()
    if duplicate:
        conn.execute('DELETE FROM memory_tasks WHERE "id" = %s', (task_id,))
        return "merged"

    conn.execute(
        """
        UPDATE memory_tasks
        SET "status" = 'pending', "lastError" = %(error)s, "lockedAt" = NULL,
            "attempts" = %(attempts)s,
            "updatedAt" = COALESCE(%(now)s, CURRENT_TIMESTAMP)
        WHERE "id" = %(task_id)s
        """,
        {"error": truncated, "task_id": task_id, "attempts": attempts, "now": now},
    )
    return "retry"


def requeue_stale_running(
    conn: psycopg.Connection, *, older_than_seconds: int = 600
) -> int:
    """把卡在 running 的任务放回队列。

    worker 进程被强杀时事务会回滚，任务本该自动回到 pending；但如果是在提交之后
    才崩，任务就会永远停在 running。这个兜底由启动时调用一次，避免任务静默丢失。

    同一用户同一类型可能卡住不止一条（入队与 claim 交替发生时），而 pending 状态上
    有唯一索引，一次性全恢复会撞约束。所以每组只恢复最早的一条，其余留到下次启动，
    反正它们要做的是同一件事。
    """
    cur = conn.execute(
        """
        WITH stale AS (
            SELECT "id",
                   row_number() OVER (
                       PARTITION BY "userId", "kind" ORDER BY "createdAt", "id"
                   ) AS rn
            FROM memory_tasks
            WHERE "status" = 'running'
              AND "lockedAt" < CURRENT_TIMESTAMP - make_interval(secs => %s::int)
        )
        UPDATE memory_tasks
        SET "status" = 'pending', "lockedAt" = NULL, "updatedAt" = CURRENT_TIMESTAMP
        WHERE "id" IN (SELECT "id" FROM stale WHERE rn = 1)
          AND NOT EXISTS (
              SELECT 1 FROM memory_tasks t2
              WHERE t2."userId" = memory_tasks."userId"
                AND t2."kind" = memory_tasks."kind"
                AND t2."status" = 'pending'
          )
        """,
        (older_than_seconds,),
    )
    return cur.rowcount


def stats(conn: psycopg.Connection) -> Dict[str, int]:
    rows = conn.execute(
        'SELECT "status", count(*) AS n FROM memory_tasks GROUP BY "status"'
    ).fetchall()
    return {row["status"]: int(row["n"]) for row in rows}
