"""长期记忆：结构化用户画像与知识条目。

画像相对上游的改动最大。上游把画像存成一整段自由文本，每次分析后整体替换，
于是每一轮都在赌 LLM 会不会把上次写进去的信息重新抄一遍——抄漏了就永久丢失。
这里改成 JSONB 结构化存储，LLM 只需产出"本次新观察到的字段"，合并由代码完成，
历史信息不会因为模型偷懒而消失。并发写入用 version 乐观锁，利用时间戳更新。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence

import psycopg
from psycopg.types.json import Jsonb

from ..db import new_id, to_vector_param

# 画像的字段划分。LLM 只被允许往这几个键里写，避免结构随模型发挥而漂移。
PROFILE_FIELDS = ("communication", "concerns", "petFacts", "healthWatch")

_EMPTY_MARKERS = {"none", "null", "n/a", "无", "暂无", "没有", "未知"}


def _is_empty(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, str):
        return not value.strip() or value.strip().lower() in _EMPTY_MARKERS
    if isinstance(value, (list, tuple, dict, set)):
        return len(value) == 0
    return False


def merge_profile(old: Dict[str, Any], new: Dict[str, Any]) -> Dict[str, Any]:
    """字段级增量合并。

    规则刻意保守，宁可留着旧信息也不轻易抹掉：

    - 字典递归合并（petFacts 按宠物分组，合并时不会因为这次只聊了一只猫
      就把另一只狗的信息删掉）
    - 列表并集且保序去重（关注点会累积，不会因为本轮没提就消失）
    - 标量以新值覆盖（沟通偏好这类会变化的属性应当跟随最新观察）
    - 新值为空或是"无/暂无"这类占位符时直接忽略，不覆盖已有内容
    """
    merged = dict(old or {})
    for key, value in (new or {}).items():
        if _is_empty(value):
            continue
        current = merged.get(key)
        if isinstance(current, dict) and isinstance(value, dict):
            merged[key] = merge_profile(current, value)
        elif isinstance(current, list) and isinstance(value, (list, tuple)):
            seen = list(current)
            for item in value:
                if item not in seen:
                    seen.append(item)
            merged[key] = seen
        else:
            merged[key] = value
    return merged


def get_profile(conn: psycopg.Connection, user_id: str) -> Dict[str, Any]:
    row = conn.execute(
        'SELECT "profile", "version", "updatedAt" FROM memory_profiles WHERE "userId" = %s',
        (user_id,),
    ).fetchone()
    if not row:
        return {"profile": {}, "version": 0, "updatedAt": None}
    return {
        "profile": row["profile"] or {},
        "version": int(row["version"]),
        "updatedAt": row["updatedAt"],
    }


def update_profile(
    conn: psycopg.Connection,
    *,
    user_id: str,
    delta: Dict[str, Any],
    now: datetime,
    max_retries: int = 3,
) -> Optional[Dict[str, Any]]:
    """把画像增量合并进去，返回合并后的画像；无有效增量时返回 None。

    乐观锁：读到的 version 必须与写入时一致，否则说明另一个 worker 刚改过，
    重读再合并一次。冲突窗口极小（同一用户的提升本就有咨询锁串行化），
    这里只是兜底。
    """
    filtered = {k: v for k, v in (delta or {}).items() if k in PROFILE_FIELDS and not _is_empty(v)}
    if not filtered:
        return None

    for _ in range(max_retries):
        current = get_profile(conn, user_id)
        merged = merge_profile(current["profile"], filtered)
        version = current["version"]

        if version == 0 and current["updatedAt"] is None:
            cur = conn.execute(
                """
                INSERT INTO memory_profiles ("userId", "profile", "version", "updatedAt")
                VALUES (%(user_id)s, %(profile)s, 1, %(now)s)
                ON CONFLICT ("userId") DO NOTHING
                """,
                {"user_id": user_id, "profile": Jsonb(merged), "now": now},
            )
            if cur.rowcount:
                return merged
            # 插入被并发抢先，退回更新路径重来一轮。
            continue

        cur = conn.execute(
            """
            UPDATE memory_profiles
            SET "profile" = %(profile)s,
                "version" = "version" + 1,
                "updatedAt" = %(now)s
            WHERE "userId" = %(user_id)s AND "version" = %(version)s
            """,
            {
                "profile": Jsonb(merged),
                "now": now,
                "user_id": user_id,
                "version": version,
            },
        )
        if cur.rowcount:
            return merged

    return None


# ---------------------------------------------------------------- 知识条目


def add_knowledge(
    conn: psycopg.Connection,
    *,
    user_id: str,
    content: str,
    embedding: Optional[Sequence[float]] = None,
    pet_id: Optional[str] = None,
    source: str = "extraction",
    now: Optional[datetime] = None,
) -> Optional[str]:
    """写入一条知识，重复内容直接跳过（靠 (userId, md5(content)) 唯一索引）。

    返回新条目 id；命中去重时返回 None。
    """
    text = (content or "").strip()
    if _is_empty(text):
        return None

    row = conn.execute(
        """
        INSERT INTO memory_knowledge
            ("id", "userId", "petId", "content", "embedding", "source", "createdAt")
        VALUES (%(id)s, %(user_id)s, %(pet_id)s, %(content)s, %(embedding)s, %(source)s,
                COALESCE(%(now)s, CURRENT_TIMESTAMP))
        ON CONFLICT ("userId", md5("content")) DO NOTHING
        RETURNING "id"
        """,
        {
            "id": new_id(),
            "user_id": user_id,
            "pet_id": pet_id,
            "content": text,
            "embedding": to_vector_param(embedding),
            "source": source,
            "now": now,
        },
    ).fetchone()
    return row["id"] if row else None


def search_knowledge(
    conn: psycopg.Connection,
    *,
    user_id: str,
    embedding: Sequence[float],
    top_k: int,
    now: datetime,
    pet_id: Optional[str] = None,
    min_similarity: float = 0.0,
) -> List[Dict[str, Any]]:
    """向量检索知识条目，命中的记一次 hit。

    hitCount 决定知识库满了之后谁先被淘汰：从没被检索到的条目最先出局。
    """
    if not embedding:
        return []

    rows = conn.execute(
        """
        SELECT "id", "content", "petId", "source", "createdAt",
               1 - ("embedding" <=> %(embedding)s::vector) AS similarity
        FROM memory_knowledge
        WHERE "userId" = %(user_id)s
          AND "embedding" IS NOT NULL
          AND (%(pet_id)s::text IS NULL OR "petId" IS NULL OR "petId" = %(pet_id)s)
        ORDER BY "embedding" <=> %(embedding)s::vector
        LIMIT %(limit)s
        """,
        {
            "embedding": to_vector_param(embedding),
            "user_id": user_id,
            "pet_id": pet_id,
            "limit": top_k,
        },
    ).fetchall()

    hits = [row for row in rows if float(row["similarity"]) >= min_similarity]
    if hits:
        conn.execute(
            """
            UPDATE memory_knowledge
            SET "hitCount" = "hitCount" + 1, "lastHitAt" = %(now)s
            WHERE "id" = ANY(%(ids)s)
            """,
            {"now": now, "ids": [row["id"] for row in hits]},
        )
    return [
        {
            "id": row["id"],
            "content": row["content"],
            "pet_id": row["petId"],
            "source": row["source"],
            "similarity": float(row["similarity"]),
            "created_at": row["createdAt"],
        }
        for row in hits
    ]


def count_knowledge(conn: psycopg.Connection, user_id: str) -> int:
    row = conn.execute(
        'SELECT count(*) AS n FROM memory_knowledge WHERE "userId" = %s', (user_id,)
    ).fetchone()
    return int(row["n"]) if row else 0


def enforce_knowledge_capacity(
    conn: psycopg.Connection, *, user_id: str, capacity: int
) -> int:
    """知识库超容量时淘汰"最冷且最旧"的条目。

    上游按纯时间顺序淘汰（deque maxlen），会把一条早期录入但反复被查到的关键事实
    挤掉。这里先看命中次数再看时间，被用到的知识能活得更久。
    """
    cur = conn.execute(
        """
        DELETE FROM memory_knowledge
        WHERE "id" IN (
            SELECT "id" FROM memory_knowledge
            WHERE "userId" = %(user_id)s
            ORDER BY "hitCount" DESC, "createdAt" DESC, "id" DESC
            OFFSET %(capacity)s
        )
        """,
        {"user_id": user_id, "capacity": max(0, capacity)},
    )
    return cur.rowcount
