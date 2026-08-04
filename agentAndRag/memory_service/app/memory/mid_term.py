"""中期记忆：按话题聚合的段与页，热度管理与汰换都在这一层。

与上游 MemoryOS 的关键差异：

1. 段与页落表，热度落列。上游把全部 session 放在进程内字典里，再用一个内存 heap
   维护热度排序，多 worker 各自持有一份必然发散。
2. 汰换按实时热度排序，而不是上游那种独立于热度的 LFU 全字典扫描。
3. 检索走 pgvector 的 HNSW 索引，而不是每次查询重建一个 faiss 扁平索引。

热度公式在 heat.py 里定义，本模块的 ``HEAT_SQL`` 是它的 SQL 镜像——因为按热度排序
必须在数据库里完成（时间衰减要实时算，不能用写入时缓存的旧值，否则一个曾经很热
但早已无人问津的段会永远排在前面，躲过汰换）。两份实现的一致性由
tests/test_storage.py::test_sql_heat_matches_python_heat 锁定。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence

import psycopg

from ..db import new_id, to_vector_param
from ..heat import HeatParams, jaccard, segment_heat, topic_score

# heat.py::segment_heat 的 SQL 等价形式。
# GREATEST(0, ...) 对应 Python 侧"未来时间戳钳到 0 衰减量"的处理。
HEAT_SQL = """
    %(alpha)s * ln(1 + GREATEST(0, "visitCount"))
  + %(beta)s  * ln(1 + GREATEST(0, "pageCount"))
  + %(gamma)s * exp(
        - GREATEST(0, EXTRACT(EPOCH FROM (%(now)s::timestamp - "lastVisitAt")) / 3600.0)
        / %(tau)s
    )
"""

_MIN_TAU = 1e-6


def _heat_params(params: HeatParams, now: datetime) -> Dict[str, Any]:
    return {
        "alpha": params.alpha,
        "beta": params.beta,
        "gamma": params.gamma,
        # tau 为 0 在 Python 侧表示"不衰减"，SQL 里直接除会炸，
        # 用极小值让 exp(-huge) 趋近 0；config 层已保证正常取值远大于此。
        "tau": params.tau_hours if params.tau_hours > 0 else _MIN_TAU,
        "now": now,
    }


# ---------------------------------------------------------------- 写入


def create_segment(
    conn: psycopg.Connection,
    *,
    user_id: str,
    summary: str,
    keywords: Sequence[str],
    embedding: Optional[Sequence[float]],
    now: datetime,
    pet_id: Optional[str] = None,
    params: HeatParams = HeatParams(),
) -> str:
    segment_id = new_id()
    heat = segment_heat(0, 0, now, now, params)
    conn.execute(
        """
        INSERT INTO memory_segments
            ("id", "userId", "petId", "summary", "keywords", "summaryEmbedding",
             "visitCount", "pageCount", "heat", "lastVisitAt", "createdAt", "updatedAt")
        VALUES (%(id)s, %(user_id)s, %(pet_id)s, %(summary)s, %(keywords)s, %(embedding)s,
                0, 0, %(heat)s, %(now)s, %(now)s, %(now)s)
        """,
        {
            "id": segment_id,
            "user_id": user_id,
            "pet_id": pet_id,
            "summary": summary,
            "keywords": list(keywords),
            "embedding": to_vector_param(embedding),
            "heat": heat,
            "now": now,
        },
    )
    return segment_id


def add_pages(
    conn: psycopg.Connection,
    *,
    segment_id: str,
    user_id: str,
    pages: Sequence[Dict[str, Any]],
    now: datetime,
    params: HeatParams = HeatParams(),
) -> List[str]:
    """把若干轮对话挂到段下，并同步刷新段的页数、访问时间与热度。

    pages 中每项需含 user_input / agent_response，可选 embedding / pet_id /
    prev_page_id / created_at。
    """
    if not pages:
        return []

    ids: List[str] = []
    for page in pages:
        page_id = page.get("id") or new_id()
        conn.execute(
            """
            INSERT INTO memory_pages
                ("id", "segmentId", "userId", "petId", "userInput", "agentResponse",
                 "embedding", "prevPageId", "createdAt")
            VALUES (%(id)s, %(segment_id)s, %(user_id)s, %(pet_id)s, %(user_input)s,
                    %(agent_response)s, %(embedding)s, %(prev_page_id)s,
                    COALESCE(%(created_at)s, %(now)s))
            """,
            {
                "id": page_id,
                "segment_id": segment_id,
                "user_id": user_id,
                "pet_id": page.get("pet_id"),
                "user_input": page.get("user_input", ""),
                "agent_response": page.get("agent_response", ""),
                "embedding": to_vector_param(page.get("embedding")),
                "prev_page_id": page.get("prev_page_id"),
                "created_at": page.get("created_at"),
                "now": now,
            },
        )
        ids.append(page_id)

    conn.execute(
        """
        UPDATE memory_segments
        SET "pageCount" = "pageCount" + %(added)s,
            "lastVisitAt" = %(now)s,
            "updatedAt" = %(now)s
        WHERE "id" = %(segment_id)s
        """,
        {"added": len(ids), "now": now, "segment_id": segment_id},
    )
    _sync_heat(conn, segment_id, now, params)
    return ids


SUMMARY_SEPARATOR = "；"
MAX_SUMMARY_PARTS = 3
MAX_KEYWORDS = 8


def merge_summary(old: str, new: str, max_parts: int = MAX_SUMMARY_PARTS) -> str:
    """把新主题摘要并进段摘要，保留最近若干条。

    段摘要原本建段时写一次就再不更新，于是真实模型跑 200 轮后出现了"摘要写着鸡肉
    过敏、段里实际装着 33 页体重与喂养"这种情况。摘要既是检索第一层的召回依据，
    又是汰换时沉淀成长期知识的原文，失真的代价不小。

    直接替换会丢掉早先的话题，无限追加会让摘要越滚越长，再调一次 LLM 做融合则会
    破坏"单轮提升固定 3 次调用"的成本承诺。折中是有界拼接：只留最近 max_parts 条，
    零 LLM 成本，且长度有上限。
    """
    parts = [p.strip() for p in old.split(SUMMARY_SEPARATOR) if p.strip()]
    candidate = new.strip()
    if candidate and candidate not in parts:
        parts.append(candidate)
    return SUMMARY_SEPARATOR.join(parts[-max_parts:])


def refresh_summary(
    conn: psycopg.Connection,
    *,
    segment_id: str,
    summary: str,
    keywords: Sequence[str],
    now: datetime,
    embed: Optional[Callable[[str], Sequence[float]]] = None,
) -> str:
    """合并新对话后刷新段的摘要、关键词与向量，返回合并后的摘要。

    向量必须基于**合并后**的摘要重算，所以这里收一个编码回调而不是现成的向量——
    调用方拿不到合并结果，没法在外面先编码好。

    关键词取并集并限长：段代表的话题会随合并扩大，关键词跟着扩大才能被检索到，
    但不设上限的话，一个长期活跃的段最终会匹配上任何查询。
    """
    row = conn.execute(
        'SELECT "summary", "keywords" FROM memory_segments WHERE "id" = %s',
        (segment_id,),
    ).fetchone()
    if row is None:
        return summary

    merged_summary = merge_summary(row["summary"] or "", summary)

    existing = list(row["keywords"] or [])
    for word in keywords:
        if word and word not in existing:
            existing.append(word)
    merged_keywords = existing[-MAX_KEYWORDS:]

    embedding = embed(merged_summary) if embed else None

    conn.execute(
        """
        UPDATE memory_segments
        SET "summary" = %(summary)s,
            "keywords" = %(keywords)s,
            "summaryEmbedding" = COALESCE(%(embedding)s::vector, "summaryEmbedding"),
            "updatedAt" = %(now)s
        WHERE "id" = %(segment_id)s
        """,
        {
            "summary": merged_summary,
            "keywords": merged_keywords,
            "embedding": to_vector_param(embedding),
            "now": now,
            "segment_id": segment_id,
        },
    )
    return merged_summary


def _sync_heat(
    conn: psycopg.Connection, segment_id: str, now: datetime, params: HeatParams
) -> None:
    """把 heat 列刷成当前值。

    该列只用于观测与统计；所有需要排序的地方都用 HEAT_SQL 实时计算，
    因为存储值无法反映此刻的时间衰减。
    """
    conn.execute(
        f'UPDATE memory_segments SET "heat" = ({HEAT_SQL}) WHERE "id" = %(segment_id)s',
        {**_heat_params(params, now), "segment_id": segment_id},
    )


# ---------------------------------------------------------------- 话题合并


def find_best_segment(
    conn: psycopg.Connection,
    *,
    user_id: str,
    embedding: Optional[Sequence[float]],
    keywords: Sequence[str],
    threshold: float,
    keyword_weight: float = 1.0,
    pet_id: Optional[str] = None,
    candidates: int = 10,
) -> Optional[Dict[str, Any]]:
    """找出最适合并入的已有话题段，没有够格的就返回 None（调用方新建段）。

    先用向量索引取候选，再在 Python 侧叠加关键词 Jaccard——关键词是数组列，
    在 SQL 里算 Jaccard 既啰嗦又用不上索引，候选集只有十几条时不值得。

    宠物标签必须**完全相同**才允许合并（两边都为空也算相同）。放宽任何一侧都会让
    段与其页的 petId 对不上：段级过滤会漏，段摘要被汰换时沉淀成知识还会张冠李戴。
    检索侧不适用这条——那里让无标签段也被召回是对的，因为它们装的是通用信息。
    """
    if not embedding:
        return None

    rows = conn.execute(
        """
        SELECT "id", "summary", "keywords",
               1 - ("summaryEmbedding" <=> %(embedding)s::vector) AS similarity
        FROM memory_segments
        WHERE "userId" = %(user_id)s
          AND "summaryEmbedding" IS NOT NULL
          AND "petId" IS NOT DISTINCT FROM %(pet_id)s::text
        ORDER BY "summaryEmbedding" <=> %(embedding)s::vector
        LIMIT %(candidates)s
        """,
        {
            "embedding": to_vector_param(embedding),
            "user_id": user_id,
            "pet_id": pet_id,
            "candidates": candidates,
        },
    ).fetchall()

    best: Optional[Dict[str, Any]] = None
    for row in rows:
        score = topic_score(
            float(row["similarity"]),
            jaccard(keywords, row["keywords"] or []),
            keyword_weight,
        )
        if score >= threshold and (best is None or score > best["score"]):
            best = {"id": row["id"], "summary": row["summary"], "score": score}
    return best


# ---------------------------------------------------------------- 检索


def search(
    conn: psycopg.Connection,
    *,
    user_id: str,
    embedding: Sequence[float],
    top_k_segments: int,
    top_k_pages: int,
    now: datetime,
    params: HeatParams = HeatParams(),
    pet_id: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """检索相关对话页，并把命中的段计一次访问。

    访问计数会抬高热度，这正是"被反复问到的话题更容易沉淀为长期画像"的机制。
    """
    if not embedding:
        return []

    segments = conn.execute(
        """
        SELECT "id", "summary",
               1 - ("summaryEmbedding" <=> %(embedding)s::vector) AS similarity
        FROM memory_segments
        WHERE "userId" = %(user_id)s
          AND "summaryEmbedding" IS NOT NULL
          AND (%(pet_id)s::text IS NULL OR "petId" IS NULL OR "petId" = %(pet_id)s)
        ORDER BY "summaryEmbedding" <=> %(embedding)s::vector
        LIMIT %(limit)s
        """,
        {
            "embedding": to_vector_param(embedding),
            "user_id": user_id,
            "pet_id": pet_id,
            "limit": top_k_segments,
        },
    ).fetchall()

    if not segments:
        return []

    segment_ids = [row["id"] for row in segments]
    pages = conn.execute(
        """
        SELECT "id", "segmentId", "userInput", "agentResponse", "createdAt",
               1 - ("embedding" <=> %(embedding)s::vector) AS similarity
        FROM memory_pages
        WHERE "segmentId" = ANY(%(segment_ids)s)
          AND "embedding" IS NOT NULL
        ORDER BY "embedding" <=> %(embedding)s::vector
        LIMIT %(limit)s
        """,
        {
            "embedding": to_vector_param(embedding),
            "segment_ids": segment_ids,
            "limit": top_k_pages,
        },
    ).fetchall()

    hit_segment_ids = sorted({row["segmentId"] for row in pages})
    if hit_segment_ids:
        touch(conn, hit_segment_ids, now=now, params=params)

    summaries = {row["id"]: row["summary"] for row in segments}
    return [
        {
            "page_id": row["id"],
            "segment_id": row["segmentId"],
            "segment_summary": summaries.get(row["segmentId"], ""),
            "user_input": row["userInput"],
            "agent_response": row["agentResponse"],
            "created_at": row["createdAt"],
            "similarity": float(row["similarity"]),
        }
        for row in pages
    ]


def touch(
    conn: psycopg.Connection,
    segment_ids: Sequence[str],
    *,
    now: datetime,
    params: HeatParams = HeatParams(),
) -> None:
    """记一次访问：访问计数加一、刷新最后访问时间、重算热度。"""
    if not segment_ids:
        return
    conn.execute(
        f"""
        UPDATE memory_segments
        SET "visitCount" = "visitCount" + 1,
            "lastVisitAt" = %(now)s,
            "updatedAt" = %(now)s
        WHERE "id" = ANY(%(ids)s)
        """,
        {"now": now, "ids": list(segment_ids)},
    )
    conn.execute(
        f'UPDATE memory_segments SET "heat" = ({HEAT_SQL}) WHERE "id" = ANY(%(ids)s)',
        {**_heat_params(params, now), "ids": list(segment_ids)},
    )


# ---------------------------------------------------------------- 提升与汰换


def hottest_segment(
    conn: psycopg.Connection,
    *,
    user_id: str,
    threshold: float,
    now: datetime,
    params: HeatParams = HeatParams(),
    min_unanalyzed: int = 1,
) -> Optional[Dict[str, Any]]:
    """取热度最高、越过阈值、且积攒了足够多未分析页的段。

    两道门槛缺一不可。热度决定"这个话题值不值得沉淀"，未分析页数决定"现在是不是
    时候"——只看热度的话，一个热段每新增一页就会被重新分析一次，画像反复重写，
    LLM 成本几乎与对话量成正比，热度阈值形同虚设。

    重复分析由页上的 analyzed 标记挡住。这也是不能照搬上游做法的原因：上游靠
    "分析后把 visitCount 与 pageCount 一起归零"来压热度，代价是段有多长这个客观
    事实被抹掉，热度语义就失真了。这里只归零访问计数。
    """
    row = conn.execute(
        f"""
        SELECT s."id", s."summary", s."petId", s."visitCount", s."pageCount",
               ({HEAT_SQL}) AS heat
        FROM memory_segments s
        WHERE s."userId" = %(user_id)s
          AND (
              SELECT count(*) FROM memory_pages p
              WHERE p."segmentId" = s."id" AND p."analyzed" = false
          ) >= %(min_unanalyzed)s
          AND ({HEAT_SQL}) >= %(threshold)s
        ORDER BY heat DESC
        LIMIT 1
        """,
        {
            **_heat_params(params, now),
            "user_id": user_id,
            "threshold": threshold,
            "min_unanalyzed": max(1, min_unanalyzed),
        },
    ).fetchone()
    return dict(row) if row else None


def unanalyzed_pages(
    conn: psycopg.Connection, segment_id: str, limit: int = 50
) -> List[Dict[str, Any]]:
    return conn.execute(
        """
        SELECT "id", "userInput", "agentResponse", "createdAt"
        FROM memory_pages
        WHERE "segmentId" = %(segment_id)s AND "analyzed" = false
        ORDER BY "createdAt", "id"
        LIMIT %(limit)s
        """,
        {"segment_id": segment_id, "limit": limit},
    ).fetchall()


def mark_analyzed(
    conn: psycopg.Connection,
    *,
    segment_id: str,
    page_ids: Sequence[str],
    now: datetime,
    params: HeatParams = HeatParams(),
) -> None:
    """标记页已分析，并把该段的访问计数清零、降温。

    注意只清 visitCount，保留 pageCount：段有多长是客观事实，抹掉它会让热度公式
    失去"内容体量"这一维度。
    """
    if page_ids:
        conn.execute(
            'UPDATE memory_pages SET "analyzed" = true WHERE "id" = ANY(%s)',
            (list(page_ids),),
        )
    conn.execute(
        """
        UPDATE memory_segments
        SET "visitCount" = 0,
            "lastAnalyzedAt" = %(now)s,
            "updatedAt" = %(now)s
        WHERE "id" = %(segment_id)s
        """,
        {"now": now, "segment_id": segment_id},
    )
    _sync_heat(conn, segment_id, now, params)


def count_segments(conn: psycopg.Connection, user_id: str) -> int:
    row = conn.execute(
        'SELECT count(*) AS n FROM memory_segments WHERE "userId" = %s', (user_id,)
    ).fetchone()
    return int(row["n"]) if row else 0


def overflow_segments(
    conn: psycopg.Connection,
    *,
    user_id: str,
    capacity: int,
    now: datetime,
    params: HeatParams = HeatParams(),
) -> List[Dict[str, Any]]:
    """列出超出容量、应被汰换的最冷段（不删除）。

    按实时热度降序排，前 capacity 个是要留下的，OFFSET 之后的就是最冷的那批。
    调用方负责先把摘要沉淀为长期知识再删除。
    """
    rows = conn.execute(
        f"""
        SELECT "id", "summary", "keywords", "petId", "summaryEmbedding",
               ({HEAT_SQL}) AS heat
        FROM memory_segments
        WHERE "userId" = %(user_id)s
        ORDER BY ({HEAT_SQL}) DESC, "createdAt" DESC
        OFFSET %(capacity)s
        """,
        {**_heat_params(params, now), "user_id": user_id, "capacity": max(0, capacity)},
    ).fetchall()
    return [dict(row) for row in rows]


def delete_segments(conn: psycopg.Connection, segment_ids: Sequence[str]) -> int:
    """删除段，页靠外键级联一起删掉。"""
    if not segment_ids:
        return 0
    cur = conn.execute(
        'DELETE FROM memory_segments WHERE "id" = ANY(%s)', (list(segment_ids),)
    )
    return cur.rowcount


def heat_distribution(
    conn: psycopg.Connection,
    *,
    user_id: str,
    now: datetime,
    params: HeatParams = HeatParams(),
) -> Dict[str, float]:
    """段热度的分位数，供 /stats 与长跑模拟观测参数是否合理。"""
    row = conn.execute(
        f"""
        SELECT count(*) AS n,
               COALESCE(min(h), 0) AS min,
               COALESCE(percentile_cont(0.5) WITHIN GROUP (ORDER BY h), 0) AS p50,
               COALESCE(percentile_cont(0.9) WITHIN GROUP (ORDER BY h), 0) AS p90,
               COALESCE(max(h), 0) AS max
        FROM (
            SELECT ({HEAT_SQL}) AS h
            FROM memory_segments
            WHERE "userId" = %(user_id)s
        ) t
        """,
        {**_heat_params(params, now), "user_id": user_id},
    ).fetchone()
    if not row:
        return {"n": 0, "min": 0.0, "p50": 0.0, "p90": 0.0, "max": 0.0}
    return {
        "n": int(row["n"]),
        "min": float(row["min"]),
        "p50": float(row["p50"]),
        "p90": float(row["p90"]),
        "max": float(row["max"]),
    }
