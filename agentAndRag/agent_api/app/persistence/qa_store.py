"""SQLite-backed Q&A record storage for PetMind.

Stores every question/answer pair with metadata (tools used, RAG hits,
response time, etc.) for admin review and knowledge-gap analysis.
"""
from __future__ import annotations

import asyncio
import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


def _default_db_path() -> Path:
    repo_root = Path(__file__).resolve().parents[3]
    return repo_root / "agent_api_logs" / "petmind_qa.db"


_DB_PATH: Optional[str] = None


def _get_db_path() -> str:
    global _DB_PATH
    if _DB_PATH is None:
        _DB_PATH = os.getenv("QA_DB_PATH", str(_default_db_path()))
    return _DB_PATH


def _connect() -> sqlite3.Connection:
    path = _get_db_path()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(path, timeout=10)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA busy_timeout=5000")
    conn.row_factory = sqlite3.Row
    return conn


_CREATE_TABLE = """
CREATE TABLE IF NOT EXISTS qa_records (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    ts              TEXT    NOT NULL,
    date_key        TEXT    NOT NULL,
    question        TEXT    NOT NULL,
    answer          TEXT    NOT NULL DEFAULT '',
    model           TEXT    NOT NULL DEFAULT '',
    tools_used      TEXT    NOT NULL DEFAULT '[]',
    rag_hit_count   INTEGER NOT NULL DEFAULT 0,
    rag_best_score  REAL    NOT NULL DEFAULT 0.0,
    used_web_search INTEGER NOT NULL DEFAULT 0,
    response_time_ms INTEGER NOT NULL DEFAULT 0,
    source_ip       TEXT    NOT NULL DEFAULT '',
    user_role       TEXT    NOT NULL DEFAULT ''
);
"""

_MIGRATE_SQL = [
    "ALTER TABLE qa_records ADD COLUMN rag_best_score REAL NOT NULL DEFAULT 0.0",
    "ALTER TABLE qa_records ADD COLUMN request_id TEXT NOT NULL DEFAULT ''",
    "ALTER TABLE qa_records ADD COLUMN feedback_rating INTEGER NOT NULL DEFAULT 0",
    "ALTER TABLE qa_records ADD COLUMN feedback_comment TEXT NOT NULL DEFAULT ''",
    "ALTER TABLE qa_records ADD COLUMN feedback_ts TEXT NOT NULL DEFAULT ''",
]

_CREATE_INDEXES = [
    "CREATE INDEX IF NOT EXISTS idx_qa_date_key ON qa_records(date_key);",
    "CREATE INDEX IF NOT EXISTS idx_qa_ts ON qa_records(ts);",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_qa_request_id ON qa_records(request_id) WHERE request_id != '';",
]


def init_db() -> None:
    conn = _connect()
    try:
        conn.execute(_CREATE_TABLE)
        for sql in _MIGRATE_SQL:
            try:
                conn.execute(sql)
            except sqlite3.OperationalError:
                pass  # column already exists
        for idx_sql in _CREATE_INDEXES:
            conn.execute(idx_sql)
        conn.commit()
        print(f"[qa_store] DB ready at {_get_db_path()}")
    finally:
        conn.close()


def _save_record_sync(
    question: str,
    answer: str,
    model: str = "",
    tools_used: Optional[List[str]] = None,
    rag_hit_count: int = 0,
    rag_best_score: float = 0.0,
    used_web_search: bool = False,
    response_time_ms: int = 0,
    source_ip: str = "",
    user_role: str = "",
    request_id: str = "",
) -> int:
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    date_key = time.strftime("%Y-%m-%d")
    conn = _connect()
    try:
        cur = conn.execute(
            """INSERT INTO qa_records
               (ts, date_key, question, answer, model, tools_used,
                rag_hit_count, rag_best_score, used_web_search, response_time_ms,
                source_ip, user_role, request_id)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                now, date_key, question, answer, model,
                json.dumps(tools_used or [], ensure_ascii=False),
                rag_hit_count, round(rag_best_score, 4), int(used_web_search),
                response_time_ms, source_ip, user_role, request_id,
            ),
        )
        conn.commit()
        return cur.lastrowid  # type: ignore[return-value]
    finally:
        conn.close()


async def save_qa_record(**kwargs: Any) -> int:
    """Async wrapper — runs the blocking SQLite insert in a thread."""
    return await asyncio.to_thread(_save_record_sync, **kwargs)


def _query_history_sync(
    page: int = 1,
    page_size: int = 20,
    date_from: Optional[str] = None,
    date_to: Optional[str] = None,
    keyword: Optional[str] = None,
) -> Dict[str, Any]:
    conditions: List[str] = []
    params: List[Any] = []

    if date_from:
        conditions.append("date_key >= ?")
        params.append(date_from)
    if date_to:
        conditions.append("date_key <= ?")
        params.append(date_to)
    if keyword:
        conditions.append("(question LIKE ? OR answer LIKE ?)")
        params.extend([f"%{keyword}%", f"%{keyword}%"])

    where = (" WHERE " + " AND ".join(conditions)) if conditions else ""

    conn = _connect()
    try:
        total = conn.execute(f"SELECT COUNT(*) FROM qa_records{where}", params).fetchone()[0]
        offset = (max(page, 1) - 1) * page_size
        rows = conn.execute(
            f"SELECT * FROM qa_records{where} ORDER BY id DESC LIMIT ? OFFSET ?",
            params + [page_size, offset],
        ).fetchall()
        records = [dict(r) for r in rows]
        for rec in records:
            try:
                rec["tools_used"] = json.loads(rec.get("tools_used") or "[]")
            except Exception:
                pass
        return {"total": total, "page": page, "page_size": page_size, "records": records}
    finally:
        conn.close()


async def query_qa_history(**kwargs: Any) -> Dict[str, Any]:
    return await asyncio.to_thread(_query_history_sync, **kwargs)


def _get_stats_sync(
    date_from: Optional[str] = None,
    date_to: Optional[str] = None,
) -> Dict[str, Any]:
    conditions: List[str] = []
    params: List[Any] = []
    if date_from:
        conditions.append("date_key >= ?")
        params.append(date_from)
    if date_to:
        conditions.append("date_key <= ?")
        params.append(date_to)
    where = (" WHERE " + " AND ".join(conditions)) if conditions else ""

    conn = _connect()
    try:
        row = conn.execute(
            f"""SELECT
                    COUNT(*)            AS total,
                    AVG(response_time_ms) AS avg_response_ms,
                    SUM(used_web_search)  AS web_search_count,
                    AVG(rag_hit_count)    AS avg_rag_hits
                FROM qa_records{where}""",
            params,
        ).fetchone()

        daily = conn.execute(
            f"""SELECT date_key, COUNT(*) AS count
                FROM qa_records{where}
                GROUP BY date_key ORDER BY date_key DESC LIMIT 30""",
            params,
        ).fetchall()

        model_dist = conn.execute(
            f"""SELECT model, COUNT(*) AS count
                FROM qa_records{where}
                GROUP BY model ORDER BY count DESC""",
            params,
        ).fetchall()

        return {
            "total": row["total"],
            "avg_response_ms": round(row["avg_response_ms"] or 0, 1),
            "web_search_count": row["web_search_count"] or 0,
            "avg_rag_hits": round(row["avg_rag_hits"] or 0, 2),
            "daily": [dict(d) for d in daily],
            "model_distribution": [dict(m) for m in model_dist],
        }
    finally:
        conn.close()


async def get_qa_stats(**kwargs: Any) -> Dict[str, Any]:
    return await asyncio.to_thread(_get_stats_sync, **kwargs)


_RAG_GAP_SCORE_THRESHOLD = 0.55


def _get_knowledge_gaps_sync(
    date_from: Optional[str] = None,
    date_to: Optional[str] = None,
    min_occurrences: int = 1,
    limit: int = 50,
) -> Dict[str, Any]:
    """Find questions where RAG had no useful hits — potential knowledge-base gaps.

    A gap is defined as:
    - RAG returned zero hits, OR
    - RAG returned hits but the best rerank score was below the relevance
      threshold (content was retrieved but not actually relevant to the query).
    """
    conditions = [f"(rag_hit_count = 0 OR (rag_hit_count > 0 AND rag_best_score < {_RAG_GAP_SCORE_THRESHOLD}))"]
    params: List[Any] = []
    if date_from:
        conditions.append("date_key >= ?")
        params.append(date_from)
    if date_to:
        conditions.append("date_key <= ?")
        params.append(date_to)
    where = " WHERE " + " AND ".join(conditions)

    conn = _connect()
    try:
        rows = conn.execute(
            f"""SELECT question, COUNT(*) AS occurrences,
                       MAX(used_web_search) AS ever_web_searched,
                       MAX(ts) AS last_asked,
                       MAX(rag_best_score) AS max_rag_score
                FROM qa_records{where}
                GROUP BY question
                HAVING COUNT(*) >= ?
                ORDER BY occurrences DESC
                LIMIT ?""",
            params + [min_occurrences, limit],
        ).fetchall()
        return {
            "total_gap_questions": len(rows),
            "gaps": [dict(r) for r in rows],
        }
    finally:
        conn.close()


async def get_knowledge_gaps(**kwargs: Any) -> Dict[str, Any]:
    return await asyncio.to_thread(_get_knowledge_gaps_sync, **kwargs)


# ---------------------------------------------------------------------------
# Feedback
# ---------------------------------------------------------------------------

def _submit_feedback_sync(
    request_id: str,
    rating: int,
    comment: str = "",
) -> bool:
    """Update feedback columns for the record matching *request_id*.

    Returns True on success, False if not found or already rated.
    """
    if not request_id:
        return False
    conn = _connect()
    try:
        row = conn.execute(
            "SELECT id, feedback_rating FROM qa_records WHERE request_id = ?",
            (request_id,),
        ).fetchone()
        if not row:
            return False
        if row["feedback_rating"] != 0:
            return False  # already rated
        now = time.strftime("%Y-%m-%dT%H:%M:%S")
        conn.execute(
            "UPDATE qa_records SET feedback_rating = ?, feedback_comment = ?, feedback_ts = ? WHERE id = ?",
            (rating, comment, now, row["id"]),
        )
        conn.commit()
        return True
    finally:
        conn.close()


async def submit_feedback(**kwargs: Any) -> bool:
    return await asyncio.to_thread(_submit_feedback_sync, **kwargs)


def _get_feedback_stats_sync(
    date_from: Optional[str] = None,
    date_to: Optional[str] = None,
) -> Dict[str, Any]:
    conditions: List[str] = []
    params: List[Any] = []
    if date_from:
        conditions.append("date_key >= ?")
        params.append(date_from)
    if date_to:
        conditions.append("date_key <= ?")
        params.append(date_to)
    where = (" WHERE " + " AND ".join(conditions)) if conditions else ""

    rated_where = (where + " AND " if where else " WHERE ") + "feedback_rating > 0"

    conn = _connect()
    try:
        total_row = conn.execute(f"SELECT COUNT(*) AS c FROM qa_records{where}", params).fetchone()
        total = total_row["c"]

        rated_row = conn.execute(
            f"SELECT COUNT(*) AS c, AVG(feedback_rating) AS avg_r FROM qa_records{rated_where}", params,
        ).fetchone()
        rated_count = rated_row["c"]
        avg_rating = round(rated_row["avg_r"] or 0, 2)

        dist_rows = conn.execute(
            f"SELECT feedback_rating AS r, COUNT(*) AS c FROM qa_records{rated_where} GROUP BY feedback_rating ORDER BY r",
            params,
        ).fetchall()
        distribution = {str(r["r"]): r["c"] for r in dist_rows}

        recent = conn.execute(
            f"""SELECT request_id, question, feedback_rating, feedback_comment, feedback_ts
                FROM qa_records{rated_where}
                ORDER BY feedback_ts DESC LIMIT 50""",
            params,
        ).fetchall()

        return {
            "total_answers": total,
            "rated_count": rated_count,
            "avg_rating": avg_rating,
            "distribution": distribution,
            "recent_feedback": [dict(r) for r in recent],
        }
    finally:
        conn.close()


async def get_feedback_stats(**kwargs: Any) -> Dict[str, Any]:
    return await asyncio.to_thread(_get_feedback_stats_sync, **kwargs)
