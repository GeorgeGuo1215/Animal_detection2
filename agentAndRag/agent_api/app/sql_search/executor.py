from __future__ import annotations

from datetime import date, datetime
from decimal import Decimal
from typing import Any, Dict, List, Tuple

from .config import MysqlConfig
from .pool import get_pool


def _serialize_cell(v: Any) -> Any:
    if v is None:
        return None
    if isinstance(v, datetime):
        return v.isoformat(sep=" ", timespec="milliseconds")
    if isinstance(v, date):
        return v.isoformat()
    if isinstance(v, Decimal):
        return str(v)
    if isinstance(v, (bytes, bytearray)):
        return v.decode("utf-8", errors="replace")
    return v


def execute_readonly(sql: str, params: List[Any], cfg: MysqlConfig) -> Tuple[List[Dict[str, Any]], int]:
    # Borrow a pooled connection instead of opening one per query.
    with get_pool(cfg).connection() as conn:
        with conn.cursor() as cur:
            cur.execute(sql, params)
            rows = list(cur.fetchall())
            # Serialize non-JSON-serializable values for LLM / JSON
            out: List[Dict[str, Any]] = []
            for row in rows:
                clean = {k: _serialize_cell(v) for k, v in row.items()}
                out.append(clean)
            return out, len(out)
