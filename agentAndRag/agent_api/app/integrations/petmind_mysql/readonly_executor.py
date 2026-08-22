from __future__ import annotations

from datetime import date, datetime
from decimal import Decimal
from typing import Any, Dict, List, Tuple

from .config import MysqlConfig
from .connection_pool import get_pool


def _serialize_cell(v: Any) -> Any:
    """将日期、Decimal、bytes 等单元格值转为 JSON 可序列化类型。"""
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
    """从连接池借用连接执行只读 SELECT，并序列化行数据。"""
    # 借用池化连接，避免每次查询新建连接。
    with get_pool(cfg).connection() as conn:
        with conn.cursor() as cur:
            cur.execute(sql, params)
            rows = list(cur.fetchall())
            # 序列化 LLM / JSON 无法直接处理的值
            out: List[Dict[str, Any]] = []
            for row in rows:
                clean = {k: _serialize_cell(v) for k, v in row.items()}
                out.append(clean)
            return out, len(out)
