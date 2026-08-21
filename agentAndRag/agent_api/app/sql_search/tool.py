from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field, field_validator

from ..context.request_context import get_request_animal_id
from .config import load_mysql_config
from .executor import execute_readonly
from .query_compiler import compile_select
from .schema_catalog import validate_table


class WhereClause(BaseModel):
    """单条 WHERE 条件。"""

    column: str
    op: Literal["eq", "ne", "gt", "gte", "lt", "lte", "in", "between", "like"] = "eq"
    value: Any = None


class OrderByClause(BaseModel):
    """单条 ORDER BY 子句。"""

    column: str
    direction: Literal["asc", "desc"] = "asc"


class SqlSearchIntent(BaseModel):
    """第一层结构化查询意图（工具参数）。"""

    database: str = Field(default="petmind", description="必须与配置的 MySQL 库名一致")
    target: Literal["single_table"] = "single_table"
    table: str = Field(default="daily_reports", min_length=1)
    columns: Optional[List[str]] = Field(default=None, description="省略则返回白名单内全部列")
    where: Optional[List[WhereClause]] = None
    order_by: Optional[List[OrderByClause]] = None
    limit: int = Field(default=50, ge=1)

    @field_validator("table")
    @classmethod
    def _table_lower(cls, v: str) -> str:
        """将表名规范为小写。"""
        return (v or "daily_reports").strip().lower()


def _merge_animal_scope(
    animal_id: str,
    user_where: Optional[List[Dict[str, Any]]],
) -> List[Dict[str, Any]]:
    """前置强制 animal_id 过滤，并去掉用户传入的 animal_id 以防越权。"""
    cleaned: List[Dict[str, Any]] = []
    for w in user_where or []:
        col = str(w.get("column", "")).strip().lower()
        if col == "animal_id":
            continue
        cleaned.append(dict(w))
    scope = {"column": "animal_id", "op": "eq", "value": animal_id}
    return [scope] + cleaned


def sql_search_tool(
    database: Optional[str] = None,
    target: str = "single_table",
    table: Optional[str] = None,
    columns: Optional[List[str]] = None,
    where: Optional[List[Dict[str, Any]]] = None,
    order_by: Optional[List[Dict[str, Any]]] = None,
    limit: int = 50,
    **extra: Any,
) -> Dict[str, Any]:
    """在白名单表上执行只读 SELECT；必须带有请求级 animal_id（见 request_context）。"""
    _ = extra
    cfg = load_mysql_config()

    animal_id = get_request_animal_id()
    if not animal_id:
        return {
            "ok": False,
            "error": "ANIMAL_ID_REQUIRED",
            "message": "sql.search is only available when the request includes a non-empty animal_id "
            "(JSON field animal_id or X-Animal-Id header).",
            "row_count": 0,
            "rows": [],
        }

    tbl = (table or "daily_reports").strip().lower()
    if not tbl:
        tbl = "daily_reports"

    try:
        intent = SqlSearchIntent(
            database=database or cfg.database,
            target="single_table",
            table=tbl,
            columns=columns,
            where=[WhereClause(**w) for w in (where or [])] if where else None,
            order_by=[OrderByClause(**o) for o in (order_by or [])] if order_by else None,
            limit=limit,
        )
    except Exception as e:  # noqa: BLE001
        return {
            "ok": False,
            "error": "INVALID_INTENT",
            "message": str(e),
            "row_count": 0,
            "rows": [],
        }

    if intent.database != cfg.database:
        return {
            "ok": False,
            "error": "INVALID_DATABASE",
            "message": f"database must be {cfg.database!r} (configured PETMIND_MYSQL_DATABASE)",
            "row_count": 0,
            "rows": [],
        }

    try:
        t = validate_table(intent.table)
    except ValueError as e:
        return {
            "ok": False,
            "error": "INVALID_TABLE",
            "message": str(e),
            "row_count": 0,
            "rows": [],
        }

    eff_limit = min(intent.limit, cfg.max_limit)
    user_where_list = [w.model_dump() for w in intent.where] if intent.where else []
    wnorm = _merge_animal_scope(animal_id, user_where_list)

    try:
        sql, params = compile_select(
            table=t,
            columns=intent.columns,
            where=wnorm,
            order_by=[o.model_dump() for o in intent.order_by] if intent.order_by else None,
            limit=eff_limit,
        )
    except ValueError as e:
        return {
            "ok": False,
            "error": "COMPILE_ERROR",
            "message": str(e),
            "row_count": 0,
            "rows": [],
        }

    try:
        rows, n = execute_readonly(sql, params, cfg)
    except Exception as e:  # noqa: BLE001
        return {
            "ok": False,
            "error": "EXECUTION_ERROR",
            "message": str(e),
            "row_count": 0,
            "rows": [],
        }

    out: Dict[str, Any] = {
        "ok": True,
        "database": cfg.database,
        "table": t,
        "scoped_animal_id": animal_id,
        "row_count": n,
        "rows": rows,
    }
    if cfg.debug_sql:
        out["compiled_sql"] = sql
        out["compiled_params"] = params
    return out
