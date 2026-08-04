from __future__ import annotations

import re
from typing import Any, List, Tuple

from .schema_catalog import column_allowed, validate_columns, validate_table


def _quote_ident(name: str) -> str:
    if not re.match(r"^[a-zA-Z_][a-zA-Z0-9_]*$", name):
        raise ValueError(f"Invalid identifier: {name!r}")
    return f"`{name}`"


def _compile_where_clause(
    table: str,
    where: list[dict[str, Any]] | None,
    params: List[Any],
) -> str:
    if not where:
        return ""

    parts: list[str] = []
    for clause in where:
        col = str(clause.get("column", "")).strip()
        op = str(clause.get("op", "eq")).strip().lower()
        cname = column_allowed(table, col)

        ident = _quote_ident(cname)
        if op == "eq":
            params.append(clause.get("value"))
            parts.append(f"{ident} = %s")
        elif op == "ne":
            params.append(clause.get("value"))
            parts.append(f"{ident} <> %s")
        elif op == "gt":
            params.append(clause.get("value"))
            parts.append(f"{ident} > %s")
        elif op == "gte":
            params.append(clause.get("value"))
            parts.append(f"{ident} >= %s")
        elif op == "lt":
            params.append(clause.get("value"))
            parts.append(f"{ident} < %s")
        elif op == "lte":
            params.append(clause.get("value"))
            parts.append(f"{ident} <= %s")
        elif op == "in":
            vals = clause.get("value")
            if not isinstance(vals, list) or not vals:
                raise ValueError("op 'in' requires non-empty list value")
            placeholders = ", ".join(["%s"] * len(vals))
            params.extend(vals)
            parts.append(f"{ident} IN ({placeholders})")
        elif op == "between":
            vals = clause.get("value")
            if not isinstance(vals, list) or len(vals) != 2:
                raise ValueError("op 'between' requires value as [low, high]")
            params.append(vals[0])
            params.append(vals[1])
            parts.append(f"{ident} BETWEEN %s AND %s")
        elif op == "like":
            params.append(clause.get("value"))
            parts.append(f"{ident} LIKE %s")
        else:
            raise ValueError(f"Unsupported where op: {op!r}")

    return " WHERE " + " AND ".join(parts)


def compile_select(
    *,
    table: str,
    columns: list[str] | None,
    where: list[dict[str, Any]] | None,
    order_by: list[dict[str, Any]] | None,
    limit: int,
) -> Tuple[str, List[Any]]:
    t = validate_table(table)
    cols = validate_columns(t, columns)
    col_sql = ", ".join(_quote_ident(c) for c in cols)

    params: List[Any] = []
    sql_parts = [f"SELECT {col_sql} FROM {_quote_ident(t)}"]
    sql_parts.append(_compile_where_clause(t, where, params))

    if order_by:
        ob_parts = []
        for ob in order_by:
            cn = str(ob.get("column", "")).strip()
            direction = str(ob.get("direction", "asc")).strip().lower()
            if direction not in ("asc", "desc"):
                raise ValueError(f"Invalid order direction: {direction!r}")
            cname = column_allowed(t, cn)
            ob_parts.append(f"{_quote_ident(cname)} {direction.upper()}")
        sql_parts.append(" ORDER BY " + ", ".join(ob_parts))

    sql_parts.append(" LIMIT %s")
    params.append(limit)

    return "".join(sql_parts), params
