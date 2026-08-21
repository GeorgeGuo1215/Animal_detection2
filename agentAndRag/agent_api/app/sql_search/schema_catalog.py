from __future__ import annotations

from typing import Dict, FrozenSet, Optional

# sql.search 白名单。此处每张表必须有 `animal_id` 列，以便请求级强制过滤
# （见 tool._merge_animal_scope）隔离到一只宠物。
#
# 逐秒原始时序（vitals_samples / temp_samples / accel_samples）故意不在此：
# 它们没有 animal_id 列，且会用大量行淹没 LLM。
# 请改用固定 SQL 的 `vitals.summary` 工具查询。
TABLE_COLUMNS: Dict[str, FrozenSet[str]] = {
    "daily_reports": frozenset(
        {
            "id",
            "report_date",
            "animal_id",
            "risk_level",
            "confidence",
            "report_text",
            "report_json",
            "evidence_json",
            "agent_trace_id",
            "created_at",
            "updated_at",
        }
    ),
    "animals": frozenset(
        {
            "id",
            "animal_id",
            "species",
            "name",
            "breed",
            "sex",
            "age_months",
            "weight_kg",
            "created_at",
            "updated_at",
        }
    ),
    # sensor_events：上传窗口元数据。故意排除 `raw_payload`（大 JSON）以保持对 LLM 友好。
    "sensor_events": frozenset(
        {
            "id",
            "event_id",
            "ts",
            "timezone",
            "animal_id",
            "device_id",
            "window_start",
            "window_end",
            "notes",
            "tags",
            "location_lat",
            "location_lng",
            "location_accuracy_m",
            "created_at",
        }
    ),
}

ALLOWED_TABLES: FrozenSet[str] = frozenset(TABLE_COLUMNS.keys())

# 必须强制 animal_id 范围过滤的表。当前白名单表都有 animal_id 列，因此与 ALLOWED_TABLES 一致。
ANIMAL_SCOPED_TABLES: FrozenSet[str] = frozenset(TABLE_COLUMNS.keys())

# 遗留常量（保留以稳定导入）
HEAVY_SAMPLE_TABLES: FrozenSet[str] = frozenset()
HEAVY_MAX_LIMIT: int = 100


def normalize_column(name: str) -> str:
    """规范化列名为小写并去除空白。"""
    return name.strip().lower()


def validate_table(table: str) -> str:
    """校验表名是否在白名单内。"""
    t = table.strip().lower()
    if t not in ALLOWED_TABLES:
        raise ValueError(f"Unknown or disallowed table: {table!r}. Allowed: {sorted(ALLOWED_TABLES)}")
    return t


def validate_columns(table: str, columns: Optional[list]) -> list[str]:
    """校验并规范化列名；未指定则返回该表白名单全部列。"""
    t = validate_table(table)
    allowed = TABLE_COLUMNS[t]
    if not columns:
        return sorted(allowed)
    out: list[str] = []
    for c in columns:
        cn = normalize_column(c)
        if cn not in allowed:
            raise ValueError(f"Column {c!r} is not allowed on table {t}. Allowed: {sorted(allowed)}")
        out.append(cn)
    return out


def column_allowed(table: str, column: str) -> str:
    """确认单列属于该表白名单并返回规范化名。"""
    t = validate_table(table)
    cn = normalize_column(column)
    if cn not in TABLE_COLUMNS[t]:
        raise ValueError(f"Column {column!r} is not allowed on table {t}")
    return cn
