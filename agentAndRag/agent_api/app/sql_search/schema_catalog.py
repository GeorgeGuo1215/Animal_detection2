from __future__ import annotations

from typing import Dict, FrozenSet, Optional

# Whitelist for sql.search. Every table here MUST own an `animal_id` column so the
# mandatory request-scoped filter (see tool._merge_animal_scope) can isolate one pet.
#
# Raw per-second time-series (vitals_samples / temp_samples / accel_samples) are NOT
# here on purpose: they have no animal_id column and would flood the LLM with rows.
# Query them through the dedicated, fixed-SQL `vitals.summary` tool instead.
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
    # sensor_events: upload-window metadata. `raw_payload` (large JSON) is intentionally
    # excluded to keep responses LLM-friendly.
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

# Tables that require a mandatory animal_id scope filter. All currently whitelisted
# tables carry an animal_id column, so the set mirrors ALLOWED_TABLES.
ANIMAL_SCOPED_TABLES: FrozenSet[str] = frozenset(TABLE_COLUMNS.keys())

# Legacy constants (kept for stable imports)
HEAVY_SAMPLE_TABLES: FrozenSet[str] = frozenset()
HEAVY_MAX_LIMIT: int = 100


def normalize_column(name: str) -> str:
    return name.strip().lower()


def validate_table(table: str) -> str:
    t = table.strip().lower()
    if t not in ALLOWED_TABLES:
        raise ValueError(f"Unknown or disallowed table: {table!r}. Allowed: {sorted(ALLOWED_TABLES)}")
    return t


def validate_columns(table: str, columns: Optional[list]) -> list[str]:
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
    t = validate_table(table)
    cn = normalize_column(column)
    if cn not in TABLE_COLUMNS[t]:
        raise ValueError(f"Column {column!r} is not allowed on table {t}")
    return cn
