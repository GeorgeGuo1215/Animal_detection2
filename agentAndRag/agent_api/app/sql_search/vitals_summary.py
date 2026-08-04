"""vitals.summary —— 生理时序聚合工具（固定 SQL，非自由查询）。

时序表 vitals_samples / temp_samples 没有 animal_id 列，且是逐秒原始数据
（accel 甚至上千行）。直接放进 sql.search 既不安全也会淹没 LLM。这里用写死的
JOIN sensor_events + 聚合，强制按请求级 animal_id 过滤，只返回 HR/RR/体温的
统计摘要（count/min/max/avg + 时间范围），供临床/营养专家评估生理状态。
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from ..context.request_context import get_request_animal_id
from .config import load_mysql_config
from .executor import execute_readonly


_VITALS_SQL = (
    "SELECT COUNT(v.id) AS n, "
    "MIN(v.hr_bpm) AS hr_min, MAX(v.hr_bpm) AS hr_max, AVG(v.hr_bpm) AS hr_avg, "
    "MIN(v.rr_bpm) AS rr_min, MAX(v.rr_bpm) AS rr_max, AVG(v.rr_bpm) AS rr_avg, "
    "MIN(e.ts) AS first_ts, MAX(e.ts) AS last_ts "
    "FROM vitals_samples v JOIN sensor_events e ON v.event_pk = e.id "
    "WHERE e.animal_id = %s"
)

_TEMP_SQL = (
    "SELECT COUNT(t.id) AS n, "
    "MIN(t.temp_c) AS temp_min, MAX(t.temp_c) AS temp_max, AVG(t.temp_c) AS temp_avg "
    "FROM temp_samples t JOIN sensor_events e ON t.event_pk = e.id "
    "WHERE e.animal_id = %s"
)

# Optional recency window appended when `days` is provided.
_DAYS_CLAUSE = " AND e.ts >= NOW() - INTERVAL %s DAY"


def _num(v: Any) -> Optional[float]:
    if v is None:
        return None
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return v
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _round(v: Optional[float], ndigits: int) -> Optional[float]:
    return round(v, ndigits) if isinstance(v, (int, float)) else None


def vitals_summary_tool(
    days: Optional[int] = None,
    **extra: Any,
) -> Dict[str, Any]:
    """Aggregate HR/RR/temperature for the request-scoped animal.

    Args:
        days: if provided (>0), only include samples from the last N days; else all history.
    """
    _ = extra
    cfg = load_mysql_config()

    animal_id = get_request_animal_id()
    if not animal_id:
        return {
            "ok": False,
            "error": "ANIMAL_ID_REQUIRED",
            "message": "vitals.summary is only available when the request includes a non-empty "
            "animal_id (JSON field animal_id or X-Animal-Id header).",
        }

    use_days = isinstance(days, int) and days > 0
    vitals_sql = _VITALS_SQL + (_DAYS_CLAUSE if use_days else "")
    temp_sql = _TEMP_SQL + (_DAYS_CLAUSE if use_days else "")
    vitals_params = [animal_id] + ([days] if use_days else [])
    temp_params = [animal_id] + ([days] if use_days else [])

    try:
        vrows, _ = execute_readonly(vitals_sql, vitals_params, cfg)
        trows, _ = execute_readonly(temp_sql, temp_params, cfg)
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": "EXECUTION_ERROR", "message": str(e)}

    v = vrows[0] if vrows else {}
    t = trows[0] if trows else {}
    v_n = int(v.get("n") or 0)
    t_n = int(t.get("n") or 0)

    out: Dict[str, Any] = {
        "ok": True,
        "animal_id": animal_id,
        "window": {"days": days if use_days else "all"},
        "vitals": {
            "sample_count": v_n,
            "hr_bpm": {
                "min": _num(v.get("hr_min")),
                "max": _num(v.get("hr_max")),
                "avg": _round(_num(v.get("hr_avg")), 1),
            },
            "rr_bpm": {
                "min": _num(v.get("rr_min")),
                "max": _num(v.get("rr_max")),
                "avg": _round(_num(v.get("rr_avg")), 1),
            },
            "first_ts": v.get("first_ts"),
            "last_ts": v.get("last_ts"),
        },
        "temperature": {
            "sample_count": t_n,
            "temp_c": {
                "min": _num(t.get("temp_min")),
                "max": _num(t.get("temp_max")),
                "avg": _round(_num(t.get("temp_avg")), 2),
            },
        },
    }
    if v_n == 0 and t_n == 0:
        out["note"] = "No vitals/temperature samples found for this animal in the selected window."
    if cfg.debug_sql:
        out["compiled_sql"] = {"vitals": vitals_sql, "temperature": temp_sql}
    return out
