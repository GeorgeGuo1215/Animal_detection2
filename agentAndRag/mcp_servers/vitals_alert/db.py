"""vitals_alert 数据访问 —— 直连 PetHealth 的 PostgreSQL。

MCP 子进程是按调用拉起、用完即退的，所以这里不做连接池：一次调用开一个连接，
交给 with 关闭。聚合与越界计数全部用 FILTER 在库侧算完，避免把原始采样行拉回
Python（复合主键 (petId, timestamp) 正好覆盖这个查询）。

psycopg 的依赖只落在本模块，且连接工厂 _connect 可被替换，单测据此注入假连接。
"""
from __future__ import annotations

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

import psycopg
from psycopg import sql
from psycopg.rows import dict_row

from .config import HEART_RATE, RESPIRATORY_RATE, VitalsConfig, normalize_species, ranges_for

logger = logging.getLogger(__name__)

# 返回给 LLM 的异常样本条数。只是给出"长什么样"的证据，不是完整清单。
SAMPLE_LIMIT = 5

# hours 显式转 int：psycopg 把 Python int 作为"未指定类型"发出（oid 0），
# 而 make_interval 用的是具名实参，别让类型解析去猜。阈值是 float，psycopg
# 已经带上 float8 类型，无需额外转换。
_WINDOW = sql.SQL(
    '"petId" = %(pet_id)s AND "timestamp" >= now() - make_interval(hours => %(hours)s::int)'
)


class VitalsDbError(RuntimeError):
    """连接或查询失败。server 层据此返回 DB_UNAVAILABLE，而不是让子进程崩掉。"""


def species_sql(pet_table: str) -> sql.Composed:
    return sql.SQL('SELECT "type" AS species FROM {pet} WHERE "id" = %(pet_id)s').format(
        pet=sql.Identifier(pet_table)
    )


def stats_sql(metric_table: str) -> sql.Composed:
    return sql.SQL(
        "SELECT COUNT(*) AS n, "
        'COUNT("heartRate") AS hr_n, '
        'AVG("heartRate")::float8 AS hr_avg, '
        'MIN("heartRate") AS hr_min, '
        'MAX("heartRate") AS hr_max, '
        'COUNT(*) FILTER (WHERE "heartRate" > %(hr_max)s) AS hr_above, '
        'COUNT(*) FILTER (WHERE "heartRate" < %(hr_min)s) AS hr_below, '
        'COUNT("respiratoryRate") AS rr_n, '
        'AVG("respiratoryRate")::float8 AS rr_avg, '
        'MIN("respiratoryRate") AS rr_min, '
        'MAX("respiratoryRate") AS rr_max, '
        'COUNT(*) FILTER (WHERE "respiratoryRate" > %(rr_max)s) AS rr_above, '
        'COUNT(*) FILTER (WHERE "respiratoryRate" < %(rr_min)s) AS rr_below, '
        'MIN("timestamp") AS first_ts, '
        'MAX("timestamp") AS last_ts '
        "FROM {metric} WHERE {window}"
    ).format(metric=sql.Identifier(metric_table), window=_WINDOW)


def samples_sql(metric_table: str) -> sql.Composed:
    return sql.SQL(
        'SELECT "timestamp", "heartRate" AS hr, "respiratoryRate" AS rr '
        "FROM {metric} WHERE {window} AND ("
        '"heartRate" > %(hr_max)s OR "heartRate" < %(hr_min)s '
        'OR "respiratoryRate" > %(rr_max)s OR "respiratoryRate" < %(rr_min)s'
        ') ORDER BY "timestamp" DESC LIMIT %(limit)s::int'
    ).format(metric=sql.Identifier(metric_table), window=_WINDOW)


def _connect(cfg: VitalsConfig) -> Any:
    """Open a short-lived read-only connection. Replaced by tests."""
    return psycopg.connect(
        cfg.dsn,
        connect_timeout=cfg.connect_timeout,
        autocommit=True,
    )


def _iso(value: Any) -> Optional[str]:
    if isinstance(value, datetime):
        return value.isoformat()
    return value if value is None else str(value)


def _bound_params(pet_id: str, hours: int, ranges: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
    return {
        "pet_id": pet_id,
        "hours": hours,
        "hr_min": ranges[HEART_RATE]["min"],
        "hr_max": ranges[HEART_RATE]["max"],
        "rr_min": ranges[RESPIRATORY_RATE]["min"],
        "rr_max": ranges[RESPIRATORY_RATE]["max"],
    }


def query_vitals(
    cfg: VitalsConfig,
    pet_id: str,
    hours: int,
    species: Optional[str] = None,
) -> Dict[str, Any]:
    """Fetch species, aggregate stats and abnormal samples in one connection.

    Args:
        cfg: resolved configuration (DSN, table names, thresholds).
        pet_id: Pet.id / PetHealthMetric.petId.
        hours: look-back window in hours.
        species: explicit species; skips the Pet lookup when provided.

    Returns:
        dict with species / species_source / pet_found / ranges / stats / abnormal_samples.

    Raises:
        VitalsDbError: connection or query failure.
    """
    try:
        with _connect(cfg) as conn:
            with conn.cursor(row_factory=dict_row) as cur:
                if species:
                    resolved = normalize_species(species)
                    source = "argument"
                    pet_found = True
                else:
                    cur.execute(species_sql(cfg.pet_table), {"pet_id": pet_id})
                    row = cur.fetchone()
                    pet_found = row is not None
                    raw = (row or {}).get("species")
                    resolved = normalize_species(raw)
                    source = "database" if raw else "fallback"

                if not pet_found:
                    return {
                        "species": resolved,
                        "species_source": source,
                        "pet_found": False,
                        "ranges": ranges_for(cfg, resolved),
                        "stats": {},
                        "abnormal_samples": [],
                    }

                ranges = ranges_for(cfg, resolved)
                params = _bound_params(pet_id, hours, ranges)

                cur.execute(stats_sql(cfg.metric_table), params)
                stats = dict(cur.fetchone() or {})

                samples: List[Dict[str, Any]] = []
                if int(stats.get("n") or 0) > 0:
                    cur.execute(
                        samples_sql(cfg.metric_table),
                        {**params, "limit": SAMPLE_LIMIT},
                    )
                    samples = [
                        {
                            "timestamp": _iso(r.get("timestamp")),
                            "heart_rate": r.get("hr"),
                            "respiratory_rate": r.get("rr"),
                        }
                        for r in (cur.fetchall() or [])
                    ]
    except Exception as e:  # noqa: BLE001 - 统一收口，交给 server 层转成 DB_UNAVAILABLE
        raise VitalsDbError(str(e)) from e

    stats["first_ts"] = _iso(stats.get("first_ts"))
    stats["last_ts"] = _iso(stats.get("last_ts"))
    return {
        "species": resolved,
        "species_source": source,
        "pet_found": True,
        "ranges": ranges,
        "stats": stats,
        "abnormal_samples": samples,
    }
