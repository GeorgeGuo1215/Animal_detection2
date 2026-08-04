"""Vitals_Alert MCP Server —— 心率/呼吸率异常告警。

按 pet_id + 回看窗口查询 PetHealth 的 PostgreSQL，返回均值统计、按物种阈值的
越界计数与告警等级。所有失败都以 JSON status 返回（而不是抛异常），这样 LLM
拿到的永远是可解释的结构化结果。

Exposes one tool:
  - check_vitals: Aggregate HR/RR for one pet and flag threshold breaches
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict, Iterable, List, Optional

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

from .config import HEART_RATE, RESPIRATORY_RATE, load_config, normalize_species
from .db import VitalsDbError, query_vitals

logger = logging.getLogger(__name__)

server = Server("vitals_alert")

DEFAULT_HOURS = 24
MIN_HOURS = 1
MAX_HOURS = 720

_SEVERITY = {"normal": 0, "warning": 1, "alert": 2}

_METRICS = (
    (HEART_RATE, "hr", "心率", "bpm"),
    (RESPIRATORY_RATE, "rr", "呼吸率", "次/分"),
)


def coerce_hours(raw: Any) -> int:
    """Clamp the look-back window into [MIN_HOURS, MAX_HOURS]."""
    try:
        hours = int(raw)
    except (TypeError, ValueError):
        return DEFAULT_HOURS
    return max(MIN_HOURS, min(MAX_HOURS, hours))


def classify_metric(
    sample_count: int,
    avg: Optional[float],
    abnormal_count: int,
    bounds: Dict[str, float],
    abnormal_ratio: float,
) -> str:
    """Grade one metric as normal / warning / alert.

    均值本身越界说明是持续性偏离，直接判 alert；偶发越界看占比，低于阈值算
    warning，达到阈值升级为 alert。
    """
    if sample_count <= 0:
        return "normal"
    if avg is not None and (avg < bounds["min"] or avg > bounds["max"]):
        return "alert"
    if abnormal_count <= 0:
        return "normal"
    if abnormal_count / sample_count >= abnormal_ratio:
        return "alert"
    return "warning"


def classify_alert(levels: Iterable[str]) -> str:
    """Overall level is the worst of the per-metric levels."""
    worst = "normal"
    for level in levels:
        if _SEVERITY.get(level, 0) > _SEVERITY[worst]:
            worst = level
    return worst


def _round(value: Any, ndigits: int = 1) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return round(float(value), ndigits)
    return None


def _int(value: Any) -> int:
    try:
        return int(value or 0)
    except (TypeError, ValueError):
        return 0


def _metric_samples(
    rows: List[Dict[str, Any]],
    key: str,
    bounds: Dict[str, float],
) -> List[Dict[str, Any]]:
    """Keep only the rows where this specific metric is out of range."""
    out = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if value < bounds["min"] or value > bounds["max"]:
                out.append({"timestamp": row.get("timestamp"), "value": value})
    return out


def _describe(label: str, unit: str, stats: Dict[str, Any], abnormal: Dict[str, Any]) -> str:
    if stats["sample_count"] <= 0:
        return f"{label}无有效采样"
    avg = stats["avg"]
    text = f"{label}均值 {avg} {unit}" if avg is not None else f"{label}均值不可用"
    count = abnormal["count"]
    if count <= 0:
        return text + "，未见越界"
    parts = []
    if abnormal["above"]:
        parts.append(f"{abnormal['above']} 次偏高")
    if abnormal["below"]:
        parts.append(f"{abnormal['below']} 次偏低")
    return f"{text}，{count} 次越界（{'、'.join(parts)}）"


def build_report(
    pet_id: str,
    hours: int,
    payload: Dict[str, Any],
    abnormal_ratio: float,
) -> Dict[str, Any]:
    """Turn the raw DB payload into the tool's response contract."""
    stats = payload["stats"]
    ranges = payload["ranges"]
    total = _int(stats.get("n"))

    result: Dict[str, Any] = {
        "status": "OK",
        "pet_id": pet_id,
        "species": payload["species"],
        "species_source": payload["species_source"],
        "window": {
            "hours": hours,
            "first_sample_at": stats.get("first_ts"),
            "last_sample_at": stats.get("last_ts"),
        },
        "sample_count": total,
        "thresholds": {metric: ranges[metric] for metric, _, _, _ in _METRICS},
        "abnormal": {},
    }

    levels = []
    descriptions = []
    for metric, prefix, label, unit in _METRICS:
        bounds = ranges[metric]
        n = _int(stats.get(f"{prefix}_n"))
        above = _int(stats.get(f"{prefix}_above"))
        below = _int(stats.get(f"{prefix}_below"))
        count = above + below
        avg = _round(stats.get(f"{prefix}_avg"))

        block = {
            "avg": avg,
            "min": stats.get(f"{prefix}_min"),
            "max": stats.get(f"{prefix}_max"),
            "sample_count": n,
            "unit": unit,
        }
        abnormal = {
            "count": count,
            "above": above,
            "below": below,
            "ratio": round(count / n, 4) if n else 0.0,
            "samples": _metric_samples(payload["abnormal_samples"], metric, bounds),
        }
        result[metric] = block
        result["abnormal"][metric] = abnormal

        levels.append(classify_metric(n, avg, count, bounds, abnormal_ratio))
        descriptions.append(_describe(label, unit, block, abnormal))

    result["alert_level"] = classify_alert(levels)
    result["summary"] = f"近 {hours} 小时共 {total} 条采样；" + "；".join(descriptions) + "。"
    return result


def check_vitals(
    pet_id: Optional[str] = None,
    hours: Any = DEFAULT_HOURS,
    species: Optional[str] = None,
) -> Dict[str, Any]:
    """Aggregate one pet's HR/RR and flag threshold breaches."""
    if not pet_id or not str(pet_id).strip():
        return {
            "status": "INVALID_ARGUMENT",
            "message": "pet_id is required (Pet.id / PetHealthMetric.petId).",
        }

    pet_id = str(pet_id).strip()
    window = coerce_hours(hours)
    cfg = load_config()

    try:
        payload = query_vitals(
            cfg,
            pet_id,
            window,
            species=normalize_species(species) if species else None,
        )
    except VitalsDbError as e:
        return {
            "status": "DB_UNAVAILABLE",
            "pet_id": pet_id,
            "message": f"Cannot query the vitals database: {e}",
        }

    if not payload["pet_found"]:
        return {
            "status": "PET_NOT_FOUND",
            "pet_id": pet_id,
            "message": f"No pet with id {pet_id!r} in table {cfg.pet_table!r}.",
        }

    if _int(payload["stats"].get("n")) <= 0:
        return {
            "status": "NO_DATA",
            "pet_id": pet_id,
            "species": payload["species"],
            "window": {"hours": window},
            "message": f"No vitals samples for this pet in the last {window} hours.",
        }

    return build_report(pet_id, window, payload, cfg.abnormal_ratio)


@server.list_tools()
async def list_tools() -> list[Tool]:
    return [
        Tool(
            name="check_vitals",
            description=(
                "Check one pet's heart rate and respiratory rate against species-specific "
                "normal ranges over a recent time window. Reads the PetHealth time-series "
                "database and returns average/min/max, out-of-range counts with sample "
                "evidence, and an alert level (normal/warning/alert). "
                "Use this when asked whether a pet's vitals are abnormal, or for its "
                "recent average heart/respiratory rate."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "pet_id": {
                        "type": "string",
                        "description": "Pet identifier (Pet.id / PetHealthMetric.petId)",
                    },
                    "hours": {
                        "type": "integer",
                        "description": (
                            f"Look-back window in hours, {MIN_HOURS}-{MAX_HOURS}"
                        ),
                        "default": DEFAULT_HOURS,
                    },
                    "species": {
                        "type": "string",
                        "enum": ["DOG", "CAT", "BIRD", "RABBIT", "OTHER"],
                        "description": (
                            "Optional species override. When omitted it is read from the "
                            "Pet table; thresholds differ a lot per species."
                        ),
                    },
                },
                "required": ["pet_id"],
            },
        ),
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    if name == "check_vitals":
        result = check_vitals(
            pet_id=arguments.get("pet_id"),
            hours=arguments.get("hours", DEFAULT_HOURS),
            species=arguments.get("species"),
        )
    else:
        result = {"error": f"Unknown tool: {name}"}

    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def run_server():
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())
