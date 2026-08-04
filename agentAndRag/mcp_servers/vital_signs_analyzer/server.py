"""
Vital Signs Analyzer MCP Server.

Exposes one tool:
  - analyze_vitals: Analyze heart rate / respiratory rate time-series data
    and return structured clinical summary with anomaly detection.

Pure numpy implementation, no external ML dependencies.
"""
from __future__ import annotations

import json
import logging
import math
from typing import Any

import numpy as np
from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

logger = logging.getLogger(__name__)

server = Server("vital_signs_analyzer")

# ---- Species normal ranges ----

SPECIES_RANGES: dict[str, dict[str, Any]] = {
    "dog": {
        "hr": (60, 140),
        "rr": (10, 30),
        "temp": (38.0, 39.2),
        "label": "犬",
    },
    "cat": {
        "hr": (140, 220),
        "rr": (20, 30),
        "temp": (38.1, 39.2),
        "label": "猫",
    },
    "panda": {
        "hr": (60, 100),
        "rr": (10, 20),
        "temp": (36.5, 38.5),
        "label": "大熊猫",
    },
}


# ---- Core analysis functions ----

def _basic_stats(values: np.ndarray) -> dict[str, float]:
    if len(values) == 0:
        return {}
    return {
        "mean": round(float(np.mean(values)), 2),
        "median": round(float(np.median(values)), 2),
        "std": round(float(np.std(values, ddof=1)) if len(values) > 1 else 0.0, 2),
        "min": round(float(np.min(values)), 2),
        "max": round(float(np.max(values)), 2),
        "count": int(len(values)),
    }


def _hrv_metrics(rr_intervals_ms: np.ndarray) -> dict[str, float]:
    """Compute HRV metrics from RR intervals (milliseconds between beats)."""
    if len(rr_intervals_ms) < 3:
        return {}
    diffs = np.diff(rr_intervals_ms)
    sdnn = float(np.std(rr_intervals_ms, ddof=1))
    rmssd = float(np.sqrt(np.mean(diffs ** 2)))
    pnn50 = float(np.sum(np.abs(diffs) > 50) / len(diffs) * 100) if len(diffs) > 0 else 0.0
    return {
        "sdnn_ms": round(sdnn, 2),
        "rmssd_ms": round(rmssd, 2),
        "pnn50_pct": round(pnn50, 2),
    }


def _trend(times: np.ndarray, values: np.ndarray) -> dict[str, Any]:
    """Linear regression trend detection."""
    if len(values) < 3:
        return {"direction": "insufficient_data"}
    t = times - times[0]
    if np.std(t) == 0:
        return {"direction": "stable", "slope_per_min": 0.0}
    coeffs = np.polyfit(t, values, 1)
    slope_per_min = float(coeffs[0]) * 60.0
    if abs(slope_per_min) < 0.5:
        direction = "stable"
    elif slope_per_min > 0:
        direction = "rising"
    else:
        direction = "falling"
    return {
        "direction": direction,
        "slope_per_min": round(slope_per_min, 3),
    }


def _detect_anomalies(
    values: np.ndarray,
    times: np.ndarray,
    normal_range: tuple[float, float],
    signal_name: str,
) -> list[dict[str, Any]]:
    """Detect values outside normal range and sudden changes."""
    anomalies: list[dict[str, Any]] = []
    lo, hi = normal_range

    mean_val = float(np.mean(values))
    if mean_val < lo:
        anomalies.append({
            "type": f"{signal_name}_low",
            "message": f"平均{signal_name} {mean_val:.1f} 低于正常范围下限 {lo}",
            "severity": "warning" if mean_val < lo * 0.8 else "watch",
        })
    elif mean_val > hi:
        anomalies.append({
            "type": f"{signal_name}_high",
            "message": f"平均{signal_name} {mean_val:.1f} 高于正常范围上限 {hi}",
            "severity": "warning" if mean_val > hi * 1.2 else "watch",
        })

    # Sudden changes (>3 sigma)
    if len(values) > 3:
        diffs = np.abs(np.diff(values))
        std_diff = float(np.std(diffs, ddof=1)) if len(diffs) > 1 else 1.0
        mean_diff = float(np.mean(diffs))
        threshold = mean_diff + 3 * max(std_diff, 0.1)
        spikes = np.where(diffs > threshold)[0]
        for idx in spikes[:5]:  # report max 5
            anomalies.append({
                "type": f"{signal_name}_sudden_change",
                "message": f"t={float(times[idx + 1]):.0f}s 处{signal_name}突变 "
                           f"{values[idx]:.1f} → {values[idx + 1]:.1f}",
                "severity": "watch",
                "time_s": float(times[idx + 1]),
            })

    return anomalies


def _detect_apnea(rr_samples: list[dict], threshold_s: float = 8.0) -> list[dict[str, Any]]:
    """Detect respiratory pauses (gaps where RR drops to near zero or is absent)."""
    if len(rr_samples) < 3:
        return []
    anomalies: list[dict[str, Any]] = []
    times = [s["t_s"] for s in rr_samples]
    rrs = [s.get("rr") or 0.0 for s in rr_samples]

    for i in range(1, len(rrs)):
        gap = times[i] - times[i - 1]
        if gap > threshold_s and rrs[i - 1] < 3.0:
            anomalies.append({
                "type": "apnea_suspected",
                "message": f"t={times[i - 1]:.0f}-{times[i]:.0f}s 疑似呼吸暂停 ({gap:.1f}s)",
                "severity": "warning",
                "duration_s": round(gap, 1),
            })

    # Also detect very low RR sustained periods
    vals = np.array(rrs, dtype=float)
    low_mask = vals < 5.0
    if np.sum(low_mask) > 0:
        low_pct = float(np.sum(low_mask) / len(vals) * 100)
        if low_pct > 10:
            anomalies.append({
                "type": "rr_very_low",
                "message": f"{low_pct:.0f}% 的时间呼吸率 < 5 bpm",
                "severity": "warning",
            })

    return anomalies


def _build_clinical_summary(
    species: str,
    stats: dict,
    anomalies: list[dict],
    hr_trend: dict,
    rr_trend: dict,
    alert_level: str,
) -> str:
    """Generate a Chinese clinical summary for the LLM to use."""
    sp = SPECIES_RANGES.get(species, SPECIES_RANGES["dog"])
    label = sp["label"]
    lines: list[str] = []

    lines.append(f"[{label}生理数据分析报告]")

    hr_stats = stats.get("hr", {})
    rr_stats = stats.get("rr", {})

    if hr_stats:
        hr_lo, hr_hi = sp["hr"]
        lines.append(
            f"心率：均值 {hr_stats.get('mean', '-')} bpm "
            f"(正常范围 {hr_lo}-{hr_hi})，"
            f"标准差 {hr_stats.get('std', '-')}，"
            f"趋势{_trend_zh(hr_trend)}"
        )
    if rr_stats:
        rr_lo, rr_hi = sp["rr"]
        lines.append(
            f"呼吸率：均值 {rr_stats.get('mean', '-')} bpm "
            f"(正常范围 {rr_lo}-{rr_hi})，"
            f"标准差 {rr_stats.get('std', '-')}，"
            f"趋势{_trend_zh(rr_trend)}"
        )

    if stats.get("temp"):
        t_lo, t_hi = sp["temp"]
        lines.append(
            f"体温：均值 {stats['temp'].get('mean', '-')} °C "
            f"(正常范围 {t_lo}-{t_hi})"
        )

    if anomalies:
        lines.append(f"异常发现 ({len(anomalies)} 项):")
        for a in anomalies[:8]:
            lines.append(f"  - [{a['severity']}] {a['message']}")

    lines.append(f"综合告警级别: {alert_level}")
    return "\n".join(lines)


def _trend_zh(t: dict) -> str:
    d = t.get("direction", "")
    mapping = {"rising": "上升", "falling": "下降", "stable": "稳定"}
    return mapping.get(d, "数据不足")


def _compute_alert_level(anomalies: list[dict]) -> str:
    severities = [a.get("severity", "normal") for a in anomalies]
    if "critical" in severities:
        return "critical"
    if severities.count("warning") >= 2:
        return "warning"
    if "warning" in severities:
        return "watch"
    if "watch" in severities:
        return "watch"
    return "normal"


# ---- Main analysis entry ----

def analyze_vitals(
    *,
    species: str = "dog",
    weight_kg: float = 0.0,
    hr_samples: list[dict] | None = None,
    rr_samples: list[dict] | None = None,
    temp_samples: list[dict] | None = None,
) -> dict[str, Any]:
    """Analyze vital signs time-series and return structured clinical summary."""
    species = species.lower() if species else "dog"
    if species not in SPECIES_RANGES:
        species = "dog"
    sp = SPECIES_RANGES[species]

    result: dict[str, Any] = {
        "species": species,
        "weight_kg": weight_kg,
        "basic_stats": {},
        "hrv_metrics": {},
        "trends": {},
        "anomalies": [],
        "alert_level": "normal",
        "clinical_summary": "",
    }

    all_anomalies: list[dict] = []

    # Heart rate analysis
    if hr_samples:
        hr_vals = np.array([s.get("hr", 0) for s in hr_samples if s.get("hr") is not None], dtype=float)
        hr_times = np.array([s.get("t_s", 0) for s in hr_samples if s.get("hr") is not None], dtype=float)
        if len(hr_vals) > 0:
            result["basic_stats"]["hr"] = _basic_stats(hr_vals)
            result["trends"]["hr"] = _trend(hr_times, hr_vals)
            all_anomalies.extend(_detect_anomalies(hr_vals, hr_times, sp["hr"], "心率"))

            # HRV from beat-to-beat intervals
            if len(hr_vals) >= 3:
                rr_intervals = 60000.0 / hr_vals  # ms per beat
                result["hrv_metrics"] = _hrv_metrics(rr_intervals)

    # Respiratory rate analysis
    if rr_samples:
        rr_vals = np.array([s.get("rr", 0) for s in rr_samples if s.get("rr") is not None], dtype=float)
        rr_times = np.array([s.get("t_s", 0) for s in rr_samples if s.get("rr") is not None], dtype=float)
        if len(rr_vals) > 0:
            result["basic_stats"]["rr"] = _basic_stats(rr_vals)
            result["trends"]["rr"] = _trend(rr_times, rr_vals)
            all_anomalies.extend(_detect_anomalies(rr_vals, rr_times, sp["rr"], "呼吸率"))
            all_anomalies.extend(_detect_apnea(rr_samples))

    # Temperature analysis
    if temp_samples:
        temp_vals = np.array([s.get("value", 0) for s in temp_samples if s.get("value") is not None], dtype=float)
        temp_times = np.array([s.get("t_s", 0) for s in temp_samples if s.get("value") is not None], dtype=float)
        if len(temp_vals) > 0:
            result["basic_stats"]["temp"] = _basic_stats(temp_vals)
            result["trends"]["temp"] = _trend(temp_times, temp_vals)
            all_anomalies.extend(_detect_anomalies(temp_vals, temp_times, sp["temp"], "体温"))

    result["anomalies"] = all_anomalies
    result["alert_level"] = _compute_alert_level(all_anomalies)
    result["clinical_summary"] = _build_clinical_summary(
        species=species,
        stats=result["basic_stats"],
        anomalies=all_anomalies,
        hr_trend=result["trends"].get("hr", {}),
        rr_trend=result["trends"].get("rr", {}),
        alert_level=result["alert_level"],
    )

    return result


# ---- MCP tool registration ----

@server.list_tools()
async def list_tools() -> list[Tool]:
    return [
        Tool(
            name="analyze_vitals",
            description=(
                "Analyze pet/animal vital signs time-series data (heart rate, respiratory rate, temperature). "
                "Returns statistical summary, HRV metrics, trend analysis, anomaly detection, and clinical interpretation. "
                "ONLY call this tool when the user provides actual physiological measurement data "
                "(e.g. heart rate samples, respiratory rate readings, sensor data). "
                "Do NOT call this for general health questions without measurement data."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "species": {
                        "type": "string",
                        "enum": ["dog", "cat", "panda"],
                        "description": "Animal species",
                        "default": "dog",
                    },
                    "weight_kg": {
                        "type": "number",
                        "description": "Animal weight in kg (used for context only)",
                        "default": 0,
                    },
                    "hr_samples": {
                        "type": "array",
                        "description": "Heart rate time-series: [{t_s: seconds, hr: bpm}, ...]",
                        "items": {
                            "type": "object",
                            "properties": {
                                "t_s": {"type": "number", "description": "Time in seconds"},
                                "hr": {"type": "number", "description": "Heart rate in bpm"},
                            },
                        },
                    },
                    "rr_samples": {
                        "type": "array",
                        "description": "Respiratory rate time-series: [{t_s: seconds, rr: bpm}, ...]",
                        "items": {
                            "type": "object",
                            "properties": {
                                "t_s": {"type": "number", "description": "Time in seconds"},
                                "rr": {"type": "number", "description": "Respiratory rate in bpm"},
                            },
                        },
                    },
                    "temp_samples": {
                        "type": "array",
                        "description": "Temperature time-series: [{t_s: seconds, value: celsius}, ...]",
                        "items": {
                            "type": "object",
                            "properties": {
                                "t_s": {"type": "number", "description": "Time in seconds"},
                                "value": {"type": "number", "description": "Temperature in Celsius"},
                            },
                        },
                    },
                },
                "required": [],
            },
        ),
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    if name == "analyze_vitals":
        result = analyze_vitals(
            species=arguments.get("species", "dog"),
            weight_kg=arguments.get("weight_kg", 0),
            hr_samples=arguments.get("hr_samples"),
            rr_samples=arguments.get("rr_samples"),
            temp_samples=arguments.get("temp_samples"),
        )
    else:
        result = {"error": f"Unknown tool: {name}"}

    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def run_server():
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())
