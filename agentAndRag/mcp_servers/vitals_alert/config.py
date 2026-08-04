"""vitals_alert 配置 —— 环境变量解析与物种阈值表。

这里只做配置解析，不碰数据库、不 import psycopg，因此可以被离线单测完整覆盖。
阈值默认值抄自 PetHealth_Server 的 pet-health-ranges.ts（兽医学参考区间），
物种之间差异极大（狗心率 60-140、猫 120-220、鸟 200-600），所以不存在
"一组全局阈值"的说法，必须按物种取值。
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

DEFAULT_DSN = "postgresql://postgres:postgres@127.0.0.1:5432/PetHealth"
DEFAULT_METRIC_TABLE = "PetHealthMetric"
DEFAULT_PET_TABLE = "Pet"
DEFAULT_ABNORMAL_RATIO = 0.2
DEFAULT_CONNECT_TIMEOUT = 5

HEART_RATE = "heart_rate"
RESPIRATORY_RATE = "respiratory_rate"

FALLBACK_SPECIES = "OTHER"

# 与上游 PET_HEALTH_RANGES 保持一致；上游调整时这里要同步。
DEFAULT_THRESHOLDS: Dict[str, Dict[str, Dict[str, float]]] = {
    "DOG": {
        HEART_RATE: {"min": 60.0, "max": 140.0},
        RESPIRATORY_RATE: {"min": 10.0, "max": 35.0},
    },
    "CAT": {
        HEART_RATE: {"min": 120.0, "max": 220.0},
        RESPIRATORY_RATE: {"min": 20.0, "max": 30.0},
    },
    "BIRD": {
        HEART_RATE: {"min": 200.0, "max": 600.0},
        RESPIRATORY_RATE: {"min": 40.0, "max": 80.0},
    },
    "RABBIT": {
        HEART_RATE: {"min": 120.0, "max": 150.0},
        RESPIRATORY_RATE: {"min": 30.0, "max": 60.0},
    },
    FALLBACK_SPECIES: {
        HEART_RATE: {"min": 60.0, "max": 200.0},
        RESPIRATORY_RATE: {"min": 10.0, "max": 60.0},
    },
}

# 覆盖 JSON 里允许的指标别名。上游 TS 用 heartRate/breathRate，
# 本工具对外用 snake_case，两种写法都接受，省得使用者猜。
_METRIC_ALIASES = {
    "heart_rate": HEART_RATE,
    "heartrate": HEART_RATE,
    "hr": HEART_RATE,
    "respiratory_rate": RESPIRATORY_RATE,
    "respiratoryrate": RESPIRATORY_RATE,
    "breath_rate": RESPIRATORY_RATE,
    "breathrate": RESPIRATORY_RATE,
    "rr": RESPIRATORY_RATE,
}


@dataclass(frozen=True)
class VitalsConfig:
    dsn: str
    metric_table: str
    pet_table: str
    thresholds: Dict[str, Dict[str, Dict[str, float]]]
    abnormal_ratio: float
    connect_timeout: int


def _env(name: str, default: str) -> str:
    """Read an env var, treating blank values as unset.

    mcp_servers.json 用 ``${VAR}`` 模板透传变量，未设置时会被替换成空串，
    所以空串必须当作"没配"处理，否则会拿空 DSN 去连库。
    """
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip()
    return value if value else default


def _env_float(name: str, default: float) -> float:
    raw = _env(name, "")
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning("%s=%r is not a number; falling back to %s", name, raw, default)
        return default


def _env_int(name: str, default: int) -> int:
    raw = _env(name, "")
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        logger.warning("%s=%r is not an integer; falling back to %s", name, raw, default)
        return default


def normalize_species(value: Optional[str]) -> str:
    """Map a caller-supplied species onto a known PetType, else OTHER."""
    if not value:
        return FALLBACK_SPECIES
    key = str(value).strip().upper()
    return key if key in DEFAULT_THRESHOLDS else FALLBACK_SPECIES


def _coerce_bounds(raw: Any) -> Optional[Dict[str, float]]:
    """Validate one {"min": x, "max": y} pair; return None if unusable."""
    if not isinstance(raw, dict):
        return None
    try:
        lo = float(raw["min"])
        hi = float(raw["max"])
    except (KeyError, TypeError, ValueError):
        return None
    if lo >= hi:
        return None
    return {"min": lo, "max": hi}


def merge_thresholds(
    base: Dict[str, Dict[str, Dict[str, float]]],
    override: Any,
) -> Dict[str, Dict[str, Dict[str, float]]]:
    """Deep-merge a user override onto the built-in threshold table.

    非法的物种 / 指标 / 区间会被逐项丢弃并记 warning，其余部分照常生效，
    避免一处笔误让整份阈值失效。
    """
    merged = {
        species: {metric: dict(bounds) for metric, bounds in metrics.items()}
        for species, metrics in base.items()
    }
    if not isinstance(override, dict):
        if override is not None:
            logger.warning("threshold override must be a JSON object; ignoring")
        return merged

    for raw_species, raw_metrics in override.items():
        species = str(raw_species).strip().upper()
        if species not in merged:
            logger.warning("unknown species %r in threshold override; ignoring", raw_species)
            continue
        if not isinstance(raw_metrics, dict):
            logger.warning("threshold override for %s must be an object; ignoring", species)
            continue
        for raw_metric, raw_bounds in raw_metrics.items():
            metric = _METRIC_ALIASES.get(str(raw_metric).strip().lower().replace("-", "_"))
            if metric is None:
                logger.warning("unknown metric %r for %s; ignoring", raw_metric, species)
                continue
            bounds = _coerce_bounds(raw_bounds)
            if bounds is None:
                logger.warning("invalid bounds %r for %s.%s; ignoring", raw_bounds, species, metric)
                continue
            merged[species][metric] = bounds
    return merged


def load_thresholds() -> Dict[str, Dict[str, Dict[str, float]]]:
    raw = _env("VITALS_THRESHOLDS_JSON", "")
    if not raw:
        return merge_thresholds(DEFAULT_THRESHOLDS, None)
    try:
        override = json.loads(raw)
    except json.JSONDecodeError as e:
        logger.warning("VITALS_THRESHOLDS_JSON is not valid JSON (%s); using defaults", e)
        return merge_thresholds(DEFAULT_THRESHOLDS, None)
    return merge_thresholds(DEFAULT_THRESHOLDS, override)


def load_config() -> VitalsConfig:
    ratio = _env_float("VITALS_ABNORMAL_RATIO", DEFAULT_ABNORMAL_RATIO)
    if not 0.0 < ratio <= 1.0:
        logger.warning("VITALS_ABNORMAL_RATIO=%s out of (0, 1]; using default", ratio)
        ratio = DEFAULT_ABNORMAL_RATIO

    timeout = _env_int("VITALS_DB_CONNECT_TIMEOUT", DEFAULT_CONNECT_TIMEOUT)
    if timeout <= 0:
        timeout = DEFAULT_CONNECT_TIMEOUT

    return VitalsConfig(
        dsn=_env("VITALS_DB_DSN", DEFAULT_DSN),
        metric_table=_env("VITALS_METRIC_TABLE", DEFAULT_METRIC_TABLE),
        pet_table=_env("VITALS_PET_TABLE", DEFAULT_PET_TABLE),
        thresholds=load_thresholds(),
        abnormal_ratio=ratio,
        connect_timeout=timeout,
    )


def ranges_for(cfg: VitalsConfig, species: str) -> Dict[str, Dict[str, float]]:
    """Return the HR/RR bounds for a species, falling back to OTHER."""
    return cfg.thresholds.get(species) or cfg.thresholds[FALLBACK_SPECIES]
