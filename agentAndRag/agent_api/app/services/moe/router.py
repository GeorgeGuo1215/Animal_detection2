"""统一 MoE 任务策略的确定性专家门控。语义意图、急症、专家分数与证据需求由 task_policy 一次 LLM 调用产出；本模块只做分数钳制、softmax/top-k 门控，并强制急症时的临床专家下限。"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .experts import EXPERTS


# 急症必须有临床分诊。药学专家仅在语义分数显示涉及用药、中毒或药理安全决策时入选；
# 若对每例急症都强制药学专家，会给尿路梗阻分诊等无关病例多一次 LLM 调用。
_EMERGENCY_EXPERTS = ("clinical",)


def _env_float(name: str, default: float) -> float:
    """读取浮点环境变量。"""
    try:
        return float(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default


def _env_int(name: str, default: int) -> int:
    """读取整数环境变量。"""
    try:
        return int(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default


@dataclass
class RouterConfig:
    """门控阈值、专家上限与 softmax 温度。"""
    gating_threshold: float = field(default_factory=lambda: _env_float("MOE_GATING_THRESHOLD", 0.15))
    max_experts: int = field(default_factory=lambda: _env_int("MOE_MAX_EXPERTS", 3))
    min_relevance: float = field(default_factory=lambda: _env_float("MOE_MIN_RELEVANCE", 3.0))
    softmax_temp: float = field(default_factory=lambda: _env_float("MOE_SOFTMAX_TEMP", 1.0))


@dataclass
class RouterDecision:
    """门控后的专家选择与权重。"""
    scores: Dict[str, float]
    raw_weights: Dict[str, float]
    weights: Dict[str, float]
    selected_experts: List[str]
    emergency: bool
    out_of_scope: bool
    reason: str


def _softmax(scores: Dict[str, float], temp: float) -> Dict[str, float]:
    """对专家分数做 softmax。"""
    keys = list(scores)
    temperature = max(1e-6, float(temp))
    values = [scores[key] / temperature for key in keys]
    maximum = max(values) if values else 0.0
    exponents = [math.exp(value - maximum) for value in values]
    total = sum(exponents) or 1.0
    return {key: exponents[index] / total for index, key in enumerate(keys)}


def resolve_router_decision(
    *,
    scores: Dict[str, float],
    emergency: bool,
    reason: str,
    config: Optional[RouterConfig] = None,
) -> RouterDecision:
    """把统一策略给出的语义分数转换为确定性的专家选择与归一化权重。

    分数先钳制到 0～10，再经 softmax、相关性阈值、门控阈值和专家数上限筛选；
    急症强制补入临床专家。该函数不再读取问题关键词，也不会自行调用 LLM。
    """
    cfg = config or RouterConfig()
    normalized = {
        key: max(0.0, min(10.0, float(scores.get(key, 0.0))))
        for key in EXPERTS
    }
    raw_weights = _softmax(normalized, cfg.softmax_temp)
    max_score = max(normalized.values()) if normalized else 0.0
    out_of_scope = max_score < cfg.min_relevance and not emergency
    selected: List[str] = []
    weights: Dict[str, float] = {}

    if not out_of_scope:
        ranked = sorted(raw_weights.items(), key=lambda item: item[1], reverse=True)
        for key, weight in ranked:
            if weight >= cfg.gating_threshold and len(selected) < cfg.max_experts:
                selected.append(key)
        if not selected and ranked:
            selected.append(ranked[0][0])
        if emergency:
            for key in _EMERGENCY_EXPERTS:
                if key not in selected:
                    selected.append(key)
        subtotal = {key: raw_weights[key] for key in selected}
        total = sum(subtotal.values()) or 1.0
        weights = {key: round(value / total, 4) for key, value in subtotal.items()}

    return RouterDecision(
        scores={key: round(value, 3) for key, value in normalized.items()},
        raw_weights={key: round(value, 4) for key, value in raw_weights.items()},
        weights=weights,
        selected_experts=selected,
        emergency=bool(emergency),
        out_of_scope=out_of_scope,
        reason=str(reason or ""),
    )


__all__ = ["RouterConfig", "RouterDecision", "resolve_router_decision"]
