"""一次 LLM 调用产出 MoE 意图、路由与证据需求的语义策略。"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...prompts.intent_contracts import (
    DEFAULT_INTENT_ID,
    INTENT_SPECS,
    get_intent_spec,
    normalize_variant,
)
from ...prompts.moe_task_policy import build_task_policy_messages
from .history_context import build_fact_state_history
from .retrieval_policy import (
    CAPABILITY_TO_TOOL,
    EvidenceTask,
    assign_evidence_tasks,
)
from .router import RouterConfig, RouterDecision, resolve_router_decision
from .trace import MoETrace, extract_usage


_EXPERT_KEYS = ("clinical", "pharmacy", "nutrition", "behavior")
_REQUIREMENTS = {"required", "recommended"}


@dataclass(frozen=True)
class IntentDecision:
    """下游阶段使用的 D1-D8 主意图兼容视图。"""

    intent_id: str
    name: str
    confidence: float
    output_variant: str
    reason: str
    fallback: bool = False
    error: str = ""

    def as_dict(self) -> Dict[str, Any]:
        """转为可序列化字典。"""
        return {
            "intent_id": self.intent_id,
            "name": self.name,
            "confidence": self.confidence,
            "output_variant": self.output_variant,
            "reason": self.reason,
            "fallback": self.fallback,
            "error": self.error,
        }


@dataclass(frozen=True)
class TaskPolicyDecision:
    """统一任务策略决策：意图、专家分数、急症与证据任务。"""
    primary_intent: str
    secondary_intents: Tuple[str, ...]
    confidence: float
    output_variant: str
    scores: Dict[str, float]
    emergency: bool
    emergency_confidence: float
    emergency_evidence: Tuple[str, ...]
    evidence_tasks: Tuple[EvidenceTask, ...]
    missing_information: Tuple[str, ...]
    reason: str
    fallback: bool = False
    error: str = ""

    def as_dict(self) -> Dict[str, Any]:
        """转为可序列化字典。"""
        return {
            "primary_intent": self.primary_intent,
            "secondary_intents": list(self.secondary_intents),
            "confidence": self.confidence,
            "output_variant": self.output_variant,
            "scores": dict(self.scores),
            "emergency": {
                "value": self.emergency,
                "confidence": self.emergency_confidence,
                "evidence": list(self.emergency_evidence),
            },
            "evidence_tasks": [task.as_dict() for task in self.evidence_tasks],
            "missing_information": list(self.missing_information),
            "reason": self.reason,
            "fallback": self.fallback,
            "error": self.error,
        }

    def as_intent_decision(self) -> IntentDecision:
        """转为 IntentDecision。"""
        spec = get_intent_spec(self.primary_intent)
        return IntentDecision(
            intent_id=self.primary_intent,
            name=spec.name,
            confidence=self.confidence,
            output_variant=self.output_variant,
            reason=self.reason,
            fallback=self.fallback,
            error=self.error,
        )

    def as_router_decision(self, config: Optional[RouterConfig] = None) -> RouterDecision:
        """按分数生成 RouterDecision。"""
        return resolve_router_decision(
            scores=self.scores,
            emergency=self.emergency,
            reason=self.reason,
            config=config,
        )

    def assigned_tasks(self, selected_experts: Sequence[str]) -> Tuple[EvidenceTask, ...]:
        """把证据任务分配给入选专家。"""
        return assign_evidence_tasks(self.evidence_tasks, selected_experts)


def fallback_task_policy(reason: str, error: str = "") -> TaskPolicyDecision:
    """策略不可用时的安全回退决策。"""
    return TaskPolicyDecision(
        primary_intent=DEFAULT_INTENT_ID,
        secondary_intents=(),
        confidence=0.0,
        output_variant="default",
        scores={"clinical": 6.0, "pharmacy": 0.0, "nutrition": 0.0, "behavior": 0.0},
        emergency=False,
        emergency_confidence=0.0,
        emergency_evidence=(),
        evidence_tasks=(EvidenceTask(
            capability="local_knowledge",
            owner="clinical",
            requirement="recommended",
            reason="统一策略不可用，安全回退为临床专家自主检索",
        ),),
        missing_information=(),
        reason=reason,
        fallback=True,
        error=error,
    )


def _json_object(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    """从文本提取 JSON 对象。"""
    value = str(text or "").strip()
    if not value:
        return None, "empty task policy response"
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError):
        left, right = value.find("{"), value.rfind("}")
        if left < 0 or right <= left:
            return None, "task policy response has no JSON object"
        try:
            parsed = json.loads(value[left:right + 1])
        except (TypeError, ValueError) as exc:
            return None, f"task policy JSON parse failed: {exc}"
    return (parsed, "") if isinstance(parsed, dict) else (None, "task policy response is not an object")


def _bounded_float(value: Any, low: float, high: float, default: float = 0.0) -> float:
    """将值钳制为范围内浮点。"""
    try:
        return round(max(low, min(high, float(value))), 4)
    except (TypeError, ValueError):
        return default


def _short_strings(value: Any, *, limit: int = 12, width: int = 300) -> Tuple[str, ...]:
    """提取短字符串列表。"""
    if not isinstance(value, list):
        return ()
    return tuple(str(item).strip()[:width] for item in value[:limit] if str(item).strip())


def parse_task_policy(text: str) -> TaskPolicyDecision:
    """解析并约束统一策略模型返回的 JSON。

    校验 D1-D8 主/次意图、专家分数、急症标记与证据任务，将分数和字符串长度限制在
    安全范围；任何格式或枚举错误都回退到可解释的临床保守策略。
    """
    obj, error = _json_object(text)
    if obj is None:
        return fallback_task_policy("统一策略输出无效，回退到临床路径", error)

    primary = str(obj.get("primary_intent") or "").strip().upper()
    if primary not in INTENT_SPECS:
        return fallback_task_policy(
            "统一策略意图标签无效，回退到临床路径",
            f"invalid primary_intent: {primary or '<empty>'}",
        )
    secondary = []
    for item in obj.get("secondary_intents") or []:
        label = str(item).strip().upper()
        if label in INTENT_SPECS and label != primary and label not in secondary:
            secondary.append(label)

    raw_scores = obj.get("scores") if isinstance(obj.get("scores"), dict) else {}
    scores = {key: _bounded_float(raw_scores.get(key), 0.0, 10.0) for key in _EXPERT_KEYS}
    if max(scores.values(), default=0.0) == 0.0:
        scores["clinical"] = 6.0

    emergency_obj = obj.get("emergency") if isinstance(obj.get("emergency"), dict) else {}
    tasks = []
    seen = set()
    for raw in obj.get("evidence_tasks") or []:
        if not isinstance(raw, dict):
            continue
        capability = str(raw.get("capability") or "").strip()
        requirement = str(raw.get("requirement") or "").strip().lower()
        owner = str(raw.get("owner") or "").strip().lower()
        if capability not in CAPABILITY_TO_TOOL or requirement not in _REQUIREMENTS:
            continue
        if owner not in _EXPERT_KEYS:
            owner = ""
        query = str(raw.get("query") or "").strip()[:500]
        raw_queries = raw.get("queries") if isinstance(raw.get("queries"), list) else []
        queries = []
        for value in [query, *raw_queries, raw.get("query_zh"), raw.get("query_en")]:
            text = str(value or "").strip()[:500]
            if text and text not in queries:
                queries.append(text)
        query = queries[0] if queries else ""
        reason = str(raw.get("reason") or "").strip()[:500]
        # One expert may need multiple evidence tasks backed by the same tool.
        # Only collapse genuinely identical tasks, never distinct queries.
        key = (
            capability,
            owner,
            requirement,
            tuple(value.casefold() for value in queries),
            reason.casefold(),
        )
        if key in seen:
            continue
        seen.add(key)
        tasks.append(EvidenceTask(
            capability=capability,
            owner=owner,
            requirement=requirement,
            reason=reason,
            web_fallback_on_weak_local=bool(raw.get("web_fallback_on_weak_local", False)),
            query=query,
            queries=tuple(queries[1:]),
        ))

    return TaskPolicyDecision(
        primary_intent=primary,
        secondary_intents=tuple(secondary),
        confidence=_bounded_float(obj.get("confidence"), 0.0, 1.0),
        output_variant=normalize_variant(primary, str(obj.get("output_variant") or "default")),
        scores=scores,
        emergency=bool(emergency_obj.get("value", False)),
        emergency_confidence=_bounded_float(emergency_obj.get("confidence"), 0.0, 1.0),
        emergency_evidence=_short_strings(emergency_obj.get("evidence")),
        evidence_tasks=tuple(tasks),
        missing_information=_short_strings(obj.get("missing_information")),
        reason=str(obj.get("reason") or "").strip()[:800],
    )


async def decide_task_policy(
    *,
    query: str,
    user_role: str,
    llm: AsyncOpenAIClient,
    config: Optional[RouterConfig] = None,
    species_zh: Optional[str] = None,
    breed: Optional[str] = None,
    conversation_history: Optional[List[Dict[str, str]]] = None,
    expert_context_history: Optional[List[Dict[str, Any]]] = None,
    user_memory: Optional[str] = None,
    prompt_injection: str = "",
    recorder: Optional[MoETrace] = None,
) -> TaskPolicyDecision:
    """调用一次 LLM 同时完成意图、路由、急症与证据任务决策。

    请求会注入 D1-D8 边界、物种/品种和有限会话历史；模型输出经
    ``parse_task_policy`` 强校验，调用异常时返回临床优先的确定性回退结果。
    """
    history_context = build_fact_state_history(
        conversation_history,
        expert_context_history,
        user_memory=user_memory,
    )
    messages = build_task_policy_messages(
        query=query,
        user_role=user_role,
        history_context=history_context or None,
        species_zh=species_zh,
        breed=breed,
        prompt_injection=prompt_injection,
    )
    started = time.perf_counter()
    response: Dict[str, Any] = {}
    output = ""
    try:
        response = await llm.chat(
            messages=messages,
            temperature=0.0,
            max_tokens=1200,
            response_format={"type": "json_object"},
            thinking=False,
        )
        output = extract_text(response)
        decision = parse_task_policy(output)
    except Exception as exc:  # noqa: BLE001
        output = f"[error] {exc}"
        decision = fallback_task_policy("统一策略调用失败，回退到临床路径", str(exc))

    if recorder is not None:
        recorder.record_llm(
            stage="task_policy",
            model=getattr(llm, "model", ""),
            messages=messages,
            output=output,
            latency_ms=(time.perf_counter() - started) * 1000.0,
            usage=extract_usage(response),
            meta={"decision": decision.as_dict(), "router_config": vars(config or RouterConfig())},
        )
        recorder.task_policy_decision = decision.as_dict()
    return decision


__all__ = [
    "TaskPolicyDecision",
    "decide_task_policy",
    "fallback_task_policy",
    "parse_task_policy",
]
