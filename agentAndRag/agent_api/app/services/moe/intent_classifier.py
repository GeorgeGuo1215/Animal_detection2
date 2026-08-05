"""One-call LLM classifier for the teacher-defined doctor intents D1-D8."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...prompts.intent_contracts import (
    DEFAULT_INTENT_ID,
    INTENT_SPECS,
    get_intent_spec,
    normalize_variant,
)
from ...prompts.moe_intent_classifier import build_intent_classifier_messages
from .trace import MoETrace, extract_usage


@dataclass(frozen=True)
class IntentDecision:
    intent_id: str
    name: str
    confidence: float
    output_variant: str
    reason: str
    fallback: bool = False
    error: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "intent_id": self.intent_id,
            "name": self.name,
            "confidence": self.confidence,
            "output_variant": self.output_variant,
            "reason": self.reason,
            "fallback": self.fallback,
            "error": self.error,
        }


def fallback_intent(reason: str, error: str = "") -> IntentDecision:
    spec = get_intent_spec(DEFAULT_INTENT_ID)
    return IntentDecision(
        intent_id=spec.intent_id,
        name=spec.name,
        confidence=0.0,
        output_variant="default",
        reason=reason,
        fallback=True,
        error=error,
    )


def _json_object(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    value = str(text or "").strip()
    if not value:
        return None, "empty classifier response"
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError):
        left = value.find("{")
        right = value.rfind("}")
        if left < 0 or right <= left:
            return None, "classifier response has no JSON object"
        try:
            parsed = json.loads(value[left : right + 1])
        except (TypeError, ValueError) as exc:
            return None, f"classifier JSON parse failed: {exc}"
    if not isinstance(parsed, dict):
        return None, "classifier response is not an object"
    return parsed, ""


def parse_intent_decision(text: str) -> IntentDecision:
    obj, error = _json_object(text)
    if obj is None:
        return fallback_intent("意图分类输出无效，安全回退到临床问题分析", error)

    intent_id = str(obj.get("primary_intent") or "").strip().upper()
    if intent_id not in INTENT_SPECS:
        return fallback_intent(
            "意图分类标签不在 D1-D8，安全回退到临床问题分析",
            f"invalid primary_intent: {intent_id or '<empty>'}",
        )
    try:
        confidence = float(obj.get("confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    confidence = round(max(0.0, min(1.0, confidence)), 4)
    variant = normalize_variant(intent_id, str(obj.get("output_variant") or "default"))
    spec = get_intent_spec(intent_id)
    return IntentDecision(
        intent_id=intent_id,
        name=spec.name,
        confidence=confidence,
        output_variant=variant,
        reason=str(obj.get("reason") or "").strip()[:500],
    )


async def classify_intent(
    *,
    query: str,
    llm: AsyncOpenAIClient,
    conversation_history: Optional[List[Dict[str, str]]] = None,
    recorder: Optional[MoETrace] = None,
) -> IntentDecision:
    messages = build_intent_classifier_messages(
        query=query,
        conversation_history=conversation_history,
    )
    started = time.perf_counter()
    response: Dict[str, Any] = {}
    output = ""
    try:
        response = await llm.chat(
            messages=messages,
            temperature=0.0,
            max_tokens=256,
            response_format={"type": "json_object"},
        )
        output = extract_text(response)
        decision = parse_intent_decision(output)
    except Exception as exc:  # noqa: BLE001 - classification must fail closed into D2
        output = f"[error] {exc}"
        decision = fallback_intent("意图分类调用失败，安全回退到临床问题分析", str(exc))

    if recorder is not None:
        recorder.record_llm(
            stage="intent_classifier",
            model=getattr(llm, "model", ""),
            messages=messages,
            output=output,
            latency_ms=(time.perf_counter() - started) * 1000.0,
            usage=extract_usage(response),
            meta={"decision": decision.as_dict()},
        )
        recorder.intent_decision = decision.as_dict()
    return decision
