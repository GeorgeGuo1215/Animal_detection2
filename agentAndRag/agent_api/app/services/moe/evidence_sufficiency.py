"""Batch semantic coverage check for high-scoring local RAG evidence."""
from __future__ import annotations

import asyncio
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional

from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...prompts.moe_evidence_sufficiency import build_evidence_sufficiency_messages
from ..structured_output import safe_json_loads
from .trace import MoETrace, extract_usage


_VALID_STATUSES = frozenset({"supported", "partial", "unsupported"})


@dataclass(frozen=True)
class EvidenceSufficiencyItem:
    id: str
    expert: str
    evidence_query: str
    evidence_goal: str
    hits: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class EvidenceSufficiencyAssessment:
    status: str
    reason: str
    matched_hit_ids: tuple[str, ...] = ()

    @property
    def supported(self) -> bool:
        return self.status == "supported"

    def as_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "reason": self.reason,
            "matched_hit_ids": list(self.matched_hit_ids),
        }


def evidence_sufficiency_enabled() -> bool:
    return os.getenv("MOE_EVIDENCE_SUFFICIENCY_ENABLED", "1").strip().lower() not in {
        "0", "false", "no", "off",
    }


def _positive_int_env(name: str, default: int) -> int:
    try:
        return max(1, int(os.getenv(name, str(default)) or default))
    except (TypeError, ValueError):
        return default


def _positive_float_env(name: str, default: float) -> float:
    try:
        return max(0.1, float(os.getenv(name, str(default)) or default))
    except (TypeError, ValueError):
        return default


def _prompt_items(items: Iterable[EvidenceSufficiencyItem]) -> List[Dict[str, Any]]:
    max_hits = _positive_int_env("MOE_EVIDENCE_SUFFICIENCY_MAX_HITS", 3)
    max_chars = _positive_int_env("MOE_EVIDENCE_SUFFICIENCY_HIT_MAX_CHARS", 1200)
    payload: List[Dict[str, Any]] = []
    for item in items:
        hits = []
        for index, hit in enumerate(item.hits[:max_hits], start=1):
            text = str(hit.get("text") or "").strip()
            hits.append({
                "hit_id": f"h{index}",
                "source_path": str(hit.get("source_path") or ""),
                "text": text[:max_chars],
            })
        payload.append({
            "id": item.id,
            "expert": item.expert,
            "evidence_query": item.evidence_query,
            "evidence_goal": item.evidence_goal,
            "hits": hits,
        })
    return payload


def parse_evidence_sufficiency(
    text: str,
    *,
    expected_ids: Iterable[str],
) -> Dict[str, EvidenceSufficiencyAssessment]:
    expected = set(expected_ids)
    obj, _error = safe_json_loads(text)
    raw_items = obj.get("assessments") if isinstance(obj, dict) else None
    if not isinstance(raw_items, list):
        return {}
    parsed: Dict[str, EvidenceSufficiencyAssessment] = {}
    for raw in raw_items:
        if not isinstance(raw, dict):
            continue
        item_id = str(raw.get("id") or "").strip()
        status = str(raw.get("status") or "").strip().lower()
        if item_id not in expected or status not in _VALID_STATUSES or item_id in parsed:
            continue
        matched = raw.get("matched_hit_ids")
        parsed[item_id] = EvidenceSufficiencyAssessment(
            status=status,
            reason=str(raw.get("reason") or "").strip()[:500],
            matched_hit_ids=tuple(
                str(value)[:40] for value in matched if str(value).strip()
            ) if isinstance(matched, list) else (),
        )
    return parsed


async def assess_evidence_sufficiency(
    *,
    case_question: str,
    items: Iterable[EvidenceSufficiencyItem],
    llm: AsyncOpenAIClient,
    recorder: Optional[MoETrace] = None,
    timeout_s: Optional[float] = None,
) -> Dict[str, EvidenceSufficiencyAssessment]:
    batch = list(items)
    if not batch or not evidence_sufficiency_enabled():
        return {}
    messages = build_evidence_sufficiency_messages(
        case_question=case_question,
        items=_prompt_items(batch),
    )
    started = time.perf_counter()
    response: Dict[str, Any] = {}
    output = ""
    error = ""
    configured_limit = _positive_float_env("MOE_EVIDENCE_SUFFICIENCY_TIMEOUT_SEC", 20.0)
    limit = configured_limit if timeout_s is None else min(
        configured_limit,
        max(0.1, float(timeout_s)),
    )
    try:
        response = await asyncio.wait_for(
            llm.chat(
                messages=messages,
                temperature=0.0,
                max_tokens=min(1200, 180 + len(batch) * 150),
                response_format={"type": "json_object"},
                thinking=False,
            ),
            timeout=limit,
        )
        output = extract_text(response)
        parsed = parse_evidence_sufficiency(
            output,
            expected_ids=(item.id for item in batch),
        )
    except Exception as exc:  # noqa: BLE001
        error = str(exc)
        parsed = {}
    if recorder is not None:
        recorder.record_llm(
            stage="evidence_sufficiency",
            model=getattr(llm, "model", ""),
            messages=messages,
            output=output,
            latency_ms=(time.perf_counter() - started) * 1000.0,
            usage=extract_usage(response),
            meta={
                "batch_size": len(batch),
                "parsed_count": len(parsed),
                "timeout_s": limit,
                "error": error,
            },
        )
    return parsed


__all__ = [
    "EvidenceSufficiencyAssessment",
    "EvidenceSufficiencyItem",
    "assess_evidence_sufficiency",
    "evidence_sufficiency_enabled",
    "parse_evidence_sufficiency",
]
