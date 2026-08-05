"""Fact-state-aware history shared by every MoE reasoning stage."""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional, Sequence

from ...prompts.moe_history import FACT_STATE_HISTORY_PREFIX, HISTORY_RULES

_HISTORY_RULES = HISTORY_RULES


def _compact_expert(expert: Dict[str, Any]) -> Dict[str, Any]:
    plan_steps = []
    for step in expert.get("plan_steps") or []:
        if not isinstance(step, dict):
            continue
        plan_steps.append({
            "tool_name": step.get("tool_name"),
            "arguments": step.get("arguments") or {},
            "reason": step.get("note") or step.get("reason") or "",
            "round": step.get("round"),
        })
    return {
        "expert": expert.get("expert"),
        "conclusion": expert.get("conclusion"),
        "evidence": expert.get("evidence") or [],
        "risks": expert.get("risks") or [],
        "confidence": expert.get("confidence"),
        "tools_used": expert.get("tools_used") or [],
        "tool_decisions": plan_steps,
    }


def build_fact_state_history(
    conversation_history: Optional[Sequence[Dict[str, str]]] = None,
    expert_context_history: Optional[Sequence[Dict[str, Any]]] = None,
) -> Optional[Dict[str, Any]]:
    """Build one structured history payload consumed by all MoE stages."""
    entries: List[Dict[str, Any]] = []
    for message in conversation_history or []:
        role = str(message.get("role") or "")
        content = str(message.get("content") or "").strip()
        if role not in {"user", "assistant"} or not content:
            continue
        entries.append({
            "state": "user_report" if role == "user" else "assistant_inference",
            "role": role,
            "role_label": (
                "user [用户原始陈述，仍需区分观察、转述与确诊信息]"
                if role == "user"
                else "assistant [未验证模型输出，不能作为患者事实]"
            ),
            "content": content,
        })

    expert_entries: List[Dict[str, Any]] = []
    for context in expert_context_history or []:
        if not isinstance(context, dict):
            continue
        experts = context.get("experts") or context.get("expert_opinions") or []
        expert_entries.append({
            "state": "expert_inference",
            "turn_index": context.get("turn_index"),
            "router": context.get("router"),
            "experts": [_compact_expert(item) for item in experts if isinstance(item, dict)],
            "critic": context.get("critic"),
        })

    if not entries and not expert_entries:
        return None
    return {
        "fact_state_rules": _HISTORY_RULES,
        "conversation": entries,
        "prior_subagent_context": expert_entries,
    }


def fact_state_history_text(
    conversation_history: Optional[Sequence[Dict[str, str]]] = None,
    expert_context_history: Optional[Sequence[Dict[str, Any]]] = None,
) -> str:
    payload = build_fact_state_history(conversation_history, expert_context_history)
    if payload is None:
        return ""
    return (
        FACT_STATE_HISTORY_PREFIX + json.dumps(payload, ensure_ascii=False, default=str)
    )
