"""边界审核专家 Critic：召回前对专家草案做安全与边界校验。

审核维度：责任边界 / 事实一致性 / 用药安全 / 数据一致性 / 合规与免责。
输出 verdict：
- pass   : 草案安全，可直接合成；
- revise : 存在可修补问题，把 constraints 注入最终生成 prompt（强制免责/降确定性/标红用药）；
- block  : 命中硬性安全红线，一票否决 → 编排层走安全兜底话术。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...prompts.moe_critic import (
    CRITIC_SYS_OWNER,
    CRITIC_SYS_VET,
    OWNER_FALLBACK_CONSTRAINT,
    VET_FALLBACK_CONSTRAINT,
)
from ..structured_output import safe_json_loads
from .trace import MoETrace, extract_usage
from .history_context import build_fact_state_history


_CRITIC_SYS_OWNER = CRITIC_SYS_OWNER
_CRITIC_SYS_VET = CRITIC_SYS_VET
_OWNER_FALLBACK_CONSTRAINT = OWNER_FALLBACK_CONSTRAINT
_VET_FALLBACK_CONSTRAINT = VET_FALLBACK_CONSTRAINT


@dataclass
class CriticResult:
    verdict: str = "pass"
    issues: List[str] = field(default_factory=list)
    constraints: List[str] = field(default_factory=list)
    reason: str = ""

    @property
    def blocked(self) -> bool:
        return self.verdict == "block"


async def review(
    *,
    query: str,
    expert_opinions: List[Dict[str, Any]],
    emergency: bool,
    llm: AsyncOpenAIClient,
    user_role: str = "pet_owner",
    conversation_history: Optional[List[Dict[str, str]]] = None,
    expert_context_history: Optional[List[Dict[str, Any]]] = None,
    user_memory: Optional[str] = None,
    recorder: Optional[MoETrace] = None,
) -> CriticResult:
    opinions_brief = [
        {
            "expert": o.get("name_zh") or o.get("expert"),
            "weight": o.get("weight"),
            "confidence": o.get("confidence"),
            "conclusion": o.get("conclusion"),
            "risks": o.get("risks"),
        }
        for o in expert_opinions
    ]
    payload = {
        "user_question": query,
        "user_role": user_role,
        "emergency": emergency,
        "expert_opinions": opinions_brief,
    }
    history_context = build_fact_state_history(
        conversation_history,
        expert_context_history,
        user_memory=user_memory,
    )
    if history_context:
        payload["history_context"] = history_context
    user_payload = json.dumps(payload, ensure_ascii=False)
    critic_sys = _CRITIC_SYS_VET if user_role == "veterinarian" else _CRITIC_SYS_OWNER
    messages = [
        {"role": "system", "content": critic_sys},
        {"role": "user", "content": user_payload},
    ]

    t0 = time.perf_counter()
    try:
        resp = await llm.chat(
            messages=messages,
            temperature=0.1,
            max_tokens=400,
            response_format={"type": "json_object"},
            thinking=False,
        )
        latency = (time.perf_counter() - t0) * 1000.0
        text = extract_text(resp)
        if recorder is not None:
            recorder.record_llm(
                stage="critic",
                model=getattr(llm, "model", ""),
                messages=messages,
                output=text,
                latency_ms=latency,
                usage=extract_usage(resp),
            )
        obj, parse_error = safe_json_loads(text)
        fallback = _VET_FALLBACK_CONSTRAINT if user_role == "veterinarian" else _OWNER_FALLBACK_CONSTRAINT
        result = CriticResult(
            verdict="revise",
            issues=[f"审核输出无法解析：{parse_error or 'missing JSON object'}"],
            constraints=[fallback],
            reason="critic 输出解析失败的安全回退",
        )
        if isinstance(obj, dict):
            verdict = str(obj.get("verdict") or "revise").strip().lower()
            if verdict not in ("pass", "revise", "block"):
                verdict = "revise"
            result = CriticResult(verdict=verdict)
            iss = obj.get("issues")
            result.issues = [str(x) for x in iss] if isinstance(iss, list) else ([str(iss)] if iss else [])
            cons = obj.get("constraints")
            result.constraints = [str(x) for x in cons] if isinstance(cons, list) else ([str(cons)] if cons else [])
            result.reason = str(obj.get("reason") or "")
            if verdict == "revise" and not result.constraints:
                result.constraints = [fallback]
    except Exception as exc:  # noqa: BLE001
        latency = (time.perf_counter() - t0) * 1000.0
        if recorder is not None:
            recorder.record_llm(
                stage="critic",
                model=getattr(llm, "model", ""),
                messages=messages,
                output=f"[error] {exc}",
                latency_ms=latency,
                meta={"error": str(exc)},
            )
        fallback = _VET_FALLBACK_CONSTRAINT if user_role == "veterinarian" else _OWNER_FALLBACK_CONSTRAINT
        result = CriticResult(
            verdict="revise",
            issues=[f"审核 LLM 失败：{exc}"],
            constraints=[fallback],
            reason="critic 调用失败的安全回退",
        )

    if recorder is not None:
        recorder.critic_result = {
            "verdict": result.verdict,
            "issues": result.issues,
            "constraints": result.constraints,
            "reason": result.reason,
        }
    return result
