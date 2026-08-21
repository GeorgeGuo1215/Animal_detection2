from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.prompts.moe_answer_safety import build_answer_safety_messages
from app.services.moe.orchestrator import (
    MoEOrchestrator,
    OrchestratorConfig,
    _answer_requires_evidence_repair,
    _explicit_forbidden_claims,
    _remove_forbidden_claims,
)
from app.tools.tool_registry import ToolRegistry


def _audit(status: str):
    return [{"expert": "pharmacy", "goal": "核对洗脱期", "status": status}]


def test_repair_gate_only_triggers_for_weak_evidence_and_high_risk_specifics():
    """验证普通患者数值不误触发，弱证据下的剂量/固定时长才触发。"""
    assert _answer_requires_evidence_repair("患犬已连续使用14天。", _audit("partial")) is False
    assert _answer_requires_evidence_repair("洗脱期通常为7天。", _audit("partial")) is True
    assert _answer_requires_evidence_repair("美洛昔康0.2 mg/kg。", _audit("unknown")) is True
    assert _answer_requires_evidence_repair("洗脱期通常为7天。", _audit("supported")) is False


def test_safety_prompt_preserves_intent_contract_and_forbids_partial_numeric_claims():
    """验证安全编辑继续服从 D1-D8 输出契约，且不把 partial 当作数值证据。"""
    messages = build_answer_safety_messages(
        query="不要给固定洗脱天数",
        answer="**方案分层**\n洗脱期约7天。",
        evidence_audit=_audit("partial"),
        intent_contract="D5 分节：治疗目标、方案分层、用药核对、监测与复查、禁忌与红旗",
        user_role="veterinarian",
        forbidden_claims=["洗脱期约7天。"],
    )
    serialized = str(messages)
    assert "保留原有 D1-D8 输出分节" in serialized
    assert "partial" in serialized and "不能支撑具体处方剂量" in serialized
    assert "D5 分节" in serialized
    assert "forbidden_claims" in serialized and "7天" in serialized


def test_explicit_no_fixed_values_preserves_patient_history_but_flags_new_recommendations():
    """用户病例中的14天不是违规值，模型新增的24小时和5天会进入禁用清单。"""
    query = "犬已连续使用泼尼松龙14天；请勿猜固定洗脱天数。"
    answer = (
        "患犬已连续使用14天。"
        "两药不得在24小时内联用。"
        "洗脱期约5天。"
    )
    claims = _explicit_forbidden_claims(query, answer)
    joined = " ".join(claims)
    assert "14天" not in joined
    assert "24小时" in joined
    assert "5天" in joined


def test_deterministic_safety_guard_removes_residual_forbidden_values():
    """即使安全编辑模型仍复述禁用数值，程序级末端守卫也会移除对应句。"""
    answer = "**方案分层**\n两药不得在24小时内联用。洗脱期约5天。应个体化评估。"
    revised = _remove_forbidden_claims(
        answer,
        ["两药不得在24小时内联用。", "洗脱期约5天。"],
    )
    assert "24小时" not in revised
    assert "5天" not in revised
    assert "应个体化评估" in revised


def test_deterministic_safety_guard_preserves_markdown_line_breaks():
    """删去违规句后仍保留 D5 标题、段落和列表换行。"""
    answer = (
        "**治疗目标**  \n先完成个体评估。\n\n"
        "**方案分层**  \n- 两药不得在24小时内联用。\n"
        "- 洗脱期须个体化。\n\n"
        "**用药核对**  \n不提供固定剂量。"
    )
    revised = _remove_forbidden_claims(answer, ["两药不得在24小时内联用。"])
    assert "24小时" not in revised
    assert "**治疗目标**  \n" in revised
    assert "\n\n**方案分层**  \n" in revised
    assert "\n- 洗脱期须个体化" in revised
    assert "\n\n**用药核对**  \n" in revised


class _RepairLLM:
    model = "fake"

    async def chat(self, **kwargs):
        self.messages = kwargs["messages"]
        return {
            "choices": [{
                "message": {"content": "**方案分层**\n洗脱期无法给出固定天数，须个体化核定。"},
                "finish_reason": "stop",
            }],
            "usage": {},
        }


def test_conditional_repair_replaces_numeric_draft_once():
    """验证有界安全编辑会替换弱证据数值草案，而不会启动检索或专家循环。"""
    llm = _RepairLLM()
    orchestrator = MoEOrchestrator(
        registry=ToolRegistry(),
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", max_tokens=500),
    )
    opinions = [{
        "expert": "pharmacy",
        "tool_results": [{
            "tool_name": "rag.search",
            "arguments": {"query": "canine washout"},
            "evidence_goal": "核对洗脱期",
            "scope": "expert",
            "sufficiency": {"status": "partial", "reason": "未支持固定天数"},
        }],
    }]

    answer, applied = asyncio.run(orchestrator._repair_answer_if_needed(
        query="不要给固定洗脱天数",
        answer="**方案分层**\n洗脱期通常为7天。",
        opinions=opinions,
        recorder=None,
    ))

    assert applied is True
    assert "7天" not in answer
    assert "无法给出固定天数" in answer
    assert len(llm.messages) == 2
