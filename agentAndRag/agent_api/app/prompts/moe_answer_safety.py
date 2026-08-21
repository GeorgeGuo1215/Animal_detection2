"""MoE 终答的条件式证据安全编辑提示词。"""
from __future__ import annotations

import json
from typing import Any, Dict, List


ANSWER_SAFETY_SYSTEM_PROMPT = (
    "你是兽医 Agent 的最终证据安全编辑器，不重新诊断、不重新检索、不增加新事实。"
    "你的唯一任务是修订给定 Markdown 终答，使其严格服从 evidence_audit 和用户限制。\n"
    "保留原有 D1-D8 输出分节、顺序、患者已提供的数值事实、已经充分支持的非数值结论和引用。\n"
    "supported 可支撑对应目标；partial 只能支撑 matched 片段中的定性部分，不能支撑具体处方剂量、"
    "固定洗脱/减量时长、数值阈值或固定监测频率；unsupported/unknown 完全不可引用。\n"
    "删除不满足上述条件的剂量、天数、频率、阈值及其引用，并改写为条件性原则或明确无法核实。"
    "用户明确要求不提供的固定值必须完全删除，不能以‘经验参考’‘通常建议’或免责声明绕过。\n"
    "Critic 和专家意见不是证据；不得因为它们要求补数值就保留未核实数值。"
    "只输出修订后的完整 Markdown 正文，不要 JSON、代码围栏、解释、审计过程或思维链。"
)


def build_answer_safety_messages(
    *,
    query: str,
    answer: str,
    evidence_audit: List[Dict[str, Any]],
    intent_contract: str,
    user_role: str,
    forbidden_claims: List[str] | None = None,
) -> List[Dict[str, str]]:
    """组装终答条件式安全编辑消息。"""
    payload = {
        "query": str(query or ""),
        "user_role": str(user_role or ""),
        "intent_output_contract": str(intent_contract or ""),
        "evidence_audit": evidence_audit,
        "forbidden_claims": list(forbidden_claims or []),
        "draft_answer": str(answer or ""),
    }
    return [
        {"role": "system", "content": ANSWER_SAFETY_SYSTEM_PROMPT},
        {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
    ]


__all__ = ["ANSWER_SAFETY_SYSTEM_PROMPT", "build_answer_safety_messages"]
