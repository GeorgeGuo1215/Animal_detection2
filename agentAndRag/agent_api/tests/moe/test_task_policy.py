from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.retrieval_policy import (  # noqa: E402
    EvidenceTask,
    assign_evidence_tasks,
    resolve_retrieval_requirement,
)
from app.services.moe.task_policy import (  # noqa: E402
    decide_task_policy,
    parse_task_policy,
)
from app.services.moe.trace import MoETrace  # noqa: E402
from app.prompts.moe_task_policy import TASK_POLICY_SYSTEM_PROMPT  # noqa: E402


def _payload(**overrides):
    value = {
        "primary_intent": "D5",
        "secondary_intents": ["D2"],
        "confidence": 0.94,
        "output_variant": "default",
        "scores": {"clinical": 8.0, "pharmacy": 9.0, "nutrition": 0.0, "behavior": 0.0},
        "emergency": {"value": False, "confidence": 0.9, "evidence": []},
        "evidence_tasks": [
            {
                "capability": "medication_reference",
                "owner": "pharmacy",
                "requirement": "required",
                "reason": "需要核验具体用药安全",
                "web_fallback_on_weak_local": True,
            },
            {
                "capability": "current_web",
                "owner": "clinical",
                "requirement": "recommended",
                "reason": "近期证据可提高质量",
                "web_fallback_on_weak_local": False,
            },
        ],
        "missing_information": ["肾功能"],
        "reason": "治疗决策并包含鉴别",
    }
    value.update(overrides)
    return value


def test_semantic_policy_parses_intent_route_and_evidence_tasks():
    decision = parse_task_policy(json.dumps(_payload(), ensure_ascii=False))

    assert decision.primary_intent == "D5"
    assert decision.secondary_intents == ("D2",)
    assert decision.scores["pharmacy"] == 9.0
    assert decision.evidence_tasks[0].capability == "medication_reference"
    assert decision.evidence_tasks[0].web_fallback_on_weak_local is True
    assert decision.missing_information == ("肾功能",)


def test_invalid_policy_falls_back_without_mandatory_retrieval():
    decision = parse_task_policy("not-json")
    requirement = resolve_retrieval_requirement(
        expert_key="clinical",
        evidence_tasks=decision.evidence_tasks,
    )

    assert decision.fallback is True
    assert decision.primary_intent == "D2"
    assert requirement.required_tools == ()
    assert requirement.recommended_tools == ("rag.search",)


def test_evidence_task_has_one_active_owner_and_is_not_duplicated():
    tasks = assign_evidence_tasks(
        [EvidenceTask("medication_reference", "pharmacy", "required", "drug safety")],
        ["clinical", "pharmacy"],
    )
    clinical = resolve_retrieval_requirement(expert_key="clinical", evidence_tasks=tasks)
    pharmacy = resolve_retrieval_requirement(expert_key="pharmacy", evidence_tasks=tasks)

    assert clinical.required_tools == ()
    assert pharmacy.required_tools == ("rag.search",)


def test_missing_owner_is_reassigned_to_an_active_expert():
    tasks = assign_evidence_tasks(
        [EvidenceTask("medication_reference", "pharmacy", "required", "drug safety")],
        ["clinical"],
    )
    assert tasks[0].owner == "clinical"
    assert resolve_retrieval_requirement(
        expert_key="clinical", evidence_tasks=tasks
    ).required_tools == ("rag.search",)


def test_d1_can_still_require_evidence_when_semantic_policy_requests_it():
    policy = _payload(
        primary_intent="D1",
        secondary_intents=["D6"],
        output_variant="soap",
        evidence_tasks=[{
            "capability": "current_web",
            "owner": "clinical",
            "requirement": "required",
            "reason": "用户要求核对当前指南来源",
        }],
    )
    decision = parse_task_policy(json.dumps(policy, ensure_ascii=False))
    requirement = resolve_retrieval_requirement(
        expert_key="clinical",
        evidence_tasks=decision.evidence_tasks,
    )
    assert decision.primary_intent == "D1"
    assert decision.output_variant == "soap"
    assert requirement.required_tools == ("mcp.web_search.web_search",)


class _PolicyLLM:
    model = "policy-test"

    async def chat(self, **kwargs):
        return {
            "choices": [{"message": {"content": json.dumps(_payload(), ensure_ascii=False)}}],
            "usage": {"prompt_tokens": 20, "completion_tokens": 10, "total_tokens": 30},
        }


def test_policy_call_records_one_structured_trace_stage():
    trace = MoETrace(question="case", user_role="veterinarian")
    decision = asyncio.run(decide_task_policy(
        query="请制定治疗并核验药物安全",
        user_role="veterinarian",
        llm=_PolicyLLM(),
        recorder=trace,
    ))

    assert decision.primary_intent == "D5"
    assert [call.stage for call in trace.llm_calls] == ["task_policy"]
    assert trace.task_policy_decision["evidence_tasks"][0]["owner"] == "pharmacy"


def test_unified_prompt_contains_all_intent_boundaries_and_routing_guidance():
    for intent_id in (f"D{i}" for i in range(1, 9)):
        assert intent_id in TASK_POLICY_SYSTEM_PROMPT
    for boundary in (
        "D3 规划尚未完成的检查；D4 解读已经给出的具体报告",
        "D5 面向具体患者制定治疗、用药与监测决策；D6 是脱离具体患者决策",
        "D7 必须依赖既往病例状态",
        "普通病例存在高风险鉴别不自动改为 D8",
    ):
        assert boundary in TASK_POLICY_SYSTEM_PROMPT
    for guidance in (
        "临床专家为主，关注事实抽取",
        "临床与药理专家优先",
        "优先保留上一轮相关专家",
        "临床专家负责风险分级",
    ):
        assert guidance in TASK_POLICY_SYSTEM_PROMPT
