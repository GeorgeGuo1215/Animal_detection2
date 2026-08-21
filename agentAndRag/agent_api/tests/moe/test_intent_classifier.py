from __future__ import annotations

import asyncio
import json
import os
import sys
from typing import Any, Dict, List

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.prompts.intent_contracts import (  # noqa: E402
    INTENT_SPECS,
    build_intent_aggregator_injection,
    intent_required_sections,
)
from app.services.moe.critic import CriticResult  # noqa: E402
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig  # noqa: E402
from app.services.moe.trace import MoETrace  # noqa: E402


def _response(content: str) -> Dict[str, Any]:
    """构造测试用的模型响应。"""
    return {
        "choices": [{"message": {"content": content}, "finish_reason": "stop"}],
        "usage": {"prompt_tokens": 10, "completion_tokens": 5, "total_tokens": 15},
    }


class _IntentAwareLLM:
    model = "intent-test"

    def __init__(self, fixed_intent: str = "D3") -> None:
        """初始化该测试替身。"""
        self.fixed_intent = fixed_intent
        self.policy_calls = 0
        self.final_calls = 0

    async def chat(self, *, messages: List[Dict[str, str]], **_: Any) -> Dict[str, Any]:
        """测试用假 LLM 聊天实现。"""
        if "统一任务策略分类器" in messages[0]["content"]:
            self.policy_calls += 1
            payload = json.loads(messages[-1]["content"])
            current = str(payload.get("current_query") or "")
            intent = current[1:3] if current.startswith("[D") else self.fixed_intent
            await asyncio.sleep(0)
            return _response(json.dumps({
                "primary_intent": intent,
                "secondary_intents": [],
                "confidence": 0.93,
                "output_variant": "default",
                "scores": {"clinical": 8, "pharmacy": 0, "nutrition": 0, "behavior": 0},
                "emergency": {"value": False, "confidence": 0.9, "evidence": []},
                "evidence_tasks": [],
                "missing_information": [],
                "reason": "test",
            }))
        self.final_calls += 1
        return _response("final answer")


class _FastOrchestrator(MoEOrchestrator):
    async def _run_experts(self, query, decision, recorder):
        """驱动专家执行路径的测试替身。"""
        return []

    async def _critique(self, query, opinions, emergency, recorder):
        """测试用审核器替身。"""
        return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")


class _BlockedOrchestrator(_FastOrchestrator):
    async def _critique(self, query, opinions, emergency, recorder):
        """测试用审核器替身。"""
        return CriticResult(
            verdict="block",
            issues=["requested action is unsafe"],
            constraints=["withhold unsafe instructions"],
            reason="safety boundary",
        )


def test_all_eight_contracts_have_aggregator_injections():
    """验证全部八种契约都向综合器注入了对应约束。"""
    for intent_id, spec in INTENT_SPECS.items():
        aggregator = build_intent_aggregator_injection(intent_id, 0.8)
        assert intent_id in aggregator and spec.name in aggregator
        for section in intent_required_sections(intent_id):
            assert section in aggregator


def test_d1_and_d6_variants_are_data_driven():
    """验证 D1 与 D6 输出变体由数据驱动。"""
    assert "S（主观）" in build_intent_aggregator_injection("D1", 1.0, "soap")
    assert "Problem List" in build_intent_aggregator_injection("D1", 1.0, "problem_list")
    assert "剂量依据" in build_intent_aggregator_injection("D6", 1.0, "dose")
    assert "指南组织" in build_intent_aggregator_injection("D6", 1.0, "guideline")


def test_one_unified_policy_call_precedes_normal_moe_run_and_reaches_aggregator():
    """验证统一策略会先于正常 MoE 运行，并到达综合器。"""
    llm = _IntentAwareLLM("D3")
    orch = _FastOrchestrator(
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
    )
    answer, _ = asyncio.run(orch.run(query="请规划检查路径"))
    assert answer == "final answer"
    assert llm.policy_calls == 1
    assert llm.final_calls == 1
    assert orch.last_run_context["intent"]["intent_id"] == "D3"
    assert orch._active_intent_decision.intent_id == "D3"


def test_parallel_requests_do_not_share_intent_state():
    """验证并行请求不会共享意图状态。"""
    shared_llm = _IntentAwareLLM()

    async def run_one(intent_id: str) -> str:
        """只跑单条用例。"""
        orch = _FastOrchestrator(
            llm=shared_llm,
            config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
        )
        await orch.run(query=f"[{intent_id}] concurrent case")
        return orch.last_run_context["intent"]["intent_id"]

    async def run_all() -> List[str]:
        """跑完全部用例或批次。"""
        return await asyncio.gather(*(run_one(f"D{i}") for i in range(1, 9)))

    assert asyncio.run(run_all()) == [f"D{i}" for i in range(1, 9)]
    assert shared_llm.policy_calls == 8


def test_d8_critic_block_still_synthesizes_safe_contract_response():
    """验证 D8 被审核阻断后仍按安全契约综合答复。"""
    llm = _IntentAwareLLM("D8")
    trace = MoETrace(question="unsafe request", user_role="veterinarian")
    orch = _BlockedOrchestrator(
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
    )

    answer, trace = asyncio.run(orch.run(query="请给危险操作步骤", recorder=trace))

    assert answer == "final answer"
    assert llm.final_calls == 1
    assert trace is not None and trace.blocked is False
    assert orch.last_run_context["critic"]["verdict"] == "block"


def test_d5_critic_block_still_synthesizes_treatment_contract_response():
    """验证 D5 被审核阻断后仍按治疗契约综合答复。"""
    llm = _IntentAwareLLM("D5")
    trace = MoETrace(question="unsafe medication request", user_role="veterinarian")
    orch = _BlockedOrchestrator(
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
    )

    answer, trace = asyncio.run(orch.run(query="请核对高风险药物组合", recorder=trace))

    assert answer == "final answer"
    assert llm.final_calls == 1
    assert trace is not None and trace.blocked is False
    assert orch.last_run_context["critic"]["verdict"] == "block"
