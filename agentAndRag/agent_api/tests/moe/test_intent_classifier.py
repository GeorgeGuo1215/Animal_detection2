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
    build_intent_router_injection,
    intent_required_sections,
)
from app.services.moe.critic import CriticResult  # noqa: E402
from app.services.moe.intent_classifier import classify_intent, parse_intent_decision  # noqa: E402
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig  # noqa: E402
from app.services.moe.router import RouterDecision  # noqa: E402
from app.services.moe.trace import MoETrace  # noqa: E402


def _response(content: str) -> Dict[str, Any]:
    return {
        "choices": [{"message": {"content": content}, "finish_reason": "stop"}],
        "usage": {"prompt_tokens": 10, "completion_tokens": 5, "total_tokens": 15},
    }


class _IntentAwareLLM:
    model = "intent-test"

    def __init__(self, fixed_intent: str = "D3") -> None:
        self.fixed_intent = fixed_intent
        self.classifier_calls = 0
        self.final_calls = 0

    async def chat(self, *, messages: List[Dict[str, str]], **_: Any) -> Dict[str, Any]:
        if "任务分类器" in messages[0]["content"]:
            self.classifier_calls += 1
            payload = json.loads(messages[-1]["content"])
            current = str(payload.get("current_query") or "")
            intent = current[1:3] if current.startswith("[D") else self.fixed_intent
            await asyncio.sleep(0)
            return _response(json.dumps({
                "primary_intent": intent,
                "confidence": 0.93,
                "output_variant": "default",
                "reason": "test",
            }))
        self.final_calls += 1
        return _response("final answer")


class _FastOrchestrator(MoEOrchestrator):
    async def _route(self, query, recorder):
        return RouterDecision(
            scores={"clinical": 8.0}, raw_weights={"clinical": 1.0},
            weights={"clinical": 1.0}, selected_experts=["clinical"],
            emergency=False, out_of_scope=False, reason="test",
        )

    async def _run_experts(self, query, decision, recorder):
        return []

    async def _critique(self, query, opinions, emergency, recorder):
        return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")


class _BlockedD8Orchestrator(_FastOrchestrator):
    async def _critique(self, query, opinions, emergency, recorder):
        return CriticResult(
            verdict="block",
            issues=["requested action is unsafe"],
            constraints=["withhold unsafe instructions"],
            reason="safety boundary",
        )


def test_all_eight_contracts_have_router_and_aggregator_injections():
    for intent_id, spec in INTENT_SPECS.items():
        router = build_intent_router_injection(intent_id, 0.8)
        aggregator = build_intent_aggregator_injection(intent_id, 0.8)
        assert intent_id in router and spec.name in router
        assert intent_id in aggregator and spec.name in aggregator
        for section in intent_required_sections(intent_id):
            assert section in aggregator


def test_d1_and_d6_variants_are_data_driven():
    assert "S（主观）" in build_intent_aggregator_injection("D1", 1.0, "soap")
    assert "Problem List" in build_intent_aggregator_injection("D1", 1.0, "problem_list")
    assert "剂量依据" in build_intent_aggregator_injection("D6", 1.0, "dose")
    assert "指南组织" in build_intent_aggregator_injection("D6", 1.0, "guideline")


def test_classifier_records_decision_and_one_llm_call():
    llm = _IntentAwareLLM("D4")
    trace = MoETrace(question="interpret", user_role="veterinarian")
    decision = asyncio.run(classify_intent(query="解读这份CBC", llm=llm, recorder=trace))
    assert decision.intent_id == "D4"
    assert llm.classifier_calls == 1
    assert trace.intent_decision["intent_id"] == "D4"
    assert [call.stage for call in trace.llm_calls] == ["intent_classifier"]


def test_classifier_malformed_json_falls_back_to_d2():
    decision = parse_intent_decision("not-json")
    assert decision.intent_id == "D2"
    assert decision.fallback is True
    assert decision.error


def test_one_classifier_call_precedes_normal_moe_run_and_reaches_aggregator():
    llm = _IntentAwareLLM("D3")
    orch = _FastOrchestrator(
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
    )
    answer, _ = asyncio.run(orch.run(query="请规划检查路径"))
    assert answer == "final answer"
    assert llm.classifier_calls == 1
    assert llm.final_calls == 1
    assert orch.last_run_context["intent"]["intent_id"] == "D3"
    assert orch._active_intent_decision.intent_id == "D3"


def test_parallel_requests_do_not_share_intent_state():
    shared_llm = _IntentAwareLLM()

    async def run_one(intent_id: str) -> str:
        orch = _FastOrchestrator(
            llm=shared_llm,
            config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
        )
        await orch.run(query=f"[{intent_id}] concurrent case")
        return orch.last_run_context["intent"]["intent_id"]

    async def run_all() -> List[str]:
        return await asyncio.gather(*(run_one(f"D{i}") for i in range(1, 9)))

    assert asyncio.run(run_all()) == [f"D{i}" for i in range(1, 9)]
    assert shared_llm.classifier_calls == 8


def test_d8_critic_block_still_synthesizes_safe_contract_response():
    llm = _IntentAwareLLM("D8")
    trace = MoETrace(question="unsafe request", user_role="veterinarian")
    orch = _BlockedD8Orchestrator(
        llm=llm,
        config=OrchestratorConfig(user_role="veterinarian", allowed_tools=[]),
    )

    answer, trace = asyncio.run(orch.run(query="请给危险操作步骤", recorder=trace))

    assert answer == "final answer"
    assert llm.final_calls == 1
    assert trace is not None and trace.blocked is False
    assert orch.last_run_context["critic"]["verdict"] == "block"
