from __future__ import annotations

import asyncio
import json
import os
import sys
from copy import deepcopy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.prompts.moe import build_pethealth_vitals_injection
from app.prompts.moe_task_policy import build_task_policy_messages
from app.schemas.openai_schemas import ChatCompletionRequest
from app.services.moe.critic import CriticResult
from app.services.moe.experts import EXPERTS, ExpertAgentSession
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig
from app.services.moe.router import RouterDecision
from app.tools.tool_registry import ToolRegistry, ToolSpec


def _decision() -> RouterDecision:
    return RouterDecision(
        scores={"clinical": 8.0},
        raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0},
        selected_experts=["clinical"],
        emergency=False,
        out_of_scope=False,
        reason="test",
    )


class _FinalLLM:
    model = "fake"

    def __init__(self) -> None:
        self.messages = []

    async def chat(self, messages=None, **kwargs):
        self.messages.append(deepcopy(messages or []))
        return {"choices": [{"message": {"content": "final answer"}, "finish_reason": "stop"}]}


class _NoopLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        return {
            "choices": [{
                "message": {
                    "content": json.dumps({
                        "action": "final",
                        "opinion": {
                            "conclusion": "ok",
                            "evidence": [],
                            "risks": [],
                            "confidence": 0.5,
                        },
                    })
                }
            }]
        }


def test_schema_accepts_pethealth_server_context_and_keeps_legacy_default():
    legacy = ChatCompletionRequest(messages=[{"role": "user", "content": "hi"}])
    assert legacy.pethealth_server is None

    req = ChatCompletionRequest(
        messages=[{"role": "user", "content": "hi"}],
        pethealth_server={
            "animal_id": "pet_1",
            "heart_rate_abnormal": True,
            "vitals_window_hours": 6,
        },
    )

    assert req.pethealth_server is not None
    assert req.pethealth_server.animal_id == "pet_1"
    assert req.pethealth_server.heart_rate_abnormal is True
    assert req.pethealth_server.vitals_window_hours == 6


def test_pethealth_injection_requires_abnormal_flag_and_animal_id():
    assert build_pethealth_vitals_injection(
        animal_id="pet_1", heart_rate_abnormal=False, stage="router"
    ) == ""
    assert build_pethealth_vitals_injection(
        animal_id="", heart_rate_abnormal=True, stage="router"
    ) == ""

    text = build_pethealth_vitals_injection(
        animal_id="pet_1",
        heart_rate_abnormal=True,
        vitals_window_hours=6,
        stage="aggregator",
    )

    assert "PetHealth_Server 外部体征提示注入" in text
    assert "pet_1" in text
    assert '"hours":6' in text
    assert "不是诊断、不是病史确认" in text
    assert "pethealth_vitals_result" in text
    assert "禁止编造心率数值" in text


def test_task_policy_messages_include_pethealth_injection():
    injection = build_pethealth_vitals_injection(
        animal_id="pet_1",
        heart_rate_abnormal=True,
        vitals_window_hours=24,
        stage="router",
    )
    messages = build_task_policy_messages(
        query="它今天怎么样？", user_role="pet_owner", prompt_injection=injection
    )
    system_prompt = messages[0]["content"]

    assert "PetHealth_Server 外部体征提示注入" in system_prompt
    assert "优先考虑临床专家参与" in system_prompt
    assert "不得只因外部 flag 就判定急症" in system_prompt


def test_pethealth_injection_does_not_enter_expert_prompt():
    registry = ToolRegistry()
    registry.register(
        ToolSpec(
            "mcp.vitals_alert.check_vitals",
            "check vitals",
            {"type": "object"},
            lambda **kwargs: {"status": "OK"},
        )
    )
    session = ExpertAgentSession(
        expert=EXPERTS["clinical"],
        query="它今天怎么样？",
        weight=1.0,
        registry=registry,
        llm=_NoopLLM(),
        request_allowed_tools=["mcp.vitals_alert.check_vitals"],
    )

    assert "PetHealth_Server 外部体征提示注入" not in session.messages[0]["content"]


def test_aggregator_prompt_and_payload_include_pethealth_vitals_result():
    orch = MoEOrchestrator(
        config=OrchestratorConfig(
            user_role="pet_owner",
            pethealth_server={
                "animal_id": "pet_1",
                "heart_rate_abnormal": True,
                "vitals_window_hours": 12,
            },
        )
    )
    messages = orch._build_synthesis_messages(
        query="它今天心率是不是危险？",
        opinions=[],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=_decision(),
        pethealth_vitals_result={
            "tool_name": "mcp.vitals_alert.check_vitals",
            "arguments": {"pet_id": "pet_1", "hours": 12},
            "ok": True,
            "result": {"status": "OK", "alert_level": "alert", "heart_rate": {"avg": 180}},
        },
    )

    assert "PetHealth_Server 外部体征提示注入" in messages[0]["content"]
    assert "终答必须结合本次 MoE payload 中的 `pethealth_vitals_result`" in messages[0]["content"]
    assert "外部异常 flag 只能作为触发核实和提醒的原因" in messages[0]["content"]
    assert '"pethealth_server"' in messages[1]["content"]
    assert '"pethealth_vitals_result"' in messages[1]["content"]
    assert '"alert_level": "alert"' in messages[1]["content"]


def test_moe_pethealth_check_uses_pethealth_animal_id_as_pet_id():
    calls = []
    registry = ToolRegistry()

    async def check_vitals(**kwargs):
        calls.append(dict(kwargs))
        return {"status": "OK", "alert_level": "alert", "heart_rate": {"avg": 188}}

    registry.register(
        ToolSpec(
            "mcp.vitals_alert.check_vitals",
            "check vitals",
            {"type": "object"},
            check_vitals,
        )
    )

    class _PetHealthOnlyOrchestrator(MoEOrchestrator):
        async def _run_experts(self, query, decision, recorder):
            return []

        async def _critique(self, query, opinions, emergency, recorder):
            return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")

    llm = _FinalLLM()
    orch = _PetHealthOnlyOrchestrator(
        registry=registry,
        llm=llm,
        config=OrchestratorConfig(
            allowed_tools=["mcp.vitals_alert.check_vitals"],
            pethealth_server={
                "animal_id": "pet_from_pethealth",
                "heart_rate_abnormal": True,
                "vitals_window_hours": 6,
            },
        ),
    )

    answer, _ = asyncio.run(orch.run(query="它今天还好吗？"))

    assert answer == "final answer"
    assert calls == [{"pet_id": "pet_from_pethealth", "hours": 6}]
    assert orch.last_run_context["pethealth_vitals"]["result"]["alert_level"] == "alert"
    assert '"pethealth_vitals_result"' in llm.messages[-1][1]["content"]
