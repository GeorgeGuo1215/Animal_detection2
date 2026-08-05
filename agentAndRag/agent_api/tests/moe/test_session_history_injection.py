from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.critic import CriticResult, review
from app.services.moe.experts import EXPERTS, ExpertAgentSession
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig
from app.services.moe.router import RouterConfig, RouterDecision, _build_router_messages, route
from app.tools.tool_registry import ToolRegistry


class _CaptureLLM:
    model = "capture"

    def __init__(self):
        self.messages = []

    async def chat(self, messages=None, **kwargs):
        self.messages.append(messages or [])
        return {"choices": [{"message": {"content": json.dumps({
            "verdict": "pass", "issues": [], "constraints": [], "reason": "ok",
        }, ensure_ascii=False)}}]}


class _RouterLLM:
    model = "router-test"

    def __init__(self, scores):
        self.scores = scores

    async def chat(self, messages=None, **kwargs):
        return {"choices": [{"message": {"content": json.dumps({
            "scores": self.scores,
            "emergency": False,
            "reason": "模型低相关性判定",
        }, ensure_ascii=False)}}]}


def _history():
    return [
        {"role": "user", "content": "用户确认事实 SENTINEL_USER"},
        {"role": "assistant", "content": "模型推断 SENTINEL_ASSISTANT"},
    ]


def _expert_history():
    return [{
        "turn_index": 1,
        "router": {"selected_experts": ["clinical"]},
        "experts": [{
            "expert": "clinical",
            "conclusion": "专家推断 SENTINEL_EXPERT",
            "evidence": ["检索证据摘要 SENTINEL_EVIDENCE"],
            "risks": [],
            "tools_used": ["rag.search"],
            "plan_steps": [{"tool_name": "rag.search", "arguments": {"query": "canine cough"}}],
            "tool_results": [{"result": "RAW_RESULT_MUST_NOT_BE_REINJECTED"}],
        }],
        "critic": {"verdict": "pass"},
    }]


def _assert_history(content):
    assert "SENTINEL_USER" in content
    assert "SENTINEL_ASSISTANT" in content
    assert "SENTINEL_EXPERT" in content
    assert "SENTINEL_EVIDENCE" in content
    assert "canine cough" in content
    assert "user_report" in content
    assert "assistant_inference" in content
    assert "expert_inference" in content
    assert "RAW_RESULT_MUST_NOT_BE_REINJECTED" not in content


def test_fact_state_history_reaches_router_expert_critic_and_aggregator():
    history = _history()
    expert_history = _expert_history()

    router_messages = _build_router_messages(
        "当前问题", "pet_owner", conversation_history=history,
        expert_context_history=expert_history,
    )
    _assert_history(router_messages[-1]["content"])

    expert = ExpertAgentSession(
        expert=EXPERTS["clinical"], query="当前问题", weight=1.0,
        registry=ToolRegistry(), llm=_CaptureLLM(),
        conversation_history=history, expert_context_history=expert_history,
    )
    _assert_history(expert.messages[-1]["content"])

    critic_llm = _CaptureLLM()
    asyncio.run(review(
        query="当前问题", expert_opinions=[], emergency=False, llm=critic_llm,
        conversation_history=history, expert_context_history=expert_history,
    ))
    _assert_history(critic_llm.messages[0][-1]["content"])

    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner"))
    synthesis = orch._build_synthesis_messages(
        query="当前问题",
        opinions=[],
        critic=CriticResult(verdict="pass"),
        decision=RouterDecision(
            scores={"clinical": 8}, raw_weights={"clinical": 1.0},
            weights={"clinical": 1.0}, selected_experts=["clinical"],
            emergency=False, out_of_scope=False, reason="test",
        ),
        conversation_history=history,
        expert_context_history=expert_history,
    )
    _assert_history(synthesis[-1]["content"])


def test_cross_session_memory_reaches_router_expert_critic_without_local_history():
    memory = "【近期对话】\n- 用户: 我的狗叫球鼠，昨晚呕吐\n  助手: SOAP摘要"

    router_messages = _build_router_messages(
        "你还记得球鼠吗",
        "pet_owner",
        user_memory=memory,
    )
    router_payload = router_messages[-1]["content"]
    assert "cross_session_memory" in router_payload
    assert "球鼠" in router_payload
    assert "user_memory" in router_payload

    expert = ExpertAgentSession(
        expert=EXPERTS["clinical"],
        query="你还记得球鼠吗",
        weight=1.0,
        registry=ToolRegistry(),
        llm=_CaptureLLM(),
        user_memory=memory,
    )
    assert "球鼠" in expert.messages[-1]["content"]
    assert "cross_session_memory" in expert.messages[-1]["content"]

    critic_llm = _CaptureLLM()
    asyncio.run(review(
        query="你还记得球鼠吗",
        expert_opinions=[],
        emergency=False,
        llm=critic_llm,
        user_memory=memory,
    ))
    assert "球鼠" in critic_llm.messages[0][-1]["content"]


def test_router_prompt_defines_history_aware_scope_without_upgrading_facts():
    messages = _build_router_messages(
        "这个为什么？", "pet_owner", conversation_history=_history(),
        expert_context_history=_expert_history(),
    )
    system_prompt = messages[0]["content"]

    assert "结合 history_context 与当前问题" in system_prompt
    assert "证据来源与指南" in system_prompt
    assert "不能永久放行" in system_prompt
    assert "不代表诊断或其他事实已经被用户确认" in system_prompt


def test_router_preserves_short_contextual_veterinary_followup():
    async def scenario():
        decision = await route(
            query="这个为什么？",
            user_role="pet_owner",
            llm=_RouterLLM({key: 1 for key in EXPERTS}),
            config=RouterConfig(min_relevance=3.0),
            conversation_history=_history(),
            expert_context_history=_expert_history(),
        )

        assert not decision.out_of_scope
        assert "clinical" in decision.selected_experts
        assert decision.scores["clinical"] == 6.0
        assert "上下文追问" in decision.reason

    asyncio.run(scenario())


def test_router_rejects_explicit_unrelated_topic_shift_despite_history():
    async def scenario():
        decision = await route(
            query="给我写一个Python排序算法，这个要详细说明",
            user_role="pet_owner",
            llm=_RouterLLM({key: 1 for key in EXPERTS}),
            config=RouterConfig(min_relevance=3.0),
            conversation_history=_history(),
            expert_context_history=_expert_history(),
        )

        assert decision.out_of_scope
        assert decision.selected_experts == []

    asyncio.run(scenario())


def test_router_does_not_use_history_without_contextual_followup_signal():
    async def scenario():
        decision = await route(
            query="介绍一下今天的国际新闻",
            user_role="pet_owner",
            llm=_RouterLLM({key: 1 for key in EXPERTS}),
            config=RouterConfig(min_relevance=3.0),
            conversation_history=_history(),
            expert_context_history=_expert_history(),
        )

        assert decision.out_of_scope

    asyncio.run(scenario())


def test_router_keeps_memory_recall_in_scope_with_cross_session_memory():
    async def scenario():
        decision = await route(
            query="你还记得我的宠物叫什么吗",
            user_role="pet_owner",
            llm=_RouterLLM({key: 1 for key in EXPERTS}),
            config=RouterConfig(min_relevance=3.0),
            user_memory="【近期对话】\n- 用户: 我的狗叫球鼠",
        )

        assert not decision.out_of_scope
        assert "clinical" in decision.selected_experts
        assert "跨会话用户记忆" in decision.reason

    asyncio.run(scenario())
