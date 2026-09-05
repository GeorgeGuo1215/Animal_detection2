from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.prompts.moe_task_policy import TASK_POLICY_SYSTEM_PROMPT
from agent_api.app.services.moe.critic import CriticResult, review
from agent_api.app.services.moe.expert_runtime.service import EXPERTS, ExpertAgentSession
from agent_api.app.services.moe.orchestration.service import MoEOrchestrator, OrchestratorConfig
from agent_api.app.services.moe.router import RouterDecision
from agent_api.app.services.moe.task_policy import decide_task_policy
from agent_api.app.tools.tool_registry import ToolRegistry


def _policy_response() -> dict:
    """构造任务策略模型的假响应。"""
    return {
        "primary_intent": "D7",
        "secondary_intents": [],
        "confidence": 0.9,
        "output_variant": "default",
        "scores": {"clinical": 8, "pharmacy": 0, "nutrition": 0, "behavior": 0},
        "emergency": {"value": False, "confidence": 0.9, "evidence": []},
        "evidence_tasks": [],
        "missing_information": [],
        "reason": "history-aware follow-up",
    }


class _CaptureLLM:
    model = "capture"

    def __init__(self, payload=None):
        """初始化该测试替身。"""
        self.messages = []
        self.payload = payload

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        self.messages.append(messages or [])
        content = self.payload or {
            "verdict": "pass", "issues": [], "constraints": [], "reason": "ok",
        }
        return {"choices": [{"message": {"content": json.dumps(content, ensure_ascii=False)}}]}


def _history():
    """构造会话历史消息。"""
    return [
        {"role": "user", "content": "用户确认事实 SENTINEL_USER"},
        {"role": "assistant", "content": "模型推断 SENTINEL_ASSISTANT"},
    ]


def _expert_history():
    """构造注入给专家的历史。"""
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
    """断言历史消息符合预期。"""
    assert "SENTINEL_USER" in content
    assert "SENTINEL_ASSISTANT" in content
    assert "SENTINEL_EXPERT" in content
    assert "SENTINEL_EVIDENCE" in content
    assert "canine cough" in content
    assert "user_report" in content
    assert "assistant_inference" in content
    assert "expert_inference" in content
    assert "RAW_RESULT_MUST_NOT_BE_REINJECTED" not in content


def test_fact_state_history_reaches_policy_expert_critic_and_aggregator():
    """验证事实状态历史能到达策略、专家、审核器和综合器。"""
    history = _history()
    expert_history = _expert_history()

    policy_llm = _CaptureLLM(_policy_response())
    asyncio.run(decide_task_policy(
        query="当前问题",
        user_role="pet_owner",
        llm=policy_llm,
        conversation_history=history,
        expert_context_history=expert_history,
    ))
    _assert_history(policy_llm.messages[0][-1]["content"])

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

    orchestrator = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner"))
    synthesis = orchestrator._build_synthesis_messages(
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


def test_cross_session_memory_reaches_policy_expert_and_critic():
    """验证跨会话记忆能到达策略、专家和审核器。"""
    memory = "【近期对话】\n- 用户: 我的狗叫球鼠，昨晚呕吐\n  助手: SOAP摘要"
    policy_llm = _CaptureLLM(_policy_response())
    asyncio.run(decide_task_policy(
        query="你还记得球鼠吗",
        user_role="pet_owner",
        llm=policy_llm,
        user_memory=memory,
    ))
    policy_payload = policy_llm.messages[0][-1]["content"]
    assert "cross_session_memory" in policy_payload
    assert "球鼠" in policy_payload
    assert "user_memory" in policy_payload

    expert = ExpertAgentSession(
        expert=EXPERTS["clinical"], query="你还记得球鼠吗", weight=1.0,
        registry=ToolRegistry(), llm=_CaptureLLM(), user_memory=memory,
    )
    assert "球鼠" in expert.messages[-1]["content"]
    assert "cross_session_memory" in expert.messages[-1]["content"]

    critic_llm = _CaptureLLM()
    asyncio.run(review(
        query="你还记得球鼠吗", expert_opinions=[], emergency=False,
        llm=critic_llm, user_memory=memory,
    ))
    assert "球鼠" in critic_llm.messages[0][-1]["content"]


def test_unified_policy_prompt_defines_history_scope_without_upgrading_facts():
    """验证统一策略提示词界定历史范围，且不会把假设升级成事实。"""
    assert "必须结合历史事实与当前问题判断连续语义" in TASK_POLICY_SYSTEM_PROMPT
    assert "复诊追问和对上一轮证据/指南的追问仍属于宠物健康上下文" in TASK_POLICY_SYSTEM_PROMPT
    assert "不代表诊断或其他事实已经被用户确认" in TASK_POLICY_SYSTEM_PROMPT
    assert "显式转向编程、股票、新闻等无关主题时不得因历史而放行" in TASK_POLICY_SYSTEM_PROMPT
