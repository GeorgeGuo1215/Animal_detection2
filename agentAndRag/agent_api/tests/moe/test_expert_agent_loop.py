from __future__ import annotations

import asyncio
import json
import os
import sys
from collections import defaultdict
from copy import deepcopy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.critic import CriticResult
from app.services.moe.experts import (
    EXPERTS,
    ExpertAgentSession,
    ExpertConfig,
    ExpertLoopConfig,
    run_expert,
    run_expert_sessions,
)
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig
from app.services.moe.router import RouterDecision
from app.services.moe.tool_broker import ToolBroker
from app.tools.tool_registry import ToolRegistry, ToolSpec


def _response(payload):
    return {"choices": [{"message": {"content": json.dumps(payload, ensure_ascii=False)}}]}


def _final(conclusion="ok"):
    return {
        "action": "final",
        "opinion": {
            "conclusion": conclusion,
            "evidence": [],
            "risks": [],
            "confidence": 0.8,
        },
    }


def _registry(call_log):
    registry = ToolRegistry()

    async def rag(**kwargs):
        call_log.append(("rag.search", dict(kwargs)))
        return {"hits": [{"score": 0.9, "source_path": "book", "text": "evidence"}]}

    async def web(**kwargs):
        call_log.append(("mcp.web_search.web_search", dict(kwargs)))
        return {"results": [{"title": "guideline"}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    registry.register(ToolSpec("mcp.web_search.web_search", "web", {"type": "object"}, web))
    return registry


class SequenceLLM:
    model = "fake"

    def __init__(self, payloads):
        self.payloads = list(payloads)
        self.messages = []

    async def chat(self, messages=None, **kwargs):
        self.messages.append(deepcopy(messages or []))
        return _response(self.payloads.pop(0))


def test_expert_persona_and_query_are_present_from_first_round_without_tools():
    llm = SequenceLLM([_final("independent")])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"],
        query="cat vomiting",
        weight=1.0,
        registry=ToolRegistry(),
        llm=llm,
        request_allowed_tools=[],
    ))

    assert len(llm.messages) == 1
    assert EXPERTS["clinical"].persona in llm.messages[0][0]["content"]
    assert "cat vomiting" in llm.messages[0][1]["content"]
    assert result["conclusion"] == "independent"
    assert result["tools_used"] == []


def test_pharmacy_prompt_requires_alternative_for_confirmed_contraindicated_combination():
    session = ExpertAgentSession(
        expert=EXPERTS["pharmacy"],
        query="Can these two drugs be used together?",
        weight=1.0,
        registry=ToolRegistry(),
        llm=SequenceLLM([_final()]),
        request_allowed_tools=[],
    )

    system_prompt = session.messages[0]["content"]
    assert "可靠证据已确认两种药物属于禁忌联用" in system_prompt
    assert "不能只给出停用或禁用结论" in system_prompt
    assert "至少一条可执行的替代路径" in system_prompt
    assert "优先替换哪一种药" in system_prompt
    assert "切换/洗脱和监测边界" in system_prompt
    assert "不得编造" in system_prompt
    assert "需要补充的信息" in system_prompt
    assert "替代路径必须真正消除禁忌组合" in system_prompt
    assert "换用同类低风险药" in system_prompt
    assert "增加支持性治疗或加强监测" in system_prompt
    assert "替代药物不得仍属于冲突类别" in system_prompt


def test_clinical_prompt_prefers_rag_and_web_for_important_cases():
    session = ExpertAgentSession(
        expert=EXPERTS["clinical"],
        query="犬突发呼吸困难，请给出鉴别诊断和处置优先级",
        weight=1.0,
        registry=_registry([]),
        llm=SequenceLLM([_final()]),
    )

    system_prompt = session.messages[0]["content"]
    assert "重要场景双检索" in system_prompt
    assert "诊断或鉴别诊断、急症风险判断" in system_prompt
    assert "rag.search 与 mcp.web_search.web_search" in system_prompt
    assert "应先后调用二者并综合证据" in system_prompt
    assert "每轮仍只调用一个工具" in system_prompt
    assert "首次检索结果不足或未命中，不是跳过另一类检索的理由" in system_prompt


def test_pharmacy_prompt_prefers_rag_and_web_for_high_risk_medication_questions():
    session = ExpertAgentSession(
        expert=EXPERTS["pharmacy"],
        query="请评估两种药的相互作用、禁忌和安全替代方案",
        weight=1.0,
        registry=_registry([]),
        llm=SequenceLLM([_final()]),
    )

    system_prompt = session.messages[0]["content"]
    assert "重要场景双检索" in system_prompt
    assert "具体药物剂量、联合用药与相互作用、禁忌、物种毒性" in system_prompt
    assert "rag.search 与 mcp.web_search.web_search" in system_prompt
    assert "应先后调用二者并综合" in system_prompt
    assert "近期药品资料、指南或外部安全证据" in system_prompt


def test_dual_retrieval_policy_is_scoped_to_clinical_and_pharmacy_experts():
    for key in ("nutrition", "behavior"):
        session = ExpertAgentSession(
            expert=EXPERTS[key],
            query="普通咨询",
            weight=1.0,
            registry=_registry([]),
            llm=SequenceLLM([_final()]),
        )
        assert "重要场景双检索" not in session.messages[0]["content"]


def test_all_experts_forbid_mitigation_as_permission_for_contraindicated_combination():
    for expert in EXPERTS.values():
        session = ExpertAgentSession(
            expert=expert,
            query="Can these contraindicated drugs be used together?",
            weight=1.0,
            registry=ToolRegistry(),
            llm=SequenceLLM([_final()]),
            request_allowed_tools=[],
        )

        system_prompt = session.messages[0]["content"]
        assert "禁忌联用硬约束" in system_prompt
        assert "替代方案必须移除或替换至少一种冲突药物或药物类别" in system_prompt
        assert "都不能解除禁忌" in system_prompt
        assert "支持性处理只能作为意外暴露后的风险处置" in system_prompt
        assert "不得编造统一天数" in system_prompt
        assert "输出前必须删除任何允许该禁忌组合重叠的句子" in system_prompt
        assert "任何字段都禁止给出数字洗脱天数" in system_prompt
        assert "不能换成仍属于冲突类别的药物" in system_prompt
        assert "适应证尚未明确时" in system_prompt
        assert "不得无条件指定立即停用其中某一种药" in system_prompt
        assert "对需要渐减的长期用药避免骤停" in system_prompt


def test_expert_can_use_rag_then_web_then_return_final_opinion():
    calls = []
    llm = SequenceLLM([
        {"action": "tool", "tool_name": "rag.search", "arguments": {"query": "feline UTI"}},
        {
            "action": "tool",
            "tool_name": "mcp.web_search.web_search",
            "arguments": {"query": "current feline UTI guideline"},
        },
        _final("combined evidence"),
    ])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"],
        query="feline urinary signs",
        weight=1.0,
        registry=_registry(calls),
        llm=llm,
    ))

    assert [name for name, _ in calls] == ["rag.search", "mcp.web_search.web_search"]
    assert EXPERTS["clinical"].rag_categories == calls[0][1]["category"]
    assert any("TOOL_RESULT" in message["content"] for message in llm.messages[1])
    assert sum("TOOL_RESULT" in message["content"] for message in llm.messages[2]) == 2
    assert all(message["role"] != "tool" for messages in llm.messages for message in messages)
    assert result["conclusion"] == "combined evidence"


def test_expert_retries_non_english_rag_query_in_english():
    calls = []
    llm = SequenceLLM([
        {"action": "tool", "tool_name": "rag.search", "arguments": {"query": "猫尿道梗阻"}},
        {
            "action": "tool",
            "tool_name": "rag.search",
            "arguments": {"query": "feline urethral obstruction"},
        },
        _final("english evidence"),
    ])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"],
        query="猫排尿困难",
        weight=1.0,
        registry=_registry(calls),
        llm=llm,
    ))

    assert [arguments["query"] for name, arguments in calls if name == "rag.search"] == [
        "feline urethral obstruction"
    ]
    assert result["tool_results"][0]["result"]["code"] == "RAG_QUERY_MUST_BE_ENGLISH"
    assert result["conclusion"] == "english evidence"


def _matching_expert(key, persona):
    return ExpertConfig(
        key=key,
        name_zh=key,
        persona=persona,
        allowed_tools=["rag.search"],
        rag_query_hint="",
        rag_categories=["clinical.shared"],
    )


class PerExpertLLM:
    model = "fake"

    def __init__(self, tool_experts):
        self.tool_experts = set(tool_experts)
        self.calls = defaultdict(int)
        self.messages = defaultdict(list)

    async def chat(self, messages=None, **kwargs):
        system = (messages or [{}])[0].get("content", "")
        key = "alpha" if "persona-alpha" in system else "beta"
        self.calls[key] += 1
        self.messages[key].append(deepcopy(messages or []))
        if key in self.tool_experts and self.calls[key] == 1:
            return _response({
                "action": "tool",
                "tool_name": "rag.search",
                "arguments": {"query": "shared case"},
            })
        return _response(_final(key))


def test_identical_rag_requests_are_executed_once_and_returned_to_each_expert():
    calls = []
    registry = _registry(calls)
    llm = PerExpertLLM({"alpha", "beta"})
    sessions = [
        ExpertAgentSession(
            expert=_matching_expert("alpha", "persona-alpha"),
            query="case",
            weight=0.5,
            registry=registry,
            llm=llm,
        ),
        ExpertAgentSession(
            expert=_matching_expert("beta", "persona-beta"),
            query="case",
            weight=0.5,
            registry=registry,
            llm=llm,
        ),
    ]

    results = asyncio.run(run_expert_sessions(
        sessions=sessions,
        broker=ToolBroker(registry=registry, allowed_tools=None),
    ))

    assert [name for name, _ in calls] == ["rag.search"]
    assert all(result["tool_results"][0]["shared"] for result in results)
    assert all("TOOL_RESULT" in llm.messages[key][1][-1]["content"] for key in ("alpha", "beta"))


def test_tool_result_is_not_added_to_another_experts_context():
    calls = []
    registry = _registry(calls)
    llm = PerExpertLLM({"alpha"})
    sessions = [
        ExpertAgentSession(
            expert=_matching_expert("alpha", "persona-alpha"), query="case", weight=0.5,
            registry=registry, llm=llm,
        ),
        ExpertAgentSession(
            expert=_matching_expert("beta", "persona-beta"), query="case", weight=0.5,
            registry=registry, llm=llm,
        ),
    ]

    asyncio.run(run_expert_sessions(
        sessions=sessions,
        broker=ToolBroker(registry=registry, allowed_tools=None),
    ))

    assert any("TOOL_RESULT" in message["content"] for message in llm.messages["alpha"][1])
    assert all("TOOL_RESULT" not in message["content"] for message in llm.messages["beta"][0])


def test_fast_expert_submits_rag_without_waiting_for_slow_expert_planning():
    events = []
    registry = ToolRegistry()

    async def rag(**kwargs):
        events.append("rag_started")
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))

    class StaggeredLLM:
        model = "fake"

        def __init__(self):
            self.calls = defaultdict(int)

        async def chat(self, messages=None, **kwargs):
            system = (messages or [{}])[0].get("content", "")
            key = "alpha" if "persona-alpha" in system else "beta"
            self.calls[key] += 1
            if key == "beta" and self.calls[key] == 1:
                await asyncio.sleep(0.03)
                events.append("slow_plan_finished")
                return _response(_final("beta"))
            if key == "alpha" and self.calls[key] == 1:
                return _response({
                    "action": "tool",
                    "tool_name": "rag.search",
                    "arguments": {"query": "fast expert query"},
                })
            return _response(_final("alpha"))

    sessions = [
        ExpertAgentSession(
            expert=_matching_expert("alpha", "persona-alpha"), query="case", weight=0.5,
            registry=registry, llm=StaggeredLLM(),
        ),
        ExpertAgentSession(
            expert=_matching_expert("beta", "persona-beta"), query="case", weight=0.5,
            registry=registry, llm=StaggeredLLM(),
        ),
    ]

    asyncio.run(run_expert_sessions(
        sessions=sessions,
        broker=ToolBroker(registry=registry, allowed_tools=None),
    ))

    assert events.index("rag_started") < events.index("slow_plan_finished")


class RepeatLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        if any("REPEATED_TOOL_CALL" in message.get("content", "") for message in messages or []):
            return _response(_final("stopped repeat"))
        return _response({"action": "tool", "tool_name": "rag.search", "arguments": {"query": "same"}})


def test_repeated_identical_tool_call_is_blocked():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"],
        query="case",
        weight=1.0,
        registry=_registry(calls),
        llm=RepeatLLM(),
        loop_config=ExpertLoopConfig(max_rounds=4, max_tool_calls=4, timeout_s=1, max_repeated_calls=1),
    ))

    assert [name for name, _ in calls] == ["rag.search"]
    assert result["rounds"] == 3
    assert result["conclusion"] == "stopped repeat"


class BudgetLLM:
    model = "fake"

    def __init__(self):
        self.calls = 0

    async def chat(self, messages=None, **kwargs):
        self.calls += 1
        if any("本轮必须返回 action=final" in message.get("content", "") for message in messages or []):
            return _response(_final("budget final"))
        return _response({
            "action": "tool",
            "tool_name": "rag.search",
            "arguments": {"query": f"query-{self.calls}"},
        })


def test_tool_call_budget_forces_a_final_round():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=_registry(calls), llm=BudgetLLM(),
        loop_config=ExpertLoopConfig(max_rounds=5, max_tool_calls=1, timeout_s=1, max_repeated_calls=1),
    ))

    assert len(calls) == 1
    assert result["rounds"] == 2
    assert result["conclusion"] == "budget final"


def test_round_budget_reserves_last_round_for_final_opinion():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=_registry(calls), llm=BudgetLLM(),
        loop_config=ExpertLoopConfig(max_rounds=2, max_tool_calls=4, timeout_s=1, max_repeated_calls=1),
    ))

    assert len(calls) == 1
    assert result["rounds"] == 2
    assert result["conclusion"] == "budget final"


def test_default_expert_round_limit_is_six():
    assert ExpertLoopConfig().max_rounds == 6


class PenultimatePromptLLM:
    model = "fake"

    def __init__(self):
        self.calls = 0
        self.messages = []

    async def chat(self, messages=None, **kwargs):
        self.calls += 1
        self.messages.append(deepcopy(messages or []))
        if any("本轮必须返回 action=final" in message.get("content", "") for message in messages or []):
            return _response(_final("closed on final round"))
        return _response({
            "action": "tool",
            "tool_name": "rag.search",
            "arguments": {"query": f"feline evidence round {self.calls}"},
        })


def test_penultimate_round_warns_that_next_round_must_be_final():
    calls = []
    llm = PenultimatePromptLLM()
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=_registry(calls), llm=llm,
        loop_config=ExpertLoopConfig(
            max_rounds=3, max_tool_calls=4, timeout_s=1, max_repeated_calls=1
        ),
    ))

    assert any(
        "这是倒数第二轮" in message.get("content", "")
        for message in llm.messages[1]
    )
    assert any(
        "本轮必须返回 action=final" in message.get("content", "")
        for message in llm.messages[2]
    )
    assert result["rounds"] == 3
    assert result["conclusion"] == "closed on final round"


class FinalRoundViolationLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        return _response({
            "action": "tool",
            "tool_name": "rag.search",
            "arguments": {"query": "feline emergency"},
        })


def test_final_round_tool_request_becomes_structured_fallback_without_execution():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=_registry(calls), llm=FinalRoundViolationLLM(),
        loop_config=ExpertLoopConfig(max_rounds=2, max_tool_calls=4, timeout_s=1),
    ))

    assert len(calls) == 1
    assert result["rounds"] == 2
    assert result["confidence"] == 0.0
    assert "最终轮未按要求" in result["conclusion"]


class SlowLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        await asyncio.sleep(0.05)
        return _response(_final())


def test_expert_timeout_returns_a_fallback_opinion():
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=SlowLLM(), request_allowed_tools=[],
        loop_config=ExpertLoopConfig(max_rounds=5, max_tool_calls=4, timeout_s=0.01, max_repeated_calls=1),
    ))

    assert result["confidence"] == 0.0
    assert "超时限制" in result["risks"][0]


def test_slow_tool_is_bounded_by_the_expert_timeout():
    registry = ToolRegistry()

    async def slow_rag(**kwargs):
        await asyncio.sleep(0.05)
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, slow_rag))
    llm = SequenceLLM([
        {"action": "tool", "tool_name": "rag.search", "arguments": {"query": "slow"}},
    ])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=registry, llm=llm,
        loop_config=ExpertLoopConfig(max_rounds=5, max_tool_calls=4, timeout_s=0.01, max_repeated_calls=1),
    ))

    assert result["confidence"] == 0.0
    assert result["tool_results"][0]["result"]["code"] == "EXPERT_TOOL_TIMEOUT"


def test_slow_tool_reserves_time_for_a_forced_final_round():
    registry = ToolRegistry()

    async def slow_rag(**kwargs):
        await asyncio.sleep(0.2)
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, slow_rag))
    llm = BudgetLLM()
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=registry, llm=llm,
        loop_config=ExpertLoopConfig(
            max_rounds=6,
            max_tool_calls=4,
            timeout_s=0.12,
            finalize_reserve_s=0.08,
            max_repeated_calls=1,
        ),
    ))

    assert result["tool_results"][0]["result"]["code"] == "EXPERT_TOOL_TIMEOUT"
    assert result["rounds"] == 2
    assert result["conclusion"] == "budget final"


class FinalLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        return {"choices": [{"message": {"content": "final answer"}}]}


def test_critic_runs_only_after_all_experts_complete():
    events = []

    class TrackingOrchestrator(MoEOrchestrator):
        async def _route(self, query, recorder):
            return RouterDecision(
                scores={"clinical": 1.0}, raw_weights={"clinical": 1.0},
                weights={"clinical": 1.0}, selected_experts=["clinical"],
                emergency=False, out_of_scope=False, reason="test",
            )

        async def _run_experts(self, query, decision, recorder):
            events.append("experts_started")
            await asyncio.sleep(0.01)
            events.append("experts_completed")
            return []

        async def _critique(self, query, opinions, emergency, recorder):
            assert events[-1] == "experts_completed"
            events.append("critic_started")
            return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")

    orchestrator = TrackingOrchestrator(
        registry=ToolRegistry(), llm=FinalLLM(), config=OrchestratorConfig(max_tokens=100)
    )
    answer, _ = asyncio.run(orchestrator.run(query="case"))

    assert answer == "final answer"
    assert events == ["experts_started", "experts_completed", "critic_started"]
