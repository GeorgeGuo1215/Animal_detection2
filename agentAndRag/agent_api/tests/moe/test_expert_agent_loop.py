from __future__ import annotations

import asyncio
import json
import os
import sys
from collections import defaultdict
from copy import deepcopy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.experts import (  # noqa: E402
    EXPERTS,
    ExpertAgentSession,
    ExpertConfig,
    ExpertLoopConfig,
    run_expert,
    run_expert_sessions,
)
from app.services.moe.retrieval_policy import RetrievalRequirement  # noqa: E402
from app.services.moe.tool_broker import ToolBroker  # noqa: E402
from app.services.moe.trace import MoETrace  # noqa: E402
from app.tools.tool_registry import ToolRegistry, ToolSpec  # noqa: E402


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


def _retrieval(*, required=(), recommended=(), web_fallback=False, reason="semantic policy", queries=()):
    return RetrievalRequirement(
        required_tools=tuple(required),
        recommended_tools=tuple(recommended),
        require_web_on_rag_failure=web_fallback,
        reason=reason,
        tool_queries=tuple(queries),
    )


def _registry(call_log, *, sufficient=True):
    registry = ToolRegistry()

    async def rag(**kwargs):
        call_log.append(("rag.search", dict(kwargs)))
        hits = [
            {"score": 0.91, "source_path": "book-a", "text": "evidence a"},
            {"score": 0.82, "source_path": "book-b", "text": "evidence b"},
        ] if sufficient else []
        return {"hits": hits}

    async def web(**kwargs):
        call_log.append(("mcp.web_search.web_search", dict(kwargs)))
        return {"results": [{"title": "guideline"}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    registry.register(ToolSpec("mcp.web_search.web_search", "web", {"type": "object"}, web))
    return registry


class SequenceLLM:
    model = "fake"

    def __init__(self, payloads, *, delay=0.0):
        self.payloads = list(payloads)
        self.messages = []
        self.delay = delay

    async def chat(self, messages=None, **kwargs):
        self.messages.append(deepcopy(messages or []))
        if self.delay:
            await asyncio.sleep(self.delay)
        return _response(self.payloads.pop(0))


def test_expert_persona_and_query_are_present_in_single_pass_without_tools():
    llm = SequenceLLM([_final("independent")])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="cat vomiting", weight=1.0,
        registry=ToolRegistry(), llm=llm, request_allowed_tools=[],
    ))

    assert len(llm.messages) == 1
    assert EXPERTS["clinical"].persona in llm.messages[0][0]["content"]
    assert "cat vomiting" in llm.messages[0][1]["content"]
    assert "唯一的专家意见生成轮次" in llm.messages[0][-1]["content"]
    assert result["conclusion"] == "independent"
    assert result["rounds"] == 1
    assert result["tools_used"] == []


def test_expert_prompt_is_single_pass_and_never_exposes_tool_schemas():
    session = ExpertAgentSession(
        expert=EXPERTS["clinical"], query="犬呼吸困难", weight=1.0,
        registry=_registry([]), llm=SequenceLLM([_final()]),
        retrieval_requirement=_retrieval(recommended=("rag.search",)),
    )
    system_prompt = session.messages[0]["content"]
    assert "只进行一次专业归纳" in system_prompt
    assert "不得调用工具" in system_prompt
    assert "input_schema" not in system_prompt
    assert "action=tool" not in system_prompt
    assert '{"action":"final","opinion":' in system_prompt


def test_pharmacy_safety_contract_remains_scoped_to_pharmacy_expert():
    for key, expert in EXPERTS.items():
        session = ExpertAgentSession(
            expert=expert, query="Can these contraindicated drugs be used together?",
            weight=1.0, registry=ToolRegistry(), llm=SequenceLLM([_final()]),
            request_allowed_tools=[],
        )
        system_prompt = session.messages[0]["content"]
        if key == "pharmacy":
            assert "禁忌联用硬约束" in system_prompt
            assert "替代方案必须移除或替换至少一种冲突药物或药物类别" in system_prompt
        else:
            assert "禁忌联用硬约束" not in system_prompt


def test_assigned_rag_and_web_execute_before_one_expert_generation():
    calls = []
    llm = SequenceLLM([_final("combined evidence")])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="feline urinary signs", weight=1.0,
        registry=_registry(calls), llm=llm,
        retrieval_requirement=_retrieval(
            recommended=("rag.search", "mcp.web_search.web_search"),
            queries=(("rag.search", "feline lower urinary tract disease"),
                     ("mcp.web_search.web_search", "current feline urinary guideline")),
        ),
    ))

    assert sorted(name for name, _ in calls) == ["mcp.web_search.web_search", "rag.search"]
    assert len(llm.messages) == 1
    assert sum(message["content"].startswith("TOOL_RESULT") for message in llm.messages[0]) == 2
    assert result["conclusion"] == "combined evidence"
    assert result["rounds"] == 1
    assert result["attempted_tools"] == ["rag.search", "mcp.web_search.web_search"]
    assert set(result["successful_tools"]) == {"rag.search", "mcp.web_search.web_search"}


def test_task_policy_english_query_is_used_for_rag():
    calls = []
    asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="猫排尿困难", weight=1.0,
        registry=_registry(calls), llm=SequenceLLM([_final()]),
        retrieval_requirement=_retrieval(
            required=("rag.search",),
            queries=(("rag.search", "feline urethral obstruction emergency triage"),),
        ),
    ))
    rag_args = next(arguments for name, arguments in calls if name == "rag.search")
    assert rag_args["query"] == "feline urethral obstruction emergency triage"
    assert rag_args["category"] == EXPERTS["clinical"].rag_categories


def test_non_english_assigned_rag_query_falls_back_to_expert_hint():
    calls = []
    asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="猫排尿困难", weight=1.0,
        registry=_registry(calls), llm=SequenceLLM([_final()]),
        retrieval_requirement=_retrieval(
            required=("rag.search",), queries=(("rag.search", "猫尿道梗阻"),),
        ),
    ))
    rag_args = next(arguments for name, arguments in calls if name == "rag.search")
    assert rag_args["query"] == EXPERTS["clinical"].rag_query_hint


def test_weak_required_rag_adds_one_web_fallback_before_final():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="核对犬用药禁忌", weight=1.0,
        registry=_registry(calls, sufficient=False), llm=SequenceLLM([_final("verified")]),
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            queries=(("rag.search", "canine drug contraindication"),),
        ),
    ))
    assert sorted(name for name, _ in calls) == ["mcp.web_search.web_search", "rag.search"]
    assert result["required_tools"] == ["rag.search", "mcp.web_search.web_search"]
    assert result["pending_tools"] == []
    assert result["conclusion"] == "verified"


def test_explicitly_disabled_tools_preserve_disable_semantics():
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="请检索", weight=1.0,
        registry=_registry(calls), llm=SequenceLLM([_final("no tools")]),
        request_allowed_tools=[],
        retrieval_requirement=_retrieval(required=("rag.search", "mcp.web_search.web_search")),
    ))
    assert calls == []
    assert result["unavailable_required_tools"] == ["rag.search", "mcp.web_search.web_search"]
    assert result["conclusion"] == "no tools"


def test_invalid_final_envelope_gets_one_format_repair_only():
    llm = SequenceLLM([
        {"conclusion": "legacy", "evidence": [], "risks": [], "confidence": 0.8},
        _final("canonical"),
    ])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=llm, request_allowed_tools=[],
    ))
    assert len(llm.messages) == 2
    assert "FORMAT_REPAIR" in llm.messages[1][-1]["content"]
    assert result["conclusion"] == "canonical"
    assert result["rounds"] == 1


def test_two_invalid_outputs_return_safe_single_pass_fallback():
    llm = SequenceLLM([{}, {}])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=llm, request_allowed_tools=[],
    ))
    assert len(llm.messages) == 2
    assert result["confidence"] == 0.0
    assert "未生成有效结构化结论" in result["conclusion"]


def _matching_expert(key, persona):
    return ExpertConfig(
        key=key, name_zh=key, persona=persona, allowed_tools=["rag.search"],
        rag_query_hint="shared case", rag_categories=["clinical.shared"],
    )


class PerExpertFinalLLM:
    model = "fake"

    def __init__(self):
        self.messages = defaultdict(list)

    async def chat(self, messages=None, **kwargs):
        system = (messages or [{}])[0].get("content", "")
        key = "alpha" if "persona-alpha" in system else "beta"
        self.messages[key].append(deepcopy(messages or []))
        return _response(_final(key))


def test_identical_assigned_rag_calls_are_deduplicated_across_experts():
    calls = []
    registry = _registry(calls)
    llm = PerExpertFinalLLM()
    requirement = _retrieval(
        recommended=("rag.search",), queries=(("rag.search", "shared case"),),
    )
    sessions = [
        ExpertAgentSession(
            expert=_matching_expert("alpha", "persona-alpha"), query="case", weight=0.5,
            registry=registry, llm=llm, retrieval_requirement=requirement,
        ),
        ExpertAgentSession(
            expert=_matching_expert("beta", "persona-beta"), query="case", weight=0.5,
            registry=registry, llm=llm, retrieval_requirement=requirement,
        ),
    ]
    results = asyncio.run(run_expert_sessions(
        sessions=sessions, broker=ToolBroker(registry=registry, allowed_tools=None),
    ))
    assert [name for name, _ in calls] == ["rag.search"]
    assert all(result["tool_results"][0]["shared"] for result in results)
    assert all(
        any("TOOL_RESULT" in message["content"] for message in llm.messages[key][0])
        for key in ("alpha", "beta")
    )


def test_each_expert_only_receives_its_own_assigned_tool_results():
    calls = []
    registry = _registry(calls)
    llm = PerExpertFinalLLM()
    sessions = [
        ExpertAgentSession(
            expert=_matching_expert("alpha", "persona-alpha"), query="case", weight=0.5,
            registry=registry, llm=llm,
            retrieval_requirement=_retrieval(recommended=("rag.search",)),
        ),
        ExpertAgentSession(
            expert=_matching_expert("beta", "persona-beta"), query="case", weight=0.5,
            registry=registry, llm=llm, retrieval_requirement=_retrieval(),
        ),
    ]
    asyncio.run(run_expert_sessions(
        sessions=sessions, broker=ToolBroker(registry=registry, allowed_tools=None),
    ))
    assert any(message["content"].startswith("TOOL_RESULT") for message in llm.messages["alpha"][0])
    assert all(not message["content"].startswith("TOOL_RESULT") for message in llm.messages["beta"][0])


def test_expert_timeout_returns_fallback_without_additional_rounds():
    llm = SequenceLLM([_final()], delay=0.05)
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=llm, request_allowed_tools=[],
        loop_config=ExpertLoopConfig(timeout_s=0.01, repair_attempts=0),
    ))
    assert result["confidence"] == 0.0
    assert result["rounds"] == 1
    assert len(llm.messages) == 1


def test_default_execution_config_is_single_pass_with_one_format_repair():
    config = ExpertLoopConfig()
    assert config.final_max_tokens == 1400
    assert config.repair_attempts == 1
    assert config.timeout_s == 120.0


def test_trace_names_final_and_format_repair_without_round_loop():
    trace = MoETrace(question="case", user_role="veterinarian")
    asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=SequenceLLM([{}, _final("fixed")]),
        request_allowed_tools=[], recorder=trace,
    ))
    assert [call.stage for call in trace.llm_calls] == [
        "expert:clinical:final", "expert:clinical:format_repair:1",
    ]
