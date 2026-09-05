from __future__ import annotations

import asyncio
import json
import os
import sys
from collections import defaultdict
from copy import deepcopy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.services.moe.expert_runtime.service import (  # noqa: E402
    EXPERTS,
    ExpertAgentSession,
    ExpertConfig,
    ExpertLoopConfig,
    run_expert,
    run_expert_sessions,
)
from agent_api.app.services.moe.retrieval_policy import RetrievalRequirement  # noqa: E402
from agent_api.app.services.moe.tool_broker import ToolBroker  # noqa: E402
from agent_api.app.services.moe.trace import MoETrace  # noqa: E402
from agent_api.app.tools.tool_registry import ToolRegistry, ToolSpec  # noqa: E402


def _response(payload):
    """构造测试用的模型响应。"""
    return {"choices": [{"message": {"content": json.dumps(payload, ensure_ascii=False)}}]}


def _final(conclusion="ok"):
    """取出最终回答文本。"""
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
    """构造或拦截一次检索调用。"""
    return RetrievalRequirement(
        required_tools=tuple(required),
        recommended_tools=tuple(recommended),
        require_web_on_rag_failure=web_fallback,
        reason=reason,
        tool_queries=tuple(queries),
    )


def _registry(call_log, *, sufficient=True):
    """返回测试用的工具注册表。"""
    registry = ToolRegistry()

    async def rag(**kwargs):
        """本用例中拦截 RAG 调用的假实现。"""
        call_log.append(("rag.search", dict(kwargs)))
        hits = [
            {"score": 0.91, "source_path": "book-a", "text": "evidence a"},
            {"score": 0.82, "source_path": "book-b", "text": "evidence b"},
        ] if sufficient else []
        return {"hits": hits}

    async def web(**kwargs):
        """网页检索替身。"""
        call_log.append(("mcp.web_search.web_search", dict(kwargs)))
        return {"results": [{"title": "guideline"}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    registry.register(ToolSpec("mcp.web_search.web_search", "web", {"type": "object"}, web))
    return registry


class SequenceLLM:
    model = "fake"

    def __init__(self, payloads, *, delay=0.0):
        """初始化该测试替身。"""
        self.payloads = list(payloads)
        self.messages = []
        self.delay = delay

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        self.messages.append(deepcopy(messages or []))
        if self.delay:
            await asyncio.sleep(self.delay)
        return _response(self.payloads.pop(0))


class SufficiencyAwareLLM:
    model = "fake"

    def __init__(self, status_by_query):
        """初始化该测试替身。"""
        self.status_by_query = dict(status_by_query)
        self.messages = []
        self.sufficiency_batches = []

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        copied = deepcopy(messages or [])
        self.messages.append(copied)
        if copied and "证据覆盖审计器" in copied[0].get("content", ""):
            payload = json.loads(copied[-1]["content"])
            items = payload["evidence_items"]
            self.sufficiency_batches.append(items)
            return _response({
                "assessments": [
                    {
                        "id": item["id"],
                        "status": self.status_by_query.get(
                            item["evidence_query"], "unsupported"
                        ),
                        "reason": "fixture coverage decision",
                        "matched_hit_ids": ["h1"],
                    }
                    for item in items
                ],
            })
        return _response(_final("audited evidence"))


def test_expert_persona_and_query_are_present_in_single_pass_without_tools():
    """验证无工具单次生成时仍包含专家人设与查询。"""
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
    """验证专家提示词是单次生成，且从不暴露工具 schema。"""
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
    """验证药房安全契约只作用于药房专家。"""
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
    """验证分配的 RAG 与网页检索会在专家生成前执行。"""
    calls = []
    llm = SequenceLLM([{}, _final("combined evidence")])
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
    assert len(llm.messages) == 2
    assert sum(message["content"].startswith("TOOL_RESULT") for message in llm.messages[-1]) == 2
    assert result["conclusion"] == "combined evidence"
    assert result["rounds"] == 1
    assert result["attempted_tools"] == ["rag.search", "mcp.web_search.web_search"]
    assert set(result["successful_tools"]) == {"rag.search", "mcp.web_search.web_search"}


def test_task_policy_english_query_is_used_for_rag():
    """验证任务策略给 RAG 用的是英文查询。"""
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
    assert rag_args["rerank"] is True


def test_distinct_queries_for_the_same_tool_are_all_executed():
    """验证同一工具的不同查询都会执行。"""
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="犬药物切换", weight=1.0,
        registry=_registry(calls), llm=SequenceLLM([_final()]),
        retrieval_requirement=_retrieval(
            required=("rag.search",),
            queries=(
                ("rag.search", "canine corticosteroid NSAID washout interval"),
                ("rag.search", "canine meloxicam gastrointestinal monitoring"),
            ),
        ),
    ))

    rag_queries = [arguments["query"] for name, arguments in calls if name == "rag.search"]
    assert rag_queries == [
        "canine corticosteroid NSAID washout interval",
        "canine meloxicam gastrointestinal monitoring",
    ]
    assert result["attempted_tools"] == ["rag.search", "rag.search"]
    assert len(result["tool_results"]) == 2
    assert result["pending_tools"] == []


def test_non_english_assigned_rag_query_falls_back_to_expert_hint():
    """验证非英文的已分配 RAG 查询会回退到专家提示。"""
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


def test_weak_required_rag_adds_one_expanded_rag_and_bilingual_web_wave():
    """验证弱 RAG 仅追加一次扩类检索，并以中英查询并发补证。"""
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="核对犬用药禁忌", weight=1.0,
        registry=_registry(calls, sufficient=False), llm=SequenceLLM([{}, _final("verified")]),
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            reason="核对犬用药禁忌",
            queries=(("rag.search", "canine drug contraindication"),),
        ),
    ))
    rag_calls = [arguments for name, arguments in calls if name == "rag.search"]
    web_calls = [arguments for name, arguments in calls if name == "mcp.web_search.web_search"]
    assert len(rag_calls) == 2
    assert rag_calls[0]["category"] == EXPERTS["pharmacy"].rag_categories
    assert "category" not in rag_calls[1]
    assert {item["query"] for item in web_calls} == {
        "canine drug contraindication",
        "兽医 犬用药禁忌",
    }
    assert result["required_tools"] == ["rag.search", "mcp.web_search.web_search"]
    assert result["pending_tools"] == []
    assert result["rounds"] == 2
    assert result["conclusion"] == "verified"


def test_high_score_but_semantically_unsupported_rag_adds_web_fallback():
    """验证高分但语义撑不住的 RAG 会追加网页兜底。"""
    calls = []
    query = "canine corticosteroid NSAID washout interval"
    llm = SufficiencyAwareLLM({query: "unsupported"})
    result = asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="犬药物切换", weight=1.0,
        registry=_registry(calls), llm=llm,
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            reason="核对糖皮质激素切换非甾体抗炎药的洗脱要求",
            queries=(("rag.search", query),),
        ),
    ))

    rag_calls = [arguments for name, arguments in calls if name == "rag.search"]
    web_calls = [arguments for name, arguments in calls if name == "mcp.web_search.web_search"]
    assert len(rag_calls) == 2
    assert "category" in rag_calls[0] and "category" not in rag_calls[1]
    assert {item["query"] for item in web_calls} == {
        query,
        "犬 兽医 糖皮质激素切换非甾体抗炎药的洗脱要求",
    }
    assert len(llm.sufficiency_batches) == 2
    assert [item["status"] for item in result["evidence_sufficiency"]] == [
        "unsupported", "unsupported", "unsupported", "unsupported",
    ]
    assert all(item["method"] == "semantic_llm" for item in result["evidence_sufficiency"])


def test_web_fallback_never_submits_task_reason_as_chinese_query():
    """验证药物补查使用简短兽医查询，而不是把中文任务说明整句交给 Web。"""
    calls = []
    reason = (
        "用户明确要求查询曲马多具体剂量，且未指明疾病，"
        "需检索权威药物参考以提供准确剂量范围。"
    )
    asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="查一下具体曲马多药物剂量", weight=1.0,
        registry=_registry(calls, sufficient=False), llm=SequenceLLM([{}, _final()]),
        species_en="canine", species_zh="犬",
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            reason=reason, queries=(("rag.search", "canine tramadol dosage"),),
        ),
    ))

    web_queries = [
        arguments["query"]
        for name, arguments in calls
        if name == "mcp.web_search.web_search"
    ]
    assert set(web_queries) == {
        "canine tramadol dosage",
        "犬 兽医 曲马多具体剂量",
    }
    assert reason not in web_queries
    assert all("用户明确要求" not in query and "需检索" not in query for query in web_queries)


def test_high_score_and_semantically_supported_rag_does_not_add_web():
    """验证高分且语义站得住的 RAG 不会再加网页检索。"""
    calls = []
    query = "feline urethral obstruction emergency triage"
    llm = SufficiencyAwareLLM({query: "supported"})
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="猫排尿困难", weight=1.0,
        registry=_registry(calls), llm=llm,
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            queries=(("rag.search", query),),
        ),
    ))

    assert [name for name, _ in calls] == ["rag.search"]
    assert len(llm.sufficiency_batches) == 1
    assert result["evidence_sufficiency"][0]["status"] == "supported"
    assert result["required_tools"] == ["rag.search"]


def test_multiple_high_score_rag_tasks_are_batched_per_bounded_wave():
    """验证多条 RAG 任务在每一有界波次只做一次批量充分性判断。"""
    calls = []
    first = "canine corticosteroid NSAID washout interval"
    second = "canine meloxicam gastrointestinal monitoring"
    llm = SufficiencyAwareLLM({first: "supported", second: "partial"})
    result = asyncio.run(run_expert(
        expert=EXPERTS["pharmacy"], query="犬药物切换", weight=1.0,
        registry=_registry(calls), llm=llm,
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            reason="核对犬药物切换和胃肠道监测",
            queries=(("rag.search", first), ("rag.search", second)),
        ),
    ))

    assert len(llm.sufficiency_batches) == 2
    assert len(llm.sufficiency_batches[0]) == 2
    assert len(llm.sufficiency_batches[1]) == 3
    rag_calls = [arguments for name, arguments in calls if name == "rag.search"]
    assert len(rag_calls) == 3
    assert all("category" in item for item in rag_calls[:2])
    assert "category" not in rag_calls[2]
    web_queries = {
        arguments["query"]
        for name, arguments in calls
        if name == "mcp.web_search.web_search"
    }
    assert {second, "兽医 犬药物切换和胃肠道监测"}.issubset(web_queries)
    assert [item["status"] for item in result["evidence_sufficiency"]] == [
        "supported", "partial", "partial", "partial", "unsupported",
    ]


def test_invalid_sufficiency_response_conservatively_adds_web():
    """验证充分性判断非法时会保守地追加网页检索。"""
    calls = []
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="猫排尿困难", weight=1.0,
        registry=_registry(calls), llm=SequenceLLM([{}, {}, _final("safe fallback")]),
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            reason="核对猫尿道梗阻分诊证据",
            queries=(("rag.search", "feline urinary obstruction triage"),),
        ),
    ))

    rag_calls = [arguments for name, arguments in calls if name == "rag.search"]
    web_calls = [arguments for name, arguments in calls if name == "mcp.web_search.web_search"]
    assert len(rag_calls) == 2
    assert "category" in rag_calls[0] and "category" not in rag_calls[1]
    assert {item["query"] for item in web_calls} == {
        "feline urinary obstruction triage",
        "兽医 猫尿道梗阻分诊证据",
    }
    assert [item["status"] for item in result["evidence_sufficiency"]] == [
        "unknown", "unknown", "unknown", "unknown",
    ]
    assert result["conclusion"] == "safe fallback"


def test_sufficiency_audit_can_be_disabled_without_changing_numeric_gate(monkeypatch):
    """验证充分性审计可关闭且不影响数值门控。"""
    monkeypatch.setenv("MOE_EVIDENCE_SUFFICIENCY_ENABLED", "0")
    calls = []
    llm = SequenceLLM([_final("numeric only")])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="猫排尿困难", weight=1.0,
        registry=_registry(calls), llm=llm,
        retrieval_requirement=_retrieval(
            required=("rag.search",), web_fallback=True,
            queries=(("rag.search", "feline urinary obstruction triage"),),
        ),
    ))

    assert [name for name, _ in calls] == ["rag.search"]
    assert len(llm.messages) == 1
    assert result["evidence_sufficiency"] == []


def test_explicitly_disabled_tools_preserve_disable_semantics():
    """验证显式禁用工具会保持「已禁用」语义。"""
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
    """验证非法的最终信封只会做一次格式修复。"""
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
    """验证两次非法输出后返回安全的单次兜底。"""
    llm = SequenceLLM([{}, {}])
    result = asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=llm, request_allowed_tools=[],
    ))
    assert len(llm.messages) == 2
    assert result["confidence"] == 0.0
    assert "未生成有效结构化结论" in result["conclusion"]


def _matching_expert(key, persona):
    """找出与当前任务匹配的专家。"""
    return ExpertConfig(
        key=key, name_zh=key, persona=persona, allowed_tools=["rag.search"],
        rag_query_hint="shared case", rag_categories=["clinical.shared"],
    )


class PerExpertFinalLLM:
    model = "fake"

    def __init__(self):
        """初始化该测试替身。"""
        self.messages = defaultdict(list)

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        system = (messages or [{}])[0].get("content", "")
        key = "alpha" if "persona-alpha" in system else "beta"
        self.messages[key].append(deepcopy(messages or []))
        return _response(_final(key))


def test_identical_assigned_rag_calls_are_deduplicated_across_experts():
    """验证专家之间相同的 RAG 调用会被去重。"""
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
    """验证每位专家只收到自己被分配的工具结果。"""
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
    """验证专家超时返回兜底，不会再开额外轮次。"""
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
    """验证默认执行配置是单次生成外加一次格式修复。"""
    config = ExpertLoopConfig()
    assert config.final_max_tokens == 1400
    assert config.repair_attempts == 1
    assert config.timeout_s == 120.0


def test_trace_names_final_and_format_repair_without_round_loop():
    """验证追踪阶段名为 final / format_repair，而不是轮次循环。"""
    trace = MoETrace(question="case", user_role="veterinarian")
    asyncio.run(run_expert(
        expert=EXPERTS["clinical"], query="case", weight=1.0,
        registry=ToolRegistry(), llm=SequenceLLM([{}, _final("fixed")]),
        request_allowed_tools=[], recorder=trace,
    ))
    assert [call.stage for call in trace.llm_calls] == [
        "expert:clinical:final", "expert:clinical:format_repair:1",
    ]
