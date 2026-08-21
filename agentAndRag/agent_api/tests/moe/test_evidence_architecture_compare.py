"""A/B/C 隔离评测脚本的离线纯函数测试。"""
from __future__ import annotations

import argparse
import asyncio
import json

import pytest

from agent_api.tests.moe.run_evidence_architecture_compare_live import (
    CompareRunner,
    EvalCase,
    CompareRunner,
    LLMCall,
    ModeResult,
    _main,
    _flatten_evidence,
    build_summary,
    clean_direct_text,
    load_cases,
    normalize_case,
    percentile,
    policy_evidence_goals,
    redact_sensitive,
    render_summary,
    retrieval_metrics,
    source_matches,
    validate_output_structure,
    validate_fixture_contracts,
)


def test_normalize_case_accepts_future_fixture_variants():
    case = normalize_case({
        "id": "drug-1",
        "query": "犬泼尼松换卡洛芬需要核对什么？",
        "goals": [{"goal": "核对联用禁忌"}, "核对洗脱期"],
        "expected_book_id": "058",
        "expected_sources": ["books/083.mmd"],
        "category": "pharmacy.papich",
        "queries": [
            {"tool": "rag.search", "query": "canine corticosteroid NSAID washout"},
            {"tool": "web", "query": "official canine NSAID label corticosteroid"},
        ],
        "expected_domain": "fda.gov",
        "difficulty": "hard",
        "expected_intent": "D5",
        "output_variant": "default",
        "required_sections": ["治疗目标", "方案分层"],
        "history": [
            {"role": "user", "content": "上次每月发作一次"},
            {"role": "assistant", "content": "请记录发作持续时间"},
            {"role": "system", "content": "不应进入病例历史"},
        ],
    }, 1)

    assert case.case_id == "drug-1"
    assert case.evidence_goals == ("核对联用禁忌", "核对洗脱期")
    assert case.expected_sources == ("books/083.mmd", "058")
    assert case.categories == ("pharmacy.papich",)
    assert case.seed_rag_queries == ("canine corticosteroid NSAID washout",)
    assert case.seed_web_queries == ("official canine NSAID label corticosteroid",)
    assert case.requires_web is True
    assert case.expected_intent == "D5"
    assert case.output_variant == "default"
    assert case.required_sections == ("治疗目标", "方案分层")
    assert case.conversation_history == (
        {"role": "user", "content": "上次每月发作一次"},
        {"role": "assistant", "content": "请记录发作持续时间"},
    )
    assert case.metadata == {"difficulty": "hard"}


def test_normalize_case_requires_question():
    with pytest.raises(ValueError, match="question is required"):
        normalize_case({"case_id": "missing"}, 1)


def test_load_cases_supports_wrapped_root_and_rejects_duplicate_ids(tmp_path):
    path = tmp_path / "cases.json"
    path.write_text(json.dumps({"cases": [
        {"case_id": "a", "question": "q1"},
        {"case_id": "b", "question": "q2"},
    ]}), encoding="utf-8")
    assert [case.case_id for case in load_cases(path)] == ["a", "b"]

    path.write_text(json.dumps([
        {"case_id": "a", "question": "q1"},
        {"case_id": "a", "question": "q2"},
    ]), encoding="utf-8")
    with pytest.raises(ValueError, match="unique"):
        load_cases(path)


def test_load_cases_supports_generated_items_schema_without_runtime_label_leak(tmp_path):
    path = tmp_path / "generated.json"
    path.write_text(json.dumps({"items": [{
        "id": "EA-001",
        "question": "如何评估前脑疾病？",
        "atomic_evidence_goals": ["识别临床表现", "确定检查"],
        "suggested_queries": ["forebrain disease examination dog", "前脑疾病检查"],
        "source_path": "books/058.mmd",
        "book_id": "058",
        "evidence_excerpt": "A neurological examination should be performed.",
        "expected_intent": "D2",
        "output_variant": "default",
        "required_sections": ["问题表示", "鉴别诊断", "高风险项", "信息缺口", "下一步验证"],
    }]}), encoding="utf-8")
    case = load_cases(path)[0]
    assert case.evidence_goals == ("识别临床表现", "确定检查")
    assert case.expected_sources == ("books/058.mmd", "058")
    assert case.seed_rag_queries == ("forebrain disease examination dog", "前脑疾病检查")
    assert case.reference_answer == "A neurological examination should be performed."
    # 运行期证据目标只来自 Task Policy，不使用上面的标签和建议查询。
    assert policy_evidence_goals({}, case.question) == (case.question,)


def test_fixture_contract_validation_matches_production_d1_d8_registry():
    valid = normalize_case({
        "case_id": "d2", "question": "猫排尿困难",
        "expected_intent": "D2", "output_variant": "default",
        "required_sections": ["问题表示", "鉴别诊断", "高风险项", "信息缺口", "下一步验证"],
        "expected_sources": ["058"],
    }, 1)
    assert validate_fixture_contracts([valid], strict=True) == []

    invalid = normalize_case({
        "case_id": "bad", "question": "猫排尿困难",
        "expected_intent": "D2", "output_variant": "soap",
        "required_sections": ["S（主观）"],
    }, 2)
    issues = validate_fixture_contracts([invalid], strict=True)
    assert any("invalid output_variant" in issue for issue in issues)
    assert any("required_sections mismatch" in issue for issue in issues)
    assert any("expected_sources" in issue for issue in issues)


def test_source_match_and_retrieval_metrics_cover_books_and_domains():
    evidence = [
        {"book_id": "058", "source_path": "books/058.mmd", "text": "irrelevant first"},
        {"source_file": "083.mmd", "text": "second source"},
        {"url": "https://www.fda.gov/animal-veterinary/drug-label", "title": "label"},
    ]
    assert source_matches("058.pdf", evidence[0])
    assert source_matches("books/083.mmd", evidence[1])
    metrics = retrieval_metrics(
        evidence,
        expected_sources=("083",),
        expected_domains=("fda.gov",),
        k=3,
    )
    assert metrics == {
        "recall_at_k": 1.0,
        "mrr": 0.5,
        "expected_source_coverage": 1.0,
        "expected_domain_hit": 1.0,
    }


def test_retrieval_metrics_without_labels_does_not_claim_recall():
    metrics = retrieval_metrics(
        [{"book_id": "058"}], expected_sources=(), expected_domains=(), k=5,
    )
    assert metrics["recall_at_k"] == 0.0
    assert metrics["mrr"] == 0.0
    assert metrics["expected_source_coverage"] == 0.0


def test_flatten_evidence_deduplicates_chunks_but_keeps_query_provenance():
    calls = [
        {"tool": "rag.search", "query": "q1", "goal_id": "g1", "result": {
            "hits": [{"chunk_id": "c1", "text": "alpha"}, {"chunk_id": "c2", "text": "beta"}],
        }},
        {"tool": "rag.search", "query": "q2", "goal_id": "g2", "result": {
            "hits": [{"chunk_id": "c1", "text": "alpha"}],
        }},
        {"tool": "mcp.web_search.web_search", "query": "q3", "goal_id": "g3", "result": {
            "results": [{"url": "https://example.test/x", "content": "gamma"}],
        }},
    ]
    flattened = _flatten_evidence(calls)
    assert [item.get("chunk_id") or item.get("url") for item in flattened] == [
        "c1", "c2", "https://example.test/x",
    ]
    assert flattened[0]["query"] == "q1"
    assert flattened[2]["tool"] == "mcp.web_search.web_search"


def test_summary_aggregates_modes_and_renders_markdown():
    results = [
        ModeResult(
            case_id="c1", mode="B", recall_at_k=1.0, mrr=0.5,
            semantic_supported_rate=0.5, semantic_confidence=0.8,
            answer_accuracy=0.7, citation_precision=1.0,
            unsupported_claims=1, total_tokens=100, total_ms=1000,
            tool_calls=[{}], llm_calls=[{}], answer_grounding="partial",
        ),
        ModeResult(
            case_id="c2", mode="B", recall_at_k=0.0, mrr=0.0,
            semantic_supported_rate=1.0, semantic_confidence=0.9,
            answer_accuracy=0.9, citation_precision=0.5,
            total_tokens=200, total_ms=3000, tool_calls=[{}, {}],
            llm_calls=[{}, {}], answer_grounding="supported",
        ),
    ]
    summary = build_summary(results)
    mode = summary["modes"]["B"]
    assert mode["cases"] == 2
    assert mode["recall_at_k"] == 0.5
    assert mode["answer_accuracy"] == 0.8
    assert mode["total_tokens"] == 300
    assert mode["latency_ms_p50"] == 2000.0
    assert mode["latency_ms_p95"] == 2900.0
    assert "| B | 2 |" in render_summary(summary)


def test_percentile_handles_empty_single_and_interpolation():
    assert percentile([], 0.95) == 0.0
    assert percentile([7], 0.95) == 7.0
    assert percentile([0, 100], 0.95) == pytest.approx(95.0)


def test_validate_output_structure_requires_all_sections_in_contract_order():
    sections = ("问题表示", "鉴别诊断", "高风险项")
    answer = "**问题表示**\n猫排尿困难\n**鉴别诊断**\n尿闭\n**高风险项**\n高钾"
    order, passed = validate_output_structure(answer, sections)
    assert order == list(sections)
    assert passed is True

    order, passed = validate_output_structure(
        "**鉴别诊断**\n尿闭\n**问题表示**\n猫排尿困难", sections,
    )
    assert order == ["鉴别诊断", "问题表示"]
    assert passed is False
    _, passed = validate_output_structure("# 问题表示\n**问题表示**", ("问题表示",))
    assert passed is False


def test_policy_evidence_goals_never_uses_fixture_labels():
    policy = {"evidence_tasks": [
        {"reason": "核对NSAID与激素联用风险", "query": "ignored when reason exists"},
        {"query": "canine NSAID renal monitoring"},
    ]}
    assert policy_evidence_goals(policy, "fallback question") == (
        "核对NSAID与激素联用风险", "canine NSAID renal monitoring",
    )
    assert policy_evidence_goals({}, "fallback question") == ("fallback question",)


def test_query_planner_hard_stops_when_task_policy_has_no_evidence_tasks():
    runner = object.__new__(CompareRunner)
    case = EvalCase(case_id="no-tools", question="只整理成SOAP", evidence_goals=("整理",))
    calls = []
    planned = asyncio.run(runner._plan(
        case,
        calls,
        task_policy={"primary_intent": "D1", "evidence_tasks": []},
        selected_experts=("clinical",),
    ))
    assert planned == []
    assert calls == []

def test_redact_sensitive_masks_loaded_api_key(monkeypatch):
    monkeypatch.setenv("TEST_VENDOR_API_KEY", "top-secret-value")
    assert redact_sensitive("failed with top-secret-value") == "failed with ***REDACTED***"


def test_clean_direct_text_only_removes_outer_markdown_fence():
    assert clean_direct_text("```markdown\n**正文**\n```") == "**正文**"
    assert clean_direct_text("前言\n```json\n{}\n```\n结尾") == "前言\n```json\n{}\n```\n结尾"


def test_synthesis_accepts_long_chinese_markdown_without_json_parsing():
    long_detail = "病例事实包含引号\"、反斜杠\\与换行；不得因 JSON 转义丢失。" * 180
    body = (
        "**问题表示**\n" + long_detail + "\n"
        "**鉴别诊断**\n待核实\n"
        "**高风险项**\n尿闭风险\n"
        "**信息缺口**\n缺少检查\n"
        "**下一步验证**\n尽快就诊"
    )

    class FakeLLM:
        def __init__(self):
            self.kwargs = None

        async def chat(self, **kwargs):
            self.kwargs = kwargs
            return {
                "choices": [{"message": {"content": f"```markdown\n{body}\n```"}}],
                "usage": {"prompt_tokens": 10, "completion_tokens": 1400, "total_tokens": 1410},
            }

    runner = object.__new__(CompareRunner)
    runner.llm = FakeLLM()
    runner.max_tokens = 2200
    calls: list[LLMCall] = []
    answer = asyncio.run(runner._synthesize(
        normalize_case({"id": "long", "question": "猫排尿困难"}, 1),
        evidence=[], assessments=[], calls=calls,
        intent_id="D2", output_variant="default", evidence_required=False,
    ))
    assert answer == body
    assert "response_format" not in runner.llm.kwargs
    assert calls[0].stage == "synthesis"
    assert calls[0].total_tokens == 1410
    assert calls[0].error == ""


def test_direct_text_preserves_partial_output_and_records_length_finish_reason():
    class LengthLimitedLLM:
        async def chat(self, **_kwargs):
            return {
                "choices": [{"message": {"content": "**问题表示**\n部分输出"}, "finish_reason": "length"}],
                "usage": {"total_tokens": 99},
            }

    runner = object.__new__(CompareRunner)
    runner.llm = LengthLimitedLLM()
    calls: list[LLMCall] = []
    output = asyncio.run(runner._llm_text(
        stage="synthesis", system="system", payload={"question": "q"},
        calls=calls, max_tokens=100,
    ))
    assert output == "**问题表示**\n部分输出"
    assert calls[0].error == "finish_reason=length; direct text may be incomplete"


def test_dry_run_writes_preview_without_initializing_external_services(tmp_path):
    fixture = tmp_path / "fixture.json"
    fixture.write_text(json.dumps([{
        "case_id": "dry-1", "question": "猫尿闭如何分级？",
        "expected_sources": ["058"],
    }]), encoding="utf-8")
    out_dir = tmp_path / "out"
    args = argparse.Namespace(
        fixtures=str(fixture), limit=1, mode="all", top_k=5,
        max_queries=3, max_tokens=1000, concurrency=1,
        out_dir=str(out_dir), resume=False, dry_run=True,
    )
    assert asyncio.run(_main(args)) == 0
    preview = json.loads((out_dir / "fixture_preview.json").read_text(encoding="utf-8"))
    assert preview["cases"][0]["case_id"] == "dry-1"
    assert preview["modes"] == ["A", "B", "C"]
    assert not (out_dir / "results.jsonl").exists()
