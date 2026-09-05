from __future__ import annotations

from agent_api.app.prompts.moe_aggregator import build_aggregator_prompt
from agent_api.app.services.moe.orchestration.service import (
    _collect_retrieved_sources,
    _evidence_audit_summary,
)


def _opinion(status: str, *, matched_hit_ids=()):
    return [{
        "expert": "clinical",
        "tool_results": [{
            "tool_name": "rag.search",
            "ok": True,
            "sufficiency": {
                "status": status,
                "reason": "fixture",
                "matched_hit_ids": list(matched_hit_ids),
            },
            "result": {"hits": [
                {"source_path": "books/a.mmd", "text": "first evidence"},
                {"source_path": "books/b.mmd", "text": "second evidence"},
            ]},
        }],
    }]


def test_unsupported_and_unknown_rag_results_are_not_citable():
    """验证未支持或未知的本地证据完全不会进入终答引用目录。"""
    assert _collect_retrieved_sources(_opinion("unsupported")) == []
    assert _collect_retrieved_sources(_opinion("unknown")) == []


def test_partial_rag_result_only_exposes_explicitly_matched_fragments():
    """验证部分支持只放行充分性审计明确匹配的片段。"""
    sources = _collect_retrieved_sources(_opinion("partial", matched_hit_ids=("h2",)))

    assert len(sources) == 1
    assert sources[0]["source_path"] == "books/b.mmd"
    assert sources[0]["excerpt"] == "second evidence"
    assert sources[0]["evidence_status"] == "partial"


def test_supported_rag_result_keeps_all_returned_sources():
    """验证充分支持时仍保留原有多来源引用能力。"""
    sources = _collect_retrieved_sources(_opinion("supported", matched_hit_ids=("h1",)))
    assert [item["source_path"] for item in sources] == ["books/a.mmd", "books/b.mmd"]


def test_web_results_require_audited_support_before_becoming_citable():
    """验证网页检索成功不等于证据成功，未知/不支持来源不会进入引用目录。"""
    base = {
        "expert": "pharmacy",
        "tool_results": [{
            "tool_name": "mcp.web_search.web_search",
            "ok": True,
            "result": {"results": [{
                "title": "Commercial dosage page",
                "url": "https://example.test/dose",
                "content": "A specific dose is advertised here.",
            }]},
        }],
    }
    assert _collect_retrieved_sources([base]) == []
    base["tool_results"][0]["sufficiency"] = {
        "status": "unsupported", "matched_hit_ids": [],
    }
    assert _collect_retrieved_sources([base]) == []
    base["tool_results"][0]["sufficiency"] = {
        "status": "partial", "matched_hit_ids": ["h1"],
    }
    sources = _collect_retrieved_sources([base])
    assert len(sources) == 1
    assert sources[0]["url"] == "https://example.test/dose"


def test_aggregator_prompt_contains_program_level_conservative_fallback():
    """验证证据不足时的保守表达是终答层硬规则，而非仅依赖二次补查。"""
    prompt = build_aggregator_prompt(
        base_prompt="base",
        user_role="veterinarian",
        has_retrieved_sources=True,
        emergency=False,
        critic_constraints=[],
    )

    assert "unsupported、unknown" in prompt
    assert "禁止用模型记忆补齐具体剂量、阈值、洗脱天数" in prompt
    assert "partial 只支撑" in prompt


def test_evidence_audit_uses_best_result_from_bounded_fallback_wave():
    """验证同一目标首轮不足、扩类补齐后，终答只按合并后的充分状态判断。"""
    opinion = [{
        "expert": "pharmacy",
        "tool_results": [
            {
                "tool_name": "rag.search",
                "arguments": {"query": "canine NSAID washout"},
                "evidence_goal": "核对洗脱要求",
                "scope": "expert",
                "sufficiency": {"status": "unsupported", "reason": "分类内未命中"},
            },
            {
                "tool_name": "rag.search",
                "arguments": {"query": "canine NSAID washout"},
                "evidence_goal": "核对洗脱要求",
                "scope": "expanded",
                "sufficiency": {
                    "status": "supported",
                    "reason": "扩类命中直接证据",
                    "matched_hit_ids": ["h1"],
                },
            },
        ],
    }]

    audit = _evidence_audit_summary(opinion)
    assert len(audit) == 1
    assert audit[0]["status"] == "supported"
    assert audit[0]["reason"] == "扩类命中直接证据"
    assert [item["scope"] for item in audit[0]["attempts"]] == ["expert", "expanded"]
