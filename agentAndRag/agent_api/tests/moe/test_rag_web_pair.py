"""离线单测：医学类 rag+web 联用补齐；Critic 兽医回退约束。

Run: pytest tests/moe/test_rag_web_pair.py
"""
from __future__ import annotations

import asyncio
import os
import sys
from typing import Any, Dict, List
from unittest.mock import AsyncMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.plan_and_solve import ensure_rag_and_web_tool_steps
from app.services.moe.critic import (
    _OWNER_FALLBACK_CONSTRAINT,
    _VET_FALLBACK_CONSTRAINT,
    review,
)
from app.services.moe.orchestrator import _block_fallback_text, _opinions_used_web


def test_ensure_pairs_rag_and_web_for_medical():
    steps = [{"type": "tool", "tool_name": "rag.search", "arguments": {"query": "vomit dog"}}]
    out = ensure_rag_and_web_tool_steps(
        steps,
        visible_tools=["rag.search", "mcp.web_search.web_search", "sql.search"],
        query="金毛犬呕吐鉴别诊断",
        rag_query="golden retriever vomit differential",
    )
    names = [s["tool_name"] for s in out]
    assert "rag.search" in names
    assert "mcp.web_search.web_search" in names


def test_ensure_adds_rag_when_only_web():
    steps = [{
        "type": "tool",
        "tool_name": "mcp.web_search.web_search",
        "arguments": {"query": "犬胰腺炎"},
    }]
    out = ensure_rag_and_web_tool_steps(
        steps,
        visible_tools=["rag.search", "mcp.web_search.web_search"],
        query="犬急性胰腺炎用药边界",
    )
    names = [s["tool_name"] for s in out]
    assert names[0] == "rag.search"
    assert "mcp.web_search.web_search" in names


def test_ensure_skips_when_web_unavailable():
    steps = [{"type": "tool", "tool_name": "rag.search", "arguments": {}}]
    out = ensure_rag_and_web_tool_steps(
        steps,
        visible_tools=["rag.search"],
        query="犬呕吐鉴别",
    )
    assert len(out) == 1
    assert out[0]["tool_name"] == "rag.search"


def test_ensure_skips_pure_vitals_plan():
    steps = [{"type": "tool", "tool_name": "vitals.summary", "arguments": {}}]
    out = ensure_rag_and_web_tool_steps(
        steps,
        visible_tools=["rag.search", "mcp.web_search.web_search", "vitals.summary"],
        query="看看我家狗今天心率呼吸体温",
    )
    assert [s["tool_name"] for s in out] == ["vitals.summary"]


def test_ensure_skips_non_medical():
    steps = [{"type": "tool", "tool_name": "rag.search", "arguments": {}}]
    out = ensure_rag_and_web_tool_steps(
        steps,
        visible_tools=["rag.search", "mcp.web_search.web_search"],
        query="推荐一本科幻小说",
    )
    assert len(out) == 1


def test_opinions_used_web_detects_tools():
    assert _opinions_used_web([{
        "tools_used": ["rag.search", "mcp.web_search.web_search"],
    }])
    assert not _opinions_used_web([{"tools_used": ["rag.search"]}])


def test_block_fallback_role_split():
    owner = _block_fallback_text("pet_owner")
    vet = _block_fallback_text("veterinarian")
    assert "线下" in owner or "兽医" in owner
    assert "请立即就医" not in vet
    assert "联系或前往专业兽医" not in vet
    assert "AI 助手" in vet or "临床参考" in vet or "院内" in vet


def test_critic_fallback_constraint_role_split():
    async def _run(role: str) -> List[str]:
        llm = AsyncMock()
        llm.chat = AsyncMock(side_effect=RuntimeError("boom"))
        llm.model = "test"
        result = await review(
            query="犬呕吐",
            expert_opinions=[],
            emergency=False,
            llm=llm,
            user_role=role,
        )
        return result.constraints

    owner_c = asyncio.run(_run("pet_owner"))
    vet_c = asyncio.run(_run("veterinarian"))
    assert owner_c == [_OWNER_FALLBACK_CONSTRAINT]
    assert vet_c == [_VET_FALLBACK_CONSTRAINT]
    assert "信息不足且无明确红旗时，先追问关键症状" in owner_c[0]
    assert "已有明确红旗时不得延迟紧急就医建议" in owner_c[0]
    assert "线下就医" not in vet_c[0]
    assert "宠主" in vet_c[0]
