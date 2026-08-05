from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.critic import review
from app.services.moe.router import route


class MalformedLLM:
    model = "fake"

    def __init__(self, content):
        self.content = content

    async def chat(self, **kwargs):
        return {"choices": [{"message": {"content": self.content}}]}


def test_malformed_critic_output_requires_revision():
    result = asyncio.run(review(
        query="cat case",
        expert_opinions=[],
        emergency=False,
        llm=MalformedLLM("not json"),
        user_role="veterinarian",
    ))

    assert result.verdict == "revise"
    assert result.constraints
    assert "解析失败" in result.reason


def test_missing_critic_verdict_requires_revision():
    result = asyncio.run(review(
        query="cat case",
        expert_opinions=[],
        emergency=False,
        llm=MalformedLLM('{"issues": []}'),
    ))

    assert result.verdict == "revise"
    assert result.constraints


def test_malformed_router_output_falls_back_to_clinical_expert():
    decision = asyncio.run(route(
        query="cat urinary obstruction",
        user_role="veterinarian",
        llm=MalformedLLM("not json"),
    ))

    assert decision.out_of_scope is False
    assert decision.selected_experts == ["clinical"]
    assert decision.scores["clinical"] == 6.0


def test_router_json_without_valid_scores_falls_back_to_clinical_expert():
    decision = asyncio.run(route(
        query="cat urinary obstruction",
        user_role="veterinarian",
        llm=MalformedLLM('{"scores": {}, "emergency": false}'),
    ))

    assert decision.out_of_scope is False
    assert decision.selected_experts == ["clinical"]
