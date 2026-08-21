from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.critic import review
from app.services.moe.task_policy import parse_task_policy


class MalformedLLM:
    model = "fake"

    def __init__(self, content):
        """初始化该测试替身。"""
        self.content = content

    async def chat(self, **kwargs):
        """测试用假 LLM 聊天实现。"""
        return {"choices": [{"message": {"content": self.content}}]}


def test_malformed_critic_output_requires_revision():
    """验证审核器输出畸形时会要求修订。"""
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
    """验证缺少审核裁决时会要求修订。"""
    result = asyncio.run(review(
        query="cat case",
        expert_opinions=[],
        emergency=False,
        llm=MalformedLLM('{"issues": []}'),
    ))

    assert result.verdict == "revise"
    assert result.constraints


def test_malformed_unified_policy_falls_back_to_clinical_expert():
    """验证畸形统一策略会回退到临床专家。"""
    policy = parse_task_policy("not json")
    decision = policy.as_router_decision()
    assert decision.out_of_scope is False
    assert decision.selected_experts == ["clinical"]
    assert decision.scores["clinical"] == 6.0
