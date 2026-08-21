"""Assert each MoE expert forces rag.search category to its rag_categories."""
from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.experts import (
    EXPERTS,
    _BEHAVIOR_RAG_CATEGORIES,
    _CLINICAL_RAG_CATEGORIES,
    _NUTRITION_RAG_CATEGORIES,
    _PHARMACY_RAG_CATEGORIES,
    run_expert,
)
from app.services.moe.retrieval_policy import RetrievalRequirement


class FakeRegistry:
    def __init__(self):
        """初始化该测试替身。"""
        self.calls = []

    def list_tools(self):
        """列出测试替身所暴露的工具。"""
        from app.tools.tool_registry import ToolSpec

        async def _rag(**kwargs):
            """返回测试用的假 RAG 检索结果。"""
            return {"hits": []}

        return [ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_rag)]

    def get(self, name):
        """按键读取测试替身中的值。"""
        return next((tool for tool in self.list_tools() if tool.name == name), None)

    async def call(self, name, args):
        """触发一次测试替身调用。"""
        self.calls.append((name, dict(args)))
        return {"hits": []}


class FakeLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        return {
            "choices": [
                {
                    "message": {
                        "content": json.dumps(
                            {
                                "action": "final",
                                "opinion": {
                                    "conclusion": "ok",
                                    "evidence": [],
                                    "risks": [],
                                    "confidence": 0.5,
                                },
                            }
                        )
                    }
                }
            ]
        }


def _run(expert_key: str):
    """运行本用例的异步主体。"""
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS[expert_key],
            query="test question",
            weight=0.5,
            registry=reg,
            llm=llm,
            recorder=None,
            retrieval_requirement=RetrievalRequirement(
                required_tools=(), recommended_tools=("rag.search",),
                require_web_on_rag_failure=False, reason="test assignment",
                tool_queries=(("rag.search", "test question"),),
            ),
        )
    )
    assert reg.calls, f"{expert_key}: rag.search not called"
    return reg.calls[0][1]


def test_clinical_forces_rag_categories():
    """验证临床专家会强制使用对应 RAG 类目。"""
    args = _run("clinical")
    assert args["category"] == list(_CLINICAL_RAG_CATEGORIES)


def test_nutrition_forces_rag_categories():
    """验证营养专家会强制使用对应 RAG 类目。"""
    args = _run("nutrition")
    assert args["category"] == list(_NUTRITION_RAG_CATEGORIES)


def test_pharmacy_forces_rag_categories():
    """验证药房专家会强制使用对应 RAG 类目。"""
    args = _run("pharmacy")
    assert args["category"] == list(_PHARMACY_RAG_CATEGORIES)


def test_behavior_forces_rag_categories():
    """验证行为专家会强制使用对应 RAG 类目。"""
    args = _run("behavior")
    assert args["category"] == list(_BEHAVIOR_RAG_CATEGORIES)


def test_task_driven_rag_always_uses_expert_category_scope():
    """验证任务驱动的 RAG 始终使用专家类目作用域。"""
    args = _run("pharmacy")
    assert "wrong.category" not in args["category"]
    assert "pharmacy.*" in args["category"]
