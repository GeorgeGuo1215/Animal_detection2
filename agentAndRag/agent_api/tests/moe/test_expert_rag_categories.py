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


class FakeRegistry:
    def __init__(self):
        self.calls = []

    def list_tools(self):
        from app.tools.tool_registry import ToolSpec

        async def _rag(**kwargs):
            return {"hits": []}

        return [ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_rag)]

    def get(self, name):
        return next((tool for tool in self.list_tools() if tool.name == name), None)

    async def call(self, name, args):
        self.calls.append((name, dict(args)))
        return {"hits": []}


class FakeLLM:
    model = "fake"

    def __init__(self):
        self.calls = 0

    async def chat(self, messages=None, **kwargs):
        self.calls += 1
        if self.calls == 1:
            return {
                "choices": [
                    {
                        "message": {
                            "content": json.dumps(
                                {
                                    "action": "tool",
                                    "tool_name": "rag.search",
                                    "arguments": {"query": "x", "category": "wrong.category"},
                                }
                            )
                        }
                    }
                ]
            }
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
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS[expert_key],
            query="test question",
            weight=0.5,
            registry=reg,
            llm=llm,
            recorder=None,
        )
    )
    assert reg.calls, f"{expert_key}: rag.search not called"
    return reg.calls[0][1]


def test_clinical_forces_rag_categories():
    args = _run("clinical")
    assert args["category"] == list(_CLINICAL_RAG_CATEGORIES)


def test_nutrition_forces_rag_categories():
    args = _run("nutrition")
    assert args["category"] == list(_NUTRITION_RAG_CATEGORIES)


def test_pharmacy_forces_rag_categories():
    args = _run("pharmacy")
    assert args["category"] == list(_PHARMACY_RAG_CATEGORIES)


def test_behavior_forces_rag_categories():
    args = _run("behavior")
    assert args["category"] == list(_BEHAVIOR_RAG_CATEGORIES)


def test_planner_wrong_category_overwritten():
    """Planner may pass category=wrong.category; expert must overwrite."""
    args = _run("pharmacy")
    assert "wrong.category" not in args["category"]
    assert "pharmacy.*" in args["category"]
