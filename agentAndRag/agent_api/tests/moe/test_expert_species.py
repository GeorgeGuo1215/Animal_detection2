"""Verify run_expert injects species/breed into the RAG query and the expert payload.

Uses fake registry + fake LLM (no DB / no network). ASCII sentinels only.
Run: pytest tests/moe/test_expert_species.py
"""
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.experts import EXPERTS, run_expert, _SPECIES_BREED_GUARD


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
        self.calls.append((name, args))
        return {"hits": []}


class FakeLLM:
    model = "fake"

    def __init__(self):
        self.last_messages = None
        self.plan_tools = ["rag.search"]
        self.calls = 0

    async def chat(self, messages=None, **kwargs):
        self.last_messages = messages
        self.calls += 1
        if self.calls <= len(self.plan_tools):
            content = json.dumps({
                "action": "tool",
                "tool_name": self.plan_tools[self.calls - 1],
                "arguments": {},
                "reason": "test",
            })
            return {"choices": [{"message": {"content": content}}]}
        content = json.dumps(
            {
                "action": "final",
                "opinion": {"conclusion": "ok", "evidence": [], "risks": [], "confidence": 0.5},
            }
        )
        return {"choices": [{"message": {"content": content}}]}


def _user_content(messages):
    return [m for m in messages if m["role"] == "user"][0]["content"]


def _system_content(messages):
    return [m for m in messages if m["role"] == "system"][0]["content"]


def test_run_expert_injects_species():
    reg, llm = FakeRegistry(), FakeLLM()
    res = asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="my pet keeps vomiting",
            weight=0.8,
            registry=reg,
            llm=llm,
            species_en="cat",
            species_zh="ZH_SENTINEL",
            recorder=None,
        )
    )
    assert reg.calls, "rag.search was not called"
    rag_args = reg.calls[0][1]
    assert "cat" in rag_args["query"]

    user_content = _user_content(llm.last_messages)
    assert "ZH_SENTINEL" in user_content
    assert "species" in user_content
    assert res["expert"] == "clinical"


def test_run_expert_injects_breed_and_guard():
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="bulldog exercise plan",
            weight=0.8,
            registry=reg,
            llm=llm,
            species_en="dog",
            species_zh="犬（狗）",
            breed="French Bulldog",
            recorder=None,
        )
    )
    assert "French Bulldog" in reg.calls[0][1]["query"]
    sys_content = _system_content(llm.last_messages)
    assert "物种/品种特异化" in sys_content or _SPECIES_BREED_GUARD[:12] in sys_content
    user_content = _user_content(llm.last_messages)
    assert "French Bulldog" in user_content
    assert "breed" in user_content


def test_run_expert_without_species():
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="my pet keeps vomiting",
            weight=0.8,
            registry=reg,
            llm=llm,
            recorder=None,
        )
    )
    user_content = _user_content(llm.last_messages)
    assert "species" not in user_content


def test_run_expert_no_forced_rag_when_other_tools_planned():
    """Planner chose only web_search → must NOT force-insert rag.search."""
    from app.tools.tool_registry import ToolSpec

    class Reg(FakeRegistry):
        def list_tools(self):
            async def _web(**kwargs):
                return {"results": []}

            async def _rag(**kwargs):
                return {"hits": []}

            return [
                ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_rag),
                ToolSpec(
                    name="mcp.web_search.web_search",
                    description="fake web",
                    input_schema={"type": "object"},
                    handler=_web,
                ),
            ]

        def get(self, name):
            return next((tool for tool in self.list_tools() if tool.name == name), None)

    reg, llm = Reg(), FakeLLM()
    llm.plan_tools = ["mcp.web_search.web_search"]
    res = asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="latest feline UTI guideline",
            weight=0.8,
            registry=reg,
            llm=llm,
            recorder=None,
        )
    )
    assert "mcp.web_search.web_search" in res["tools_used"]
    assert "rag.search" not in res["tools_used"]
