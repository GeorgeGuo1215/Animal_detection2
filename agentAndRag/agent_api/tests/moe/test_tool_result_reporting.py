from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.routers.routes_openai import (
    _stream_multi_turn_agent,
    _make_decide_prompt,
    _summarize_tool_result,
    _tool_complete_line_and_detail,
)
from app.tools.tool_registry import ToolRegistry, ToolSpec


def _web_result():
    return {
        "status": "OK",
        "count": 1,
        "results": [{
            "title": "Clinical guideline",
            "url": "https://example.test/guideline",
            "content": "Current veterinary evidence.",
        }],
    }


def test_web_result_summary_does_not_collapse_to_status_only():
    summary = _summarize_tool_result(_web_result())

    assert "Clinical guideline" in summary
    assert "https://example.test/guideline" in summary
    assert summary != "status=OK "


def test_multiturn_decision_prompt_counts_web_results():
    prompt = _make_decide_prompt(
        "current evidence",
        [{"tool_name": "mcp.web_search.web_search", "result": _web_result()}],
        ["mcp.web_search.web_search"],
    )

    assert '"hits_count": 1' in prompt
    assert "Clinical guideline" in prompt


def test_debug_tool_completion_records_arguments_and_web_result():
    _, detail = _tool_complete_line_and_detail(
        "mcp.web_search.web_search",
        _web_result(),
        arguments={"query": "feline urinary obstruction"},
        round_num=2,
        include_debug_result=True,
    )

    assert detail["arguments"]["query"] == "feline urinary obstruction"
    assert detail["result"]["results"][0]["title"] == "Clinical guideline"
    assert detail["hits_count"] == 1


def test_multiturn_stops_before_repeating_an_exact_tool_call(monkeypatch):
    calls = []
    registry = ToolRegistry()

    async def web_search(**kwargs):
        calls.append(dict(kwargs))
        return _web_result()

    registry.register(ToolSpec(
        "mcp.web_search.web_search", "web", {"type": "object"}, web_search,
    ))

    class DecisionLLM:
        model = "fake"

        async def chat(self, **kwargs):
            decision = {
                "action": "call_tool",
                "tool_name": "mcp.web_search.web_search",
                "arguments": {"query": "same query"},
                "reason": "test",
            }
            return {"choices": [{"message": {"content": json.dumps(decision)}}]}

    class StreamLLM:
        async def chat_stream(self, **kwargs):
            yield "final"

    import app.routers.routes_openai as routes

    monkeypatch.setattr(routes, "get_registry", lambda: registry)
    monkeypatch.setattr(routes, "get_shared_async_client", lambda: DecisionLLM())
    monkeypatch.setattr(routes, "get_shared_async_stream_client", lambda: StreamLLM())

    async def collect():
        return [chunk async for chunk in _stream_multi_turn_agent(
            request_id="test",
            model="agent-multi-turn",
            query="question",
            system_context="",
            conversation_history=[],
            temperature=0.2,
            max_tokens=100,
            allowed_tools=["mcp.web_search.web_search"],
            debug_timing=True,
        )]

    chunks = asyncio.run(collect())

    assert calls == [{"query": "same query"}]
    assert any('"agent_status":"decision_complete"' in chunk for chunk in chunks)
    assert any('"agent_status":"tool_skipped"' in chunk for chunk in chunks)
