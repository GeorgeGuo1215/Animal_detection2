from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.routers.routes_openai import _resolve_request_allowed_tools
from app.schemas.openai_schemas import ChatCompletionRequest, ChatMessage


AVAILABLE = {"rag.search", "mcp.vitals_alert.check_vitals", "mcp.web_search.web_search"}


def _tool(name):
    return {"type": "function", "function": {"name": name, "parameters": {"type": "object"}}}


def _request(**kwargs):
    return ChatCompletionRequest(messages=[ChatMessage(role="user", content="case")], **kwargs)


def test_explicit_empty_tools_disables_all_tools():
    assert _resolve_request_allowed_tools(_request(tools=[]), AVAILABLE) == []


def test_tool_choice_none_overrides_declared_tools():
    request = _request(tools=[_tool("rag.search")], tool_choice="none")
    assert _resolve_request_allowed_tools(request, AVAILABLE) == []


def test_structured_tool_choice_selects_one_declared_function():
    request = _request(
        tools=[_tool("rag.search"), _tool("mcp.web_search.web_search")],
        tool_choice={"type": "function", "function": {"name": "rag.search"}},
    )
    assert _resolve_request_allowed_tools(request, AVAILABLE) == ["rag.search"]


def test_undeclared_structured_tool_choice_disables_tools():
    request = _request(
        tools=[_tool("rag.search")],
        tool_choice={"type": "function", "function": {"name": "mcp.web_search.web_search"}},
    )
    assert _resolve_request_allowed_tools(request, AVAILABLE) == []


def test_default_tools_include_pethealth_vitals_alert_when_registered():
    assert "mcp.vitals_alert.check_vitals" in _resolve_request_allowed_tools(_request(), AVAILABLE)
