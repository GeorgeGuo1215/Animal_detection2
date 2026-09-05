from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.routers.routes_openai import _resolve_request_allowed_tools
from agent_api.app.schemas.openai_schemas import ChatCompletionRequest, ChatMessage


AVAILABLE = {"rag.search", "mcp.vitals_alert.check_vitals", "mcp.web_search.web_search"}


def _tool(name):
    """拦截一次工具调用。"""
    return {"type": "function", "function": {"name": name, "parameters": {"type": "object"}}}


def _request(**kwargs):
    """构造测试用的 HTTP 请求对象。"""
    return ChatCompletionRequest(messages=[ChatMessage(role="user", content="case")], **kwargs)


def test_explicit_empty_tools_disables_all_tools():
    """验证显式传入空工具列表会关掉所有工具。"""
    assert _resolve_request_allowed_tools(_request(tools=[]), AVAILABLE) == []


def test_tool_choice_none_overrides_declared_tools():
    """验证 tool_choice=none 会覆盖已声明工具。"""
    request = _request(tools=[_tool("rag.search")], tool_choice="none")
    assert _resolve_request_allowed_tools(request, AVAILABLE) == []


def test_structured_tool_choice_selects_one_declared_function():
    """验证结构化 tool_choice 只会选中一个已声明函数。"""
    request = _request(
        tools=[_tool("rag.search"), _tool("mcp.web_search.web_search")],
        tool_choice={"type": "function", "function": {"name": "rag.search"}},
    )
    assert _resolve_request_allowed_tools(request, AVAILABLE) == ["rag.search"]


def test_undeclared_structured_tool_choice_disables_tools():
    """验证未声明的结构化 tool_choice 会禁用工具。"""
    request = _request(
        tools=[_tool("rag.search")],
        tool_choice={"type": "function", "function": {"name": "mcp.web_search.web_search"}},
    )
    assert _resolve_request_allowed_tools(request, AVAILABLE) == []


def test_default_tools_include_pethealth_vitals_alert_when_registered():
    """验证注册后默认工具包含 PetHealth 体征告警。"""
    assert "mcp.vitals_alert.check_vitals" in _resolve_request_allowed_tools(_request(), AVAILABLE)
