from __future__ import annotations

import json
import os
import sys

import pytest
from pydantic import ValidationError

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.routers.sse import SSE_DONE, openai_sse_chunk
from agent_api.app.schemas.openai_schemas import ChatCompletionRequest
from agent_api.app.services.agent_execution import (
    AGENT_MODEL_ID,
    build_moe_orchestrator,
    public_moe_allowed_tools,
)
from agent_api.app.tools.tool_registry import ToolRegistry


def test_agent_model_is_moe_only():
    """验证对外暴露的 agent 模型只有 MoE 这一条路径。"""
    assert AGENT_MODEL_ID == "agent-moe"
    assert ChatCompletionRequest(messages=[{"role": "user", "content": "狗吐了"}]).model == AGENT_MODEL_ID

    with pytest.raises(ValidationError):
        ChatCompletionRequest(
            model="agent-plan-solve",
            messages=[{"role": "user", "content": "狗吐了"}],
        )


def test_public_moe_default_tools_are_restricted(monkeypatch):
    """验证公开 MoE 默认工具集是受限的。"""
    monkeypatch.delenv("AGENT_PUBLIC_MOE_ALLOWED_TOOLS", raising=False)
    allowed = public_moe_allowed_tools([
        "rag.search", "mcp.web_search.web_search", "sql.search", "unsafe.tool",
    ])

    assert allowed == ["rag.search", "mcp.web_search.web_search"]


def test_moe_factory_caps_aggregator_output_at_default_budget(monkeypatch):
    """验证 MoE 工厂把综合器输出限制在默认预算内。"""
    monkeypatch.delenv("MOE_FINAL_ANSWER_MAX_TOKENS", raising=False)
    orchestrator = build_moe_orchestrator(
        registry=ToolRegistry(), max_tokens=9000, allowed_tools=[],
    )

    assert orchestrator.config.max_tokens == 2500


def test_final_answer_budget_follows_env_override(monkeypatch):
    """验证最终回答预算会跟随环境变量覆盖。"""
    monkeypatch.setenv("MOE_FINAL_ANSWER_MAX_TOKENS", "1200")

    capped = build_moe_orchestrator(registry=ToolRegistry(), max_tokens=9000, allowed_tools=[])
    defaulted = build_moe_orchestrator(registry=ToolRegistry(), allowed_tools=[])
    under_budget = build_moe_orchestrator(registry=ToolRegistry(), max_tokens=800, allowed_tools=[])

    assert capped.config.max_tokens == 1200
    assert defaulted.config.max_tokens == 1200
    assert under_budget.config.max_tokens == 800
    assert capped._aggregator_max_tokens("狗吐了") == 1200


def test_invalid_final_answer_budget_env_falls_back_to_default(monkeypatch):
    """验证非法的回答预算环境变量会回退到默认值。"""
    monkeypatch.setenv("MOE_FINAL_ANSWER_MAX_TOKENS", "not-a-number")

    orchestrator = build_moe_orchestrator(registry=ToolRegistry(), allowed_tools=[])

    assert orchestrator.config.max_tokens == 2500


def test_debug_timing_is_disabled_by_default():
    """验证调试计时默认关闭。"""
    request = ChatCompletionRequest(messages=[{"role": "user", "content": "狗吐了"}])

    assert request.debug_timing is False


def test_shared_sse_encoder_preserves_truncated_finish_reason():
    """验证共享 SSE 编码器会保留 truncated 结束原因。"""
    chunk = openai_sse_chunk(
        request_id="chatcmpl-test", model="agent-moe", created=1,
        content="partial", status="streaming", detail={}, finish="truncated",
    )
    payload = json.loads(chunk.removeprefix("data: ").strip())

    assert payload["choices"][0]["finish_reason"] == "truncated"
    assert payload["choices"][0]["delta"]["content"] == "partial"
    assert SSE_DONE == "data: [DONE]\n\n"
