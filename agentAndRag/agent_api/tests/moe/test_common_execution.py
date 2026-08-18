from __future__ import annotations

import json
import os
import sys

import pytest
from pydantic import ValidationError

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.routers.sse import SSE_DONE, openai_sse_chunk
from app.schemas.openai_schemas import ChatCompletionRequest
from app.services.agent_execution import (
    AGENT_MODEL_ID,
    build_moe_orchestrator,
    public_moe_allowed_tools,
)
from app.tools.tool_registry import ToolRegistry


def test_agent_model_is_moe_only():
    assert AGENT_MODEL_ID == "agent-moe"
    assert ChatCompletionRequest(messages=[{"role": "user", "content": "狗吐了"}]).model == AGENT_MODEL_ID

    with pytest.raises(ValidationError):
        ChatCompletionRequest(
            model="agent-plan-solve",
            messages=[{"role": "user", "content": "狗吐了"}],
        )


def test_public_moe_default_tools_are_restricted(monkeypatch):
    monkeypatch.delenv("AGENT_PUBLIC_MOE_ALLOWED_TOOLS", raising=False)
    allowed = public_moe_allowed_tools([
        "rag.search", "mcp.web_search.web_search", "sql.search", "unsafe.tool",
    ])

    assert allowed == ["rag.search", "mcp.web_search.web_search"]


def test_moe_factory_caps_aggregator_output_at_default_budget(monkeypatch):
    monkeypatch.delenv("MOE_FINAL_ANSWER_MAX_TOKENS", raising=False)
    orchestrator = build_moe_orchestrator(
        registry=ToolRegistry(), max_tokens=9000, allowed_tools=[],
    )

    assert orchestrator.config.max_tokens == 2500


def test_final_answer_budget_follows_env_override(monkeypatch):
    monkeypatch.setenv("MOE_FINAL_ANSWER_MAX_TOKENS", "1200")

    capped = build_moe_orchestrator(registry=ToolRegistry(), max_tokens=9000, allowed_tools=[])
    defaulted = build_moe_orchestrator(registry=ToolRegistry(), allowed_tools=[])
    under_budget = build_moe_orchestrator(registry=ToolRegistry(), max_tokens=800, allowed_tools=[])

    assert capped.config.max_tokens == 1200
    assert defaulted.config.max_tokens == 1200
    assert under_budget.config.max_tokens == 800
    assert capped._aggregator_max_tokens("狗吐了") == 1200


def test_invalid_final_answer_budget_env_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("MOE_FINAL_ANSWER_MAX_TOKENS", "not-a-number")

    orchestrator = build_moe_orchestrator(registry=ToolRegistry(), allowed_tools=[])

    assert orchestrator.config.max_tokens == 2500


def test_debug_timing_is_disabled_by_default():
    request = ChatCompletionRequest(messages=[{"role": "user", "content": "狗吐了"}])

    assert request.debug_timing is False


def test_shared_sse_encoder_preserves_truncated_finish_reason():
    chunk = openai_sse_chunk(
        request_id="chatcmpl-test", model="agent-moe", created=1,
        content="partial", status="streaming", detail={}, finish="truncated",
    )
    payload = json.loads(chunk.removeprefix("data: ").strip())

    assert payload["choices"][0]["finish_reason"] == "truncated"
    assert payload["choices"][0]["delta"]["content"] == "partial"
    assert SSE_DONE == "data: [DONE]\n\n"
