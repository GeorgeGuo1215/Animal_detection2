from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.concurrency import AsyncResourceLimiter
from app.llm.llm_client_stream import AsyncOpenAIStreamClient
from app.llm.openai_compat import build_chat_payload
from app.schemas.openai_schemas import ChatCompletionChoice, ChatMessage
from app.services.moe.critic import CriticResult
from app.services.moe.orchestrator import (
    MoEOrchestrator,
    OrchestratorConfig,
    normalize_finish_reason,
)
from app.tools.tool_registry import ToolRegistry


class _StageOrchestrator(MoEOrchestrator):
    async def _run_experts(self, query, decision, recorder):
        return [{
            "expert": "clinical", "name_zh": "clinical", "weight": 1.0,
            "conclusion": "opinion", "evidence": [], "risks": [], "confidence": 0.8,
            "rag_hits": 0, "rag_best_score": 0.0, "tools_used": [],
        }]

    async def _critique(self, query, opinions, emergency, recorder):
        return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")


class _LengthLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        return {
            "choices": [{"message": {"content": "partial"}, "finish_reason": "length"}],
            "usage": {},
        }


class _LengthStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        yield {"content": "partial", "finish_reason": None}
        yield {"content": None, "finish_reason": "length"}


class _EmptyStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        self.last_kwargs = kwargs
        yield {"content": None, "finish_reason": "stop"}


class _InterruptedStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        yield {"content": "partial", "finish_reason": None}
        raise RuntimeError("upstream disconnected")


class _FallbackAnswerLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        self.last_kwargs = kwargs
        return {
            "choices": [{"message": {"content": "fallback answer"}, "finish_reason": "stop"}],
            "usage": {},
        }


class _EmptyAnswerLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        return {
            "choices": [{"message": {"content": ""}, "finish_reason": "stop"}],
            "usage": {},
        }


def test_length_is_normalized_to_truncated():
    assert normalize_finish_reason("length") == "truncated"
    assert normalize_finish_reason("max_tokens") == "truncated"
    assert normalize_finish_reason("stop") == "stop"


def test_schema_accepts_truncated_finish_reason():
    choice = ChatCompletionChoice(
        message=ChatMessage(role="assistant", content="partial"),
        finish_reason="truncated",
    )
    assert choice.finish_reason == "truncated"


def test_chat_payload_can_disable_deepseek_thinking_for_low_latency_stages():
    payload = build_chat_payload(
        model="deepseek-v4-flash",
        messages=[{"role": "user", "content": "case"}],
        temperature=0.0,
        max_tokens=100,
        thinking=False,
    )

    assert payload["thinking"] == {"type": "disabled"}


def test_non_stream_orchestrator_preserves_truncation():
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_LengthLLM(),
        config=OrchestratorConfig(max_tokens=10, allowed_tools=[]),
    )

    answer, _ = asyncio.run(orchestrator.run(query="case"))

    assert answer == "partial"
    assert orchestrator.last_finish_reason == "truncated"


def test_stream_orchestrator_emits_truncated_final_event():
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_LengthLLM(), stream_llm=_LengthStreamLLM(),
        config=OrchestratorConfig(max_tokens=10, allowed_tools=[]),
    )

    async def collect():
        return [event async for event in orchestrator.stream(query="case")]

    events = asyncio.run(collect())

    assert "".join(event["content"] for event in events).endswith("partial")
    assert events[-1]["finish"] == "truncated"
    assert orchestrator.last_finish_reason == "truncated"


def test_empty_aggregator_stream_uses_non_stream_fallback_and_emits_content():
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_FallbackAnswerLLM(), stream_llm=_EmptyStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        return [event async for event in orchestrator.stream(query="case")]

    events = asyncio.run(collect())
    assert "".join(event["content"] for event in events).endswith("fallback answer")
    assert any(
        event.get("status") == "generating" and (event.get("detail") or {}).get("retry") == 1
        for event in events
    )
    assert events[-1]["finish"] == "stop"
    assert orchestrator.stream_llm.last_kwargs["thinking"] is False
    assert orchestrator.llm.last_kwargs["thinking"] is False


def test_empty_stream_and_empty_fallback_raise_explicit_generation_error():
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_EmptyAnswerLLM(), stream_llm=_EmptyStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        return [event async for event in orchestrator.stream(query="case")]

    with pytest.raises(RuntimeError, match="no visible content after fallback"):
        asyncio.run(collect())


def test_interrupted_partial_stream_is_not_marked_as_normal_completion():
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_FallbackAnswerLLM(), stream_llm=_InterruptedStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        return [event async for event in orchestrator.stream(query="case")]

    with pytest.raises(RuntimeError, match="interrupted after partial output"):
        asyncio.run(collect())


def test_stream_client_preserves_upstream_length_reason(monkeypatch: pytest.MonkeyPatch):
    import app.llm.llm_client_stream as stream_module

    class FakeResponse:
        def raise_for_status(self):
            return None

        async def aiter_lines(self):
            yield 'data: {"choices":[{"delta":{"content":"partial"},"finish_reason":null}]}'
            yield 'data: {"choices":[{"delta":{},"finish_reason":"length"}]}'
            yield "data: [DONE]"

    class FakeContext:
        async def __aenter__(self):
            return FakeResponse()

        async def __aexit__(self, exc_type, exc, tb):
            return False

    class FakeClient:
        def stream(self, *args, **kwargs):
            return FakeContext()

    class Limits:
        acquire_timeout_s = 1.0
        llm = AsyncResourceLimiter("llm", 1)

    monkeypatch.setattr(stream_module, "get_resource_limits", lambda: Limits())
    client = AsyncOpenAIStreamClient.__new__(AsyncOpenAIStreamClient)
    client.base_url = "https://example.test"
    client.api_key = "test"
    client.model = "test"
    client._client = FakeClient()

    async def collect():
        return [event async for event in client.chat_stream_events(
            messages=[{"role": "user", "content": "case"}], max_tokens=10
        )]

    events = asyncio.run(collect())
    assert events == [
        {"content": "partial", "finish_reason": None},
        {"content": None, "finish_reason": "length"},
    ]
