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
        """驱动专家执行路径的测试替身。"""
        return [{
            "expert": "clinical", "name_zh": "clinical", "weight": 1.0,
            "conclusion": "opinion", "evidence": [], "risks": [], "confidence": 0.8,
            "rag_hits": 0, "rag_best_score": 0.0, "tools_used": [],
        }]

    async def _critique(self, query, opinions, emergency, recorder):
        """测试用审核器替身。"""
        return CriticResult(verdict="pass", issues=[], constraints=[], reason="ok")


class _LengthLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        return {
            "choices": [{"message": {"content": "partial"}, "finish_reason": "length"}],
            "usage": {},
        }


class _LengthStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        """产出测试用的聊天流事件。"""
        yield {"content": "partial", "finish_reason": None}
        yield {"content": None, "finish_reason": "length"}


class _EmptyStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        """产出测试用的聊天流事件。"""
        self.last_kwargs = kwargs
        yield {"content": None, "finish_reason": "stop"}


class _InterruptedStreamLLM:
    model = "fake"

    async def chat_stream_events(self, messages=None, **kwargs):
        """产出测试用的聊天流事件。"""
        yield {"content": "partial", "finish_reason": None}
        raise RuntimeError("upstream disconnected")


class _FallbackAnswerLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        self.last_kwargs = kwargs
        return {
            "choices": [{"message": {"content": "fallback answer"}, "finish_reason": "stop"}],
            "usage": {},
        }


class _EmptyAnswerLLM:
    model = "fake"

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        return {
            "choices": [{"message": {"content": ""}, "finish_reason": "stop"}],
            "usage": {},
        }


def test_length_is_normalized_to_truncated():
    """验证 length 结束原因会被归一成 truncated。"""
    assert normalize_finish_reason("length") == "truncated"
    assert normalize_finish_reason("max_tokens") == "truncated"
    assert normalize_finish_reason("stop") == "stop"


def test_schema_accepts_truncated_finish_reason():
    """验证 schema 接受 truncated 结束原因。"""
    choice = ChatCompletionChoice(
        message=ChatMessage(role="assistant", content="partial"),
        finish_reason="truncated",
    )
    assert choice.finish_reason == "truncated"


def test_chat_payload_can_disable_deepseek_thinking_for_low_latency_stages():
    """验证低延迟阶段可以关闭 DeepSeek thinking。"""
    payload = build_chat_payload(
        model="deepseek-v4-flash",
        messages=[{"role": "user", "content": "case"}],
        temperature=0.0,
        max_tokens=100,
        thinking=False,
    )

    assert payload["thinking"] == {"type": "disabled"}


def test_non_stream_orchestrator_preserves_truncation():
    """验证非流式编排器会保留截断结束原因。"""
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_LengthLLM(),
        config=OrchestratorConfig(max_tokens=10, allowed_tools=[]),
    )

    answer, _ = asyncio.run(orchestrator.run(query="case"))

    assert answer == "partial"
    assert orchestrator.last_finish_reason == "truncated"


def test_stream_orchestrator_emits_truncated_final_event():
    """验证流式编排器会发出 truncated 最终事件。"""
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_LengthLLM(), stream_llm=_LengthStreamLLM(),
        config=OrchestratorConfig(max_tokens=10, allowed_tools=[]),
    )

    async def collect():
        """收集流式事件或调用记录。"""
        return [event async for event in orchestrator.stream(query="case")]

    events = asyncio.run(collect())

    assert "".join(event["content"] for event in events).endswith("partial")
    assert events[-1]["finish"] == "truncated"
    assert orchestrator.last_finish_reason == "truncated"


def test_empty_aggregator_stream_uses_non_stream_fallback_and_emits_content():
    """验证综合器空流会走非流式兜底并仍能发出内容。"""
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_FallbackAnswerLLM(), stream_llm=_EmptyStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        """收集流式事件或调用记录。"""
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
    """验证空流且兜底也为空时会抛出明确的生成错误。"""
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_EmptyAnswerLLM(), stream_llm=_EmptyStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        """收集流式事件或调用记录。"""
        return [event async for event in orchestrator.stream(query="case")]

    with pytest.raises(RuntimeError, match="no visible content after fallback"):
        asyncio.run(collect())


def test_interrupted_partial_stream_resets_and_uses_complete_fallback():
    """验证流被打断后会重置，并改用完整兜底回答。"""
    orchestrator = _StageOrchestrator(
        registry=ToolRegistry(), llm=_FallbackAnswerLLM(), stream_llm=_InterruptedStreamLLM(),
        config=OrchestratorConfig(max_tokens=100, allowed_tools=[]),
    )

    async def collect():
        """收集流式事件或调用记录。"""
        return [event async for event in orchestrator.stream(query="case")]

    events = asyncio.run(collect())
    reset_index = next(index for index, event in enumerate(events) if event.get("status") == "answer_reset")
    assert any(event.get("content") == "partial" for event in events[:reset_index])
    assert "".join(
        event.get("content") or "" for event in events[reset_index + 1:]
    ) == "fallback answer"
    assert events[-1]["finish"] == "stop"
    assert orchestrator.last_finish_reason == "stop"


def test_stream_client_preserves_upstream_length_reason(monkeypatch: pytest.MonkeyPatch):
    """验证流式客户端保留上游的 length 结束原因。"""
    import app.llm.llm_client_stream as stream_module

    class FakeResponse:
        def raise_for_status(self):
            """在测试替身里按状态码决定是否抛错。"""
            return None

        async def aiter_lines(self):
            """异步逐行产出测试预置的 SSE 文本。"""
            yield 'data: {"choices":[{"delta":{"content":"partial"},"finish_reason":null}]}'
            yield 'data: {"choices":[{"delta":{},"finish_reason":"length"}]}'
            yield "data: [DONE]"

    class FakeContext:
        async def __aenter__(self):
            """异步进入上下文并返回自身。"""
            return FakeResponse()

        async def __aexit__(self, exc_type, exc, tb):
            """异步退出上下文。"""
            return False

    class FakeClient:
        def stream(self, *args, **kwargs):
            """测试用流式输出实现。"""
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
        """收集流式事件或调用记录。"""
        return [event async for event in client.chat_stream_events(
            messages=[{"role": "user", "content": "case"}], max_tokens=10
        )]

    events = asyncio.run(collect())
    assert events == [
        {"content": "partial", "finish_reason": None},
        {"content": None, "finish_reason": "length"},
    ]
