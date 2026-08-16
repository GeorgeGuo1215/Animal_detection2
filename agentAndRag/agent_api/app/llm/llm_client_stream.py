"""
Streaming LLM client for SSE responses.
"""
from __future__ import annotations

import json
from typing import Any, AsyncIterator, Dict, Iterator, List, Optional

import httpx

from ..concurrency import get_resource_limits
from .openai_compat import (
    authorization_headers,
    build_chat_payload,
    completion_url,
    create_async_http_client,
    httpx_trust_env,
    require_api_key,
    resolve_settings,
)


class OpenAIStreamClient:
    """
    OpenAI-compatible streaming client.

    Yields content chunks from SSE stream.
    """

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        timeout_s: float = 300.0,
    ) -> None:
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        self.timeout_s = float(timeout_s)

    def chat_stream(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        thinking: Optional[bool] = None,
    ) -> Iterator[str]:
        """
        Stream chat completion, yielding content chunks.

        Yields each content delta as a string.
        """
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, stream=True, thinking=thinking,
        )

        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            with httpx.Client(timeout=self.timeout_s, trust_env=httpx_trust_env()) as client:
                with client.stream(
                    "POST", completion_url(self.base_url),
                    headers=authorization_headers(self.api_key), json=payload,
                ) as response:
                    response.raise_for_status() # 如果 status_code 是 4xx 或 5xx，抛出 HTTPStatusError
                    for line in response.iter_lines():
                        if not line:
                            continue
                        if line.startswith("data: "):
                            data = line[6:] # data: 后的 {json}
                            if data == "[DONE]":
                                break
                            try:
                                chunk = json.loads(data) # 将 {json} 转换为 Python 字典
                                delta = chunk.get("choices", [{}])[0].get("delta", {})
                                content = delta.get("content")
                                if content:
                                    yield content
                            except json.JSONDecodeError:
                                continue

    def chat_stream_full(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        thinking: Optional[bool] = None,
    ) -> str:
        """
        Stream chat completion and return full content.
        """
        chunks = []
        for chunk in self.chat_stream(
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            thinking=thinking,
        ):
            chunks.append(chunk)
        return "".join(chunks)


class AsyncOpenAIStreamClient:
    """Async streaming client with connection pooling."""

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
    ) -> None:
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        self._client = create_async_http_client()

    async def chat_stream(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        thinking: Optional[bool] = None,
    ) -> AsyncIterator[str]:
        events = self.chat_stream_events(
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            thinking=thinking,
        )
        try:
            async for event in events:
                content = event.get("content")
                if content:
                    yield str(content)
        finally:
            await events.aclose()

    async def chat_stream_events(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        thinking: Optional[bool] = None,
    ) -> AsyncIterator[Dict[str, Optional[str]]]:
        """Yield content deltas plus the upstream finish reason."""
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, stream=True, thinking=thinking,
        )

        limits = get_resource_limits()
        async with limits.llm.slot(timeout_s=limits.acquire_timeout_s):
            async with self._client.stream(
                "POST", completion_url(self.base_url),
                headers=authorization_headers(self.api_key), json=payload,
            ) as response:
                response.raise_for_status()
                async for line in response.aiter_lines():
                    if not line:
                        continue
                    if line.startswith("data: "):
                        data = line[6:]
                        if data == "[DONE]":
                            break
                        try:
                            chunk = json.loads(data)
                            choice = chunk.get("choices", [{}])[0]
                            delta = choice.get("delta", {})
                            content = delta.get("content")
                            finish_reason = choice.get("finish_reason")
                            if content or finish_reason:
                                yield {
                                    "content": str(content) if content else None,
                                    "finish_reason": str(finish_reason) if finish_reason else None,
                                }
                        except json.JSONDecodeError:
                            continue

    async def chat_stream_full(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        thinking: Optional[bool] = None,
    ) -> str:
        chunks: List[str] = []
        async for chunk in self.chat_stream(
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            thinking=thinking,
        ):
            chunks.append(chunk)
        return "".join(chunks)

    async def close(self) -> None:
        await self._client.aclose()


# --- Shared singleton (default env config) ---------------------------------
# Reuse one streaming client across requests so its httpx connection pool is
# shared rather than rebuilt (and leaked) per request.
_SHARED_ASYNC_STREAM_CLIENT: Optional["AsyncOpenAIStreamClient"] = None


def get_shared_async_stream_client() -> "AsyncOpenAIStreamClient":
    global _SHARED_ASYNC_STREAM_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_STREAM_CLIENT is None:
        _SHARED_ASYNC_STREAM_CLIENT = AsyncOpenAIStreamClient()
    return _SHARED_ASYNC_STREAM_CLIENT


async def aclose_shared_async_stream_client() -> None:
    global _SHARED_ASYNC_STREAM_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_STREAM_CLIENT is not None:
        await _SHARED_ASYNC_STREAM_CLIENT.close()
        _SHARED_ASYNC_STREAM_CLIENT = None
