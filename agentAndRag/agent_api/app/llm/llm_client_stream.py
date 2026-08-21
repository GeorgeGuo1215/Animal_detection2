"""面向 SSE 响应的流式 LLM 客户端。"""
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
    """OpenAI 兼容流式客户端，从 SSE 中产出内容增量。"""

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        timeout_s: float = 300.0,
    ) -> None:
        """按参数或环境变量初始化同步流式客户端。"""
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
        """流式调用 chat completion，逐块产出 content delta。"""
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
        """流式调用 chat completion 并拼接为完整文本返回。"""
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
    """带连接池的异步流式客户端。"""

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
    ) -> None:
        """按参数或环境变量初始化异步流式客户端。"""
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
        """异步流式调用，仅产出 content 字符串增量。"""
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
        """产出 content delta 以及上游 finish_reason。"""
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
        """异步流式调用并拼接为完整文本返回。"""
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
        """关闭底层 httpx 客户端。"""
        await self._client.aclose()


# --- 共享单例（默认环境配置）---
# 跨请求复用同一个流式客户端，共享 httpx 连接池，避免每次重建导致泄漏。
_SHARED_ASYNC_STREAM_CLIENT: Optional["AsyncOpenAIStreamClient"] = None


def get_shared_async_stream_client() -> "AsyncOpenAIStreamClient":
    """获取默认环境配置的共享异步流式客户端（懒加载）。"""
    global _SHARED_ASYNC_STREAM_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_STREAM_CLIENT is None:
        _SHARED_ASYNC_STREAM_CLIENT = AsyncOpenAIStreamClient()
    return _SHARED_ASYNC_STREAM_CLIENT


async def aclose_shared_async_stream_client() -> None:
    """关闭并清空共享异步流式客户端。"""
    global _SHARED_ASYNC_STREAM_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_STREAM_CLIENT is not None:
        await _SHARED_ASYNC_STREAM_CLIENT.close()
        _SHARED_ASYNC_STREAM_CLIENT = None
