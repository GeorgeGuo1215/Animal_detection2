"""统一的 OpenAI 兼容聊天客户端。

异步非流式与流式请求共用一个 ``httpx.AsyncClient`` 和连接池；资源槽位继续负责
应用层在途调用上限，连接池只负责 TCP/Keep-Alive 容量，两者职责不同。
"""

from __future__ import annotations

import json
from typing import Any, AsyncIterator, Dict, Iterator, List, Optional

import httpx

from ...concurrency import get_resource_limits
from .config import (
    authorization_headers,
    build_chat_payload,
    completion_url,
    create_async_http_client,
    httpx_trust_env,
    load_generation_settings,
    load_transport_settings,
    require_api_key,
    resolve_settings,
)
from .sse_parser import parse_chat_sse_line


class OpenAIChatClient:
    """共享连接池的异步聊天客户端，分别暴露 complete 与 stream 方法。"""

    def __init__(
        self, *, base_url: Optional[str] = None, api_key: Optional[str] = None,
        model: Optional[str] = None, client: Optional[httpx.AsyncClient] = None,
    ) -> None:
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        self._client = client or create_async_http_client()

    async def chat(
        self, *, messages: List[Dict[str, Any]], temperature: Optional[float] = None,
        max_tokens: Optional[int] = None, response_format: Optional[Dict[str, Any]] = None,
        thinking: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """执行一次非流式补全；保留 ``chat`` 名称兼容现有编排器。"""
        require_api_key(self.api_key)
        generation = load_generation_settings()
        payload = build_chat_payload(
            model=self.model,
            messages=messages,
            temperature=generation.default_temperature if temperature is None else temperature,
            max_tokens=generation.default_max_tokens if max_tokens is None else max_tokens,
            response_format=response_format,
            thinking=thinking,
        )
        limits = get_resource_limits()
        async with limits.llm.slot(timeout_s=limits.acquire_timeout_s):
            response = await self._client.post(
                completion_url(self.base_url),
                headers=authorization_headers(self.api_key),
                json=payload,
            )
            response.raise_for_status()
            return response.json()

    async def chat_stream_events(
        self, *, messages: List[Dict[str, Any]], temperature: Optional[float] = None,
        max_tokens: Optional[int] = None, thinking: Optional[bool] = None,
    ) -> AsyncIterator[Dict[str, Optional[str]]]:
        """执行流式补全，产出文本增量与上游 finish_reason。"""
        require_api_key(self.api_key)
        generation = load_generation_settings()
        payload = build_chat_payload(
            model=self.model,
            messages=messages,
            temperature=generation.default_temperature if temperature is None else temperature,
            max_tokens=generation.default_max_tokens if max_tokens is None else max_tokens,
            stream=True,
            thinking=thinking,
        )
        limits = get_resource_limits()
        async with limits.llm.slot(timeout_s=limits.acquire_timeout_s):
            async with self._client.stream(
                "POST",
                completion_url(self.base_url),
                headers=authorization_headers(self.api_key),
                json=payload,
            ) as response:
                response.raise_for_status()
                async for line in response.aiter_lines():
                    event = parse_chat_sse_line(line)
                    if event is not None:
                        yield event

    async def chat_stream(
        self, *, messages: List[Dict[str, Any]], temperature: Optional[float] = None,
        max_tokens: Optional[int] = None, thinking: Optional[bool] = None,
    ) -> AsyncIterator[str]:
        """仅产出可见文本增量，并在消费者提前关闭时释放资源槽位。"""
        events = self.chat_stream_events(
            messages=messages, temperature=temperature, max_tokens=max_tokens, thinking=thinking,
        )
        try:
            async for event in events:
                content = event.get("content")
                if content:
                    yield str(content)
        finally:
            await events.aclose()

    async def chat_stream_full(self, **kwargs: Any) -> str:
        """消费流并返回完整可见文本。"""
        chunks = [chunk async for chunk in self.chat_stream(**kwargs)]
        return "".join(chunks)

    async def close(self) -> None:
        """关闭共享的底层连接池。"""
        await self._client.aclose()


class SyncOpenAIChatClient:
    """兼容少量同步脚本的单一客户端；生产请求使用异步共享客户端。"""

    def __init__(
        self, *, base_url: Optional[str] = None, api_key: Optional[str] = None,
        model: Optional[str] = None, timeout_s: Optional[float] = None,
    ) -> None:
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        transport = load_transport_settings()
        self.timeout_s = timeout_s if timeout_s is not None else transport.read_timeout_s

    def chat(
        self, *, messages: List[Dict[str, Any]], temperature: Optional[float] = None,
        max_tokens: Optional[int] = None, response_format: Optional[Dict[str, Any]] = None,
        thinking: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """同步执行非流式补全。"""
        require_api_key(self.api_key)
        generation = load_generation_settings()
        payload = build_chat_payload(
            model=self.model, messages=messages,
            temperature=generation.default_temperature if temperature is None else temperature,
            max_tokens=generation.default_max_tokens if max_tokens is None else max_tokens,
            response_format=response_format, thinking=thinking,
        )
        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            with httpx.Client(timeout=self.timeout_s, trust_env=httpx_trust_env()) as client:
                response = client.post(
                    completion_url(self.base_url),
                    headers=authorization_headers(self.api_key), json=payload,
                )
                response.raise_for_status()
                return response.json()

    def chat_stream(
        self, *, messages: List[Dict[str, Any]], temperature: Optional[float] = None,
        max_tokens: Optional[int] = None, thinking: Optional[bool] = None,
    ) -> Iterator[str]:
        """同步执行流式补全。"""
        require_api_key(self.api_key)
        generation = load_generation_settings()
        payload = build_chat_payload(
            model=self.model, messages=messages,
            temperature=generation.default_temperature if temperature is None else temperature,
            max_tokens=generation.default_max_tokens if max_tokens is None else max_tokens,
            stream=True, thinking=thinking,
        )
        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            with httpx.Client(timeout=self.timeout_s, trust_env=httpx_trust_env()) as client:
                with client.stream(
                    "POST", completion_url(self.base_url),
                    headers=authorization_headers(self.api_key), json=payload,
                ) as response:
                    response.raise_for_status()
                    for line in response.iter_lines():
                        event = parse_chat_sse_line(line)
                        if event and event.get("content"):
                            yield str(event["content"])

    def chat_stream_full(self, **kwargs: Any) -> str:
        """同步消费流并拼接正文。"""
        return "".join(self.chat_stream(**kwargs))


AsyncOpenAIClient = OpenAIChatClient
AsyncOpenAIStreamClient = OpenAIChatClient
OpenAICompatClient = SyncOpenAIChatClient
OpenAIStreamClient = SyncOpenAIChatClient

_SHARED_ASYNC_CLIENT: Optional[OpenAIChatClient] = None


def get_shared_async_client() -> OpenAIChatClient:
    """返回同时服务流式与非流式调用的唯一共享客户端。"""
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_CLIENT is None:
        _SHARED_ASYNC_CLIENT = OpenAIChatClient()
    return _SHARED_ASYNC_CLIENT


def get_shared_async_stream_client() -> OpenAIChatClient:
    """兼容旧入口；与非流式入口返回同一实例和连接池。"""
    return get_shared_async_client()


async def aclose_shared_async_client() -> None:
    """关闭并清空唯一共享 LLM 客户端。"""
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    client = _SHARED_ASYNC_CLIENT
    _SHARED_ASYNC_CLIENT = None
    if client is not None:
        await client.close()


async def aclose_shared_async_stream_client() -> None:
    """兼容旧关闭入口；统一委托给共享客户端。"""
    await aclose_shared_async_client()


def extract_text(response: Dict[str, Any]) -> str:
    """从 OpenAI 兼容响应提取 assistant 文本。"""
    try:
        return (response["choices"][0]["message"].get("content") or "").strip()
    except Exception:  # noqa: BLE001
        return json.dumps(response, ensure_ascii=False)[:2000]
