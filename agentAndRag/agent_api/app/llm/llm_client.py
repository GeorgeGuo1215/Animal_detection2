from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

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


class OpenAICompatClient:
    """最小 OpenAI 兼容同步客户端，适用于 OpenAI / DeepSeek 等 `POST {base_url}/v1/chat/completions` 网关。"""

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        timeout_s: float = 300.0,
    ) -> None:
        """按参数或环境变量初始化同步聊天客户端。"""
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        self.timeout_s = float(timeout_s)

    def chat(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        response_format: Optional[Dict[str, Any]] = None,
        thinking: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """同步调用 chat completions，并占用 LLM 并发槽位。"""
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, response_format=response_format, thinking=thinking,
        )

        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            with httpx.Client(timeout=self.timeout_s, trust_env=httpx_trust_env()) as client:
                r = client.post(
                    completion_url(self.base_url),
                    headers=authorization_headers(self.api_key),
                    json=payload,
                )
                r.raise_for_status()
                return r.json()


class AsyncOpenAIClient:
    """带连接池的异步 OpenAI 兼容客户端。"""

    def __init__(
        self,
        *,
        base_url: Optional[str] = None,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
    ) -> None:
        """按参数或环境变量初始化异步客户端与共享 httpx 连接池。"""
        settings = resolve_settings(base_url=base_url, api_key=api_key, model=model)
        self.base_url = settings.base_url
        self.api_key = settings.api_key
        self.model = settings.model
        self._client = create_async_http_client()

    async def chat(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        response_format: Optional[Dict[str, Any]] = None,
        thinking: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """异步调用 chat completions，并占用 LLM 并发槽位。"""
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, response_format=response_format, thinking=thinking,
        )

        limits = get_resource_limits()
        async with limits.llm.slot(timeout_s=limits.acquire_timeout_s):
            r = await self._client.post(
                completion_url(self.base_url),
                headers=authorization_headers(self.api_key),
                json=payload,
            )
            r.raise_for_status()
            return r.json()

    async def close(self) -> None:
        """关闭底层 httpx 客户端。"""
        await self._client.aclose()


# --- 共享单例（默认环境配置）---
# 跨请求复用同一个 AsyncOpenAIClient，以便真正共享 httpx 连接池，避免每次重建导致泄漏。
# 需要按请求覆盖 base_url/api_key/model 的调用方应自行创建实例并 close()。
_SHARED_ASYNC_CLIENT: Optional["AsyncOpenAIClient"] = None


def get_shared_async_client() -> "AsyncOpenAIClient":
    """获取默认环境配置的共享异步客户端（懒加载）。"""
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_CLIENT is None:
        _SHARED_ASYNC_CLIENT = AsyncOpenAIClient()
    return _SHARED_ASYNC_CLIENT


async def aclose_shared_async_client() -> None:
    """关闭并清空共享异步客户端。"""
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_CLIENT is not None:
        await _SHARED_ASYNC_CLIENT.close()
        _SHARED_ASYNC_CLIENT = None


def extract_text(resp: Dict[str, Any]) -> str:
    """从 OpenAI 兼容响应中提取 assistant 文本；失败则截断序列化原文。"""
    try:
        return (resp["choices"][0]["message"].get("content") or "").strip()
    except Exception:  # noqa: BLE001
        return json.dumps(resp, ensure_ascii=False)[:2000]
