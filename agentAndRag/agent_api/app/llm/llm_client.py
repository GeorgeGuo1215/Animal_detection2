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
    """
    Minimal OpenAI-compatible client.

    Works with OpenAI / DeepSeek / other gateways that support:
      POST {base_url}/v1/chat/completions
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

    def chat(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        response_format: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, response_format=response_format,
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
    """Async version of OpenAICompatClient with connection pooling."""

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

    async def chat(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
        response_format: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        require_api_key(self.api_key)
        payload = build_chat_payload(
            model=self.model, messages=messages, temperature=temperature,
            max_tokens=max_tokens, response_format=response_format,
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
        await self._client.aclose()


# --- Shared singleton (default env config) ---------------------------------
# Reuse one AsyncOpenAIClient across requests so the httpx connection pool is
# actually shared instead of being rebuilt (and leaked) on every request.
# Callers that need per-request base_url/api_key/model must build their own
# instance and close() it themselves.
_SHARED_ASYNC_CLIENT: Optional["AsyncOpenAIClient"] = None


def get_shared_async_client() -> "AsyncOpenAIClient":
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_CLIENT is None:
        _SHARED_ASYNC_CLIENT = AsyncOpenAIClient()
    return _SHARED_ASYNC_CLIENT


async def aclose_shared_async_client() -> None:
    global _SHARED_ASYNC_CLIENT  # noqa: PLW0603
    if _SHARED_ASYNC_CLIENT is not None:
        await _SHARED_ASYNC_CLIENT.close()
        _SHARED_ASYNC_CLIENT = None


def extract_text(resp: Dict[str, Any]) -> str:
    """
    Extract assistant content from an OpenAI-compatible response.
    """
    try:
        return (resp["choices"][0]["message"].get("content") or "").strip()
    except Exception:  # noqa: BLE001
        return json.dumps(resp, ensure_ascii=False)[:2000]
