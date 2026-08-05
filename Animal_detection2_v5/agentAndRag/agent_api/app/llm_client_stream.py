"""
Streaming LLM client for SSE responses.
"""
from __future__ import annotations

import json
import os
from typing import Any, AsyncIterator, Dict, Iterator, List, Optional

import httpx


def _env(name: str, default: Optional[str] = None) -> Optional[str]:
    v = os.getenv(name)
    return v if v not in (None, "") else default


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
        self.base_url = (base_url or _env("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/")
        self.api_key = api_key or _env("OPENAI_API_KEY") or _env("DEEPSEEK_API_KEY") or ""
        self.model = model or _env("OPENAI_MODEL") or _env("DEEPSEEK_MODEL") or "deepseek-chat"
        self.timeout_s = float(timeout_s)

    def chat_stream(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
    ) -> Iterator[str]:
        """
        Stream chat completion, yielding content chunks.
        
        Yields each content delta as a string.
        """
        if not self.api_key:
            raise RuntimeError("Missing API key: set OPENAI_API_KEY (or DEEPSEEK_API_KEY).")

        url = f"{self.base_url}/chat/completions"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": float(temperature),
            "max_tokens": int(max_tokens),
            "stream": True,
        }

        with httpx.Client(timeout=self.timeout_s) as client:
            with client.stream("POST", url, headers=headers, json=payload) as response:
                response.raise_for_status()
                for line in response.iter_lines():
                    if not line:
                        continue
                    if line.startswith("data: "):
                        data = line[6:]
                        if data == "[DONE]":
                            break
                        try:
                            chunk = json.loads(data)
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
    ) -> str:
        """
        Stream chat completion and return full content.
        """
        chunks = []
        for chunk in self.chat_stream(messages=messages, temperature=temperature, max_tokens=max_tokens):
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
        self.base_url = (base_url or _env("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/")
        self.api_key = api_key or _env("OPENAI_API_KEY") or _env("DEEPSEEK_API_KEY") or ""
        self.model = model or _env("OPENAI_MODEL") or _env("DEEPSEEK_MODEL") or "deepseek-chat"
        self._client = httpx.AsyncClient(
            timeout=httpx.Timeout(connect=10, read=120, write=10, pool=30),
            limits=httpx.Limits(max_connections=20, max_keepalive_connections=10),
        )

    async def chat_stream(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
    ) -> AsyncIterator[str]:
        if not self.api_key:
            raise RuntimeError("Missing API key: set OPENAI_API_KEY (or DEEPSEEK_API_KEY).")

        url = f"{self.base_url}/chat/completions"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": float(temperature),
            "max_tokens": int(max_tokens),
            "stream": True,
        }

        async with self._client.stream("POST", url, headers=headers, json=payload) as response:
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
                        delta = chunk.get("choices", [{}])[0].get("delta", {})
                        content = delta.get("content")
                        if content:
                            yield content
                    except json.JSONDecodeError:
                        continue

    async def chat_stream_full(
        self,
        *,
        messages: List[Dict[str, Any]],
        temperature: float = 0.2,
        max_tokens: int = 768,
    ) -> str:
        chunks: List[str] = []
        async for chunk in self.chat_stream(messages=messages, temperature=temperature, max_tokens=max_tokens):
            chunks.append(chunk)
        return "".join(chunks)

    async def close(self) -> None:
        await self._client.aclose()
