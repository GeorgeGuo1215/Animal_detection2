from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import httpx


def env_value(name: str, default: Optional[str] = None) -> Optional[str]:
    value = os.getenv(name)
    return value if value not in (None, "") else default


def httpx_trust_env() -> bool:
    value = (os.getenv("HTTPX_TRUST_ENV") or "1").strip().lower()
    return value not in ("0", "false", "no", "off")


@dataclass(frozen=True)
class OpenAISettings:
    base_url: str
    api_key: str
    model: str


def resolve_settings(
    *,
    base_url: Optional[str] = None,
    api_key: Optional[str] = None,
    model: Optional[str] = None,
) -> OpenAISettings:
    return OpenAISettings(
        base_url=(base_url or env_value("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/"),
        api_key=api_key or env_value("OPENAI_API_KEY") or env_value("DEEPSEEK_API_KEY") or "",
        model=model or env_value("OPENAI_MODEL") or env_value("DEEPSEEK_MODEL") or "deepseek-v4-flash",
    )


def completion_url(base_url: str) -> str:
    return f"{base_url.rstrip('/')}/chat/completions"


def authorization_headers(api_key: str) -> Dict[str, str]:
    return {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}


def build_chat_payload(
    *,
    model: str,
    messages: List[Dict[str, Any]],
    temperature: float,
    max_tokens: int,
    response_format: Optional[Dict[str, Any]] = None,
    stream: bool = False,
    thinking: Optional[bool] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "model": model,
        "messages": messages,
        "temperature": float(temperature),
        "max_tokens": int(max_tokens),
    }
    if response_format:
        payload["response_format"] = response_format
    if stream:
        payload["stream"] = True
    if thinking is not None:
        payload["thinking"] = {"type": "enabled" if thinking else "disabled"}
    return payload


def create_async_http_client() -> httpx.AsyncClient:
    return httpx.AsyncClient(
        timeout=httpx.Timeout(connect=10, read=120, write=10, pool=30),
        limits=httpx.Limits(max_connections=20, max_keepalive_connections=10),
        trust_env=httpx_trust_env(),
    )


def require_api_key(api_key: str) -> None:
    if not api_key:
        raise RuntimeError("Missing API key: set OPENAI_API_KEY (or DEEPSEEK_API_KEY).")
