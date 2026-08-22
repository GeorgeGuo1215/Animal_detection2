from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import httpx


def env_value(name: str, default: Optional[str] = None) -> Optional[str]:
    """读取环境变量；空字符串视为未设置并回退 default。"""
    value = os.getenv(name)
    return value if value not in (None, "") else default


def httpx_trust_env() -> bool:
    """是否让 httpx 读取系统代理等环境变量（HTTPX_TRUST_ENV）。"""
    value = (os.getenv("HTTPX_TRUST_ENV") or "1").strip().lower()
    return value not in ("0", "false", "no", "off")


@dataclass(frozen=True)
class OpenAISettings:
    """OpenAI 兼容接口的 base_url、api_key 与 model。"""

    base_url: str
    api_key: str
    model: str


def _env_float(name: str, default: float, *, minimum: float = 0.0) -> float:
    """读取有下界的浮点环境变量，非法配置回退默认值。"""
    try:
        return max(minimum, float(env_value(name, str(default)) or default))
    except (TypeError, ValueError):
        return default


def _env_int(name: str, default: int, *, minimum: int = 1) -> int:
    """读取有下界的整数环境变量，非法配置回退默认值。"""
    try:
        return max(minimum, int(env_value(name, str(default)) or default))
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class LLMTransportSettings:
    """LLM HTTP 连接池和各阶段超时；与应用层并发槽位职责分离。"""

    connect_timeout_s: float
    read_timeout_s: float
    write_timeout_s: float
    pool_timeout_s: float
    max_connections: int
    max_keepalive_connections: int
    trust_env: bool


def load_transport_settings() -> LLMTransportSettings:
    """从环境变量加载共享 LLM 连接池配置。"""
    max_connections = _env_int("AGENT_LLM_MAX_CONNECTIONS", 20)
    keepalive = min(
        max_connections,
        _env_int("AGENT_LLM_MAX_KEEPALIVE_CONNECTIONS", 10),
    )
    return LLMTransportSettings(
        connect_timeout_s=_env_float("AGENT_LLM_CONNECT_TIMEOUT_SEC", 10.0, minimum=0.1),
        read_timeout_s=_env_float("AGENT_LLM_READ_TIMEOUT_SEC", 120.0, minimum=1.0),
        write_timeout_s=_env_float("AGENT_LLM_WRITE_TIMEOUT_SEC", 10.0, minimum=0.1),
        pool_timeout_s=_env_float("AGENT_LLM_POOL_TIMEOUT_SEC", 30.0, minimum=0.1),
        max_connections=max_connections,
        max_keepalive_connections=keepalive,
        trust_env=httpx_trust_env(),
    )


@dataclass(frozen=True)
class MoeGenerationSettings:
    """可运维调节的 MoE 生成预算，不包含医学策略或输出契约。"""

    default_temperature: float
    default_max_tokens: int
    request_temperature: float
    task_policy_max_tokens: int
    critic_temperature: float
    critic_max_tokens: int
    expert_temperature: float


def load_generation_settings() -> MoeGenerationSettings:
    """加载通用与阶段级生成参数。"""
    return MoeGenerationSettings(
        default_temperature=_env_float("AGENT_LLM_DEFAULT_TEMPERATURE", 0.2),
        default_max_tokens=_env_int("AGENT_LLM_DEFAULT_MAX_TOKENS", 768),
        request_temperature=_env_float("MOE_REQUEST_DEFAULT_TEMPERATURE", 0.3),
        task_policy_max_tokens=_env_int("MOE_TASK_POLICY_MAX_TOKENS", 1200),
        critic_temperature=_env_float("MOE_CRITIC_TEMPERATURE", 0.1),
        critic_max_tokens=_env_int("MOE_CRITIC_MAX_TOKENS", 400),
        expert_temperature=_env_float("MOE_EXPERT_TEMPERATURE", 0.2),
    )


def resolve_settings(
    *,
    base_url: Optional[str] = None,
    api_key: Optional[str] = None,
    model: Optional[str] = None,
) -> OpenAISettings:
    """用显式参数或环境变量拼出 OpenAI 兼容配置。"""
    return OpenAISettings(
        base_url=(base_url or env_value("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/"),
        api_key=api_key or env_value("OPENAI_API_KEY") or env_value("DEEPSEEK_API_KEY") or "",
        model=model or env_value("OPENAI_MODEL") or env_value("DEEPSEEK_MODEL") or "deepseek-v4-flash",
    )


def completion_url(base_url: str) -> str:
    """拼接 chat/completions 请求 URL。"""
    return f"{base_url.rstrip('/')}/chat/completions"


def authorization_headers(api_key: str) -> Dict[str, str]:
    """构造 Bearer 鉴权与 JSON Content-Type 头。"""
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
    """组装 OpenAI 兼容的 chat completions 请求体。"""
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


def create_async_http_client(
    settings: Optional[LLMTransportSettings] = None,
) -> httpx.AsyncClient:
    """创建带超时与连接池的 httpx 异步客户端。"""
    transport = settings or load_transport_settings()
    return httpx.AsyncClient(
        timeout=httpx.Timeout(
            connect=transport.connect_timeout_s,
            read=transport.read_timeout_s,
            write=transport.write_timeout_s,
            pool=transport.pool_timeout_s,
        ),
        limits=httpx.Limits(
            max_connections=transport.max_connections,
            max_keepalive_connections=transport.max_keepalive_connections,
        ),
        trust_env=transport.trust_env,
    )


def require_api_key(api_key: str) -> None:
    """API key 为空时抛出 RuntimeError。"""
    if not api_key:
        raise RuntimeError("Missing API key: set OPENAI_API_KEY (or DEEPSEEK_API_KEY).")
