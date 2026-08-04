"""OpenAI 兼容的 LLM 调用。

整个服务走同步模型：FastAPI 的同步路由自动跑在线程池里，worker 是普通线程，
数据库连接池也是同步的。混用 async 只会给这套以 IO 等待为主的流程增加复杂度。

LLM 只在 worker 的后台提升流程里被调用，请求路径上一次都不调。
"""

from __future__ import annotations

import json
import logging
import re
from typing import Any, Dict, List, Optional, Protocol, Sequence

import httpx

from .config import MemoryConfig

logger = logging.getLogger(__name__)

# LLM 常把 JSON 包在 ```json ... ``` 里，解析前要剥掉。
_FENCE_RE = re.compile(r"^\s*```(?:json)?\s*|\s*```\s*$", re.IGNORECASE)


class LLMError(RuntimeError):
    """LLM 调用失败。worker 捕获后按任务重试处理。"""


class LLMClient(Protocol):
    def complete(
        self,
        messages: Sequence[Dict[str, str]],
        *,
        temperature: float = 0.3,
        max_tokens: int = 1024,
    ) -> str: ...


class OpenAICompatClient:
    def __init__(
        self,
        base_url: str,
        api_key: str,
        model: str,
        timeout: float = 60.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.model = model
        self.timeout = timeout

    def complete(
        self,
        messages: Sequence[Dict[str, str]],
        *,
        temperature: float = 0.3,
        max_tokens: int = 1024,
    ) -> str:
        if not self.api_key:
            raise LLMError("LLM api key is not configured")
        payload = {
            "model": self.model,
            "messages": list(messages),
            "temperature": temperature,
            "max_tokens": max_tokens,
            "stream": False,
        }
        try:
            with httpx.Client(timeout=self.timeout) as client:
                response = client.post(
                    f"{self.base_url}/chat/completions",
                    headers={
                        "Authorization": f"Bearer {self.api_key}",
                        "Content-Type": "application/json",
                    },
                    json=payload,
                )
                response.raise_for_status()
                data = response.json()
        except httpx.HTTPError as exc:
            raise LLMError(f"LLM request failed: {exc}") from exc
        except ValueError as exc:
            raise LLMError(f"LLM returned malformed JSON body: {exc}") from exc

        try:
            return data["choices"][0]["message"]["content"] or ""
        except (KeyError, IndexError, TypeError) as exc:
            raise LLMError(f"unexpected LLM response shape: {data!r}") from exc


def strip_code_fence(text: str) -> str:
    """去掉 markdown 代码围栏，只保留里面的内容。"""
    cleaned = (text or "").strip()
    if "```" not in cleaned:
        return cleaned
    cleaned = _FENCE_RE.sub("", cleaned)
    return cleaned.strip()


def parse_json_object(text: str) -> Dict[str, Any]:
    """尽力把 LLM 输出解析成 JSON 对象。

    解析不出来返回空字典而不是抛异常：画像抽取失败只应该让这一轮没有产出，
    不该让整个提升任务失败并触发重试，否则一个话痨模型能把重试次数耗光。
    """
    cleaned = strip_code_fence(text)
    if not cleaned:
        return {}
    try:
        parsed = json.loads(cleaned)
    except json.JSONDecodeError:
        # 模型常在 JSON 前后带一句解释，退而求其次抓最外层的花括号。
        start = cleaned.find("{")
        end = cleaned.rfind("}")
        if start == -1 or end <= start:
            logger.warning("memory_service: LLM output is not JSON: %.120s", cleaned)
            return {}
        try:
            parsed = json.loads(cleaned[start : end + 1])
        except json.JSONDecodeError:
            logger.warning("memory_service: failed to salvage JSON from LLM output")
            return {}
    return parsed if isinstance(parsed, dict) else {}


def as_string_list(value: Any) -> List[str]:
    """把 LLM 可能返回的多种形态（列表、换行字符串、单值）归一成字符串列表。"""
    if value is None:
        return []
    if isinstance(value, str):
        parts = [line.strip(" -•\t") for line in value.splitlines()]
    elif isinstance(value, (list, tuple)):
        parts = [str(item).strip(" -•\t") for item in value]
    else:
        parts = [str(value).strip()]

    out: List[str] = []
    for part in parts:
        text = part.strip()
        # 模型表达"没有内容"的几种常见写法，都不该写进知识库。
        if not text or text.lower() in {"none", "null", "n/a", "无", "暂无", "没有"}:
            continue
        out.append(text)
    return out


_client: Optional[LLMClient] = None


def init_llm(cfg: MemoryConfig) -> LLMClient:
    global _client
    if _client is None:
        _client = OpenAICompatClient(
            base_url=cfg.llm_base_url,
            api_key=cfg.llm_api_key,
            model=cfg.llm_model,
            timeout=cfg.llm_timeout,
        )
    return _client


def set_llm(client: Optional[LLMClient]) -> None:
    """替换全局 LLM 客户端，供测试与长跑模拟注入确定性假实现。"""
    global _client
    _client = client


def get_llm() -> LLMClient:
    if _client is None:
        raise RuntimeError("llm client is not initialised; call init_llm() first")
    return _client
