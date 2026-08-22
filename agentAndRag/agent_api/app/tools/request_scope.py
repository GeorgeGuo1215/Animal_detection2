"""Agent 工具调用的请求级作用域。

该作用域不是 HTTP 中间件：animal_id 既可能来自已解析请求体，也必须在 SSE 生成器与
``asyncio.to_thread`` 继承的 Context 中可见。上下文退出时会复位，避免请求间身份泄漏。
"""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar, Token
from dataclasses import dataclass
from typing import Iterator, List, Optional

_REQUEST_ANIMAL_ID: ContextVar[Optional[str]] = ContextVar("request_animal_id", default=None)
SQL_SEARCH_TOOL_NAME = "sql.search"
ANIMAL_REQUIRED_TOOLS = frozenset({"sql.search", "vitals.summary"})


def _normalize_animal_id(value: Optional[str]) -> Optional[str]:
    """去除空白并将空字符串规范为 ``None``。"""
    if value is None:
        return None
    normalized = str(value).strip()
    return normalized or None


def resolve_request_animal_id(
    *, explicit: Optional[str] = None, body_animal_id: Optional[str] = None,
    header_animal_id: Optional[str] = None,
) -> Optional[str]:
    """按显式参数、请求体、请求头的顺序解析 animal_id。"""
    for value in (explicit, body_animal_id, header_animal_id):
        normalized = _normalize_animal_id(value)
        if normalized is not None:
            return normalized
    return None


@dataclass(frozen=True)
class ToolRequestScope:
    """当前工具调用能够访问的请求级身份信息。"""

    animal_id: Optional[str]


def set_request_animal_id(
    *, explicit: Optional[str] = None, body_animal_id: Optional[str] = None,
    header_animal_id: Optional[str] = None,
) -> Token[Optional[str]]:
    """兼容旧调用并返回复位 Token；新代码应优先使用绑定上下文。"""
    return _REQUEST_ANIMAL_ID.set(resolve_request_animal_id(
        explicit=explicit, body_animal_id=body_animal_id, header_animal_id=header_animal_id,
    ))


def reset_request_animal_id(token: Token[Optional[str]]) -> None:
    """使用 setter 返回的 Token 恢复上一层作用域。"""
    _REQUEST_ANIMAL_ID.reset(token)


@contextmanager
def bind_tool_request_scope(
    *, explicit: Optional[str] = None, body_animal_id: Optional[str] = None,
    header_animal_id: Optional[str] = None,
) -> Iterator[ToolRequestScope]:
    """绑定工具身份，并在正常返回、异常或取消时可靠复位。"""
    animal_id = resolve_request_animal_id(
        explicit=explicit, body_animal_id=body_animal_id, header_animal_id=header_animal_id,
    )
    token = _REQUEST_ANIMAL_ID.set(animal_id)
    try:
        yield ToolRequestScope(animal_id=animal_id)
    finally:
        _REQUEST_ANIMAL_ID.reset(token)


def get_request_animal_id() -> Optional[str]:
    """读取当前工具请求作用域绑定的 animal_id。"""
    return _REQUEST_ANIMAL_ID.get()


def filter_tools_without_animal(tool_names: List[str]) -> List[str]:
    """未绑定 animal_id 时移除只能访问个体数据的工具。"""
    if get_request_animal_id():
        return list(tool_names)
    return [name for name in tool_names if name not in ANIMAL_REQUIRED_TOOLS]
