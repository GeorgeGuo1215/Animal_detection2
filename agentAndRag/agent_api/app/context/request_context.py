"""单请求上下文（例如按 animal_id 限定 sql.search 范围）。"""

from __future__ import annotations

from contextvars import ContextVar
from typing import List, Optional

_REQUEST_ANIMAL_ID: ContextVar[Optional[str]] = ContextVar("request_animal_id", default=None)

SQL_SEARCH_TOOL_NAME = "sql.search"

# 请求未携带 animal_id 时必须隐藏的工具（类似 MCP 条件工具）。
ANIMAL_REQUIRED_TOOLS = frozenset({"sql.search", "vitals.summary"})


def _normalize_animal_id(value: Optional[str]) -> Optional[str]:
    """去除空白并将空字符串规范为 None。"""
    if value is None:
        return None
    s = str(value).strip()
    return s or None


def set_request_animal_id(
    *,
    explicit: Optional[str] = None,
    body_animal_id: Optional[str] = None,
    header_animal_id: Optional[str] = None,
) -> None:
    """解析 animal_id：显式参数优先于 body，再高于 header；空值表示未设置。"""
    aid = _normalize_animal_id(explicit)
    if aid is None:
        aid = _normalize_animal_id(body_animal_id)
    if aid is None:
        aid = _normalize_animal_id(header_animal_id)
    _REQUEST_ANIMAL_ID.set(aid)


def get_request_animal_id() -> Optional[str]:
    """读取当前请求绑定的 animal_id。"""
    return _REQUEST_ANIMAL_ID.get()


def filter_tools_without_animal(tool_names: List[str]) -> List[str]:
    """无 animal_id 时隐藏动物相关工具（sql.search、vitals.summary）。"""
    if get_request_animal_id():
        return list(tool_names)
    return [n for n in tool_names if n not in ANIMAL_REQUIRED_TOOLS]
