from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional


@dataclass(frozen=True)
class ToolSpec:
    """
    function-calling 的最小工具规格。

    - name: 稳定标识，如 "rag.search"
    - description: 简短人类可读描述
    - input_schema: 类 JSON Schema 的字典
    - handler: 同步或异步 callable(**kwargs) -> dict
    """

    name: str
    description: str
    input_schema: Dict[str, Any]
    handler: Callable[..., Any]


class ToolRegistry:
    """进程内工具注册表：注册、列举、按名调用。"""

    def __init__(self) -> None:
        """初始化空注册表。"""
        self._tools: Dict[str, ToolSpec] = {}

    def register(self, tool: ToolSpec) -> None:
        """注册工具；重名则抛出 ValueError。"""
        if tool.name in self._tools:
            raise ValueError(f"Tool already registered: {tool.name}")
        self._tools[tool.name] = tool

    def list_tools(self) -> List[ToolSpec]:
        """按名称排序返回已注册工具。"""
        return [self._tools[k] for k in sorted(self._tools.keys())]

    def get(self, name: str) -> Optional[ToolSpec]:
        """按名称查找工具，不存在则返回 None。"""
        return self._tools.get(name)

    async def call(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """透明分发到同步或异步 handler。"""
        tool = self.get(name)
        if not tool:
            raise KeyError(f"Unknown tool: {name}")
        arguments = dict(arguments or {})
        if asyncio.iscoroutinefunction(tool.handler):
            return await tool.handler(**arguments)
        return await asyncio.to_thread(tool.handler, **arguments)

    def call_sync(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """向后兼容的同步调用（非异步上下文）。"""
        tool = self.get(name)
        if not tool:
            raise KeyError(f"Unknown tool: {name}")
        arguments = dict(arguments or {})
        if asyncio.iscoroutinefunction(tool.handler):
            raise TypeError(f"Tool '{name}' is async; use 'await registry.call()' instead.")
        return tool.handler(**arguments)


_REGISTRY: Optional[ToolRegistry] = None


def get_registry() -> ToolRegistry:
    """获取全局 ToolRegistry 单例。"""
    global _REGISTRY  # noqa: PLW0603
    if _REGISTRY is None:
        _REGISTRY = ToolRegistry()
    return _REGISTRY

