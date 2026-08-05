from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional


@dataclass(frozen=True)
class ToolSpec:
    """
    A minimal tool spec for function-calling.

    - name: stable identifier, e.g. "rag.search"
    - description: short human-readable description
    - input_schema: JSON schema-like dict
    - handler: sync or async callable(**kwargs) -> dict
    """

    name: str
    description: str
    input_schema: Dict[str, Any]
    handler: Callable[..., Any]


class ToolRegistry:
    def __init__(self) -> None:
        self._tools: Dict[str, ToolSpec] = {}

    def register(self, tool: ToolSpec) -> None:
        if tool.name in self._tools:
            raise ValueError(f"Tool already registered: {tool.name}")
        self._tools[tool.name] = tool

    def list_tools(self) -> List[ToolSpec]:
        return [self._tools[k] for k in sorted(self._tools.keys())]

    def get(self, name: str) -> Optional[ToolSpec]:
        return self._tools.get(name)

    async def call(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatch to sync or async handler transparently."""
        tool = self.get(name)
        if not tool:
            raise KeyError(f"Unknown tool: {name}")
        arguments = dict(arguments or {})
        if asyncio.iscoroutinefunction(tool.handler):
            return await tool.handler(**arguments)
        return await asyncio.to_thread(tool.handler, **arguments)

    def call_sync(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Backward-compatible synchronous call (for non-async contexts)."""
        tool = self.get(name)
        if not tool:
            raise KeyError(f"Unknown tool: {name}")
        arguments = dict(arguments or {})
        if asyncio.iscoroutinefunction(tool.handler):
            raise TypeError(f"Tool '{name}' is async; use 'await registry.call()' instead.")
        return tool.handler(**arguments)


_REGISTRY: Optional[ToolRegistry] = None


def get_registry() -> ToolRegistry:
    global _REGISTRY  # noqa: PLW0603
    if _REGISTRY is None:
        _REGISTRY = ToolRegistry()
    return _REGISTRY

