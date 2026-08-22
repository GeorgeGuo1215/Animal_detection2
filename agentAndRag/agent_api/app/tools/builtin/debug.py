"""仅供内部工作流自检的调试工具。"""

from __future__ import annotations

from typing import Any, Dict

from ..tool_registry import ToolRegistry, ToolSpec


def _debug_echo_tool(**kwargs: Any) -> Dict[str, Any]:
    """原样回显参数。"""
    return {"echo": kwargs}


def register_debug_tools(registry: ToolRegistry) -> None:
    """注册不进入公开默认白名单的 debug.echo。"""
    registry.register(ToolSpec(
        name="debug.echo",
        description="Echo back arguments for internal workflow debugging.",
        input_schema={"type": "object", "properties": {}, "required": []},
        handler=_debug_echo_tool,
    ))
