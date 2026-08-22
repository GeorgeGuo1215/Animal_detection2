"""内置 ToolRegistry 契约的组合入口。"""

from .debug import register_debug_tools
from .petmind_data import register_petmind_data_tools
from .rag import register_rag_tools


def register_builtin_tools(registry) -> None:
    """注册生产内置工具；MCP 工具由独立适配器注册。"""
    register_rag_tools(registry)
    register_petmind_data_tools(registry)


__all__ = ["register_builtin_tools", "register_debug_tools"]
