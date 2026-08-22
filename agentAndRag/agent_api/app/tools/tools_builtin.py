"""兼容入口；内置工具已按能力拆分到 tools.builtin。"""

from .builtin import register_builtin_tools, register_debug_tools

__all__ = ["register_builtin_tools", "register_debug_tools"]
