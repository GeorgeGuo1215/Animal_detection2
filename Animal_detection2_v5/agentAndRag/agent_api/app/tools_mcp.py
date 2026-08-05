from __future__ import annotations

import logging
from typing import Any, Dict

from .mcp_client import call_mcp_tool, call_mcp_tool_async, list_mcp_tools
from .mcp_config import McpServerConfig, load_mcp_servers
from .tool_registry import ToolRegistry, ToolSpec

logger = logging.getLogger(__name__)


def _make_async_handler(server: McpServerConfig, tool_name: str):
    """Return an async handler that calls the MCP tool natively."""

    async def _handler(**kwargs: Any) -> Dict[str, Any]:
        return await call_mcp_tool_async(server, tool_name, kwargs)

    return _handler


def register_mcp_tools(registry: ToolRegistry) -> Dict[str, Any]:
    """
    Load MCP servers from config and register each MCP tool into ToolRegistry.
    Uses async handlers so the event loop is not blocked during MCP calls.
    """
    servers = load_mcp_servers()
    summary: Dict[str, Any] = {"servers": [], "tools": 0}

    for server in servers:
        try:
            tools = list_mcp_tools(server)
        except Exception as exc:  # noqa: BLE001
            logger.warning("MCP server %s load failed: %s", server.name, exc)
            continue

        for t in tools:
            tool_name = str(t.get("name") or "").strip()
            if not tool_name:
                continue
            name = f"mcp.{server.name}.{tool_name}"
            if registry.get(name) is not None:
                continue

            description = t.get("description") or ""
            input_schema = t.get("input_schema") or {"type": "object", "properties": {}}
            registry.register(
                ToolSpec(
                    name=name,
                    description=f"[mcp:{server.name}] {description}".strip(),
                    input_schema=input_schema,
                    handler=_make_async_handler(server, tool_name),
                )
            )
            summary["tools"] += 1

        summary["servers"].append(server.name)

    return summary
