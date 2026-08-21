from __future__ import annotations

import logging
import os
from typing import Any, Dict

from ..concurrency import get_resource_limits
from ..mcp.mcp_client import call_mcp_tool_async, list_mcp_tools, list_mcp_tools_async
from ..mcp.mcp_config import McpServerConfig, load_mcp_servers
from .tool_registry import ToolRegistry, ToolSpec

logger = logging.getLogger(__name__)


async def _direct_web_search_handler(**kwargs: Any) -> Dict[str, Any]:
    """直接调用 Tavily REST API，绕过 MCP 子进程。"""
    from mcp_servers.web_search.tavily_client import tavily_search

    query = kwargs.get("query", "")
    max_results = int(kwargs.get("max_results", 5))
    search_depth = str(kwargs.get("search_depth", "basic"))
    limits = get_resource_limits()
    async with limits.mcp.slot(timeout_s=limits.acquire_timeout_s):
        hits = await tavily_search(query=query, max_results=max_results, search_depth=search_depth)
    return {"status": "OK", "query": query, "results": hits, "count": len(hits)}


async def _direct_ingredient_check_handler(**kwargs: Any) -> Dict[str, Any]:
    """直接调用成分检查，绕过 MCP 子进程。"""
    from mcp_servers.web_search.ingredient_checker import check_ingredients

    product_name = str(kwargs.get("product_name") or "").strip()
    current_health_context = str(kwargs.get("current_health_context") or "").strip()
    limits = get_resource_limits()
    async with limits.mcp.slot(timeout_s=limits.acquire_timeout_s):
        return await check_ingredients(
            product_name=product_name,
            current_health_context=current_health_context,
        )


def _make_async_handler(server: McpServerConfig, tool_name: str):
    """返回原生调用该 MCP 工具的异步 handler。"""

    async def _handler(**kwargs: Any) -> Dict[str, Any]:
        """把关键字参数原样转发给 MCP 工具。"""
        return await call_mcp_tool_async(server, tool_name, kwargs)

    return _handler


def _register_server_tools(registry: ToolRegistry, server: McpServerConfig, tools: list[Dict[str, Any]]) -> int:
    """将 MCP 服务器上的工具注册进 ToolRegistry，返回新注册数量。"""
    registered = 0
    for tool in tools:
        tool_name = str(tool.get("name") or "").strip()
        if not tool_name:
            continue
        name = f"mcp.{server.name}.{tool_name}"
        if registry.get(name) is not None:
            continue

        description = tool.get("description") or ""
        input_schema = tool.get("input_schema") or {"type": "object", "properties": {}}
        registry.register(
            ToolSpec(
                name=name,
                description=f"[mcp:{server.name}] {description}".strip(),
                input_schema=input_schema,
                handler=_make_async_handler(server, tool_name),
            )
        )
        registered += 1
    return registered


def _register_direct_web_search(registry: ToolRegistry) -> int:
    """注册直连 Tavily 的 web_search 与 ingredient_check 工具。"""
    registered = 0
    name = "mcp.web_search.web_search"
    if registry.get(name) is None:
        registry.register(
            ToolSpec(
                name=name,
                description=(
                    "Perform a real-time web search using Tavily and return "
                    "relevant results (title, URL, content snippet)."
                ),
                input_schema={
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"},
                        "max_results": {"type": "integer", "default": 5},
                        "search_depth": {"type": "string", "enum": ["basic", "advanced"], "default": "basic"},
                    },
                    "required": ["query"],
                },
                handler=_direct_web_search_handler,
            )
        )
        registered += 1
    ingredient_name = "mcp.web_search.ingredient_check"
    if registry.get(ingredient_name) is None:
        registry.register(
            ToolSpec(
                name=ingredient_name,
                description=(
                    "Analyze a pet product's ingredients against the pet's health conditions "
                    "using web search and a contraindications database."
                ),
                input_schema={
                    "type": "object",
                    "properties": {
                        "product_name": {"type": "string"},
                        "current_health_context": {"type": "string"},
                    },
                    "required": ["product_name", "current_health_context"],
                },
                handler=_direct_ingredient_check_handler,
            )
        )
        registered += 1
    if registered:
        logger.info("Registered web_search tools via direct Tavily REST API (fast path)")
    return registered


async def register_mcp_tools_async(registry: ToolRegistry) -> Dict[str, Any]:
    """注册 MCP 工具，不创建第二个事件循环。"""
    servers = load_mcp_servers()
    summary: Dict[str, Any] = {"servers": [], "tools": 0}

    for server in servers:
        if server.name == "web_search" and os.getenv("TAVILY_API_KEY", "").strip():
            summary["tools"] += _register_direct_web_search(registry)
            summary["servers"].append(server.name)
            continue

        try:
            tools = await list_mcp_tools_async(server)
        except Exception as exc:  # noqa: BLE001
            logger.warning("MCP server %s load failed: %s", server.name, exc)
            continue
        summary["tools"] += _register_server_tools(registry, server, tools)
        summary["servers"].append(server.name)

    return summary


def register_mcp_tools(registry: ToolRegistry) -> Dict[str, Any]:
    """
    从配置加载 MCP 服务器，并将各 MCP 工具注册进 ToolRegistry。
    web_search 走直连 REST API，而非 MCP 子进程。
    """
    servers = load_mcp_servers()
    summary: Dict[str, Any] = {"servers": [], "tools": 0}

    for server in servers:
        if server.name == "web_search" and os.getenv("TAVILY_API_KEY", "").strip():
            summary["tools"] += _register_direct_web_search(registry)
            summary["servers"].append(server.name)
            continue

        try:
            tools = list_mcp_tools(server)
        except Exception as exc:  # noqa: BLE001
            logger.warning("MCP server %s load failed: %s", server.name, exc)
            continue

        summary["tools"] += _register_server_tools(registry, server, tools)
        summary["servers"].append(server.name)

    return summary
