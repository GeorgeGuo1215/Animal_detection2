from __future__ import annotations

import logging
import os
from typing import Any, Dict, List

from mcp import ClientSession  # type: ignore[import]
from mcp.client.streamable_http import (  # type: ignore[import]
    streamable_http_client,
)

logger = logging.getLogger(__name__)

_TAVILY_MCP_URL_TEMPLATE = "https://mcp.tavily.com/mcp/?tavilyApiKey={key}"


async def tavily_search(
    query: str,
    max_results: int = 5,
    search_depth: str = "basic",
) -> List[Dict[str, Any]]:
    """
    调用远程 Tavily MCP Server 的 tavily_search 工具，返回统一的搜索结果结构。

    返回格式（简化版）:
    [
      {
        "title": "...",
        "url": "...",
        "content": "..."  # 主要文本/摘要
      },
      ...
    ]
    """
    api_key = os.getenv("TAVILY_API_KEY", "").strip()
    if not api_key:
        logger.warning("TAVILY_API_KEY not set, Tavily search disabled.")
        return []

    url = _TAVILY_MCP_URL_TEMPLATE.format(key=api_key)

    try:
        async with streamable_http_client(url=url) as (read, write, _):
            async with ClientSession(read, write) as session:
                await session.initialize()
                result = await session.call_tool(
                    "tavily_search",
                    {
                        "query": query,
                        "max_results": max_results,
                        "search_depth": search_depth,
                    },
                )
    except Exception as exc:  # noqa: BLE001
        logger.warning("Tavily MCP search failed: %s", exc)
        return []

    # result.content 可能是 MCP TextContent 列表或类似结构，交给 mcp_client 规范化
    content = getattr(result, "content", None) or getattr(result, "contents", None)
    items: List[Dict[str, Any]] = []

    if isinstance(content, list):
        for c in content:
            # 兼容 pydantic model / dict / 其他
            if hasattr(c, "model_dump"):
                data = c.model_dump()  # type: ignore[assignment]
            elif isinstance(c, dict):
                data = c
            else:
                data = {"type": getattr(c, "type", "text"), "text": str(c)}

            text = data.get("text") or data.get("value") or ""
            meta = data.get("metadata") or {}
            items.append(
                {
                    "title": meta.get("title") or "",
                    "url": meta.get("url") or meta.get("source") or "",
                    "content": text,
                }
            )

    return items