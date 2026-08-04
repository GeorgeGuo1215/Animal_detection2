from __future__ import annotations

import json
import logging
import os
from typing import Any, Dict, List

import httpx

logger = logging.getLogger(__name__)

_TAVILY_REST_URL = "https://api.tavily.com/search"

_http_pool: httpx.AsyncClient | None = None


def _get_http_pool() -> httpx.AsyncClient:
    global _http_pool
    if _http_pool is None or _http_pool.is_closed:
        _http_pool = httpx.AsyncClient(
            timeout=httpx.Timeout(connect=5, read=30, write=5, pool=10),
            limits=httpx.Limits(max_connections=10, max_keepalive_connections=5),
        )
    return _http_pool


def _normalize_result_entry(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize a single search result dict to {title, url, content}."""
    return {
        "title": entry.get("title") or "",
        "url": entry.get("url") or entry.get("link") or entry.get("source") or "",
        "content": entry.get("content") or entry.get("snippet") or entry.get("text") or "",
    }


async def tavily_search(
    query: str,
    max_results: int = 5,
    search_depth: str = "basic",
) -> List[Dict[str, Any]]:
    """
    Call the Tavily REST API directly (no MCP subprocess) and return search results.

    Returns:
        [{"title": "...", "url": "...", "content": "..."}, ...]
    """
    api_key = os.getenv("TAVILY_API_KEY", "").strip()
    if not api_key:
        logger.warning("TAVILY_API_KEY not set, Tavily search disabled.")
        return []

    payload = {
        "query": query,
        "max_results": int(max_results),
        "search_depth": search_depth,
    }
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    try:
        client = _get_http_pool()
        resp = await client.post(_TAVILY_REST_URL, json=payload, headers=headers)
        resp.raise_for_status()
        data = resp.json()
    except Exception as exc:
        logger.warning("Tavily REST search failed: %s", exc)
        return []

    results = data.get("results") or []
    return [_normalize_result_entry(r) for r in results if isinstance(r, dict)]
