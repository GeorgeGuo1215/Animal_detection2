"""
Web Search MCP Server.

Exposes two tools:
  - web_search: General-purpose Tavily web search, returns raw result list
  - ingredient_check: Ingredient safety analysis against pet health conditions
"""
from __future__ import annotations

import json
import logging

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

from .ingredient_checker import check_ingredients
from .tavily_client import tavily_search

logger = logging.getLogger(__name__)

server = Server(name="web_search")


@server.list_tools()
async def list_tools() -> list[Tool]:
    return [
        Tool(
            name="web_search",
            description=(
                "Perform a real-time web search using Tavily and return a list of relevant results "
                "(title, URL, content snippet). Suitable for any query that requires up-to-date "
                "information from the internet: news, product details, research, prices, recipes, etc."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "The search query, e.g. 'best low-phosphorus dog food for CKD'",
                    },
                    "max_results": {
                        "type": "integer",
                        "description": "Maximum number of results to return (1-20). Default: 5.",
                        "default": 5,
                        "minimum": 1,
                        "maximum": 20,
                    },
                    "search_depth": {
                        "type": "string",
                        "enum": ["basic", "advanced"],
                        "description": "Search depth: 'basic' is faster, 'advanced' yields richer results. Default: 'basic'.",
                        "default": "basic",
                    },
                },
                "required": ["query"],
            },
        ),
        Tool(
            name="ingredient_check",
            description=(
                "Analyze a pet product's ingredients against the pet's health conditions using web search. "
                "Looks up the product's ingredient list online, then cross-references it with a built-in "
                "contraindications database for conditions such as CKD, diabetes, pancreatitis, food allergies, etc. "
                "Returns INSUFFICIENT_DATA if no ingredient information can be found online, "
                "prompting the user to provide the ingredient list manually (e.g. by photographing the packaging)."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "product_name": {
                        "type": "string",
                        "description": "Full product name, e.g. 'Royal Canin Urinary SO Dry Dog Food 8.8lb'",
                    },
                    "current_health_context": {
                        "type": "string",
                        "description": "Pet's current health conditions, e.g. 'Chronic Kidney Disease Stage 1'",
                    },
                },
                "required": ["product_name", "current_health_context"],
            },
        ),
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    if name == "web_search":
        hits = await tavily_search(
            query=arguments["query"],
            max_results=int(arguments.get("max_results", 5)),
            search_depth=str(arguments.get("search_depth", "basic")),
        )
        result = {"status": "OK", "query": arguments["query"], "results": hits, "count": len(hits)}

    elif name == "ingredient_check":
        result = await check_ingredients(
            product_name=arguments["product_name"],
            current_health_context=arguments["current_health_context"],
        )
    else:
        result = {"error": f"Unknown tool: {name}"}

    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def run_server():
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())
