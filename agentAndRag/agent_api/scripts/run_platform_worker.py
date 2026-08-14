from __future__ import annotations

import asyncio
import os

from agent_api.app.concurrency import configure_resource_limits
from agent_api.app.llm.llm_client import aclose_shared_async_client
from agent_api.app.llm.llm_client_stream import aclose_shared_async_stream_client
from agent_api.app.memory import close_memory_client, start_memory_client
from agent_api.app.platform.database import close_platform_database, init_platform_database
from agent_api.app.platform.run_service import worker_forever
from agent_api.app.tools.tool_registry import get_registry
from agent_api.app.tools.tools_builtin import register_builtin_tools, register_debug_tools
from agent_api.app.tools.tools_mcp import register_mcp_tools_async


async def _main() -> None:
    configure_resource_limits()
    await init_platform_database()
    await start_memory_client()
    registry = get_registry()
    if registry.get("rag.search") is None:
        register_builtin_tools(registry)
        register_debug_tools(registry)
        if os.getenv("AGENT_ENABLE_MCP", "1") == "1":
            await register_mcp_tools_async(registry)
    try:
        await worker_forever()
    finally:
        await close_memory_client()
        await aclose_shared_async_client()
        await aclose_shared_async_stream_client()
        await close_platform_database()


if __name__ == "__main__":
    asyncio.run(_main())
