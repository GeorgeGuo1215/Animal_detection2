"""Entry point: python -m mcp_servers.price_watcher_pro"""
import asyncio

from .server import run_server

if __name__ == "__main__":
    asyncio.run(run_server())
