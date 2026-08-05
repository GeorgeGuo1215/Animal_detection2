"""Entry point: python -m mcp_servers.nutritional_planner"""
import asyncio

from .server import run_server

if __name__ == "__main__":
    asyncio.run(run_server())
