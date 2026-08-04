"""Entry point: python -m mcp_servers.vital_signs_analyzer"""
import asyncio

from .server import run_server

if __name__ == "__main__":
    asyncio.run(run_server())
