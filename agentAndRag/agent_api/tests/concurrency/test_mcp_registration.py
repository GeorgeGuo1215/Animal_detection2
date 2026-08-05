from __future__ import annotations

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_AGENT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_AGENT_ROOT))

from app.mcp.mcp_config import McpServerConfig  # noqa: E402
from app.concurrency import AsyncResourceLimiter  # noqa: E402
from app.tools.tool_registry import ToolRegistry  # noqa: E402
from app.tools.tools_mcp import register_mcp_tools_async  # noqa: E402


def test_async_mcp_registration_does_not_use_sync_wrapper(monkeypatch: pytest.MonkeyPatch) -> None:
    import app.tools.tools_mcp as tools_mcp

    server = McpServerConfig(name="demo", command=sys.executable, args=["-m", "demo"])

    async def _list_async(_server: McpServerConfig):
        return [{
            "name": "ping",
            "description": "Ping the demo server",
            "input_schema": {"type": "object", "properties": {}},
        }]

    monkeypatch.setattr(tools_mcp, "load_mcp_servers", lambda: [server])
    monkeypatch.setattr(tools_mcp, "list_mcp_tools_async", _list_async)
    monkeypatch.setattr(
        tools_mcp,
        "list_mcp_tools",
        lambda _server: (_ for _ in ()).throw(AssertionError("sync MCP registration path used")),
    )

    registry = ToolRegistry()
    summary = asyncio.run(register_mcp_tools_async(registry))

    assert summary == {"servers": ["demo"], "tools": 1}
    assert registry.get("mcp.demo.ping") is not None


def test_mcp_limiter_covers_session_lifecycle(monkeypatch: pytest.MonkeyPatch) -> None:
    import app.mcp.mcp_client as mcp_client

    active = 0
    max_active = 0
    limits = SimpleNamespace(
        mcp=AsyncResourceLimiter("mcp", 2),
        acquire_timeout_s=1.0,
    )

    class _StdioContext:
        async def __aenter__(self):
            return object(), object()

        async def __aexit__(self, exc_type, exc, tb):
            return False

    class _Session:
        def __init__(self, read, write):
            pass

        async def __aenter__(self):
            nonlocal active, max_active
            active += 1
            max_active = max(max_active, active)
            return self

        async def __aexit__(self, exc_type, exc, tb):
            nonlocal active
            active -= 1
            return False

        async def initialize(self) -> None:
            return None

    monkeypatch.setattr(mcp_client, "get_resource_limits", lambda: limits)
    monkeypatch.setattr(mcp_client, "StdioServerParameters", lambda **kwargs: kwargs)
    monkeypatch.setattr(mcp_client, "stdio_client", lambda params: _StdioContext())
    monkeypatch.setattr(mcp_client, "ClientSession", _Session)

    cfg = McpServerConfig(name="demo", command=sys.executable, args=["-m", "demo"])

    async def _run() -> None:
        async def _work(_session) -> str:
            await asyncio.sleep(0.02)
            return "ok"

        results = await asyncio.gather(*(mcp_client._with_session(cfg, _work) for _ in range(6)))
        assert results == ["ok"] * 6

    asyncio.run(_run())
    assert max_active == 2
    assert limits.mcp.snapshot()["active"] == 0
    assert limits.mcp.snapshot()["acquired"] == 6
