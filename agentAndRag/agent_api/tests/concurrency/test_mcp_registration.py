from __future__ import annotations

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_AGENT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_AGENT_ROOT))

from agent_api.app.mcp.mcp_config import McpServerConfig  # noqa: E402
from agent_api.app.concurrency import AsyncResourceLimiter  # noqa: E402
from agent_api.app.tools.tool_registry import ToolRegistry  # noqa: E402
from agent_api.app.tools.tools_mcp import register_mcp_tools_async  # noqa: E402


def test_async_mcp_registration_does_not_use_sync_wrapper(monkeypatch: pytest.MonkeyPatch) -> None:
    """验证异步 MCP 注册不会走同步包装。"""
    import agent_api.app.tools.tools_mcp as tools_mcp

    server = McpServerConfig(name="demo", command=sys.executable, args=["-m", "demo"])

    async def _list_async(_server: McpServerConfig):
        """异步列出 MCP 工具，用来证明没有走同步包装。"""
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
    """验证 MCP 限流覆盖会话从创建到关闭的生命周期。"""
    import agent_api.app.mcp.mcp_client as mcp_client

    active = 0
    max_active = 0
    limits = SimpleNamespace(
        mcp=AsyncResourceLimiter("mcp", 2),
        acquire_timeout_s=1.0,
    )

    class _StdioContext:
        async def __aenter__(self):
            """异步进入上下文并返回自身。"""
            return object(), object()

        async def __aexit__(self, exc_type, exc, tb):
            """异步退出上下文。"""
            return False

    class _Session:
        def __init__(self, read, write):
            """初始化该测试替身。"""
            pass

        async def __aenter__(self):
            """异步进入上下文并返回自身。"""
            nonlocal active, max_active
            active += 1
            max_active = max(max_active, active)
            return self

        async def __aexit__(self, exc_type, exc, tb):
            """异步退出上下文。"""
            nonlocal active
            active -= 1
            return False

        async def initialize(self) -> None:
            """测试替身：完成初始化。"""
            return None

    monkeypatch.setattr(mcp_client, "get_resource_limits", lambda: limits)
    monkeypatch.setattr(mcp_client, "StdioServerParameters", lambda **kwargs: kwargs)
    monkeypatch.setattr(mcp_client, "stdio_client", lambda params: _StdioContext())
    monkeypatch.setattr(mcp_client, "ClientSession", _Session)

    cfg = McpServerConfig(name="demo", command=sys.executable, args=["-m", "demo"])

    async def _run() -> None:
        """运行本用例的异步主体。"""
        async def _work(_session) -> str:
            """占用限流名额的异步工作协程。"""
            await asyncio.sleep(0.02)
            return "ok"

        results = await asyncio.gather(*(mcp_client._with_session(cfg, _work) for _ in range(6)))
        assert results == ["ok"] * 6

    asyncio.run(_run())
    assert max_active == 2
    assert limits.mcp.snapshot()["active"] == 0
    assert limits.mcp.snapshot()["acquired"] == 6
