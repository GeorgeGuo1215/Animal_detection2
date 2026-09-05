"""A-D 重构期间必须保持的模块边界与请求隔离契约。"""

from __future__ import annotations

import os
import sys
import asyncio
import json

import httpx

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.tools.request_scope import (  # noqa: E402
    bind_tool_request_scope,
    filter_tools_without_animal,
    get_request_animal_id,
)
from agent_api.app.tools.tool_registry import ToolRegistry  # noqa: E402
from agent_api.app.tools.builtin import register_builtin_tools  # noqa: E402
from agent_api.app.integrations.llm.client import OpenAIChatClient  # noqa: E402
from agent_api.app.integrations.llm.config import load_transport_settings  # noqa: E402


def test_tool_request_scope_is_nested_and_resets_after_exception() -> None:
    """嵌套请求和异常退出都不能泄漏上一只动物的身份。"""
    assert get_request_animal_id() is None
    try:
        with bind_tool_request_scope(explicit=" cat-1 "):
            assert get_request_animal_id() == "cat-1"
            with bind_tool_request_scope(body_animal_id="dog-2"):
                assert get_request_animal_id() == "dog-2"
            assert get_request_animal_id() == "cat-1"
            raise RuntimeError("cancelled request")
    except RuntimeError:
        pass
    assert get_request_animal_id() is None


def test_animal_tools_are_visible_only_inside_scoped_request() -> None:
    """缺少 animal_id 时必须过滤个体数据库工具。"""
    tools = ["rag.search", "sql.search", "vitals.summary"]
    assert filter_tools_without_animal(tools) == ["rag.search"]
    with bind_tool_request_scope(header_animal_id="cat-1"):
        assert filter_tools_without_animal(tools) == tools


def test_rag_tool_schema_does_not_expose_infrastructure_credentials() -> None:
    """模型不可控制索引路径、设备、模型地址或 API Key。"""
    registry = ToolRegistry()
    register_builtin_tools(registry)
    rag = registry.get("rag.search")
    assert rag is not None
    properties = rag.input_schema["properties"]
    forbidden = {
        "index_dir", "device", "embedding_model", "rewrite_base_url",
        "rewrite_api_key", "rewrite_model", "rerank_model",
    }
    assert forbidden.isdisjoint(properties)


def test_rag_handler_drops_forged_infrastructure_arguments(monkeypatch) -> None:
    """即使模型绕过 JSON Schema 伪造字段，handler 也不能接收内部配置。"""
    import agent_api.app.tools.builtin.rag as rag_module

    seen = {}
    monkeypatch.setattr(rag_module, "rag_search_tool", lambda **kwargs: seen.update(kwargs) or {})
    rag_module._rag_search_handler(
        query="feline cystitis",
        top_k=3,
        index_dir="C:/secret-index",
        rewrite_api_key="stolen-key",
        device="cuda:9",
    )
    assert seen == {"query": "feline cystitis", "top_k": 3}


def test_production_and_tests_use_canonical_imports() -> None:
    """旧别名不可重新引入；避免两套 app 包持有不同的单例。"""
    import ast
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    forbidden = ('app.', 'agent_api.app.llm.', 'agent_api.app.sql_search.',
                 'agent_api.app.platform.run_service', 'agent_api.app.services.moe.orchestrator',
                 'agent_api.app.services.moe.experts', 'agent_api.app.context.request_context',
                 'agent_api.app.persistence.qa_store', 'agent_api.app.persistence.trace_store',
                 'agent_api.app.tools.tools_builtin')
    violations = []
    for folder in (root / 'app', root / 'tests'):
        for path in folder.rglob('*.py'):
            source = path.read_text(encoding='utf-8-sig')
            tree = ast.parse(source)
            for node in ast.walk(tree):
                names = [node.module or ''] if isinstance(node, ast.ImportFrom) else [a.name for a in node.names] if isinstance(node, ast.Import) else []
                if any(name.startswith(forbidden) for name in names):
                    violations.append(f'{path.name}:{node.lineno}')
    assert not violations


def test_stream_and_non_stream_share_one_default_client(monkeypatch) -> None:
    """生产编排器的两种调用模式必须复用同一连接池。"""
    import agent_api.app.integrations.llm.client as client_module

    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    client_module._SHARED_ASYNC_CLIENT = None
    complete_client = client_module.get_shared_async_client()
    stream_client = client_module.get_shared_async_stream_client()
    assert complete_client is stream_client
    asyncio.run(client_module.aclose_shared_async_client())


def test_llm_transport_settings_are_environment_configurable(monkeypatch) -> None:
    """连接池容量与超时来自环境变量，且 keepalive 不超过总连接数。"""
    monkeypatch.setenv("AGENT_LLM_CONNECT_TIMEOUT_SEC", "7.5")
    monkeypatch.setenv("AGENT_LLM_MAX_CONNECTIONS", "6")
    monkeypatch.setenv("AGENT_LLM_MAX_KEEPALIVE_CONNECTIONS", "20")
    settings = load_transport_settings()
    assert settings.connect_timeout_s == 7.5
    assert settings.max_connections == 6
    assert settings.max_keepalive_connections == 6


def test_zero_temperature_is_preserved_in_llm_payload() -> None:
    """temperature=0.0 是合法配置，不能被 truthy 回退覆盖。"""
    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen.update(json.loads(request.content))
        return httpx.Response(200, json={"choices": [{"message": {"content": "ok"}}]})

    async def scenario() -> None:
        http_client = httpx.AsyncClient(transport=httpx.MockTransport(handler))
        client = OpenAIChatClient(
            base_url="https://llm.test", api_key="test", model="model", client=http_client,
        )
        await client.chat(
            messages=[{"role": "user", "content": "case"}], temperature=0.0, max_tokens=16,
        )
        await client.close()

    asyncio.run(scenario())
    assert seen["temperature"] == 0.0
    assert seen["max_tokens"] == 16
