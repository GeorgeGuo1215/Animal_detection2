from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.persistence.session_manager import SessionManager
from app.routers import routes_chat_ui
from app.schemas.chat_moe import ChatMoeCompletionRequest


class _Registry:
    def list_tools(self):
        return []


class _FakeOrchestrator:
    def __init__(self, calls):
        self.calls = calls
        self.last_run_context = {}

    async def stream(self, **kwargs):
        self.calls.append(kwargs)
        turn = len(self.calls)
        self.last_run_context = {
            "router": {"selected_experts": ["clinical"]},
            "experts": [{
                "expert": "clinical", "conclusion": f"专家结论{turn}",
                "evidence": [f"检索证据{turn}"], "risks": [], "tools_used": ["rag.search"],
                "plan_steps": [{"tool_name": "rag.search", "arguments": {"query": "cat medicine"}}],
                "tool_results": [{"tool_name": "rag.search", "ok": True, "result": {"hits": []}}],
            }],
            "critic": {"verdict": "pass"},
        }
        yield {"status": "streaming", "content": f"后端回答{turn}"}
        yield {"finish": "stop"}


def _request(payload):
    return ChatMoeCompletionRequest(**payload)


async def _consume(response):
    parts = []
    async for chunk in response.body_iterator:
        parts.append(chunk.decode() if isinstance(chunk, bytes) else chunk)
    return "".join(parts)


def test_chat_moe_uses_backend_history_and_persists_expert_context(tmp_path, monkeypatch):
    async def scenario():
        manager = SessionManager(db_path=tmp_path / "chat.db")
        session = await manager.create()
        calls = []
        monkeypatch.setattr(routes_chat_ui, "get_session_manager", lambda: manager)
        monkeypatch.setattr(routes_chat_ui, "get_registry", lambda: _Registry())
        monkeypatch.setattr(
            routes_chat_ui, "build_moe_orchestrator", lambda **kwargs: _FakeOrchestrator(calls)
        )

        first = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": session.session_id, "message": "第一轮", "user_role": "pet_owner",
        }))
        assert "后端回答1" in await _consume(first)

        second = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": session.session_id, "message": "第二轮", "user_role": "pet_owner",
        }))
        assert "后端回答2" in await _consume(second)

        assert calls[0]["conversation_history"] == []
        assert [item["content"] for item in calls[1]["conversation_history"]] == ["第一轮", "后端回答1"]
        assert calls[1]["expert_context_history"][0]["experts"][0]["conclusion"] == "专家结论1"
        persisted = await manager.get(session.session_id, touch=False)
        assert persisted is not None
        assert len(persisted.messages) == 4
        assert len(persisted.expert_contexts) == 2
        assert len(persisted.tool_results) == 2

    asyncio.run(scenario())


def test_browser_pages_do_not_store_or_slice_message_history():
    assert "const messages" not in routes_chat_ui._MOE_TEST_HTML
    assert "messages.push" not in routes_chat_ui._MOE_TEST_HTML
    assert "messages.slice" not in routes_chat_ui._MOE_TEST_HTML
    assert "session_id" in routes_chat_ui._MOE_TEST_HTML

    assert "const messages = []" in routes_chat_ui._CHAT_HTML
    assert "[...messages, { role: 'user', content: text }].slice(-11)" in routes_chat_ui._CHAT_HTML
    assert "messages.push(\n      { role: 'user', content: text }," in routes_chat_ui._CHAT_HTML
    assert "fetch('/v1/chat/completions'" in routes_chat_ui._CHAT_HTML
    assert "fetch('/chat-moe/completions'" not in routes_chat_ui._CHAT_HTML


def test_chat_moe_reports_loaded_context_metadata(tmp_path, monkeypatch):
    async def scenario():
        manager = SessionManager(db_path=tmp_path / "context_event.db")
        session = await manager.create()
        calls = []
        monkeypatch.setattr(routes_chat_ui, "get_session_manager", lambda: manager)
        monkeypatch.setattr(routes_chat_ui, "get_registry", lambda: _Registry())
        monkeypatch.setattr(
            routes_chat_ui, "build_moe_orchestrator", lambda **kwargs: _FakeOrchestrator(calls)
        )

        first = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": session.session_id, "message": "第一轮",
        }))
        await _consume(first)
        second = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": session.session_id, "message": "这个为什么？",
        }))
        body = await _consume(second)
        events = [
            json.loads(line[6:])
            for line in body.splitlines()
            if line.startswith("data: {")
        ]
        context_event = next(
            item for item in events if item.get("agent_status") == "session_context_loaded"
        )

        assert context_event["agent_detail"] == {
            "complete_turns": 1,
            "expert_context_turns": 1,
            "prior_experts": ["clinical"],
            "prior_tools": ["rag.search"],
            "prior_searches": 1,
            "prior_evidence_items": 1,
        }

    asyncio.run(scenario())


def test_openapi_documents_stateful_test_and_stateless_production_boundaries():
    from app.main import app

    schema = app.openapi()
    chat_moe = schema["paths"]["/chat-moe/completions"]["post"]
    production = schema["paths"]["/v1/chat/completions"]["post"]

    request_schema = chat_moe["requestBody"]["content"]["application/json"]["schema"]
    assert request_schema["$ref"].endswith("/ChatMoeCompletionRequest")
    assert "server history only for `/chat-moe`" in chat_moe["description"]
    assert "does not create a conversation session" in production["description"]
    assert schema["paths"]["/sessions"]["post"]["deprecated"] is True
