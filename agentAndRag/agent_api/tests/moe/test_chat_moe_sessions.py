from __future__ import annotations

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.persistence.session_manager import SessionManager
from app.prompts.moe_router import build_router_system_prompt
from app.routers import routes_chat_ui
from app.memory.identity import chat_moe_memory_user_id, normalize_test_username
from app.schemas.chat_moe import ChatMoeCompletionRequest, ChatMoeSessionRequest


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


def test_chat_moe_username_identity_is_stable_and_not_plaintext():
    assert normalize_test_username("  Test   User  ") == "Test User"
    first = chat_moe_memory_user_id("Ｔｅｓｔ User")
    second = chat_moe_memory_user_id("test   user")
    assert first == second
    assert first.startswith("chatmoe:")
    assert "test" not in first


def test_router_treats_pet_identity_memory_as_health_followup_context():
    prompt = build_router_system_prompt("clinical")
    assert "宠物身份与照护档案" in prompt
    assert "不属于闲聊" in prompt
    assert "clinical" in prompt


def test_chat_moe_shares_memory_across_sessions_but_not_users(tmp_path, monkeypatch):
    async def scenario():
        manager = SessionManager(db_path=tmp_path / "cross_session.db")
        calls = []
        stored = {}

        async def ensure_subject(**kwargs):
            return {"enabled": True, "ensured": True, "user_id": kwargs["user_id"]}

        async def load_memory(*, user_id, query, pet_id):
            del query, pet_id
            prior = stored.get(user_id)
            return (
                (f"CROSS_SESSION_MEMORY={prior}" if prior else ""),
                {"enabled": True, "loaded": True, "user_id": user_id, "context_chars": len(prior or "")},
            )

        async def write_memory(*, user_id, query, answer, **kwargs):
            del query, kwargs
            stored[user_id] = answer
            return {"stored": True, "user_id": user_id, "message_id": "memory-1"}

        monkeypatch.setattr(routes_chat_ui, "get_session_manager", lambda: manager)
        monkeypatch.setattr(routes_chat_ui, "get_registry", lambda: _Registry())
        monkeypatch.setattr(routes_chat_ui, "ensure_memory_subject", ensure_subject)
        monkeypatch.setattr(routes_chat_ui, "load_user_memory", load_memory)
        monkeypatch.setattr(routes_chat_ui, "write_user_memory", write_memory)
        monkeypatch.setattr(
            routes_chat_ui, "build_moe_orchestrator", lambda **kwargs: _FakeOrchestrator(calls)
        )

        first_session = await routes_chat_ui.create_chat_moe_session(
            ChatMoeSessionRequest(username="Alice")
        )
        first = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": first_session.session_id, "message": "记住猫叫团团",
        }))
        assert "后端回答1" in await _consume(first)

        second_session = await routes_chat_ui.create_chat_moe_session(
            ChatMoeSessionRequest(username=" alice ")
        )
        second = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": second_session.session_id, "message": "它叫什么？",
        }))
        await _consume(second)
        assert calls[1]["conversation_history"] == []
        assert "CROSS_SESSION_MEMORY=后端回答1" in calls[1]["system_context"]
        assert "CROSS_SESSION_MEMORY=后端回答1" in calls[1]["user_memory"]
        assert first_session.memory_user_id == second_session.memory_user_id

        other_session = await routes_chat_ui.create_chat_moe_session(
            ChatMoeSessionRequest(username="Bob")
        )
        third = await routes_chat_ui.chat_moe_public_completions(_request({
            "session_id": other_session.session_id, "message": "它叫什么？",
        }))
        await _consume(third)
        assert "CROSS_SESSION_MEMORY=" not in calls[2]["system_context"]
        assert other_session.memory_user_id != first_session.memory_user_id

    asyncio.run(scenario())


def test_browser_pages_do_not_store_or_slice_message_history():
    assert "const messages" not in routes_chat_ui._MOE_TEST_HTML
    assert "messages.push" not in routes_chat_ui._MOE_TEST_HTML
    assert "messages.slice" not in routes_chat_ui._MOE_TEST_HTML
    assert "session_id" in routes_chat_ui._MOE_TEST_HTML
    assert "username" in routes_chat_ui._MOE_TEST_HTML
    assert "const turns=new Map()" in routes_chat_ui._MOE_TEST_HTML
    assert "turn.root.querySelector" in routes_chat_ui._MOE_TEST_HTML
    assert "expert-card" in routes_chat_ui._MOE_TEST_HTML
    assert "crypto.randomUUID" in routes_chat_ui._MOE_TEST_HTML
    assert "Date.now()" not in routes_chat_ui._MOE_TEST_HTML
    assert "max-height:min(620px,68vh);overflow:auto" in routes_chat_ui._MOE_TEST_HTML
    assert ".workbench.collapsed .trace-body{max-height:0" in routes_chat_ui._MOE_TEST_HTML
    assert ".answer{position:relative;z-index:1" in routes_chat_ui._MOE_TEST_HTML

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
