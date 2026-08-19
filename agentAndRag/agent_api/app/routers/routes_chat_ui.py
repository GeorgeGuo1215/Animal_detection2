"""Stateful MoE diagnostic console; legacy /chat and /admin pages were removed."""
from __future__ import annotations

import hmac
import uuid
from pathlib import Path
from typing import List, Optional

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession

from ..memory import (
    chat_moe_memory_user_id,
    ensure_memory_subject,
    load_user_memory,
    normalize_test_username,
    write_user_memory,
)
from ..persistence.session_manager import get_session_manager
from ..platform.config import get_platform_settings
from ..platform.database import get_platform_session
from ..platform.dependencies import get_current_principal
from ..schemas.chat_moe import ChatMoeCompletionRequest, ChatMoeSessionRequest, ChatMoeSessionResponse
from ..services.agent_execution import build_moe_orchestrator, public_moe_allowed_tools
from ..tools.tool_registry import get_registry
from ..worker_proxy import proxy_json_to_worker, should_delegate_agent_execution, worker_token
from .sse import SSE_DONE, SSE_RESPONSE_HEADERS, openai_sse_chunk

async def _require_diagnostic_access(
    request: Request,
    session: AsyncSession = Depends(get_platform_session),
) -> None:
    """Keep the diagnostic console open in development and admin-only in production."""
    if not get_platform_settings().production:
        return
    supplied_worker_token = request.headers.get("x-petmind-worker-token", "")
    if supplied_worker_token and hmac.compare_digest(supplied_worker_token, worker_token()):
        return
    principal = await get_current_principal(request, session)
    if principal.auth_kind != "jwt" or principal.role != "SUPER_ADMIN":
        raise HTTPException(status_code=403, detail="diagnostic console requires SUPER_ADMIN")


chat_moe_router = APIRouter(dependencies=[Depends(_require_diagnostic_access)])

_MOE_TEST_HTML_PATH = Path(__file__).resolve().parents[1] / "static" / "chat_moe.html"
_MOE_TEST_HTML = _MOE_TEST_HTML_PATH.read_text(encoding="utf-8")


@chat_moe_router.get("/chat-moe", response_class=HTMLResponse)
async def chat_moe_test_ui():
    return _MOE_TEST_HTML


@chat_moe_router.post("/chat-moe/sessions", response_model=ChatMoeSessionResponse)
async def create_chat_moe_session(body: ChatMoeSessionRequest) -> ChatMoeSessionResponse:
    username = normalize_test_username(body.username)
    if not username:
        raise HTTPException(status_code=400, detail="username is required")
    memory_user_id = chat_moe_memory_user_id(username)
    memory_detail = await ensure_memory_subject(
        user_id=memory_user_id,
        display_name=username,
        source="chat-moe",
        metadata={"channel": "chat-moe", "identity_kind": "tester_username"},
    )
    session = await get_session_manager().create({
        "channel": "chat-moe", "username": username, "memory_user_id": memory_user_id,
    })
    return ChatMoeSessionResponse(
        session_id=session.session_id,
        username=username,
        memory_user_id=memory_user_id,
        memory=memory_detail,
    )


def _moe_request_id() -> str:
    return f"chatcmpl-{uuid.uuid4().hex[:24]}"


def _moe_system_context(response_lang: str, memory_injection: Optional[str] = None) -> str:
    parts: List[str] = []
    if response_lang in {"zh", "en"}:
        parts.append("请使用中文回答。" if response_lang == "zh" else "Please answer in English.")
    if memory_injection:
        parts.append(memory_injection)
    return "\n".join(parts)


@chat_moe_router.post(
    "/chat-moe/completions",
    responses={200: {"content": {"text/event-stream": {}}}},
)
async def chat_moe_public_completions(body: ChatMoeCompletionRequest, request: Request = None):
    """Use server history only for `/chat-moe`; production requests remain stateless."""
    # FastAPI always injects Request in production. Direct unit/in-process
    # callers deliberately omit it and must exercise the local orchestration
    # branch instead of accidentally inheriting a machine-level worker flag.
    if request is not None and should_delegate_agent_execution():
        return await proxy_json_to_worker(
            request,
            path="/chat-moe/completions",
            payload=body.model_dump(mode="json", exclude_none=True),
            stream=True,
        )

    session_id = body.session_id.strip()
    query = body.message.strip()
    if not session_id:
        raise HTTPException(status_code=400, detail="session_id is required")
    if not query:
        raise HTTPException(status_code=400, detail="message is required")
    manager = get_session_manager()
    session = await manager.get(session_id, touch=False)
    if session is None:
        raise HTTPException(status_code=404, detail="session not found or expired")
    memory_user_id = str(session.metadata.get("memory_user_id") or "").strip() or None
    request_id = _moe_request_id()

    async def event_generator():
        async with manager.session_lock(session_id):
            conversation_history, expert_context_history = await manager.context(session_id)
            memory_injection, memory_load_detail = await load_user_memory(
                user_id=memory_user_id, query=query, pet_id=None,
            )
            prior_experts = sorted({
                str(opinion.get("expert"))
                for context in expert_context_history
                for opinion in (context.get("experts") or [])
                if isinstance(opinion, dict) and opinion.get("expert")
            })
            prior_tools = sorted({
                str(tool_name)
                for context in expert_context_history
                for opinion in (context.get("experts") or [])
                if isinstance(opinion, dict)
                for tool_name in (opinion.get("tools_used") or []) if tool_name
            })
            prior_searches = sum(
                1
                for context in expert_context_history
                for opinion in (context.get("experts") or [])
                if isinstance(opinion, dict)
                for result in (opinion.get("tool_results") or [])
                if isinstance(result, dict) and result.get("ok")
                and str(result.get("tool_name") or "") in {"rag.search", "mcp.web_search.web_search"}
            )
            prior_evidence_items = sum(
                len(opinion.get("evidence") or [])
                for context in expert_context_history
                for opinion in (context.get("experts") or [])
                if isinstance(opinion, dict)
            )
            yield openai_sse_chunk(
                request_id=request_id, model="agent-moe", status="session_context_loaded",
                detail={
                    "complete_turns": len(conversation_history) // 2,
                    "expert_context_turns": len(expert_context_history),
                    "prior_experts": prior_experts, "prior_tools": prior_tools,
                    "prior_searches": prior_searches, "prior_evidence_items": prior_evidence_items,
                },
            )
            yield openai_sse_chunk(
                request_id=request_id, model="agent-moe",
                status="memory_context_loaded" if memory_load_detail.get("loaded") else "memory_context_skipped",
                detail=memory_load_detail,
            )
            registry = get_registry()
            orchestrator = build_moe_orchestrator(
                registry=registry,
                temperature=body.temperature,
                max_tokens=body.max_tokens,
                user_role=body.user_role,
                allowed_tools=public_moe_allowed_tools(tool.name for tool in registry.list_tools()),
            )
            final_parts: List[str] = []
            completed = False
            try:
                emitted_finish = False
                async for event in orchestrator.stream(
                    query=query,
                    system_context=_moe_system_context(body.response_lang, memory_injection),
                    conversation_history=conversation_history,
                    expert_context_history=expert_context_history,
                    user_memory=memory_injection,
                ):
                    finish = event.get("finish")
                    emitted_finish = emitted_finish or bool(finish)
                    if event.get("status") == "streaming" and event.get("content"):
                        final_parts.append(str(event["content"]))
                    elif event.get("status") == "answer_reset":
                        final_parts.clear()
                    yield openai_sse_chunk(
                        request_id=request_id, model="agent-moe",
                        status=event.get("status"), detail=event.get("detail") or {},
                        content=event.get("content") or "", finish=finish,
                    )
                if not emitted_finish:
                    yield openai_sse_chunk(
                        request_id=request_id, model="agent-moe", status="done", detail={}, finish="stop",
                    )
                completed = True
            except Exception as exc:  # noqa: BLE001
                yield openai_sse_chunk(
                    request_id=request_id, model="agent-moe", status="error",
                    detail={"message": str(exc)}, content=f"\nMoE 调用失败：{exc}", finish="stop",
                )

            if completed and final_parts:
                experts = list(orchestrator.last_run_context.get("experts") or [])
                tool_results = [
                    result for opinion in experts
                    for result in (opinion.get("tool_results") or []) if isinstance(result, dict)
                ]
                committed = await manager.commit_turn(
                    session_id, user_message=query, assistant_message="".join(final_parts),
                    expert_context=orchestrator.last_run_context, tool_results=tool_results,
                )
                if committed is not None:
                    memory_write_detail = await write_user_memory(
                        user_id=memory_user_id, query=query, answer="".join(final_parts), pet_id=None,
                        session_id=session_id,
                        turn_id=f"chat-moe:{session_id}:{len(committed.messages) // 2}",
                    )
                    yield openai_sse_chunk(
                        request_id=request_id, model="agent-moe",
                        status="memory_stored" if memory_write_detail.get("stored") else "memory_store_skipped",
                        detail=memory_write_detail,
                    )
            yield SSE_DONE

    return StreamingResponse(event_generator(), media_type="text/event-stream", headers=SSE_RESPONSE_HEADERS)
