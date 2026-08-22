"""仅由 PetMind MoE 运行时支撑的 OpenAI 兼容 API。"""
from __future__ import annotations

import json
import time
import uuid
from functools import partial
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

from ..concurrency import ResourceBusyError
from ..tools.request_scope import bind_tool_request_scope, filter_tools_without_animal
from ..memory import load_user_memory, write_user_memory
from ..features.qa_audit.repository import save_qa_record
from ..integrations.llm.config import load_generation_settings
from ..observability.jsonl_trace import new_trace_id, write_trace
from ..schemas.openai_schemas import (
    ChatCompletionChoice, ChatCompletionRequest, ChatCompletionResponse, ChatMessage, UsageInfo,
)
from ..services.agent_execution import AGENT_MODEL_ID, build_moe_orchestrator
from ..services.moe import MoETrace
from ..tools.tool_registry import get_registry
from ..worker_proxy import proxy_json_to_worker, should_delegate_agent_execution
from .sse import SSE_DONE, SSE_RESPONSE_HEADERS, openai_sse_chunk

router = APIRouter()

DEFAULT_ALLOWED_TOOLS = [
    "rag.search", "sql.search", "vitals.summary", "mcp.vitals_alert.check_vitals",
    "mcp.web_search.web_search", "mcp.web_search.ingredient_check",
    "mcp.nutritional_planner.calculate_meal_plan",
    "mcp.nutritional_planner.generate_exercise_plan",
]


def _clean_identity(value: Optional[str], max_length: int = 200) -> Optional[str]:
    """截断并规范化身份字符串。"""
    text = str(value or "").strip()
    return text[:max_length] if text else None


def _memory_user_id(req: ChatCompletionRequest, request: Request) -> Optional[str]:
    """从网关已解析的平台用户读取记忆 user_id，不信任请求体中的 user。"""
    # 记忆归属是安全边界。切勿信任调用方控制的 OpenAI ``user`` 扩展或 X-User-Id；
    # 网关会把由数据库 API key/JWT 解析出的用户写到 request.state。
    del req
    return _clean_identity(getattr(request.state, "platform_user_id", None))


def _resolve_request_allowed_tools(req: ChatCompletionRequest, available_names: set[str]) -> List[str]:
    """按 tools/tool_choice 与 animal_id 解析本请求允许的工具。"""
    if req.tools is None:
        allowed = [name for name in DEFAULT_ALLOWED_TOOLS if name in available_names]
    else:
        allowed = [
            str(tool.get("function", {}).get("name") or "").strip()
            for tool in req.tools
            if isinstance(tool, dict) and isinstance(tool.get("function"), dict)
        ]
        allowed = [name for name in allowed if name and name in available_names]
    if req.tool_choice == "none":
        allowed = []
    elif isinstance(req.tool_choice, dict):
        function = req.tool_choice.get("function")
        selected = str(function.get("name") or "").strip() if isinstance(function, dict) else ""
        allowed = [selected] if selected and selected in allowed else []
    return filter_tools_without_animal(allowed)


def _gen_id() -> str:
    """生成 chatcmpl 请求 id。"""
    return f"chatcmpl-{uuid.uuid4().hex[:24]}"


def _now_ts() -> int:
    """当前 Unix 时间戳。"""
    return int(time.time())


def _extract_user_query(messages: List[ChatMessage]) -> str:
    """从消息列表提取最近一条用户问题。"""
    for message in reversed(messages):
        if message.role == "user" and message.content:
            return message.content
    return ""


def _build_system_context(messages: List[ChatMessage]) -> str:
    """拼接 system 消息作为系统上下文。"""
    return "\n".join(
        message.content for message in messages
        if message.role == "system" and message.content
    )


def _build_conversation_history(messages: List[ChatMessage]) -> List[Dict[str, str]]:
    """提取 user/assistant 对话历史。"""
    return [
        {"role": message.role, "content": message.content}
        for message in messages
        if message.role in ("user", "assistant") and message.content
    ]


def _build_pethealth_server_context(req: ChatCompletionRequest, request: Request) -> Optional[Dict[str, Any]]:
    """在心率异常时构造 PetHealth_Server 上下文字典。"""
    context = req.pethealth_server
    if context is None or not context.heart_rate_abnormal:
        return None
    animal_id = str(
        context.animal_id or req.animal_id or request.headers.get("x-animal-id") or ""
    ).strip()
    if not animal_id:
        return None
    return {
        "animal_id": animal_id,
        "heart_rate_abnormal": True,
        "vitals_window_hours": int(context.vitals_window_hours or 24),
    }


async def _stream_moe_agent(
    request_id: str,
    model: str,
    query: str,
    system_context: str,
    conversation_history: List[Dict[str, str]],
    temperature: float,
    max_tokens: Optional[int],
    allowed_tools: Optional[List[str]],
    user_role: str = "pet_owner",
    debug_timing: bool = False,
    pethealth_server: Optional[Dict[str, Any]] = None,
    user_memory: str = "",
) -> AsyncGenerator[str, None]:
    """流式运行 MoE Agent 并产出 SSE 块。"""
    make_chunk = partial(openai_sse_chunk, request_id=request_id, created=_now_ts(), model=model)
    orchestrator = build_moe_orchestrator(
        registry=get_registry(), temperature=temperature, max_tokens=max_tokens,
        user_role=user_role, allowed_tools=allowed_tools, pethealth_server=pethealth_server,
    )
    recorder = MoETrace(question=query, user_role=user_role) if debug_timing else None
    emitted_finish = False
    try:
        async for event in orchestrator.stream(
            query=query, system_context=system_context,
            conversation_history=conversation_history, user_memory=user_memory, recorder=recorder,
        ):
            finish = event.get("finish")
            emitted_finish = emitted_finish or bool(finish)
            yield make_chunk(
                content=event.get("content") or "", status=event.get("status"),
                detail=event.get("detail"), finish=finish,
            )
        if recorder is not None:
            yield make_chunk(status="llm_trace", detail={
                "total_ms": recorder.total_ms,
                "calls": [{
                    "seq": call.seq, "stage": call.stage, "model": call.model,
                    "output": call.output, "latency_ms": call.latency_ms,
                    "prompt_tokens": call.prompt_tokens,
                    "completion_tokens": call.completion_tokens,
                    "total_tokens": call.total_tokens, "meta": call.meta,
                    "prompt_cache_hit_tokens": call.prompt_cache_hit_tokens,
                    "prompt_cache_miss_tokens": call.prompt_cache_miss_tokens,
                } for call in recorder.llm_calls],
            })
    except ResourceBusyError as exc:
        yield make_chunk(
            content="服务当前繁忙，请稍后重试。", status="busy",
            detail=exc.as_dict(), finish="stop",
        )
        yield SSE_DONE
        return
    except Exception as exc:  # noqa: BLE001
        yield make_chunk(content=f"\nMoE generation failed: {exc}", status="streaming")
    if not emitted_finish:
        yield make_chunk(finish="stop")
    yield SSE_DONE


def _collect_stream_audit(
    obj: Dict[str, Any], *, content: List[str], tools: List[str], counters: Dict[str, Any],
) -> None:
    """从公开流事件累积最终文本、去重工具名及 RAG/Web 使用指标。

    收到 ``answer_reset`` 时清空此前部分文本，确保 QA 审计只保存替代后的完整终答；
    专家与工具事件都可补充检索计数，但不会记录原始隐藏提示词。
    """
    delta = (obj.get("choices") or [{}])[0].get("delta") or {}
    status = obj.get("agent_status")
    detail = obj.get("agent_detail") or {}
    if status == "streaming" and delta.get("content"):
        content.append(delta["content"])
    elif status == "answer_reset":
        content.clear()
    elif status == "tool_complete":
        tool_name = str(detail.get("tool_name") or "")
        if tool_name and tool_name not in tools:
            tools.append(tool_name)
        if "rag" in tool_name:
            counters["rag_hits"] += int(detail.get("hits_count") or 0)
            counters["rag_best"] = max(counters["rag_best"], float(detail.get("best_score") or 0.0))
        if "web_search" in tool_name:
            counters["web"] = True
    elif status == "expert_complete":
        for tool_name in detail.get("tools_used") or []:
            if tool_name and tool_name not in tools:
                tools.append(tool_name)
            if "web_search" in tool_name:
                counters["web"] = True
        counters["rag_hits"] += int(detail.get("hits_count") or 0)
        counters["rag_best"] = max(counters["rag_best"], float(detail.get("best_score") or 0.0))


@router.post("/v1/chat/completions")
async def chat_completions(req: ChatCompletionRequest, request: Request):
    """OpenAI 兼容 chat completions；this endpoint does not create a conversation session。"""
    if should_delegate_agent_execution():
        return await proxy_json_to_worker(
            request, path="/v1/chat/completions",
            payload=req.model_dump(mode="json", exclude_none=True), stream=bool(req.stream),
        )

    trace_id = new_trace_id()
    request_id = _gen_id()
    query = _extract_user_query(req.messages)
    system_context = _build_system_context(req.messages)
    conversation_history = _build_conversation_history(req.messages)
    pethealth_context = _build_pethealth_server_context(req, request)
    memory_user_id = _memory_user_id(req, request)
    memory_pet_id = _clean_identity(req.animal_id or request.headers.get("x-animal-id"))
    memory_injection, memory_load_detail = await load_user_memory(
        user_id=memory_user_id, query=query, pet_id=memory_pet_id,
    )
    if memory_injection:
        system_context = "\n\n".join(filter(None, (system_context, memory_injection)))
    memory_session_id = _clean_identity(req.memory_session_id)
    memory_turn_id = _clean_identity(req.memory_turn_id) or request_id
    scope_kwargs = {
        "body_animal_id": req.animal_id,
        "header_animal_id": request.headers.get("x-animal-id"),
    }
    with bind_tool_request_scope(**scope_kwargs):
        available_names = {tool.name for tool in get_registry().list_tools()}
        allowed_tools = _resolve_request_allowed_tools(req, available_names)
    user_role = req.user_role or "pet_owner"
    source_ip = request.client.host if request.client else ""

    if req.stream:
        async def event_generator():
            """产出 chat completions 的 SSE 事件流。"""
            started = time.monotonic()
            content: List[str] = []
            tools: List[str] = []
            counters: Dict[str, Any] = {"rag_hits": 0, "rag_best": 0.0, "web": False}
            yield openai_sse_chunk(
                request_id=request_id, model=AGENT_MODEL_ID,
                status="memory_loaded" if memory_load_detail.get("loaded") else "memory_skipped",
                detail=memory_load_detail,
            )
            with bind_tool_request_scope(**scope_kwargs):
                source = _stream_moe_agent(
                    request_id=request_id, model=AGENT_MODEL_ID, query=query,
                    system_context=system_context, conversation_history=conversation_history,
                    temperature=(
                        req.temperature
                        if req.temperature is not None
                        else load_generation_settings().request_temperature
                    ),
                    max_tokens=req.max_tokens,
                    allowed_tools=allowed_tools, user_role=user_role,
                    debug_timing=bool(req.debug_timing), pethealth_server=pethealth_context,
                    user_memory=memory_injection,
                )
                async for chunk in source:
                    if chunk == SSE_DONE:
                        continue
                    yield chunk
                    if chunk.startswith("data: "):
                        try:
                            _collect_stream_audit(
                                json.loads(chunk[6:]), content=content, tools=tools, counters=counters,
                            )
                        except Exception:
                            pass

            answer = "".join(content)
            memory_write_detail = await write_user_memory(
                user_id=memory_user_id, query=query, answer=answer, pet_id=memory_pet_id,
                session_id=memory_session_id, turn_id=memory_turn_id,
            )
            yield openai_sse_chunk(
                request_id=request_id, model=AGENT_MODEL_ID,
                status="memory_stored" if memory_write_detail.get("stored") else "memory_store_skipped",
                detail=memory_write_detail,
            )
            yield SSE_DONE
            write_trace(
                trace_id, tool="v1.chat.completions.stream",
                request={"model": AGENT_MODEL_ID, "query": query, "allowed_tools": allowed_tools},
                response={"id": request_id, "answer_length": len(answer), "tools_called": tools},
            )
            try:
                await save_qa_record(
                    question=query, answer=answer, model=AGENT_MODEL_ID, tools_used=tools,
                    rag_hit_count=counters["rag_hits"], rag_best_score=counters["rag_best"],
                    used_web_search=counters["web"],
                    response_time_ms=int((time.monotonic() - started) * 1000),
                    source_ip=source_ip, user_role=user_role, request_id=request_id,
                )
            except Exception:
                pass

        return StreamingResponse(event_generator(), media_type="text/event-stream", headers=SSE_RESPONSE_HEADERS)

    started = time.monotonic()
    moe_trace = MoETrace(question=query, user_role=user_role)
    with bind_tool_request_scope(**scope_kwargs):
        orchestrator = build_moe_orchestrator(
            registry=get_registry(),
            temperature=(
                req.temperature
                if req.temperature is not None
                else load_generation_settings().request_temperature
            ),
            max_tokens=req.max_tokens, user_role=user_role, allowed_tools=allowed_tools,
            pethealth_server=pethealth_context,
        )
        try:
            answer, moe_trace = await orchestrator.run(
                query=query, system_context=system_context,
                conversation_history=conversation_history, user_memory=memory_injection,
                recorder=moe_trace,
            )
        except ResourceBusyError as exc:
            raise HTTPException(
                status_code=503,
                detail=exc.as_dict(),
                headers={"Retry-After": str(max(1, int(exc.timeout_s)))},
            ) from exc
    memory_write_detail = await write_user_memory(
        user_id=memory_user_id, query=query, answer=answer or "", pet_id=memory_pet_id,
        session_id=memory_session_id, turn_id=memory_turn_id,
    )
    response = ChatCompletionResponse(
        id=request_id, created=_now_ts(), model=AGENT_MODEL_ID,
        choices=[ChatCompletionChoice(
            message=ChatMessage(role="assistant", content=answer),
            finish_reason=orchestrator.last_finish_reason,
        )],
        usage=UsageInfo(), memory={"load": memory_load_detail, "write": memory_write_detail},
    )
    write_trace(
        trace_id, tool="v1.chat.completions.moe",
        request=req.model_dump(), response=response.model_dump(),
    )
    tool_names = [call.tool_name for call in moe_trace.tool_calls]
    try:
        await save_qa_record(
            question=query, answer=answer or "", model=AGENT_MODEL_ID, tools_used=tool_names,
            rag_hit_count=sum(call.hits_count for call in moe_trace.rag_calls),
            rag_best_score=max((call.best_score for call in moe_trace.rag_calls), default=0.0),
            used_web_search=any("web_search" in name for name in tool_names),
            response_time_ms=int((time.monotonic() - started) * 1000),
            source_ip=source_ip, user_role=user_role, request_id=request_id,
        )
    except Exception:
        pass
    return response


@router.get("/v1/models")
async def list_models():
    """列出可用模型。"""
    return {
        "object": "list",
        "data": [{"id": AGENT_MODEL_ID, "object": "model", "created": 0, "owned_by": "petmind"}],
    }
