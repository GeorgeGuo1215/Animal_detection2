"""
OpenAI-compatible /v1/chat/completions endpoint with streaming support.

This module provides:
1. Standard OpenAI API format
2. SSE streaming with agent status updates
3. Plan-and-Solve agent integration
4. Multi-turn tool_calls support (Agent decides when to stop)
5. Fully async LLM + tool dispatch for enterprise concurrency
"""
from __future__ import annotations

import json
import time
import uuid
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

from .llm_client import AsyncOpenAIClient, extract_text
from .llm_client_stream import AsyncOpenAIStreamClient
from .plan_and_solve import AsyncPlanAndSolveAgent, _safe_json_loads
from .schemas_openai import (
    ChatCompletionChunk,
    ChatCompletionChunkChoice,
    ChatCompletionChunkDelta,
    ChatCompletionChoice,
    ChatCompletionRequest,
    ChatCompletionResponse,
    ChatMessage,
    UsageInfo,
)
from .tool_registry import get_registry
from .trace_store import new_trace_id, write_trace


router = APIRouter()

MAX_TOOL_ROUNDS = 5

DEFAULT_ALLOWED_TOOLS = [
    "rag.search",
    "mcp.web_search.web_search",
    "mcp.web_search.ingredient_check",
    "mcp.nutritional_planner.calculate_meal_plan",
    "mcp.nutritional_planner.generate_exercise_plan",
]

_TOOL_DESCRIPTIONS = {
    "rag.search": "Search the veterinary knowledge base for medical/health information.",
    "mcp.web_search.web_search": (
        "Perform real-time web search via Tavily for any up-to-date information "
        "(news, product details, price hints, etc.). Each result contains title, url, and content snippet. "
        "When composing the final answer, cite sources inline using [^N^] where N is the result index (1-based). "
        "Always provide the source URL to the user."
    ),
    "mcp.web_search.ingredient_check": "Use web search to find product ingredients and check for conflicts with the pet's health conditions.",
    "mcp.nutritional_planner.calculate_meal_plan": "Calculate daily calorie needs (RER/MER) and next meal portion in grams.",
    "mcp.nutritional_planner.generate_exercise_plan": "Generate exercise recommendations based on calorie deficit and medical constraints.",
}


_WEB_SEARCH_CITATION_RULES = (
    "\n\n## 网络搜索结果引用规范\n"
    "当回答中使用了 web_search 工具返回的信息时，必须遵循以下引用规则：\n"
    "1. 优先使用最新、最权威的信息源（官方网站、学术机构、主流媒体），而非聚合站点。\n"
    "2. 使用行内引用标记 [^N^] 标注来源（N 为搜索结果的序号，从 1 开始）。\n"
    "3. 只引用直接支撑你回答的来源；如果去掉该来源不影响回答，则不引用。\n"
    "4. 引用具体事实（数字、日期、统计、官方声明），而非常识。\n"
    "5. 每个事实只标注一个 [^N^]，放在句末；同一段落最多一个引用标记。\n"
    "6. 在回答末尾附上「参考来源」列表，格式为：[^N^] 标题 — URL\n"
    "7. 如果搜索结果之间相互矛盾或信息不足，明确告知用户并给出下一步建议。\n"
)

def _gen_id() -> str:
    return f"chatcmpl-{uuid.uuid4().hex[:24]}"


def _now_ts() -> int:
    return int(time.time())


def _extract_user_query(messages: List[ChatMessage]) -> str:
    for msg in reversed(messages):
        if msg.role == "user" and msg.content:
            return msg.content
    return ""


def _build_system_context(messages: List[ChatMessage]) -> str:
    parts = []
    for msg in messages:
        if msg.role == "system" and msg.content:
            parts.append(msg.content)
    return "\n".join(parts) if parts else ""


def _build_conversation_history(messages: List[ChatMessage]) -> List[Dict[str, str]]:
    history = []
    for msg in messages:
        if msg.role in ("user", "assistant") and msg.content:
            history.append({"role": msg.role, "content": msg.content})
    return history


def _make_decide_prompt(
    query: str,
    tool_results: List[Dict[str, Any]],
    available_tools: List[str],
) -> str:
    tools_with_desc = [
        {"name": t, "description": _TOOL_DESCRIPTIONS.get(t, "")} for t in available_tools
    ]
    return json.dumps({
        "task": "决定下一步行动",
        "instructions": (
            "你是一个智能 Agent。根据用户问题和已有的工具调用结果，决定是否需要继续调用工具获取更多信息，还是已经可以生成最终回答。\n"
            "如果信息不足，选择调用工具；如果信息足够，选择生成最终回答。\n"
            "工具选择指南：\n"
            "- 健康/医学知识问题 → rag.search\n"
            "- 实时网络信息 / 产品信息 / 价格线索 → mcp.web_search.web_search\n"
            "- 产品成分安全性检查 → mcp.web_search.ingredient_check\n"
            "- 喂食量/热量计算 → mcp.nutritional_planner.calculate_meal_plan\n"
            "- 运动计划建议 → mcp.nutritional_planner.generate_exercise_plan\n"
            "你必须输出严格 JSON，不要输出任何额外文字。"
        ),
        "user_query": query,
        "tool_results_so_far": [
            {
                "tool_name": r.get("tool_name"),
                "hits_count": len(r.get("result", {}).get("hits", [])) if isinstance(r.get("result"), dict) else 0,
                "brief": _summarize_tool_result(r.get("result")),
            }
            for r in tool_results
        ],
        "available_tools": tools_with_desc,
        "output_format": {
            "action": "call_tool 或 final_answer",
            "tool_name": "工具名称",
            "arguments": "根据所选工具填写对应参数 (JSON object)",
            "reason": "简短说明为什么做这个决定",
        },
    }, ensure_ascii=False)


def _summarize_tool_result(result: Any, max_chars: int = 500) -> str:
    if not isinstance(result, dict):
        return str(result)[:max_chars]

    # RAG results
    hits = result.get("hits", [])
    if hits:
        summaries = []
        for h in hits[:3]:
            text = h.get("text", "")[:200]
            summaries.append(text)
        return " | ".join(summaries)[:max_chars]

    # MCP tool results -- look for common status/content patterns
    status = result.get("status", "")
    if status:
        msg = result.get("message", "")
        return f"status={status} {msg}"[:max_chars]

    content = result.get("content", [])
    if isinstance(content, list) and content:
        texts = []
        for c in content[:3]:
            if isinstance(c, dict):
                texts.append(c.get("text", str(c))[:200])
            else:
                texts.append(str(c)[:200])
        return " | ".join(texts)[:max_chars]

    return json.dumps(result, ensure_ascii=False)[:max_chars]


def _check_special_flags(result: Dict[str, Any]) -> Optional[str]:
    """Check tool results for special flags that require user interaction."""
    if not isinstance(result, dict):
        return None

    # MCP content wrapper -- unwrap text content
    content = result.get("content", [])
    if isinstance(content, list):
        for item in content:
            if isinstance(item, dict) and item.get("text"):
                try:
                    inner = json.loads(item["text"])
                    if isinstance(inner, dict):
                        result = inner
                except (json.JSONDecodeError, TypeError):
                    pass

    status = result.get("status", "")

    if status == "INSUFFICIENT_DATA":
        msg = result.get("message", "Unable to retrieve ingredient data.")
        return f"\n**[Requires User Input]** {msg}\n"

    flags = result.get("flags", [])
    if "FEEDING_INQUIRY_NEEDED" in flags:
        msg = result.get("inquiry_message", "No feeding recorded today. Has the pet eaten?")
        return f"\n**[Feeding Inquiry]** {msg}\n"

    if "OVERFED_WARNING" in flags:
        msg = result.get("overfed_message", "Calorie intake exceeds daily needs.")
        return f"\n**[Warning]** {msg}\n"

    return None


async def _stream_multi_turn_agent(
    request_id: str,
    model: str,
    query: str,
    system_context: str,
    conversation_history: List[Dict[str, str]],
    temperature: float,
    max_tokens: int,
    allowed_tools: Optional[List[str]],
    debug_timing: bool = False,
) -> AsyncGenerator[str, None]:
    created = _now_ts()
    reg = get_registry()
    llm = AsyncOpenAIClient()
    stream_llm = AsyncOpenAIStreamClient()
    agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)

    _t_start = time.perf_counter()
    _timing: List[Dict[str, Any]] = []

    def _lap(label: str, t0: float, **extra: Any) -> None:
        if debug_timing:
            _timing.append({"step": label, "ms": round((time.perf_counter() - t0) * 1000, 1), **extra})

    def make_chunk(content: str = "", status: str = None, detail: Dict = None, finish: str = None) -> str:
        chunk = ChatCompletionChunk(
            id=request_id,
            created=created,
            model=model,
            choices=[ChatCompletionChunkChoice(
                delta=ChatCompletionChunkDelta(content=content if content else None),
                finish_reason=finish,
            )],
            agent_status=status,
            agent_detail=detail,
        )
        return f"data: {chunk.model_dump_json()}\n\n"

    tool_results: List[Dict[str, Any]] = []
    available_tools = allowed_tools or ["rag.search"]

    for round_num in range(MAX_TOOL_ROUNDS):
        yield make_chunk(
            status="thinking",
            detail={"message": f"思考中... (第{round_num + 1}轮)", "round": round_num + 1}
        )

        decide_prompt = _make_decide_prompt(query, tool_results, available_tools)

        t0 = time.perf_counter()
        try:
            decide_resp = await llm.chat(
                messages=[
                    {"role": "system", "content": "你是一个智能决策 Agent。输出严格 JSON。"},
                    {"role": "user", "content": decide_prompt},
                ],
                temperature=0.1,
                max_tokens=256,
                response_format={"type": "json_object"},
            )
            _lap(f"round_{round_num+1}_decide_llm", t0)
            decide_text = extract_text(decide_resp)
            decision, err = _safe_json_loads(decide_text)

            if not decision:
                yield make_chunk(content=f"Decision parse failed: {err}\n")
                break

        except Exception as e:
            _lap(f"round_{round_num+1}_decide_llm_error", t0)
            yield make_chunk(content=f"Decision failed: {e}\n")
            break

        action = decision.get("action", "final_answer")
        reason = decision.get("reason", "")

        if action == "call_tool":
            tool_name = decision.get("tool_name", "rag.search")
            args = decision.get("arguments", {})

            if tool_name not in available_tools:
                yield make_chunk(content=f"Tool {tool_name} unavailable, skipping\n")
                continue

            if tool_name == "rag.search":
                args = agent._force_rag_search_defaults(args)

            yield make_chunk(
                content=f"\n**Round {round_num + 1} tool call**: {tool_name}\n",
                status="tool_calling",
                detail={"tool_name": tool_name, "round": round_num + 1, "reason": reason}
            )

            if args.get("query"):
                yield make_chunk(content=f"   Query: {args['query']}\n")

            t0 = time.perf_counter()
            try:
                result = await reg.call(tool_name, args)
                _lap(f"round_{round_num+1}_tool_{tool_name}", t0)
                tool_results.append({
                    "round": round_num + 1,
                    "tool_name": tool_name,
                    "arguments": args,
                    "result": result,
                })

                hits_count = len(result.get("hits", [])) if isinstance(result, dict) else 0
                yield make_chunk(
                    content=f"   Found {hits_count} relevant results\n",
                    status="tool_complete",
                    detail={"tool_name": tool_name, "hits_count": hits_count, "round": round_num + 1}
                )

                special_msg = _check_special_flags(result)
                if special_msg:
                    yield make_chunk(content=special_msg, status="user_action_needed")
            except Exception as e:
                _lap(f"round_{round_num+1}_tool_{tool_name}_error", t0)
                yield make_chunk(content=f"   Tool call failed: {e}\n")
                tool_results.append({
                    "round": round_num + 1,
                    "tool_name": tool_name,
                    "arguments": args,
                    "error": str(e),
                })

        elif action == "final_answer":
            yield make_chunk(
                content=f"\n**Generating final answer** (reason: {reason})\n",
                status="decided_final",
                detail={"reason": reason, "total_rounds": round_num + 1}
            )
            break

        else:
            yield make_chunk(content=f"Unknown action: {action}, generating answer\n")
            break

    yield make_chunk(
        content="\n**Generating response...**\n\n",
        status="generating"
    )

    sys_prompt = (
        "你是一个面向兽医/动物健康方向的 AI 助手。"
        "你拥有以下能力：知识库检索、网络搜索与成分分析、营养与运动计划制定。"
        "请用中文回答，必要时引用你从工具返回的证据片段（简短引用即可）。"
        "如果工具返回 INSUFFICIENT_DATA，请明确告知用户需要提供更多信息（如提供产品成分表）。"
        "如果工具返回 FEEDING_INQUIRY_NEEDED，请主动询问用户宠物今天是否已经进食。"
        "如果证据不足，请明确说不确定，并给出下一步建议。"
    )
    has_web_search = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    if has_web_search:
        sys_prompt += _WEB_SEARCH_CITATION_RULES
    if system_context:
        sys_prompt = f"{system_context}\n\n{sys_prompt}"

    user_content_parts = []
    if conversation_history:
        user_content_parts.append("历史对话:\n" + "\n".join(
            f"{m['role']}: {m['content']}" for m in conversation_history[-6:]
        ))

    user_content_parts.append(json.dumps({
        "query": query,
        "tool_results": tool_results,
    }, ensure_ascii=False))

    user_content = "\n\n".join(user_content_parts)

    t0 = time.perf_counter()
    try:
        async for chunk_text in stream_llm.chat_stream(
            messages=[
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": user_content},
            ],
            temperature=temperature,
            max_tokens=max_tokens,
        ):
            yield make_chunk(content=chunk_text, status="streaming")
    except Exception as e:
        yield make_chunk(content=f"\nGeneration failed: {e}")
    _lap("final_generation", t0)

    if debug_timing:
        total_ms = round((time.perf_counter() - _t_start) * 1000, 1)
        _timing.append({"step": "total", "ms": total_ms})
        yield make_chunk(
            status="timing_summary",
            detail={"timing": _timing},
        )

    yield make_chunk(finish="stop")
    yield "data: [DONE]\n\n"


async def _stream_plan_and_solve(
    request_id: str,
    model: str,
    query: str,
    system_context: str,
    temperature: float,
    max_tokens: int,
    allowed_tools: Optional[List[str]],
    debug_timing: bool = False,
) -> AsyncGenerator[str, None]:
    created = _now_ts()
    reg = get_registry()
    llm = AsyncOpenAIClient()
    stream_llm = AsyncOpenAIStreamClient()
    agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)

    _t_start = time.perf_counter()
    _timing: List[Dict[str, Any]] = []

    def _lap(label: str, t0: float, **extra: Any) -> None:
        if debug_timing:
            _timing.append({"step": label, "ms": round((time.perf_counter() - t0) * 1000, 1), **extra})

    def make_chunk(content: str = "", status: str = None, detail: Dict = None, finish: str = None) -> str:
        chunk = ChatCompletionChunk(
            id=request_id,
            created=created,
            model=model,
            choices=[ChatCompletionChunkChoice(
                delta=ChatCompletionChunkDelta(content=content if content else None),
                finish_reason=finish,
            )],
            agent_status=status,
            agent_detail=detail,
        )
        return f"data: {chunk.model_dump_json()}\n\n"

    yield make_chunk(status="planning", detail={"message": "正在制定计划..."})

    t0 = time.perf_counter()
    try:
        plan = await agent.plan(query=query, allowed_tools=allowed_tools)
        _lap("plan_llm", t0)
        yield make_chunk(
            content="**Plan complete**\n",
            status="plan_complete",
            detail={"plan": plan}
        )
    except Exception as e:
        _lap("plan_llm_error", t0)
        yield make_chunk(content=f"Planning failed: {e}", finish="stop")
        yield "data: [DONE]\n\n"
        return

    tool_results: List[Dict[str, Any]] = []
    allowed = set(allowed_tools) if allowed_tools else None

    for i, step in enumerate(plan):
        stype = step.get("type")
        if stype == "tool":
            tool_name = str(step.get("tool_name") or "")
            if not tool_name:
                continue
            if allowed is not None and tool_name not in allowed:
                continue

            args = step.get("arguments") or {}
            if not isinstance(args, dict):
                args = {}

            if tool_name == "rag.search":
                args = agent._force_rag_search_defaults(args)

            yield make_chunk(
                content=f"\n**Executing tool**: {tool_name}\n",
                status="tool_calling",
                detail={"tool_name": tool_name, "step": i}
            )

            t0 = time.perf_counter()
            try:
                result = await reg.call(tool_name, args)
                _lap(f"step_{i}_tool_{tool_name}", t0)
                tool_results.append({
                    "step": i,
                    "tool_name": tool_name,
                    "arguments": args,
                    "result": result
                })

                hits_count = len(result.get("hits", [])) if isinstance(result, dict) else 0
                yield make_chunk(
                    content=f"Found {hits_count} relevant results\n",
                    status="tool_complete",
                    detail={"tool_name": tool_name, "hits_count": hits_count}
                )

                special_msg = _check_special_flags(result)
                if special_msg:
                    yield make_chunk(content=special_msg, status="user_action_needed")
            except Exception as e:
                _lap(f"step_{i}_tool_{tool_name}_error", t0)
                yield make_chunk(content=f"Tool call failed: {e}\n")

        elif stype == "final":
            break

    yield make_chunk(
        content="\n**Generating response...**\n\n",
        status="generating"
    )

    sys_prompt = (
        "你是一个面向兽医/动物健康方向的 AI 助手。"
        "你拥有以下能力：知识库检索、网络搜索与成分分析、营养与运动计划制定。"
        "请用中文回答，必要时引用你从工具返回的证据片段（简短引用即可）。"
        "如果工具返回 INSUFFICIENT_DATA，请明确告知用户需要提供更多信息（如拍摄产品成分表）。"
        "如果工具返回 FEEDING_INQUIRY_NEEDED，请主动询问用户宠物今天是否已经进食。"
        "如果证据不足，请明确说不确定，并给出下一步建议。"
    )
    has_web_search = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    if has_web_search:
        sys_prompt += _WEB_SEARCH_CITATION_RULES
    if system_context:
        sys_prompt = f"{system_context}\n\n{sys_prompt}"

    user_content = json.dumps({
        "query": query,
        "plan": plan,
        "tool_results": tool_results,
    }, ensure_ascii=False)

    t0 = time.perf_counter()
    try:
        async for chunk_text in stream_llm.chat_stream(
            messages=[
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": user_content},
            ],
            temperature=temperature,
            max_tokens=max_tokens,
        ):
            yield make_chunk(content=chunk_text, status="streaming")
    except Exception as e:
        yield make_chunk(content=f"\nGeneration failed: {e}")
    _lap("final_generation", t0)

    if debug_timing:
        total_ms = round((time.perf_counter() - _t_start) * 1000, 1)
        _timing.append({"step": "total", "ms": total_ms})
        yield make_chunk(
            status="timing_summary",
            detail={"timing": _timing},
        )

    yield make_chunk(finish="stop")
    yield "data: [DONE]\n\n"


@router.post("/v1/chat/completions")
async def chat_completions(req: ChatCompletionRequest, request: Request):
    """
    OpenAI-compatible chat completions endpoint.

    Supports both streaming (stream=true) and non-streaming modes.

    Models:
    - "agent-plan-solve": Single-turn plan-and-solve (backward compatible)
    - "agent-multi-turn": Multi-turn agent with iterative tool calls
    """
    trace_id = new_trace_id()
    request_id = _gen_id()
    created = _now_ts()
    # 构建对话历史，查询，系统提示词
    query = _extract_user_query(req.messages)
    system_context = _build_system_context(req.messages)
    conversation_history = _build_conversation_history(req.messages)

    # 构建允许的工具列表
    allowed_tools = None
    if req.tools:
        allowed_tools = [t.get("function", {}).get("name") for t in req.tools if t.get("function")]
        allowed_tools = [n for n in allowed_tools if n]
    if not allowed_tools:
        reg = get_registry()
        available_names = {t.name for t in reg.list_tools()}
        allowed_tools = [t for t in DEFAULT_ALLOWED_TOOLS if t in available_names]
        if not allowed_tools:
            allowed_tools = ["rag.search"]

    use_multi_turn = req.model in ("agent-multi-turn", "agent-multiturn", "multi-turn")
    _debug_timing = bool(req.debug_timing)

    # 流式处理
    if req.stream:
        async def event_generator():
            if use_multi_turn:
                async for chunk in _stream_multi_turn_agent(
                    request_id=request_id,
                    model=req.model,
                    query=query,
                    system_context=system_context,
                    conversation_history=conversation_history,
                    temperature=req.temperature or 0.2,
                    max_tokens=req.max_tokens or 768,
                    allowed_tools=allowed_tools,
                    debug_timing=_debug_timing,
                ):
                    yield chunk
            else:
                async for chunk in _stream_plan_and_solve(
                    request_id=request_id,
                    model=req.model,
                    query=query,
                    system_context=system_context,
                    temperature=req.temperature or 0.2,
                    max_tokens=req.max_tokens or 768,
                    allowed_tools=allowed_tools,
                    debug_timing=_debug_timing,
                ):
                    yield chunk

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            }
        )

    else:
        try:
            _t_start = time.perf_counter()
            _timing: List[Dict[str, Any]] = []

            reg = get_registry()
            llm = AsyncOpenAIClient()
            agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)

            t0 = time.perf_counter()
            plan = await agent.plan(query=query, allowed_tools=allowed_tools)
            if _debug_timing:
                _timing.append({"step": "plan_llm", "ms": round((time.perf_counter() - t0) * 1000, 1)})

            t0 = time.perf_counter()
            answer, tool_results = await agent.solve(
                query=query,
                plan_steps=plan,
                allowed_tools=allowed_tools,
                temperature=req.temperature or 0.2,
                max_tokens=req.max_tokens or 768,
            )
            if _debug_timing:
                _timing.append({"step": "solve_all", "ms": round((time.perf_counter() - t0) * 1000, 1)})
                _timing.append({"step": "total", "ms": round((time.perf_counter() - _t_start) * 1000, 1)})

            response = ChatCompletionResponse(
                id=request_id,
                created=created,
                model=req.model,
                choices=[ChatCompletionChoice(
                    message=ChatMessage(role="assistant", content=answer),
                    finish_reason="stop",
                )],
                usage=UsageInfo(),
                plan=plan,
                tool_results=tool_results,
                timing=_timing if _debug_timing else None,
            )

            write_trace(trace_id, tool="v1.chat.completions", request=req.model_dump(), response=response.model_dump())
            return response

        except Exception as e:
            error_response = {
                "error": {
                    "message": str(e),
                    "type": "server_error",
                    "code": "agent_error",
                }
            }
            write_trace(trace_id, tool="v1.chat.completions", request=req.model_dump(), response=error_response, error=str(e))
            from fastapi import HTTPException
            raise HTTPException(status_code=500, detail=error_response)


@router.get("/v1/models")
async def list_models():
    """List available models (OpenAI-compatible)."""
    return {
        "object": "list",
        "data": [
            {
                "id": "agent-plan-solve",
                "object": "model",
                "created": 1700000000,
                "owned_by": "pethealthai",
                "description": "Single-turn plan-and-solve agent",
            },
            {
                "id": "agent-multi-turn",
                "object": "model",
                "created": 1700000000,
                "owned_by": "pethealthai",
                "description": "Multi-turn agent with iterative tool calls",
            },
        ]
    }
