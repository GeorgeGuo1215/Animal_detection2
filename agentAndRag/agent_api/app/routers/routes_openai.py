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

import asyncio
import json
import math
import time
import uuid
from functools import partial
from typing import Any, AsyncGenerator, Dict, List, Optional

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

from ..context.request_context import filter_tools_without_animal, set_request_animal_id
from ..concurrency import ResourceBusyError
from ..llm.llm_client import extract_text, get_shared_async_client
from ..llm.llm_client_stream import get_shared_async_stream_client
from ..memory import load_user_memory, write_user_memory
from ..persistence.qa_store import save_qa_record
from ..persistence.trace_store import new_trace_id, write_trace
from ..prompts.multi_turn import DECISION_INSTRUCTIONS, DECISION_SYSTEM_PROMPT
from ..schemas.openai_schemas import (
    ChatCompletionChoice,
    ChatCompletionRequest,
    ChatCompletionResponse,
    ChatMessage,
    UsageInfo,
)
from ..services.agent_execution import AgentMode, build_moe_orchestrator, resolve_agent_mode
from ..services.moe import MoETrace
from ..services.plan_and_solve import (
    AsyncPlanAndSolveAgent,
    _safe_json_loads,
    build_solve_prompt,
    ensure_rag_and_web_tool_steps,
)
from ..services.tool_call_utils import canonical_tool_call
from ..tools.tool_registry import get_registry
from .sse import SSE_DONE, SSE_RESPONSE_HEADERS, openai_sse_chunk


import re as _re

router = APIRouter()

MAX_TOOL_ROUNDS = 5

_TIMELINESS_HINTS = _re.compile(
    r"(最新|最近|近期|今年|当前|目前|202[3-9]|latest|recent|current)",
    _re.IGNORECASE,
)
_MULTI_TOOL_HINTS = _re.compile(
    r"(成分|配料|ingredient|热量|calorie|营养|nutrition|运动计划|exercise|meal\s*plan)",
    _re.IGNORECASE,
)


def _needs_planner(query: str) -> bool:
    """Return True if the query likely needs multi-tool planning.
    Simple factual questions can skip the planning LLM call."""
    return bool(_TIMELINESS_HINTS.search(query) or _MULTI_TOOL_HINTS.search(query))


def _clean_identity(value: Optional[str], max_length: int = 200) -> Optional[str]:
    text = str(value or "").strip()
    return text[:max_length] if text else None


def _memory_user_id(req: ChatCompletionRequest, request: Request) -> Optional[str]:
    return _clean_identity(
        getattr(request.state, "platform_user_id", None)
        or getattr(req, "user_id", None)
        or getattr(req, "user", None)
        or request.headers.get("x-user-id")
    )


DEFAULT_ALLOWED_TOOLS = [
    "rag.search",
    "sql.search",
    "vitals.summary",
    "mcp.vitals_alert.check_vitals",
    "mcp.web_search.web_search",
    "mcp.web_search.ingredient_check",
    "mcp.nutritional_planner.calculate_meal_plan",
    "mcp.nutritional_planner.generate_exercise_plan",
]


def _resolve_request_allowed_tools(
    req: ChatCompletionRequest,
    available_names: set[str],
) -> List[str]:
    if req.tools is None:
        allowed_tools = [name for name in DEFAULT_ALLOWED_TOOLS if name in available_names]
    else:
        allowed_tools = [
            str(tool.get("function", {}).get("name") or "").strip()
            for tool in req.tools
            if isinstance(tool, dict) and isinstance(tool.get("function"), dict)
        ]
        allowed_tools = [name for name in allowed_tools if name and name in available_names]

    if req.tool_choice == "none":
        allowed_tools = []
    elif isinstance(req.tool_choice, dict):
        function = req.tool_choice.get("function")
        selected = str(function.get("name") or "").strip() if isinstance(function, dict) else ""
        allowed_tools = [selected] if selected and selected in allowed_tools else []
    return filter_tools_without_animal(allowed_tools)

_TOOL_DESCRIPTIONS = {
    "rag.search": "Search the veterinary knowledge base for medical/health information.",
    "vitals.summary": "Aggregated HR/RR/temperature stats for the request-scoped pet (collar time-series).",
    "mcp.vitals_alert.check_vitals": (
        "Check PetHealth PostgreSQL heart-rate and respiratory-rate samples for one pet_id, "
        "returning thresholds, abnormal samples, and alert_level."
    ),
    "sql.search": (
        "Read-only query on PetMind whitelist tables: daily_reports (daily summaries), "
        "animals (pet profile incl. species/breed/age/weight), sensor_events (collar upload windows). "
        "Requires request animal_id; server enforces the animal scope. "
        "For HR/RR/temperature trends use vitals.summary; for veterinary knowledge use rag.search."
    ),
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


def _build_pethealth_server_context(
    req: ChatCompletionRequest,
    request: Request,
) -> Optional[Dict[str, Any]]:
    ctx = req.pethealth_server
    if ctx is None or not bool(ctx.heart_rate_abnormal):
        return None
    animal_id = (
        str(ctx.animal_id).strip()
        if ctx.animal_id is not None and str(ctx.animal_id).strip()
        else str(getattr(req, "animal_id", "") or request.headers.get("x-animal-id") or "").strip()
    )
    if not animal_id:
        return None
    return {
        "animal_id": animal_id,
        "heart_rate_abnormal": True,
        "vitals_window_hours": int(ctx.vitals_window_hours or 24),
    }


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
        "instructions": DECISION_INSTRUCTIONS,
        "user_query": query,
        "tool_results_so_far": [
            {
                "tool_name": r.get("tool_name"),
                "hits_count": (
                    _non_sql_tool_result_count(r.get("result"))[0]
                    if r.get("tool_name") != "sql.search" and isinstance(r.get("result"), dict)
                    else 0
                ),
                "row_count": (
                    _sql_row_count(r.get("result"))
                    if r.get("tool_name") == "sql.search" and isinstance(r.get("result"), dict)
                    else None
                ),
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


def _sql_row_count(result: Any) -> int:
    if not isinstance(result, dict):
        return 0
    rc = result.get("row_count")
    if isinstance(rc, int):
        return rc
    rows = result.get("rows", [])
    return len(rows) if isinstance(rows, list) else 0


def _non_sql_tool_result_count(result: Dict[str, Any]) -> tuple[int, float]:
    """
    Count items for SSE tool_complete / agent_detail.

    - rag.search: result["hits"] (list)
    - mcp.web_search.*: result["results"] (list) + optional result["count"]
    - Other tools: optional result["count"] or first list-valued key among common names
    """
    if not isinstance(result, dict):
        return 0, 0.0

    hits = result.get("hits")
    if isinstance(hits, list):
        best = max((float(h.get("score", 0.0)) for h in hits if isinstance(h, dict)), default=0.0)
        return len(hits), best

    results = result.get("results")
    if isinstance(results, list):
        n = len(results)
        cnt = result.get("count")
        if isinstance(cnt, int) and cnt >= 0:
            n = cnt
        return n, 0.0

    cnt = result.get("count")
    if isinstance(cnt, int) and cnt >= 0:
        return cnt, 0.0

    for key in ("items", "data", "records", "documents"):
        v = result.get(key)
        if isinstance(v, list):
            return len(v), 0.0

    return 0, 0.0


def _tool_complete_line_and_detail(
    tool_name: str,
    result: Dict[str, Any],
    *,
    arguments: Optional[Dict[str, Any]] = None,
    round_num: Optional[int] = None,
    inline_prefix: bool = False,
    include_debug_result: bool = False,
) -> tuple[str, Dict[str, Any]]:
    """SSE content line and agent_detail for tool_complete (RAG vs sql.search vs web_search, etc.)."""
    if tool_name == "sql.search":
        n = _sql_row_count(result)
        if inline_prefix:
            line = f"   {tool_name}: {n} rows\n"
        else:
            line = f"   sql.search returned {n} row(s)\n"
        detail: Dict[str, Any] = {"tool_name": tool_name, "row_count": n}
        if arguments is not None:
            detail["arguments"] = arguments
        if round_num is not None:
            detail["round"] = round_num
        return line, detail

    hits_count, best_score = _non_sql_tool_result_count(result)
    if inline_prefix:
        line = f"   {tool_name}: {hits_count} results\n"
    else:
        line = f"   Found {hits_count} relevant results\n"
    detail = {"tool_name": tool_name, "hits_count": hits_count, "best_score": best_score}
    if arguments is not None:
        detail["arguments"] = arguments
    if include_debug_result and tool_name.startswith("mcp.web_search"):
        detail["result"] = result
    if round_num is not None:
        detail["round"] = round_num
    return line, detail


def _summarize_tool_result(result: Any, max_chars: int = 500) -> str:
    if not isinstance(result, dict):
        return str(result)[:max_chars]

    # sql.search
    if "rows" in result or result.get("row_count") is not None:
        rows = result.get("rows", [])
        n = result.get("row_count")
        if not isinstance(n, int) and isinstance(rows, list):
            n = len(rows)
        if not isinstance(n, int):
            n = 0
        preview = rows[:2] if isinstance(rows, list) else []
        tail = json.dumps(preview, ensure_ascii=False)[: max_chars - 40]
        ok = result.get("ok")
        return f"sql.search ok={ok} rows={n} preview={tail}"[:max_chars]

    # RAG results
    hits = result.get("hits", [])
    if hits:
        summaries = []
        for h in hits[:3]:
            text = h.get("text", "")[:200]
            summaries.append(text)
        return " | ".join(summaries)[:max_chars]

    # Direct web-search results. Handle these before the generic status field:
    # successful responses use status=OK as well as a populated results list.
    results = result.get("results", [])
    if isinstance(results, list) and results:
        summaries = []
        for item in results[:3]:
            if not isinstance(item, dict):
                summaries.append(str(item)[:200])
                continue
            title = str(item.get("title") or "")
            url = str(item.get("url") or item.get("link") or "")
            content = str(item.get("content") or item.get("snippet") or "")[:200]
            summaries.append(f"{title} ({url}): {content}")
        return " | ".join(summaries)[:max_chars]

    content = result.get("content", [])
    if isinstance(content, list) and content:
        texts = []
        for c in content[:3]:
            if isinstance(c, dict):
                texts.append(c.get("text", str(c))[:200])
            else:
                texts.append(str(c)[:200])
        return " | ".join(texts)[:max_chars]

    # MCP tool results -- generic status/message fallback.
    status = result.get("status", "")
    if status:
        msg = result.get("message", "")
        return f"status={status} {msg}"[:max_chars]

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
    user_role: str = "pet_owner",
) -> AsyncGenerator[str, None]:
    created = _now_ts()
    reg = get_registry()
    llm = get_shared_async_client()
    stream_llm = get_shared_async_stream_client()
    agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)

    _t_start = time.perf_counter()
    _timing: List[Dict[str, Any]] = []

    def _lap(label: str, t0: float, **extra: Any) -> None:
        if debug_timing:
            _timing.append({"step": label, "ms": round((time.perf_counter() - t0) * 1000, 1), **extra})

    make_chunk = partial(openai_sse_chunk, request_id=request_id, created=created, model=model)

    tool_results: List[Dict[str, Any]] = []
    completed_tool_calls: set[str] = set()
    available_tools = list(allowed_tools) if allowed_tools is not None else ["rag.search"]

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
                    {"role": "system", "content": DECISION_SYSTEM_PROMPT},
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
            if debug_timing:
                yield make_chunk(
                    status="decision_complete",
                    detail={
                        "round": round_num + 1,
                        "output": decide_text,
                        "decision": decision,
                    },
                )

        except ResourceBusyError as e:
            _lap(f"round_{round_num+1}_decide_llm_busy", t0)
            yield make_chunk(content="服务当前繁忙，请稍后重试。", status="busy", detail=e.as_dict(), finish="stop")
            yield SSE_DONE
            return
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

            call_key = canonical_tool_call(tool_name, args)
            if call_key in completed_tool_calls:
                yield make_chunk(
                    content=f"\n**Skipping repeated tool call**: {tool_name}\n",
                    status="tool_skipped",
                    detail={
                        "tool_name": tool_name,
                        "arguments": args,
                        "round": round_num + 1,
                        "reason": "exact_duplicate",
                    },
                )
                break

            yield make_chunk(
                content=f"\n**Round {round_num + 1} tool call**: {tool_name}\n",
                status="tool_calling",
                detail={"tool_name": tool_name, "round": round_num + 1, "reason": reason}
            )

            if args.get("query"):
                yield make_chunk(content=f"   Query: {args['query']}\n")
            if tool_name == "sql.search" and args.get("table"):
                yield make_chunk(content=f"   Table: {args['table']}\n")

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
                completed_tool_calls.add(call_key)

                if not isinstance(result, dict):
                    result = {}
                line, detail = _tool_complete_line_and_detail(
                    tool_name,
                    result,
                    arguments=args,
                    round_num=round_num + 1,
                    include_debug_result=debug_timing,
                )
                yield make_chunk(content=line, status="tool_complete", detail=detail)

                special_msg = _check_special_flags(result)
                if special_msg:
                    yield make_chunk(content=special_msg, status="user_action_needed")
            except ResourceBusyError as e:
                _lap(f"round_{round_num+1}_tool_{tool_name}_busy", t0)
                yield make_chunk(content="资源当前繁忙，请稍后重试。", status="busy", detail=e.as_dict(), finish="stop")
                yield SSE_DONE
                return
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

    has_web_search = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    sys_prompt = build_solve_prompt(
        user_role=user_role, has_web_search=has_web_search, query=query, max_tokens=max_tokens,
    )
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
    except ResourceBusyError as e:
        yield make_chunk(content="服务当前繁忙，请稍后重试。", status="busy", detail=e.as_dict(), finish="stop")
        yield SSE_DONE
        return
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
    yield SSE_DONE


async def _stream_plan_and_solve(
    request_id: str,
    model: str,
    query: str,
    system_context: str,
    temperature: float,
    max_tokens: int,
    allowed_tools: Optional[List[str]],
    debug_timing: bool = False,
    user_role: str = "pet_owner",
) -> AsyncGenerator[str, None]:
    created = _now_ts()
    reg = get_registry()
    llm = get_shared_async_client()
    stream_llm = get_shared_async_stream_client()
    agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)

    _t_start = time.perf_counter()
    _timing: List[Dict[str, Any]] = []

    def _lap(label: str, t0: float, **extra: Any) -> None:
        if debug_timing:
            _timing.append({"step": label, "ms": round((time.perf_counter() - t0) * 1000, 1), **extra})

    make_chunk = partial(openai_sse_chunk, request_id=request_id, created=created, model=model)

    use_planner = _needs_planner(query)

    if use_planner:
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
        except ResourceBusyError as e:
            _lap("plan_llm_busy", t0)
            yield make_chunk(content="服务当前繁忙，请稍后重试。", status="busy", detail=e.as_dict(), finish="stop")
            yield SSE_DONE
            return
        except Exception as e:
            _lap("plan_llm_error", t0)
            yield make_chunk(content=f"Planning failed: {e}", finish="stop")
            yield SSE_DONE
            return
    else:
        plan = []
        if allowed_tools is None or "rag.search" in allowed_tools:
            plan.append(
                {"type": "tool", "tool_name": "rag.search", "arguments": {"query": query}, "note": "fast path"}
            )
        plan.append({"type": "final"})
        yield make_chunk(status="plan_complete", detail={"plan": plan, "fast_path": True})

    tool_results: List[Dict[str, Any]] = []
    allowed = set(allowed_tools) if allowed_tools is not None else None

    # --- Collect tool steps, then execute in parallel if possible ---
    tool_steps: List[Dict[str, Any]] = []
    for i, step in enumerate(plan):
        if step.get("type") == "tool":
            tool_name = str(step.get("tool_name") or "")
            if not tool_name or (allowed is not None and tool_name not in allowed):
                continue
            args = step.get("arguments") or {}
            if not isinstance(args, dict):
                args = {}
            if tool_name == "rag.search":
                args = agent._force_rag_search_defaults(args)
            tool_steps.append({"step": i, "tool_name": tool_name, "arguments": args})
        elif step.get("type") == "final":
            break

    # Medical queries: pair rag.search + web_search when both are allowed.
    _paired = ensure_rag_and_web_tool_steps(
        [{"type": "tool", "tool_name": ts["tool_name"], "arguments": ts["arguments"]} for ts in tool_steps],
        visible_tools=list(allowed) if allowed is not None else [t.name for t in reg.list_tools()],
        query=query,
    )
    if len(_paired) != len(tool_steps) or {s.get("tool_name") for s in _paired} != {t["tool_name"] for t in tool_steps}:
        tool_steps = []
        for i, s in enumerate(_paired):
            name = str(s.get("tool_name") or "")
            if not name or (allowed is not None and name not in allowed):
                continue
            args = s.get("arguments") or {}
            if not isinstance(args, dict):
                args = {}
            if name == "rag.search":
                args = agent._force_rag_search_defaults(args)
            tool_steps.append({"step": i, "tool_name": name, "arguments": args})

    if len(tool_steps) >= 2:
        yield make_chunk(
            content=f"\n**Parallel execution**: {', '.join(s['tool_name'] for s in tool_steps)}\n",
            status="tool_calling",
            detail={"parallel": True, "count": len(tool_steps)},
        )
        t0 = time.perf_counter()

        async def _run_tool(ts: Dict[str, Any]) -> Dict[str, Any]:
            try:
                result = await reg.call(ts["tool_name"], ts["arguments"])
                return {**ts, "result": result}
            except ResourceBusyError as exc:
                return {**ts, "error": str(exc), "resource_busy": exc.as_dict()}
            except Exception as exc:
                return {**ts, "error": str(exc)}

        gathered = await asyncio.gather(*[_run_tool(ts) for ts in tool_steps], return_exceptions=False)
        _lap("parallel_tools", t0)

        for tr in gathered:
            tool_results.append(tr)
            if "error" in tr:
                if tr.get("resource_busy"):
                    yield make_chunk(
                        content=f"   {tr['tool_name']}: resource busy\n",
                        status="busy",
                        detail=tr["resource_busy"],
                    )
                else:
                    yield make_chunk(content=f"   {tr['tool_name']}: failed ({tr['error']})\n")
            else:
                result = tr.get("result", {})
                if not isinstance(result, dict):
                    result = {}
                line, detail = _tool_complete_line_and_detail(
                    tr["tool_name"],
                    result,
                    arguments=tr.get("arguments"),
                    inline_prefix=True,
                    include_debug_result=debug_timing,
                )
                yield make_chunk(content=line, status="tool_complete", detail=detail)
                special_msg = _check_special_flags(result)
                if special_msg:
                    yield make_chunk(content=special_msg, status="user_action_needed")
    else:
        for ts in tool_steps:
            yield make_chunk(
                content=f"\n**Executing tool**: {ts['tool_name']}\n",
                status="tool_calling",
                detail={"tool_name": ts["tool_name"], "step": ts["step"]},
            )
            t0 = time.perf_counter()
            try:
                result = await reg.call(ts["tool_name"], ts["arguments"])
                _lap(f"step_{ts['step']}_tool_{ts['tool_name']}", t0)
                tool_results.append({**ts, "result": result})
                if not isinstance(result, dict):
                    result = {}
                line, detail = _tool_complete_line_and_detail(
                    ts["tool_name"],
                    result,
                    arguments=ts.get("arguments"),
                    include_debug_result=debug_timing,
                )
                yield make_chunk(content=line, status="tool_complete", detail=detail)
                special_msg = _check_special_flags(result)
                if special_msg:
                    yield make_chunk(content=special_msg, status="user_action_needed")
            except ResourceBusyError as e:
                _lap(f"step_{ts['step']}_tool_{ts['tool_name']}_busy", t0)
                tool_results.append({**ts, "error": str(e), "resource_busy": e.as_dict()})
                yield make_chunk(content="资源当前繁忙，请稍后重试。", status="busy", detail=e.as_dict())
            except Exception as e:
                _lap(f"step_{ts['step']}_tool_{ts['tool_name']}_error", t0)
                yield make_chunk(content=f"Tool call failed: {e}\n")
                tool_results.append({**ts, "error": str(e)})

    # --- RAG fallback: auto web search when RAG results are insufficient or irrelevant ---
    _WEB_TOOL = "mcp.web_search.web_search"
    _rag_results = [
        r for r in tool_results
        if r.get("tool_name") == "rag.search" and isinstance(r.get("result"), dict)
    ]
    rag_hits_total = sum(len(r["result"].get("hits", [])) for r in _rag_results)
    rag_best_score = max(
        (hit.get("score", 0.0) for r in _rag_results for hit in r["result"].get("hits", [])),
        default=0.0,
    )
    import os as _os
    _RAG_RELEVANCE_THRESHOLD = float(_os.getenv("RAG_RELEVANCE_THRESHOLD", "0.55"))
    _WEB_FALLBACK_MIN_HITS = int(_os.getenv("RAG_WEB_FALLBACK_MIN_HITS", "2"))
    already_has_web = any(r.get("tool_name") == _WEB_TOOL for r in tool_results)
    _need_web = (rag_hits_total < _WEB_FALLBACK_MIN_HITS) or (rag_hits_total > 0 and rag_best_score < _RAG_RELEVANCE_THRESHOLD)
    if _need_web and not already_has_web and _WEB_TOOL in (allowed_tools or []):
        _fallback_reason = (
            f"RAG only returned {rag_hits_total} hits"
            if rag_hits_total < 2
            else f"RAG best score {rag_best_score:.3f} < {_RAG_RELEVANCE_THRESHOLD} (irrelevant content)"
        )
        yield make_chunk(
            content=f"\n**Fallback**: web_search ({_fallback_reason})\n",
            status="tool_calling",
            detail={"tool_name": _WEB_TOOL, "step": "fallback", "reason": _fallback_reason},
        )
        t0 = time.perf_counter()
        try:
            web_result = await reg.call(_WEB_TOOL, {"query": query, "max_results": 5, "search_depth": "advanced"})
            _lap("fallback_web_search", t0)
            tool_results.append({"step": "fallback", "tool_name": _WEB_TOOL, "arguments": {"query": query}, "result": web_result})
            web_count = len(web_result.get("results", [])) if isinstance(web_result, dict) else 0
            yield make_chunk(
                content=f"Found {web_count} web results\n",
                status="tool_complete",
                detail={
                    "tool_name": _WEB_TOOL,
                    "hits_count": web_count,
                    "arguments": {"query": query, "max_results": 5, "search_depth": "advanced"},
                    **({"result": web_result} if debug_timing else {}),
                },
            )
        except ResourceBusyError as e:
            _lap("fallback_web_search_busy", t0)
            yield make_chunk(content="实时搜索当前繁忙。\n", status="busy", detail=e.as_dict())
        except Exception as e:
            _lap("fallback_web_search_error", t0)
            yield make_chunk(content=f"Web search fallback failed: {e}\n")

    yield make_chunk(
        content="\n**Generating response...**\n\n",
        status="generating"
    )

    has_web_search = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    sys_prompt = build_solve_prompt(
        user_role=user_role, has_web_search=has_web_search, query=query, max_tokens=max_tokens,
    )
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
    except ResourceBusyError as e:
        yield make_chunk(content="服务当前繁忙，请稍后重试。", status="busy", detail=e.as_dict(), finish="stop")
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
    yield SSE_DONE


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
    """MoE 流式：包装 MoEOrchestrator.stream 为 OpenAI 兼容 SSE chunk。"""
    created = _now_ts()

    make_chunk = partial(openai_sse_chunk, request_id=request_id, created=created, model=model)

    orch = build_moe_orchestrator(
        registry=get_registry(), temperature=temperature, max_tokens=max_tokens,
        user_role=user_role, allowed_tools=allowed_tools,
        pethealth_server=pethealth_server,
    )
    recorder = MoETrace(question=query, user_role=user_role) if debug_timing else None

    try:
        emitted_finish = False
        async for ev in orch.stream(
            query=query,
            system_context=system_context,
            conversation_history=conversation_history,
            user_memory=user_memory,
            recorder=recorder,
        ):
            finish = ev.get("finish")
            emitted_finish = emitted_finish or bool(finish)
            yield make_chunk(
                content=ev.get("content") or "",
                status=ev.get("status"),
                detail=ev.get("detail"),
                finish=finish,
            )
        if recorder is not None:
            yield make_chunk(
                status="llm_trace",
                detail={
                    "total_ms": recorder.total_ms,
                    "calls": [
                        {
                            "seq": call.seq,
                            "stage": call.stage,
                            "model": call.model,
                            "output": call.output,
                            "latency_ms": call.latency_ms,
                            "prompt_tokens": call.prompt_tokens,
                            "completion_tokens": call.completion_tokens,
                            "total_tokens": call.total_tokens,
                            "meta": call.meta,
                        }
                        for call in recorder.llm_calls
                    ],
                },
            )
    except ResourceBusyError as exc:
        yield make_chunk(content="服务当前繁忙，请稍后重试。", status="busy", detail=exc.as_dict(), finish="stop")
        yield SSE_DONE
        return
    except Exception as exc:  # noqa: BLE001
        yield make_chunk(content=f"\nMoE generation failed: {exc}")

    if not emitted_finish:
        yield make_chunk(finish="stop")
    yield SSE_DONE


@router.post("/v1/chat/completions")
async def chat_completions(req: ChatCompletionRequest, request: Request):
    """
    Stateless OpenAI-compatible chat completions endpoint.

    Supports both streaming (stream=true) and non-streaming modes. The Agent
    process does not create a conversation session or reuse prior requests;
    callers must persist and resend the complete `messages` history they need.
    QA/trace observability records are not conversation memory.

    Models:
    - "agent-plan-solve": Single-turn plan-and-solve (backward compatible)
    - "agent-multi-turn": Multi-turn agent with iterative tool calls
    - "agent-moe": MoE veterinary committee: router + weighted experts + critic gating (DEFAULT for unrecognized model names)
    """
    trace_id = new_trace_id()
    request_id = _gen_id()
    created = _now_ts()
    # 构建对话历史，查询，系统提示词
    query = _extract_user_query(req.messages)
    system_context = _build_system_context(req.messages)
    conversation_history = _build_conversation_history(req.messages)
    pethealth_server_context = _build_pethealth_server_context(req, request)
    memory_user_id = _memory_user_id(req, request)
    memory_pet_id = _clean_identity(
        getattr(req, "animal_id", None) or request.headers.get("x-animal-id")
    )
    memory_injection, memory_load_detail = await load_user_memory(
        user_id=memory_user_id,
        query=query,
        pet_id=memory_pet_id,
    )
    if memory_injection:
        system_context = "\n\n".join(part for part in (system_context, memory_injection) if part)
    memory_session_id = _clean_identity(getattr(req, "memory_session_id", None))
    memory_turn_id = _clean_identity(getattr(req, "memory_turn_id", None)) or request_id

    # 请求级 animal_id（sql.search 仅在非空时进入工具名单；由 ContextVar 供 sql_search_tool 读取）
    set_request_animal_id(
        body_animal_id=getattr(req, "animal_id", None),
        header_animal_id=request.headers.get("x-animal-id"),
    )

    # None means service defaults; an explicit [] or tool_choice=none means no tools.
    reg = get_registry()
    available_names = {tool.name for tool in reg.list_tools()}
    allowed_tools = _resolve_request_allowed_tools(req, available_names)

    # MoE committee is the default pipeline; plan / multi-turn are explicit opt-outs.
    mode = resolve_agent_mode(req.model)
    use_multi_turn = mode is AgentMode.MULTI_TURN
    use_moe = mode is AgentMode.MOE
    _debug_timing = bool(req.debug_timing)
    user_role = req.user_role or "pet_owner"

    source_ip = request.client.host if request.client else ""

    # 流式处理
    if req.stream:
        async def event_generator():
            set_request_animal_id(
                body_animal_id=getattr(req, "animal_id", None),
                header_animal_id=request.headers.get("x-animal-id"),
            )
            t0 = time.monotonic()
            collected_content = []
            collected_tools = []
            collected_timing = []
            collected_rag_hits = 0
            collected_rag_best_score = 0.0
            collected_web_search = False

            yield openai_sse_chunk(
                request_id=request_id,
                model=req.model,
                status="memory_loaded" if memory_load_detail.get("loaded") else "memory_skipped",
                detail=memory_load_detail,
            )

            if use_moe:
                source = _stream_moe_agent(
                    request_id=request_id,
                    model=req.model,
                    query=query,
                    system_context=system_context,
                    conversation_history=conversation_history,
                    temperature=req.temperature or 0.3,
                    max_tokens=req.max_tokens,
                    allowed_tools=allowed_tools,
                    user_role=user_role,
                    debug_timing=_debug_timing,
                    pethealth_server=pethealth_server_context,
                    user_memory=memory_injection,
                )
            elif use_multi_turn:
                source = _stream_multi_turn_agent(
                    request_id=request_id,
                    model=req.model,
                    query=query,
                    system_context=system_context,
                    conversation_history=conversation_history,
                    temperature=req.temperature or 0.2,
                    max_tokens=req.max_tokens or 768,
                    allowed_tools=allowed_tools,
                    debug_timing=_debug_timing,
                    user_role=user_role,
                )
            else:
                source = _stream_plan_and_solve(
                    request_id=request_id,
                    model=req.model,
                    query=query,
                    system_context=system_context,
                    temperature=req.temperature or 0.2,
                    max_tokens=req.max_tokens or 768,
                    allowed_tools=allowed_tools,
                    debug_timing=_debug_timing,
                    user_role=user_role,
                )

            async for chunk in source:
                if chunk == SSE_DONE:
                    continue
                yield chunk
                if chunk.startswith("data: ") and chunk.strip() != "data: [DONE]":
                    try:
                        obj = json.loads(chunk[6:])
                        delta = (obj.get("choices") or [{}])[0].get("delta") or {}
                        status = obj.get("agent_status")
                        detail = obj.get("agent_detail") or {}
                        if status == "streaming" and delta.get("content"):
                            collected_content.append(delta["content"])
                        elif status == "tool_complete":
                            tool_name = detail.get("tool_name", "")
                            collected_tools.append(tool_name)
                            if "rag" in tool_name:
                                collected_rag_hits += detail.get("hits_count", 0)
                                bs = detail.get("best_score", 0.0) or 0.0
                                if bs > collected_rag_best_score:
                                    collected_rag_best_score = bs
                            if "web_search" in tool_name:
                                collected_web_search = True
                        elif status == "timing_summary":
                            collected_timing = detail.get("timing", [])
                    except Exception:
                        pass

            memory_write_detail = await write_user_memory(
                user_id=memory_user_id,
                query=query,
                answer="".join(collected_content),
                pet_id=memory_pet_id,
                session_id=memory_session_id,
                turn_id=memory_turn_id,
            )
            yield openai_sse_chunk(
                request_id=request_id,
                model=req.model,
                status="memory_stored" if memory_write_detail.get("stored") else "memory_store_skipped",
                detail=memory_write_detail,
            )
            yield SSE_DONE

            write_trace(
                trace_id,
                tool="v1.chat.completions.stream",
                request={"model": req.model, "query": query, "allowed_tools": allowed_tools},
                response={
                    "id": request_id,
                    "answer_length": sum(len(s) for s in collected_content),
                    "tools_called": collected_tools,
                    "timing": collected_timing,
                },
            )
            elapsed_ms = int((time.monotonic() - t0) * 1000)
            try:
                await save_qa_record(
                    question=query, answer="".join(collected_content),
                    model=req.model, tools_used=collected_tools,
                    rag_hit_count=collected_rag_hits,
                    rag_best_score=collected_rag_best_score,
                    used_web_search=collected_web_search,
                    response_time_ms=elapsed_ms,
                    source_ip=source_ip, user_role=user_role,
                    request_id=request_id,
                )
            except Exception:
                pass

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers=SSE_RESPONSE_HEADERS,
        )

    else:
        t0_non_stream = time.monotonic()
        try:
            _t_start = time.perf_counter()
            _timing: List[Dict[str, Any]] = []

            if use_moe:
                orch = build_moe_orchestrator(
                    registry=get_registry(), temperature=req.temperature or 0.3,
                    max_tokens=req.max_tokens, user_role=user_role,
                    allowed_tools=allowed_tools,
                    pethealth_server=pethealth_server_context,
                )
                moe_trace = MoETrace(question=query, user_role=user_role)
                answer, moe_trace = await orch.run(
                    query=query,
                    system_context=system_context,
                    conversation_history=conversation_history,
                    user_memory=memory_injection,
                    recorder=moe_trace,
                )
                response = ChatCompletionResponse(
                    id=request_id,
                    created=created,
                    model=req.model,
                    choices=[ChatCompletionChoice(
                        message=ChatMessage(role="assistant", content=answer),
                        finish_reason=orch.last_finish_reason,
                    )],
                    usage=UsageInfo(),
                )
                memory_write_detail = await write_user_memory(
                    user_id=memory_user_id,
                    query=query,
                    answer=answer or "",
                    pet_id=memory_pet_id,
                    session_id=memory_session_id,
                    turn_id=memory_turn_id,
                )
                response.memory = {
                    "load": memory_load_detail,
                    "write": memory_write_detail,
                }
                write_trace(trace_id, tool="v1.chat.completions.moe", request=req.model_dump(), response=response.model_dump())
                elapsed_ms = int((time.monotonic() - t0_non_stream) * 1000)
                tool_names = [call.tool_name for call in (moe_trace.tool_calls if moe_trace else [])]
                rag_calls = moe_trace.rag_calls if moe_trace else []
                try:
                    await save_qa_record(
                        question=query, answer=answer or "", model=req.model,
                        tools_used=tool_names,
                        rag_hit_count=sum(call.hits_count for call in rag_calls),
                        rag_best_score=max((call.best_score for call in rag_calls), default=0.0),
                        used_web_search=any("web_search" in name for name in tool_names),
                        response_time_ms=elapsed_ms, source_ip=source_ip,
                        user_role=user_role, request_id=request_id,
                    )
                except Exception:
                    pass
                return response

            reg = get_registry()
            llm = get_shared_async_client()
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
                user_role=user_role,
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

            memory_write_detail = await write_user_memory(
                user_id=memory_user_id,
                query=query,
                answer=answer or "",
                pet_id=memory_pet_id,
                session_id=memory_session_id,
                turn_id=memory_turn_id,
            )
            response.memory = {
                "load": memory_load_detail,
                "write": memory_write_detail,
            }

            write_trace(trace_id, tool="v1.chat.completions", request=req.model_dump(), response=response.model_dump())

            elapsed_ms = int((time.monotonic() - t0_non_stream) * 1000)
            tools_called = [tr.get("tool_name", "") for tr in (tool_results or []) if isinstance(tr, dict)]
            rag_hits = sum(
                len(tr.get("result", {}).get("hits", []))
                for tr in (tool_results or [])
                if isinstance(tr, dict) and "rag" in tr.get("tool_name", "") and isinstance(tr.get("result"), dict)
            )
            web_used = any("web_search" in tr.get("tool_name", "") for tr in (tool_results or []) if isinstance(tr, dict))
            try:
                await save_qa_record(
                    question=query, answer=answer or "",
                    model=req.model, tools_used=tools_called,
                    rag_hit_count=rag_hits, used_web_search=web_used,
                    response_time_ms=elapsed_ms,
                    source_ip=source_ip, user_role=user_role,
                    request_id=request_id,
                )
            except Exception:
                pass

            return response

        except ResourceBusyError as e:
            error_response = {
                "error": {
                    "message": str(e),
                    "type": "server_busy",
                    "code": e.code,
                    "detail": e.as_dict()["detail"],
                }
            }
            write_trace(trace_id, tool="v1.chat.completions", request=req.model_dump(), response=error_response, error=str(e))
            raise HTTPException(
                status_code=503,
                detail=error_response,
                headers={"Retry-After": str(max(1, math.ceil(e.timeout_s)))},
            ) from e
        except Exception as e:
            error_response = {
                "error": {
                    "message": str(e),
                    "type": "server_error",
                    "code": "agent_error",
                }
            }
            write_trace(trace_id, tool="v1.chat.completions", request=req.model_dump(), response=error_response, error=str(e))
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
                "owned_by": "petmind",
                "description": "Single-turn plan-and-solve agent (opt-out of the default MoE pipeline)",
            },
            {
                "id": "agent-multi-turn",
                "object": "model",
                "created": 1700000000,
                "owned_by": "petmind",
                "description": "Multi-turn agent with iterative tool calls",
            },
            {
                "id": "agent-moe",
                "object": "model",
                "created": 1700000000,
                "owned_by": "petmind",
                "description": "MoE veterinary committee: router + weighted experts + critic gating (DEFAULT for unrecognized model names)",
            },
        ]
    }
