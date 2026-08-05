from __future__ import annotations

import os
import traceback
from typing import Any, Dict

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .auth import APIKeyAuthMiddleware, load_api_keys

from .rate_limit import RateLimitMiddleware
from .session_manager import get_session_manager
from .rag_tools import rag_reindex_tool, rag_search_tool, warmup_rag_cache
from .tool_registry import get_registry
from .tools_builtin import register_builtin_tools, register_debug_tools
from .tools_mcp import register_mcp_tools
from .routes_openai import router as openai_router
from .schemas import (
    AgentPlanAndSolveRequest,
    AgentPlanAndSolveResponse,
    JobStatusResponse,
    RagReindexRequest,
    RagSearchRequest,
    ToolCallRequest,
    ToolListResponse,
    ToolSpecOut,
    ToolResponse,
)
from .llm_client import AsyncOpenAIClient
from .plan_and_solve import AsyncPlanAndSolveAgent
from .trace_store import new_trace_id, write_trace


app = FastAPI(title="PetHealthAI Agent API", version="0.3.0")

app.include_router(openai_router)

app.add_middleware(APIKeyAuthMiddleware)

_rl_rate = float(os.getenv("AGENT_RATE_LIMIT", "30"))
_rl_burst = int(os.getenv("AGENT_RATE_BURST", str(int(_rl_rate))))
app.add_middleware(RateLimitMiddleware, rate=_rl_rate, burst=_rl_burst)

if os.getenv("AGENT_ENABLE_CORS", "0") == "1":
    _origins = [o.strip() for o in os.getenv("AGENT_CORS_ORIGINS", "*").split(",") if o.strip()]
    app.add_middleware(
        CORSMiddleware,
        allow_origins=_origins if _origins != ["*"] else ["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["Content-Type", "Authorization", "Accept", "X-Requested-With"],
    )


@app.on_event("startup")
def _startup() -> None:
    load_api_keys()

    reg = get_registry()
    if reg.get("rag.search") is None:
        register_builtin_tools(reg)
        register_debug_tools(reg)
        if os.getenv("AGENT_ENABLE_MCP", "1") == "1":
            register_mcp_tools(reg)

    if os.getenv("AGENT_WARMUP_RAG", "1") == "1":
        from pathlib import Path
        from RAG.simple_rag.config import default_config

        repo_root = Path(__file__).resolve().parents[2]
        cfg = default_config(repo_root)
        device = os.getenv("AGENT_WARMUP_DEVICE") or None
        embedding_model = os.getenv("AGENT_WARMUP_EMBEDDING_MODEL", "intfloat/multilingual-e5-small")
        rerank_model = os.getenv("AGENT_WARMUP_RERANK_MODEL", "BAAI/bge-reranker-large")
        enable_bm25 = os.getenv("AGENT_WARMUP_BM25", "1") == "1"
        enable_reranker = os.getenv("AGENT_WARMUP_RERANKER", "1") == "1"

        stats = warmup_rag_cache(
            index_dir=cfg.index_dir,
            embedding_model=embedding_model,
            device=device,
            enable_bm25=enable_bm25,
            enable_reranker=enable_reranker,
            rerank_model=rerank_model,
        )

        actual_device = device or "cpu"
        try:
            import torch
            if torch.cuda.is_available() and actual_device != "cpu":
                gpu_name = torch.cuda.get_device_name(0)
                print(f"[startup] Device: {actual_device} ({gpu_name})")
            else:
                print(f"[startup] Device: cpu" + (" (CUDA available but not selected)" if torch.cuda.is_available() else ""))
        except ImportError:
            print(f"[startup] Device: {actual_device} (torch not found, cannot detect GPU)")
        print(f"[startup] Embedding: {embedding_model}")
        print(f"[startup] Reranker: {rerank_model if enable_reranker else 'disabled'}")
        print(f"[startup] BM25: {'enabled' if enable_bm25 else 'disabled'}")
        print(f"[startup] Index: {stats['index_size']} chunks")


@app.get("/health")
def health() -> Dict[str, Any]:
    return {"ok": True}


@app.post("/tools/rag/search", response_model=ToolResponse)
def rag_search(req: RagSearchRequest) -> ToolResponse:
    trace_id = new_trace_id()
    try:
        data = rag_search_tool(**req.model_dump())
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="rag.search", request=req.model_dump(), response=resp.model_dump())
        return resp
    except Exception as e:  # noqa: BLE001
        err = {"code": "RAG_SEARCH_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="rag.search", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post("/tools/rag/reindex", response_model=ToolResponse)
def rag_reindex(req: RagReindexRequest) -> ToolResponse:
    trace_id = new_trace_id()
    try:
        data = rag_reindex_tool(**req.model_dump())
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="rag.reindex", request=req.model_dump(), response=resp.model_dump())
        return resp
    except Exception as e:  # noqa: BLE001
        err = {"code": "RAG_REINDEX_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="rag.reindex", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.get("/tools", response_model=ToolListResponse)
def tools_list() -> ToolListResponse:
    trace_id = new_trace_id()
    reg = get_registry()
    tools = [ToolSpecOut(name=t.name, description=t.description, input_schema=t.input_schema) for t in reg.list_tools()]
    resp = ToolListResponse(ok=True, trace_id=trace_id, tools=tools)
    write_trace(trace_id, tool="tools.list", request={}, response=resp.model_dump())
    return resp


@app.post("/tools/call", response_model=ToolResponse)
async def tools_call(req: ToolCallRequest) -> ToolResponse:
    trace_id = new_trace_id()
    reg = get_registry()
    try:
        data = await reg.call(req.tool_name, req.arguments)
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="tools.call", request=req.model_dump(), response=resp.model_dump())
        return resp
    except Exception as e:  # noqa: BLE001
        err = {"code": "TOOL_CALL_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="tools.call", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post("/agent/plan_and_solve", response_model=AgentPlanAndSolveResponse)
async def agent_plan_and_solve(req: AgentPlanAndSolveRequest) -> AgentPlanAndSolveResponse:
    trace_id = new_trace_id()
    try:
        reg = get_registry()
        llm = AsyncOpenAIClient(base_url=req.llm_base_url, api_key=req.llm_api_key, model=req.llm_model)
        agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)
        plan = await agent.plan(query=req.query, allowed_tools=req.allowed_tools)
        answer, tool_results = await agent.solve(
            query=req.query,
            plan_steps=plan,
            allowed_tools=req.allowed_tools,
            temperature=req.temperature,
            max_tokens=req.max_tokens,
        )
        resp = AgentPlanAndSolveResponse(ok=True, trace_id=trace_id, answer=answer, plan=plan, tool_results=tool_results)
        write_trace(trace_id, tool="agent.plan_and_solve", request=req.model_dump(), response=resp.model_dump())
        return resp
    except Exception as e:  # noqa: BLE001
        err = {"code": "AGENT_PLAN_SOLVE_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = AgentPlanAndSolveResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="agent.plan_and_solve", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post("/sessions")
async def create_session():
    mgr = get_session_manager()
    sess = await mgr.create()
    return {"ok": True, "session_id": sess.session_id}


@app.get("/sessions/{session_id}")
async def get_session(session_id: str):
    mgr = get_session_manager()
    sess = await mgr.get(session_id)
    if not sess:
        return {"ok": False, "error": "session not found or expired"}
    return {
        "ok": True,
        "session_id": sess.session_id,
        "messages": sess.messages,
        "created_at": sess.created_at,
        "last_active": sess.last_active,
    }


@app.delete("/sessions/{session_id}")
async def delete_session(session_id: str):
    mgr = get_session_manager()
    deleted = await mgr.delete(session_id)
    return {"ok": deleted}



