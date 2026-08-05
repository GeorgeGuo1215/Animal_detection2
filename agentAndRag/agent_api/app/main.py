from __future__ import annotations

import math
import os
import sys
import threading
import traceback
from pathlib import Path

# Repo root (Animal_detection/) so `import integration` works when running from agentAndRag
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from typing import Any, Dict, Optional

from fastapi import Depends, FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .context.request_context import ANIMAL_REQUIRED_TOOLS, filter_tools_without_animal, get_request_animal_id, set_request_animal_id
from .concurrency import ResourceBusyError, configure_resource_limits, get_resource_limits
from .llm.llm_client import AsyncOpenAIClient, aclose_shared_async_client, get_shared_async_client
from .llm.llm_client_stream import aclose_shared_async_stream_client
from .lifecycle_tasks.session_cleanup import (
    start_session_cleanup_task,
    stop_session_cleanup_task,
)
from .middleware.auth import APIKeyAuthMiddleware, load_api_keys
from .middleware.rate_limit import RateLimitMiddleware
from .memory import close_memory_client, memory_status, start_memory_client
from .persistence.qa_store import (
    get_feedback_stats, get_knowledge_gaps, get_qa_stats,
    init_db as _init_qa_db, query_qa_history, submit_feedback,
)
from .persistence.session_manager import get_session_manager
from .persistence.trace_store import new_trace_id, write_trace
from .routers.routes_chat_ui import router as chat_ui_router
from .routers.routes_openai import router as openai_router
from .schemas import (
    AgentPlanAndSolveRequest,
    AgentPlanAndSolveResponse,
    RagReindexRequest,
    RagSearchRequest,
    SqlSearchRequest,
    ToolCallRequest,
    ToolListResponse,
    ToolSpecOut,
    ToolResponse,
)
from .hf_local_model import is_local_path, resolve_embedding_model_id, resolve_rerank_model_id
from .services.plan_and_solve import AsyncPlanAndSolveAgent
from .sql_search import sql_search_tool
from .tools.rag_tools import rag_reindex_tool, rag_search_tool, warmup_rag_cache
from .tools.tool_registry import get_registry
from .tools.tools_builtin import register_builtin_tools, register_debug_tools
from .tools.tools_mcp import register_mcp_tools_async

from integration.api.routes_ingest import router as integration_ingest_router


app = FastAPI(title="PetMind Agent API", version="0.4.0")

app.include_router(openai_router)
app.include_router(chat_ui_router)
app.include_router(integration_ingest_router, prefix="/integration", tags=["integration"])

app.add_middleware(APIKeyAuthMiddleware)

_rl_rate = float(os.getenv("AGENT_RATE_LIMIT", "30"))
_rl_burst = int(os.getenv("AGENT_RATE_BURST", str(int(_rl_rate))))
app.add_middleware(RateLimitMiddleware, rate=_rl_rate, burst=_rl_burst)

# --- CORS：前端与 Agent 不同端口时（如 web 在 :8001、Agent 在 :8000）浏览器会拦截，需返回 Access-Control-Allow-Origin ---
_CORS_DEFAULT_ORIGINS = (
    "http://127.0.0.1:8000 http://localhost:8000 "
    "http://127.0.0.1:8001 http://localhost:8001 "
    "http://127.0.0.1:5500 http://localhost:5500 "
    "http://127.0.0.1:5173 http://localhost:5173"
)
# 任意本机端口（Live Server、python -m http.server、Vite 等），避免仅白名单漏端口导致无 CORS 头
_CORS_LOCAL_ORIGIN_REGEX = r"https?://(127\.0\.0\.1|localhost)(:\d+)?$"
_env_cors_enable = os.getenv("AGENT_ENABLE_CORS", "1").strip().lower()
_env_cors_origins = os.getenv("AGENT_CORS_ORIGINS", "").strip()
_cors_on = _env_cors_enable not in ("0", "false", "no", "off")

if _cors_on:
    if _env_cors_origins == "*":
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["Content-Type", "Authorization", "Accept", "X-Requested-With", "X-Animal-Id", "X-User-Id"],
        )
    else:
        _parts = _env_cors_origins.split(",") if _env_cors_origins else _CORS_DEFAULT_ORIGINS.split()
        _cors_allow = [o.strip() for o in _parts if o.strip()]
        app.add_middleware(
            CORSMiddleware,
            allow_origins=_cors_allow if _cors_allow else ["http://127.0.0.1:8001"],
            allow_origin_regex=_CORS_LOCAL_ORIGIN_REGEX,
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["Content-Type", "Authorization", "Accept", "X-Requested-With", "X-Animal-Id", "X-User-Id"],
        )


# --- Readiness state --------------------------------------------------------
# Liveness (/health) is up the moment the process starts; readiness (/ready)
# flips true only after the (async) RAG warmup finishes, so an orchestrator
# won't route traffic into a cold instance whose first request would be slow.
_READY: bool = False
_WARMUP_INFO: Dict[str, Any] = {"status": "pending"}


def _run_rag_warmup_unlimited() -> None:
    """Blocking RAG cache warmup, run in a background thread so it never blocks
    server startup / liveness. Marks the process ready when finished."""
    global _READY, _WARMUP_INFO
    try:
        from RAG.simple_rag.config import default_config

        repo_root = Path(__file__).resolve().parents[2]
        cfg = default_config(repo_root)
        device = os.getenv("AGENT_WARMUP_DEVICE") or None

        hf_offline = os.getenv("AGENT_HF_OFFLINE", "").strip().lower() in ("1", "true", "yes")
        if hf_offline:
            os.environ.setdefault("HF_HUB_OFFLINE", "1")
            os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
            print("[startup] Hugging Face hub offline mode (AGENT_HF_OFFLINE=1): use local model dirs or set AGENT_EMBEDDING_MODEL_PATH.")

        embedding_model = resolve_embedding_model_id(None, repo_root)
        rerank_model = resolve_rerank_model_id(None, repo_root)

        enable_bm25 = os.getenv("AGENT_WARMUP_BM25", "1") == "1"
        enable_reranker = os.getenv("AGENT_WARMUP_RERANKER", "1") == "1"
        # Offline: avoid pulling CrossEncoder from Hub if only a hub name is configured
        if hf_offline and enable_reranker and not is_local_path(rerank_model):
            print("[startup] AGENT_HF_OFFLINE: disable reranker warmup (no local path). Set AGENT_RERANKER_MODEL_PATH or models/bge-reranker-large, or AGENT_WARMUP_RERANKER=0.")
            enable_reranker = False

        skip_warmup = hf_offline and not is_local_path(embedding_model)
        if skip_warmup:
            print(
                "[startup] AGENT_HF_OFFLINE: skip RAG warmup — embedding model is not a local path. "
                "Use agentAndRag/models/multilingual-e5-small, AGENT_EMBEDDING_MODEL_PATH, "
                "or download once so it appears under HF_HOME/hub/models--org--name (snapshots/).",
            )
            _WARMUP_INFO = {"status": "skipped", "reason": "hf_offline_no_local_embedding"}
            return

        stats = warmup_rag_cache(
            index_dir=cfg.index_dir,
            embedding_model=embedding_model,
            device=device,
            enable_bm25=enable_bm25,
            enable_reranker=enable_reranker,
            rerank_model=rerank_model,
        )

        # Preload dense vectors for MoE expert category sub-indexes (BM25 built lazily on first use).
        cat_warm: list[dict] = []
        warm_cats = os.getenv("AGENT_WARMUP_CATEGORIES", "1") == "1"
        if warm_cats:
            try:
                from RAG.simple_rag.category_index import (
                    expert_category_warmup_ids,
                    resolve_category_index_dirs,
                )

                for cid in expert_category_warmup_ids():
                    dirs = resolve_category_index_dirs(repo_root=repo_root, category=cid)
                    for d in dirs:
                        if not d.exists():
                            continue
                        try:
                            st = warmup_rag_cache(
                                index_dir=d,
                                embedding_model=embedding_model,
                                device=device,
                                enable_bm25=False,
                                enable_reranker=False,
                                rerank_model=rerank_model,
                            )
                            if int(st.get("index_size") or 0) > 0:
                                cat_warm.append({"category": cid, "index_size": st.get("index_size")})
                        except Exception as cat_exc:  # noqa: BLE001
                            print(f"[startup] category warmup skip {cid}: {cat_exc}")
            except Exception as cat_outer:  # noqa: BLE001
                print(f"[startup] category warmup unavailable: {cat_outer}")

        actual_device = device or "cpu"
        try:
            import torch
            if torch.cuda.is_available() and actual_device != "cpu":
                gpu_name = torch.cuda.get_device_name(0)
                print(f"[startup] Device: {actual_device} ({gpu_name})")
            else:
                print("[startup] Device: cpu" + (" (CUDA available but not selected)" if torch.cuda.is_available() else ""))
        except ImportError:
            print(f"[startup] Device: {actual_device} (torch not found, cannot detect GPU)")
        print(f"[startup] Embedding: {embedding_model}")
        print(f"[startup] Reranker: {rerank_model if enable_reranker else 'disabled'}")
        print(f"[startup] BM25: {'enabled' if enable_bm25 else 'disabled'}")
        print(f"[startup] Index: {stats['index_size']} chunks")
        if cat_warm:
            print(f"[startup] Category indexes warmed: {len(cat_warm)}")
        _WARMUP_INFO = {
            "status": "ok",
            "index_size": stats.get("index_size"),
            "category_indexes": len(cat_warm),
        }
    except Exception as exc:  # noqa: BLE001
        print("[startup] RAG warmup failed; continuing without preloaded cache.")
        print(f"[startup] Warmup error: {exc}")
        print(traceback.format_exc())
        _WARMUP_INFO = {"status": "failed", "error": str(exc)}
    finally:
        _READY = True
        print("[startup] Readiness: /ready is now serving 200.")


def _run_rag_warmup() -> None:
    limits = get_resource_limits()
    try:
        # Warmup is a one-off background job and may legitimately wait behind
        # an early request longer than the normal request acquire timeout.
        with limits.rag.slot(timeout_s=max(limits.acquire_timeout_s, 300.0)):
            _run_rag_warmup_unlimited()
    except ResourceBusyError as exc:
        global _READY, _WARMUP_INFO
        _WARMUP_INFO = {"status": "failed", "error": str(exc)}
        _READY = True
        print(f"[startup] RAG warmup skipped: {exc}")


@app.on_event("startup")
async def _startup() -> None:
    global _READY, _WARMUP_INFO
    configure_resource_limits()
    load_api_keys()
    _init_qa_db()
    await start_memory_client()

    reg = get_registry()
    if reg.get("rag.search") is None:
        register_builtin_tools(reg)
        register_debug_tools(reg)
        if os.getenv("AGENT_ENABLE_MCP", "1") == "1":
            await register_mcp_tools_async(reg)

    await start_session_cleanup_task()

    if os.getenv("AGENT_WARMUP_RAG", "1") == "1":
        # Warm up off the startup path so uvicorn finishes startup immediately
        # and /health (liveness) responds right away; /ready flips when done.
        threading.Thread(target=_run_rag_warmup, name="rag-warmup", daemon=True).start()
    else:
        _WARMUP_INFO = {"status": "disabled"}
        _READY = True


@app.on_event("shutdown")
async def _shutdown() -> None:
    await stop_session_cleanup_task()
    await close_memory_client()
    # Release the shared LLM httpx connection pools.
    await aclose_shared_async_client()
    await aclose_shared_async_stream_client()
    # Close pooled MySQL connections used by sql.search / vitals.summary.
    try:
        from .sql_search.pool import close_pool

        close_pool()
    except Exception:  # noqa: BLE001
        pass


@app.get("/health")
def health() -> Dict[str, Any]:
    """Liveness probe — up as soon as the process is running."""
    return {"ok": True}


@app.get("/ready")
def ready() -> JSONResponse:
    """Readiness probe — 200 once RAG warmup finished, 503 while still warming."""
    status = 200 if _READY else 503
    return JSONResponse(
        status_code=status,
        content={
            "ready": _READY,
            "warmup": _WARMUP_INFO,
            "resource_limits": get_resource_limits().snapshot(),
            "memory": memory_status(),
        },
    )


@app.post("/tools/rag/search", response_model=ToolResponse)
def rag_search(req: RagSearchRequest) -> ToolResponse:
    trace_id = new_trace_id()
    try:
        data = rag_search_tool(**req.model_dump())
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="rag.search", request=req.model_dump(), response=resp.model_dump())
        return resp
    except ResourceBusyError as e:
        write_trace(trace_id, tool="rag.search", request=req.model_dump(), response=e.as_dict(), error=str(e))
        raise HTTPException(status_code=503, detail=e.as_dict(), headers={"Retry-After": str(max(1, math.ceil(e.timeout_s)))}) from e
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
    except ResourceBusyError as e:
        write_trace(trace_id, tool="rag.reindex", request=req.model_dump(), response=e.as_dict(), error=str(e))
        raise HTTPException(status_code=503, detail=e.as_dict(), headers={"Retry-After": str(max(1, math.ceil(e.timeout_s)))}) from e
    except Exception as e:  # noqa: BLE001
        err = {"code": "RAG_REINDEX_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="rag.reindex", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post("/tools/sql/search", response_model=ToolResponse)
def sql_search(request: Request, req: SqlSearchRequest) -> ToolResponse:
    trace_id = new_trace_id()
    try:
        set_request_animal_id(body_animal_id=req.animal_id, header_animal_id=request.headers.get("x-animal-id"))
        data = sql_search_tool(**req.model_dump(exclude={"animal_id"}))
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="sql.search", request=req.model_dump(), response=resp.model_dump())
        return resp
    except Exception as e:  # noqa: BLE001
        err = {"code": "SQL_SEARCH_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="sql.search", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.get("/tools", response_model=ToolListResponse)
def tools_list(request: Request) -> ToolListResponse:
    trace_id = new_trace_id()
    set_request_animal_id(header_animal_id=request.headers.get("x-animal-id"))
    reg = get_registry()
    raw = reg.list_tools()
    if not get_request_animal_id():
        raw = [t for t in raw if t.name not in ANIMAL_REQUIRED_TOOLS]
    tools = [ToolSpecOut(name=t.name, description=t.description, input_schema=t.input_schema) for t in raw]
    resp = ToolListResponse(ok=True, trace_id=trace_id, tools=tools)
    write_trace(trace_id, tool="tools.list", request={}, response=resp.model_dump())
    return resp


@app.post("/tools/call", response_model=ToolResponse)
async def tools_call(request: Request, req: ToolCallRequest) -> ToolResponse:
    trace_id = new_trace_id()
    set_request_animal_id(body_animal_id=req.animal_id, header_animal_id=request.headers.get("x-animal-id"))
    reg = get_registry()
    if req.tool_name == "sql.search" and not get_request_animal_id():
        err = {"code": "ANIMAL_ID_REQUIRED", "message": "sql.search requires animal_id (JSON field or X-Animal-Id header)."}
        return ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
    try:
        data = await reg.call(req.tool_name, req.arguments)
        resp = ToolResponse(ok=True, trace_id=trace_id, data=data)
        write_trace(trace_id, tool="tools.call", request=req.model_dump(), response=resp.model_dump())
        return resp
    except ResourceBusyError as e:
        write_trace(trace_id, tool="tools.call", request=req.model_dump(), response=e.as_dict(), error=str(e))
        raise HTTPException(status_code=503, detail=e.as_dict(), headers={"Retry-After": str(max(1, math.ceil(e.timeout_s)))}) from e
    except Exception as e:  # noqa: BLE001
        err = {"code": "TOOL_CALL_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = ToolResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="tools.call", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post("/agent/plan_and_solve", response_model=AgentPlanAndSolveResponse)
async def agent_plan_and_solve(request: Request, req: AgentPlanAndSolveRequest) -> AgentPlanAndSolveResponse:
    trace_id = new_trace_id()
    try:
        set_request_animal_id(body_animal_id=req.animal_id, header_animal_id=request.headers.get("x-animal-id"))
        reg = get_registry()
        # Reuse the shared pooled client for the default config; only build a
        # throwaway client (and close it) when the caller overrides LLM config.
        use_custom_llm = any([req.llm_base_url, req.llm_api_key, req.llm_model])
        llm = (
            AsyncOpenAIClient(base_url=req.llm_base_url, api_key=req.llm_api_key, model=req.llm_model)
            if use_custom_llm
            else get_shared_async_client()
        )
        agent = AsyncPlanAndSolveAgent(registry=reg, llm=llm)
        allowed_tools = req.allowed_tools
        if allowed_tools is not None:
            allowed_tools = filter_tools_without_animal(allowed_tools)
        try:
            plan = await agent.plan(query=req.query, allowed_tools=allowed_tools)
            answer, tool_results = await agent.solve(
                query=req.query,
                plan_steps=plan,
                allowed_tools=allowed_tools,
                temperature=req.temperature,
                max_tokens=req.max_tokens,
            )
        finally:
            if use_custom_llm:
                await llm.close()
        resp = AgentPlanAndSolveResponse(ok=True, trace_id=trace_id, answer=answer, plan=plan, tool_results=tool_results)
        write_trace(trace_id, tool="agent.plan_and_solve", request=req.model_dump(), response=resp.model_dump())
        return resp
    except ResourceBusyError as e:
        write_trace(trace_id, tool="agent.plan_and_solve", request=req.model_dump(), response=e.as_dict(), error=str(e))
        raise HTTPException(status_code=503, detail=e.as_dict(), headers={"Retry-After": str(max(1, math.ceil(e.timeout_s)))}) from e
    except Exception as e:  # noqa: BLE001
        err = {"code": "AGENT_PLAN_SOLVE_FAILED", "message": str(e), "detail": {"traceback": traceback.format_exc()}}
        resp = AgentPlanAndSolveResponse(ok=False, trace_id=trace_id, error=err)  # type: ignore[arg-type]
        write_trace(trace_id, tool="agent.plan_and_solve", request=req.model_dump(), response=resp.model_dump(), error=str(e))
        return resp


@app.post(
    "/sessions",
    deprecated=True,
    tags=["Legacy test sessions"],
    summary="Create a legacy test session",
)
async def create_session():
    """Legacy diagnostics only; `/v1/chat/completions` never reads this session."""
    mgr = get_session_manager()
    sess = await mgr.create()
    return {"ok": True, "session_id": sess.session_id}


@app.get(
    "/sessions/{session_id}",
    deprecated=True,
    tags=["Legacy test sessions"],
    summary="Inspect a legacy test session",
)
async def get_session(session_id: str):
    """Inspect test state. This endpoint is not production conversation memory."""
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


@app.delete(
    "/sessions/{session_id}",
    deprecated=True,
    tags=["Legacy test sessions"],
    summary="Delete a legacy test session",
)
async def delete_session(session_id: str):
    """Delete a test session without affecting stateless production requests."""
    mgr = get_session_manager()
    deleted = await mgr.delete(session_id)
    return {"ok": deleted}


# ---------------------------------------------------------------------------
# QA management endpoints — admin token required
# ---------------------------------------------------------------------------

_QA_ADMIN_TOKEN = os.getenv("QA_ADMIN_TOKEN", "")


async def _require_admin(request: Request) -> None:
    """Verify admin access via X-Admin-Token header.

    Separate from the general API key auth so it is never bypassed
    by AGENT_DISABLE_AUTH.  Rejects requests without the correct token
    with 403 — external users through frp do not know this token.
    """
    if not _QA_ADMIN_TOKEN:
        raise HTTPException(status_code=403, detail="QA admin token not configured on server")
    token = request.headers.get("X-Admin-Token", "").strip()
    if token != _QA_ADMIN_TOKEN:
        raise HTTPException(status_code=403, detail="Invalid or missing admin token")


@app.get("/qa/history", dependencies=[Depends(_require_admin)])
async def qa_history(
    page: int = Query(1, ge=1),
    page_size: int = Query(20, ge=1, le=100),
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
    keyword: Optional[str] = Query(None),
) -> Dict[str, Any]:
    data = await query_qa_history(
        page=page, page_size=page_size,
        date_from=date_from, date_to=date_to, keyword=keyword,
    )
    return {"ok": True, **data}


@app.get("/qa/stats", dependencies=[Depends(_require_admin)])
async def qa_stats(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
) -> Dict[str, Any]:
    data = await get_qa_stats(date_from=date_from, date_to=date_to)
    return {"ok": True, **data}


@app.get("/qa/knowledge-gaps", dependencies=[Depends(_require_admin)])
async def qa_knowledge_gaps(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
    min_occurrences: int = Query(1, ge=1),
    limit: int = Query(50, ge=1, le=200),
) -> Dict[str, Any]:
    data = await get_knowledge_gaps(
        date_from=date_from, date_to=date_to,
        min_occurrences=min_occurrences, limit=limit,
    )
    return {"ok": True, **data}


# ---------------------------------------------------------------------------
# Feedback endpoints
# ---------------------------------------------------------------------------

@app.post("/qa/feedback")
async def qa_feedback(request: Request) -> Dict[str, Any]:
    """Public endpoint — any user can submit feedback for an answer."""
    try:
        body = await request.json()
    except Exception:
        raise HTTPException(status_code=400, detail="Invalid JSON body")
    request_id = (body.get("request_id") or "").strip()
    rating = body.get("rating")
    comment = (body.get("comment") or "").strip()
    if not request_id:
        raise HTTPException(status_code=400, detail="request_id is required")
    if not isinstance(rating, int) or rating < 1 or rating > 5:
        raise HTTPException(status_code=400, detail="rating must be an integer between 1 and 5")
    ok = await submit_feedback(request_id=request_id, rating=rating, comment=comment)
    if not ok:
        raise HTTPException(status_code=404, detail="Record not found or already rated")
    return {"ok": True}


@app.get("/qa/feedback-stats", dependencies=[Depends(_require_admin)])
async def qa_feedback_stats(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
) -> Dict[str, Any]:
    data = await get_feedback_stats(date_from=date_from, date_to=date_to)
    return {"ok": True, **data}
