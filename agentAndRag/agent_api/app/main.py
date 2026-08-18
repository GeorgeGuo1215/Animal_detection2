from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

# Repo root (Animal_detection/) so `import integration` works when running from agentAndRag
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
from typing import Any, Dict, Optional

from fastapi import Depends, FastAPI, HTTPException, Query, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.responses import JSONResponse

from .concurrency import configure_resource_limits, get_resource_limits
from .llm.llm_client import aclose_shared_async_client
from .llm.llm_client_stream import aclose_shared_async_stream_client
from .lifecycle_tasks.session_cleanup import (
    start_session_cleanup_task,
    stop_session_cleanup_task,
)
from .middleware.auth import APIKeyAuthMiddleware, load_api_keys
from .middleware.platform_http import PlatformRequestMiddleware
from .middleware.platform_rate_limit import PlatformRateLimitMiddleware
from .middleware.rate_limit import RateLimitMiddleware
from .memory import close_memory_client, memory_status, start_memory_client
from .persistence.qa_store import (
    get_feedback_stats, get_knowledge_gaps, get_qa_stats,
    init_db as _init_qa_db, query_qa_history, submit_feedback,
)
from .routers.routes_chat_ui import chat_moe_router
from .routers.routes_openai import router as openai_router
from .routers.routes_platform_admin import router as platform_admin_router
from .routers.routes_platform_auth import router as platform_auth_router
from .routers.routes_platform_core import router as platform_core_router
from .tools.tool_registry import get_registry
from .tools.tools_builtin import register_builtin_tools, register_debug_tools
from .tools.tools_mcp import register_mcp_tools_async
from .runtime_warmup import warmup_rag_runtime
from .worker_proxy import should_delegate_agent_execution, worker_base_url, worker_readiness

from integration.api.routes_ingest import router as integration_ingest_router
from .platform import close_platform_database, get_platform_settings, init_platform_database
from .platform.cleanup import start_platform_cleanup_task, stop_platform_cleanup_task
from .platform.database import platform_session
from .platform.services import seed_platform_plans, seed_platform_rbac


_PLATFORM_SETTINGS = get_platform_settings()
app = FastAPI(
    title="PetMind Agent API",
    version="1.0.0",
    docs_url=None if _PLATFORM_SETTINGS.production else "/docs",
    redoc_url=None if _PLATFORM_SETTINGS.production else "/redoc",
    openapi_url=None if _PLATFORM_SETTINGS.production else "/openapi.json",
)

app.include_router(openai_router)
app.include_router(chat_moe_router)
app.include_router(integration_ingest_router, prefix="/integration", tags=["integration"])
app.include_router(platform_auth_router)
app.include_router(platform_core_router)
app.include_router(platform_admin_router)

app.add_middleware(APIKeyAuthMiddleware)

_rl_rate = float(os.getenv("AGENT_RATE_LIMIT", "30"))
_rl_burst = int(os.getenv("AGENT_RATE_BURST", str(int(_rl_rate))))
app.add_middleware(RateLimitMiddleware, rate=_rl_rate, burst=_rl_burst)
app.add_middleware(PlatformRateLimitMiddleware)
app.add_middleware(PlatformRequestMiddleware)

if _PLATFORM_SETTINGS.production:
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=list(_PLATFORM_SETTINGS.allowed_hosts))


def _platform_error(request: Request, *, status_code: int, code: str, message: str, details=None) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content={
            "code": code,
            "message": message,
            "request_id": getattr(request.state, "request_id", ""),
            "details": details,
        },
    )


@app.exception_handler(RequestValidationError)
async def _validation_error(request: Request, exc: RequestValidationError):
    if not request.url.path.startswith("/api/v1/"):
        return JSONResponse(status_code=422, content={"detail": exc.errors()})
    fields = [{"path": ".".join(str(part) for part in item["loc"]), "type": item["type"], "message": item["msg"]} for item in exc.errors()]
    return _platform_error(request, status_code=422, code="validation_error", message="Request validation failed", details={"fields": fields})


@app.exception_handler(HTTPException)
async def _http_error(request: Request, exc: HTTPException):
    if not request.url.path.startswith("/api/v1/"):
        return JSONResponse(status_code=exc.status_code, content={"detail": exc.detail}, headers=exc.headers)
    if isinstance(exc.detail, dict):
        code = str(exc.detail.get("code") or "request_failed")
        message = str(exc.detail.get("message") or "Request failed")
        details = exc.detail.get("details")
    else:
        code = str(exc.detail).lower().replace(" ", "_")[:80]
        message = str(exc.detail)
        details = None
    response = _platform_error(request, status_code=exc.status_code, code=code, message=message, details=details)
    if exc.headers:
        response.headers.update(exc.headers)
    return response

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
    if _PLATFORM_SETTINGS.production:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=[_PLATFORM_SETTINGS.frontend_origin],
            allow_credentials=True,
            allow_methods=["GET", "POST", "PATCH", "DELETE", "OPTIONS"],
            allow_headers=["Content-Type", "Authorization", "Accept", "X-API-Key", "Idempotency-Key", "Last-Event-ID", "X-Request-Id"],
            expose_headers=["X-Request-Id", "X-PetMind-Run-Id", "X-RateLimit-Limit", "X-RateLimit-Remaining", "Retry-After"],
        )
    elif _env_cors_origins == "*":
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["Content-Type", "Authorization", "Accept", "X-Requested-With", "X-Animal-Id", "X-User-Id", "X-API-Key", "Idempotency-Key", "Last-Event-ID", "X-Request-Id"],
            expose_headers=["X-Request-Id", "X-PetMind-Run-Id", "X-RateLimit-Limit", "X-RateLimit-Remaining", "Retry-After"],
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
            allow_headers=["Content-Type", "Authorization", "Accept", "X-Requested-With", "X-Animal-Id", "X-User-Id", "X-API-Key", "Idempotency-Key", "Last-Event-ID", "X-Request-Id"],
            expose_headers=["X-Request-Id", "X-PetMind-Run-Id", "X-RateLimit-Limit", "X-RateLimit-Remaining", "Retry-After"],
        )


# --- Readiness state --------------------------------------------------------
# Liveness (/health) is up the moment the process starts; readiness (/ready)
# flips true only after the (async) RAG warmup finishes, so an orchestrator
# won't route traffic into a cold instance whose first request would be slow.
_READY: bool = False
_WARMUP_INFO: Dict[str, Any] = {"status": "pending"}


def _run_rag_warmup() -> None:
    global _READY, _WARMUP_INFO
    _WARMUP_INFO = warmup_rag_runtime()
    _READY = True
    print("[startup] Readiness: /ready is now serving 200.")


@app.on_event("startup")
async def _startup() -> None:
    global _READY, _WARMUP_INFO
    configure_resource_limits()
    load_api_keys()
    _init_qa_db()
    await start_memory_client()
    settings = get_platform_settings()
    if settings.enabled:
        await init_platform_database()
        async with platform_session() as session:
            await seed_platform_plans(session)
            await seed_platform_rbac(session)
        await start_platform_cleanup_task()

    if not should_delegate_agent_execution():
        reg = get_registry()
        if reg.get("rag.search") is None:
            register_builtin_tools(reg)
            register_debug_tools(reg)
            if os.getenv("AGENT_ENABLE_MCP", "1") == "1":
                await register_mcp_tools_async(reg)

    await start_session_cleanup_task()

    if should_delegate_agent_execution():
        _WARMUP_INFO = {"status": "delegated", "worker": worker_base_url()}
        _READY = True
        print("[startup] Gateway mode: local RAG warmup disabled; Agent execution is delegated to Worker.")
    elif os.getenv("AGENT_WARMUP_RAG", "1") == "1":
        # Warm up off the startup path so uvicorn finishes startup immediately
        # and /health (liveness) responds right away; /ready flips when done.
        threading.Thread(target=_run_rag_warmup, name="rag-warmup", daemon=True).start()
    else:
        _WARMUP_INFO = {"status": "disabled"}
        _READY = True


@app.on_event("shutdown")
async def _shutdown() -> None:
    if get_platform_settings().enabled:
        await stop_platform_cleanup_task()
        await close_platform_database()
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
async def ready() -> JSONResponse:
    """Readiness probe — 200 once RAG warmup finished, 503 while still warming."""
    worker_ok = True
    worker_detail: dict[str, Any] | None = None
    if should_delegate_agent_execution():
        worker_ok, worker_detail = await worker_readiness()
    ready_now = _READY and worker_ok
    status = 200 if ready_now else 503
    return JSONResponse(
        status_code=status,
        content={
            "ready": ready_now,
            "role": "gateway" if should_delegate_agent_execution() else "standalone",
            "worker": worker_detail,
            "warmup": _WARMUP_INFO,
            "resource_limits": get_resource_limits().snapshot(),
            "memory": memory_status(),
        },
    )


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
