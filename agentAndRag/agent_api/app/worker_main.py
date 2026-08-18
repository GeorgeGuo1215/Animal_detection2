from __future__ import annotations

import asyncio
import hmac
import os
from typing import Any

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from .concurrency import configure_resource_limits, get_resource_limits
from .llm.llm_client import aclose_shared_async_client
from .llm.llm_client_stream import aclose_shared_async_stream_client
from .memory import close_memory_client, memory_status, start_memory_client
from .persistence.qa_store import init_db as init_qa_db
from .platform.database import close_platform_database, init_platform_database
from .platform.run_service import worker_forever
from .routers.routes_chat_ui import chat_moe_router
from .routers.routes_openai import router as openai_router
from .runtime_warmup import warmup_rag_runtime
from .tools.tool_registry import get_registry
from .tools.tools_builtin import register_builtin_tools, register_debug_tools
from .tools.tools_mcp import register_mcp_tools_async
from .worker_proxy import worker_token


app = FastAPI(title="PetMind Internal Agent Worker", docs_url=None, redoc_url=None, openapi_url=None)
app.include_router(openai_router)
app.include_router(chat_moe_router)

_READY = False
_WARMUP_INFO: dict[str, Any] = {"status": "pending"}
_QUEUE_TASK: asyncio.Task[None] | None = None


@app.middleware("http")
async def authenticate_internal_request(request: Request, call_next):
    if request.url.path not in {"/health", "/ready"}:
        supplied = request.headers.get("x-petmind-worker-token", "")
        if not supplied or not hmac.compare_digest(supplied, worker_token()):
            return JSONResponse(status_code=401, content={"detail": "invalid worker token"})
        if not _READY:
            return JSONResponse(
                status_code=503,
                content={"detail": "agent worker is warming up"},
                headers={"Retry-After": "3"},
            )
    return await call_next(request)


async def _initialize_worker() -> None:
    global _READY, _WARMUP_INFO, _QUEUE_TASK
    configure_resource_limits()
    init_qa_db()
    await init_platform_database()
    await start_memory_client()
    registry = get_registry()
    if registry.get("rag.search") is None:
        register_builtin_tools(registry)
        register_debug_tools(registry)
        if os.getenv("AGENT_ENABLE_MCP", "1") == "1":
            await register_mcp_tools_async(registry)

    if os.getenv("AGENT_WARMUP_RAG", "1") == "1":
        _WARMUP_INFO = await asyncio.to_thread(warmup_rag_runtime)
    else:
        _WARMUP_INFO = {"status": "disabled"}
    _QUEUE_TASK = asyncio.create_task(worker_forever(), name="platform-worker-queue")
    _READY = True
    print("[worker] Agent execution and Redis queue consumer are ready.", flush=True)


@app.on_event("startup")
async def startup() -> None:
    # Complete model/tool initialization before Uvicorn accepts execution
    # traffic. An initialization failure terminates the Worker so the service
    # supervisor can restart it instead of leaving a permanently unready shell.
    await _initialize_worker()


@app.on_event("shutdown")
async def shutdown() -> None:
    global _READY
    _READY = False
    if _QUEUE_TASK is not None and not _QUEUE_TASK.done():
        _QUEUE_TASK.cancel()
    if _QUEUE_TASK is not None:
        try:
            await asyncio.wait_for(
                asyncio.gather(_QUEUE_TASK, return_exceptions=True),
                timeout=10.0,
            )
        except TimeoutError:
            print("[worker] Queue consumer did not stop within 10s; continuing shutdown.", flush=True)
    await asyncio.wait_for(close_memory_client(), timeout=10.0)
    await asyncio.wait_for(aclose_shared_async_client(), timeout=10.0)
    await asyncio.wait_for(aclose_shared_async_stream_client(), timeout=10.0)
    await asyncio.wait_for(close_platform_database(), timeout=10.0)


@app.get("/health")
async def health() -> dict[str, Any]:
    return {"ok": True, "role": "worker"}


@app.get("/ready")
async def ready() -> JSONResponse:
    queue_alive = _QUEUE_TASK is not None and not _QUEUE_TASK.done()
    ready_now = _READY and queue_alive
    return JSONResponse(
        status_code=200 if ready_now else 503,
        content={
            "ready": ready_now,
            "role": "worker",
            "queue_consumer": queue_alive,
            "warmup": _WARMUP_INFO,
            "memory": memory_status(),
            "resource_limits": get_resource_limits().snapshot(),
        },
    )
