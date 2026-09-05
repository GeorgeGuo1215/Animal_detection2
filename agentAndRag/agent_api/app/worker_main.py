from __future__ import annotations

import asyncio
import hmac
import os
from typing import Any

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from .concurrency import configure_resource_limits, get_resource_limits
from .integrations.llm.client import aclose_shared_async_client
from .memory import close_memory_client, memory_status, start_memory_client
from .features.qa_audit.repository import init_db as init_qa_db
from .platform.database import close_platform_database, init_platform_database
from .platform.runs import close_run_queue_redis, worker_forever
from .features.chat_moe.router import chat_moe_router
from .routers.routes_openai import router as openai_router
from .runtime_warmup import warmup_rag_runtime
from .tools.tool_registry import get_registry
from .tools.builtin import register_builtin_tools, register_debug_tools
from .tools.tools_mcp import register_mcp_tools_async
from .worker_proxy import worker_token


app = FastAPI(title="PetMind Internal Agent Worker", docs_url=None, redoc_url=None, openapi_url=None)
app.include_router(openai_router)
app.include_router(chat_moe_router)

_READY = False
_WARMUP_INFO: dict[str, Any] = {"status": "pending"}
_QUEUE_TASK: asyncio.Task[None] | None = None


def _shutdown_timeout_s() -> float:
    """读取 Worker 子系统关闭等待上限。"""
    try:
        return max(1.0, float(os.getenv("AGENT_SHUTDOWN_TIMEOUT_SEC", "10")))
    except (TypeError, ValueError):
        return 10.0


@app.middleware("http")
async def authenticate_internal_request(request: Request, call_next):
    """校验内部 Worker Token；健康检查除外，未就绪时返回 503。"""
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
    """初始化数据库、记忆、工具、RAG 预热与平台队列消费者。"""
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
    """启动时先完成模型/工具初始化，再接受执行流量；失败则让进程退出以便 supervisor 重启。"""
    await _initialize_worker()


@app.on_event("shutdown")
async def shutdown() -> None:
    """停止队列消费者并关闭记忆、LLM 与平台数据库连接。"""
    global _READY
    from .observability.jsonl_trace import close_trace_writer
    await close_trace_writer()
    timeout_s = _shutdown_timeout_s()
    _READY = False
    if _QUEUE_TASK is not None and not _QUEUE_TASK.done():
        _QUEUE_TASK.cancel()
    if _QUEUE_TASK is not None:
        try:
            await asyncio.wait_for(
                asyncio.gather(_QUEUE_TASK, return_exceptions=True),
                timeout=timeout_s,
            )
        except (TimeoutError, asyncio.TimeoutError):
            print(
                f"[worker] Queue consumer did not stop within {timeout_s:g}s; continuing shutdown.",
                flush=True,
            )
    await asyncio.wait_for(close_run_queue_redis(), timeout=timeout_s)
    await asyncio.wait_for(close_memory_client(), timeout=timeout_s)
    await asyncio.wait_for(aclose_shared_async_client(), timeout=timeout_s)
    await asyncio.wait_for(close_platform_database(), timeout=timeout_s)


@app.get("/health")
async def health() -> dict[str, Any]:
    """存活探针：进程已启动即返回 ok。"""
    return {"ok": True, "role": "worker"}


@app.get("/ready")
async def ready() -> JSONResponse:
    """就绪探针：预热完成且队列消费者存活时返回 200，否则 503。"""
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
