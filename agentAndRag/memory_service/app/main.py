"""FastAPI 入口。

服务本身无状态，可以起多个实例；后台 worker 随进程一起启动，靠数据库层面的
SKIP LOCKED 与咨询锁保证多实例不重复处理。只想跑 API 不想带 worker 时，
把 MEMORY_WORKER_CONCURRENCY 设成 0 之外的做法是单独起纯 worker 进程
（见 scripts/run_worker.py）。
"""

from __future__ import annotations

import logging
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI

from . import db
from .config import MemoryConfig, load_config
from .embedding import init_embedder
from .llm import init_llm
from .routers import build_router
from .worker import WorkerPool

logger = logging.getLogger(__name__)


def _warmup_embedding_enabled() -> bool:
    return os.getenv("MEMORY_WARMUP_EMBEDDING", "1").strip().lower() in {
        "1", "true", "yes", "on",
    }


def create_app(cfg: MemoryConfig | None = None, *, with_workers: bool = True) -> FastAPI:
    config = cfg or load_config()
    embedder = init_embedder(config)
    llm = init_llm(config)
    pool = WorkerPool(config, embedder, llm) if with_workers else None

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        db.init_pool(config)
        logger.info(
            "memory_service: embedding=%s device=%s dim=%s",
            config.embedding_model, config.embedding_device, config.embedding_dim,
        )
        # `/health` must mean the service can serve a context request, not merely
        # that PostgreSQL is reachable.  Lazy model loading otherwise makes the
        # first required-memory request time out after the stack reports ready.
        if _warmup_embedding_enabled():
            embedder.embed_query("memory service readiness warmup")
            logger.info("memory_service: embedding warmup complete")
        if pool:
            pool.start()
        try:
            yield
        finally:
            if pool:
                pool.stop()
            db.close_pool()

    app = FastAPI(
        title="Pet Memory Service",
        description="PetMind记忆系统",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.include_router(build_router(config, worker_pool=pool))
    return app


app = create_app()


def main() -> None:
    import os

    import uvicorn

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    uvicorn.run(
        app,
        host=os.getenv("MEMORY_HOST", "0.0.0.0"),
        port=int(os.getenv("MEMORY_PORT", "8300")),
    )


if __name__ == "__main__":
    main()
