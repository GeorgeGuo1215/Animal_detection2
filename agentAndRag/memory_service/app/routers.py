"""HTTP 路由。

路由函数是同步的（``def`` 而非 ``async def``），FastAPI 会把它们放进线程池执行，
正好匹配同步的数据库连接池。写成 async 再在里面做阻塞查询反而会卡住事件循环。

写入路径不做任何 LLM 调用：落一行短期记忆、入一条队，立即返回。所有耗时处理
都在 worker 里发生，chat 请求不会因为记忆系统而变慢。
"""

from __future__ import annotations

import logging

import psycopg
from fastapi import APIRouter, HTTPException

from . import db
from .config import MemoryConfig
from .embedding import get_embedder
from .memory import consolidator, long_term, mid_term, queue, retriever, short_term, subjects
from .schemas import (
    ContextIn,
    ContextOut,
    HealthOut,
    MessageIn,
    MessageOut,
    ProfileOut,
    StatsOut,
    SubjectIn,
    SubjectOut,
)

logger = logging.getLogger(__name__)


def build_router(cfg: MemoryConfig, worker_pool=None) -> APIRouter:
    router = APIRouter()

    @router.post("/v1/memory/subjects/ensure", response_model=SubjectOut, tags=["memory"])
    def ensure_subject(payload: SubjectIn) -> SubjectOut:
        try:
            with db.connection() as conn:
                record = subjects.ensure(
                    conn,
                    user_id=payload.user_id,
                    display_name=payload.display_name,
                    source=payload.source,
                    metadata=payload.metadata,
                )
        except db.MemoryDbError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return SubjectOut(
            user_id=record["id"],
            display_name=record["displayName"],
            source=record["source"],
            metadata=record["metadata"] or {},
            created_at=record["createdAt"],
            updated_at=record["updatedAt"],
        )

    @router.post("/v1/memory/messages", response_model=MessageOut, tags=["memory"])
    def write_message(payload: MessageIn) -> MessageOut:
        from datetime import datetime

        now = datetime.now()
        try:
            with db.connection() as conn:
                subjects.ensure(conn, user_id=payload.user_id)
                message_id, created = short_term.append_once(
                    conn,
                    user_id=payload.user_id,
                    user_input=payload.user_input,
                    agent_response=payload.agent_response,
                    pet_id=payload.pet_id,
                    session_id=payload.session_id,
                    turn_id=payload.turn_id,
                    created_at=now,
                )
                size = short_term.count(conn, payload.user_id)
                # 攒够一批才派活，避免每轮对话都触发一次带 LLM 的提升流程。
                queued = False
                if created and consolidator.should_promote(size, cfg):
                    queued = queue.enqueue(conn, user_id=payload.user_id, now=now) is not None
        except psycopg.errors.ForeignKeyViolation as exc:
            raise HTTPException(status_code=404, detail=f"用户不存在: {payload.user_id}") from exc
        except db.MemoryDbError as exc:
            logger.exception("memory_service: write failed")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

        return MessageOut(
            id=message_id,
            queued=queued,
            short_term_size=size,
            duplicate=not created,
        )

    @router.post("/v1/memory/context", response_model=ContextOut, tags=["memory"])
    def read_context(payload: ContextIn) -> ContextOut:
        from datetime import datetime

        try:
            with db.connection() as conn:
                context = retriever.build_context(
                    conn,
                    user_id=payload.user_id,
                    query=payload.query,
                    cfg=cfg,
                    embedder=get_embedder(),
                    now=datetime.now(),
                    pet_id=payload.pet_id,
                )
        except db.MemoryDbError as exc:
            logger.exception("memory_service: context retrieval failed")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

        return ContextOut(
            **context,
            text=retriever.format_context(context) if payload.include_text else None,
        )

    @router.get("/v1/memory/profile/{user_id}", response_model=ProfileOut, tags=["memory"])
    def read_profile(user_id: str) -> ProfileOut:
        try:
            with db.connection() as conn:
                record = long_term.get_profile(conn, user_id)
        except db.MemoryDbError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

        return ProfileOut(
            user_id=user_id,
            profile=record["profile"],
            version=record["version"],
            updated_at=record["updatedAt"],
        )

    @router.get("/v1/memory/stats/{user_id}", response_model=StatsOut, tags=["memory"])
    def read_stats(user_id: str) -> StatsOut:
        from datetime import datetime

        try:
            with db.connection() as conn:
                heat = mid_term.heat_distribution(
                    conn, user_id=user_id, now=datetime.now(), params=cfg.heat
                )
                return StatsOut(
                    user_id=user_id,
                    short_term=short_term.count(conn, user_id),
                    segments=mid_term.count_segments(conn, user_id),
                    knowledge=long_term.count_knowledge(conn, user_id),
                    heat=heat,
                )
        except db.MemoryDbError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router.get("/health", response_model=HealthOut, tags=["ops"])
    def health() -> HealthOut:
        """探活。数据库不通时返回 degraded 而不是 5xx，方便编排区分"进程活着但依赖挂了"。"""
        try:
            with db.connection() as conn:
                conn.execute("SELECT 1")
                queue_stats = queue.stats(conn)
            database = "ok"
        except Exception as exc:  # noqa: BLE001
            logger.warning("memory_service: health check failed: %s", exc)
            return HealthOut(status="degraded", database="unavailable")

        return HealthOut(
            status="ok",
            database=database,
            queue=queue_stats,
            workers=worker_pool.stats if worker_pool else {},
        )

    return router
