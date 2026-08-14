from __future__ import annotations

from contextlib import asynccontextmanager
from typing import Any, AsyncIterator, Optional

from sqlalchemy.ext.asyncio import AsyncEngine, AsyncSession, async_sessionmaker, create_async_engine

from .config import get_platform_settings
from .models import Base

_engine: Optional[AsyncEngine] = None
_session_factory: Optional[async_sessionmaker[AsyncSession]] = None


def _normalize_url(url: str) -> str:
    if url.startswith("postgresql://"):
        return "postgresql+asyncpg://" + url[len("postgresql://"):]
    return url


def get_platform_engine() -> AsyncEngine:
    global _engine, _session_factory
    if _engine is None:
        settings = get_platform_settings()
        normalized_url = _normalize_url(settings.database_url)
        options: dict[str, Any] = {"pool_pre_ping": True, "future": True}
        if not normalized_url.startswith("sqlite"):
            options.update(pool_size=settings.database_pool_size, max_overflow=settings.database_max_overflow)
        _engine = create_async_engine(normalized_url, **options)
        _session_factory = async_sessionmaker(_engine, expire_on_commit=False)
    return _engine


def get_platform_session_factory() -> async_sessionmaker[AsyncSession]:
    get_platform_engine()
    assert _session_factory is not None
    return _session_factory


async def get_platform_session() -> AsyncIterator[AsyncSession]:
    factory = get_platform_session_factory()
    async with factory() as session:
        yield session


@asynccontextmanager
async def platform_session() -> AsyncIterator[AsyncSession]:
    factory = get_platform_session_factory()
    async with factory() as session:
        yield session


async def init_platform_database() -> None:
    settings = get_platform_settings()
    if not settings.enabled:
        return
    engine = get_platform_engine()
    if settings.auto_create_schema:
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)


async def close_platform_database() -> None:
    global _engine, _session_factory
    if _engine is not None:
        await _engine.dispose()
    _engine = None
    _session_factory = None
