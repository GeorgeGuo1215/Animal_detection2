from __future__ import annotations

from contextlib import asynccontextmanager
from typing import Any, AsyncIterator, Optional

from sqlalchemy.ext.asyncio import AsyncEngine, AsyncSession, async_sessionmaker, create_async_engine

from .config import get_platform_settings
from .models import Base

_engine: Optional[AsyncEngine] = None
_session_factory: Optional[async_sessionmaker[AsyncSession]] = None


def _normalize_url(url: str) -> str:
    """将 ``postgresql://`` 规范化为 SQLAlchemy 异步驱动 ``postgresql+asyncpg://``。"""
    if url.startswith("postgresql://"):
        return "postgresql+asyncpg://" + url[len("postgresql://"):]
    return url


def get_platform_engine() -> AsyncEngine:
    """懒创建并返回平台异步引擎；非 SQLite 时应用连接池配置。"""
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
    """返回异步 Session 工厂；必要时先初始化引擎。"""
    get_platform_engine()
    assert _session_factory is not None
    return _session_factory


async def get_platform_session() -> AsyncIterator[AsyncSession]:
    """FastAPI 依赖：产出一个请求级 AsyncSession。"""
    factory = get_platform_session_factory()
    async with factory() as session:
        yield session


@asynccontextmanager
async def platform_session() -> AsyncIterator[AsyncSession]:
    """上下文管理器：在服务代码中临时打开一个 AsyncSession。"""
    factory = get_platform_session_factory()
    async with factory() as session:
        yield session


async def init_platform_database() -> None:
    """平台启用且允许自动建表时，用 ``create_all`` 初始化 schema。"""
    settings = get_platform_settings()
    if not settings.enabled:
        return
    engine = get_platform_engine()
    if settings.auto_create_schema:
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)


async def close_platform_database() -> None:
    """释放引擎连接池并清空模块级单例，供应用关闭时调用。"""
    global _engine, _session_factory
    if _engine is not None:
        await _engine.dispose()
    _engine = None
    _session_factory = None
