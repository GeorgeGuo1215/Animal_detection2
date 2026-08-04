"""数据库访问层。

对外只暴露一个 ``connection()`` 上下文管理器和几个锁辅助函数，SQL 留在各记忆模块里，
不做 ORM 或 Repository 抽象——表就六张，多包一层只会让人多跳一次文件。

``_new_pool`` 与 ``connection`` 都可以被测试替换，用来注入假连接或把整个用例包在一个
可回滚的事务里。
"""

from __future__ import annotations

import logging
import uuid
from contextlib import contextmanager
from typing import Any, Iterator, List, Optional, Sequence

import psycopg
from pgvector.psycopg import register_vector
from psycopg.rows import dict_row
from psycopg_pool import ConnectionPool, PoolTimeout

from .config import MemoryConfig

logger = logging.getLogger(__name__)


class MemoryDbError(RuntimeError):
    """数据库不可用或 SQL 执行失败，由调用方转成对外的错误状态。"""


def new_id() -> str:
    """生成主键。

    pet-server 侧用 Prisma 的 cuid，但那是客户端生成的，数据库列只是 TEXT，
    所以这里用 uuid 即可，不必为了视觉一致引入额外依赖。
    """
    return uuid.uuid4().hex


def to_vector_param(value: Any) -> Optional[List[float]]:
    """把各种形态的向量统一成可以作为查询参数的 float 列表。

    向量在这套代码里有三种来源：embedder 返回的 list、numpy 数组、以及从数据库
    读回来的 pgvector ``Vector`` 对象。最后一种在"把段摘要的向量复用到知识条目"
    这类路径上会直接被当成参数传回去，不归一化就会炸在参数适配上。
    """
    if value is None:
        return None
    if hasattr(value, "to_list"):
        value = value.to_list()
    elif hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, Sequence):
        return None
    items = [float(x) for x in value]
    return items or None


_pool: Optional[ConnectionPool] = None


def _configure_connection(conn: psycopg.Connection) -> None:
    """每条连接都要注册 vector 类型，否则 list[float] 无法直接作为参数传入。"""
    register_vector(conn)


def _new_pool(cfg: MemoryConfig) -> ConnectionPool:
    """建连接池。测试可替换此函数以避免真实连库。"""
    return ConnectionPool(
        conninfo=cfg.dsn,
        min_size=cfg.pool_min,
        max_size=max(cfg.pool_min, cfg.pool_max),
        timeout=cfg.connect_timeout,
        kwargs={"row_factory": dict_row, "connect_timeout": cfg.connect_timeout},
        configure=_configure_connection,
        open=False,
    )


def init_pool(cfg: MemoryConfig) -> ConnectionPool:
    global _pool
    if _pool is None:
        _pool = _new_pool(cfg)
        _pool.open()
        logger.info("memory_service: connection pool ready (%s)", cfg.redacted_dsn())
    return _pool


def close_pool() -> None:
    global _pool
    if _pool is not None:
        _pool.close()
        _pool = None


def get_pool() -> ConnectionPool:
    if _pool is None:
        raise MemoryDbError("connection pool is not initialised; call init_pool() first")
    return _pool


@contextmanager
def connection() -> Iterator[psycopg.Connection]:
    """借一条连接，异常时回滚。

    psycopg 的池连接默认在退出 with 时提交，所以正常路径不需要显式 commit。

    只有"数据库连不上"这类故障会被包成 MemoryDbError（对外是 503）。外键冲突、
    唯一约束这些数据问题原样抛出，好让调用方按语义区分——把它们一起包掉的话，
    传了个不存在的 userId 也会被报成服务不可用。
    """
    pool = get_pool()
    try:
        with pool.connection() as conn:
            yield conn
    except (psycopg.OperationalError, psycopg.InterfaceError, PoolTimeout) as exc:
        raise MemoryDbError(str(exc)) from exc


def try_user_lock(conn: psycopg.Connection, user_id: str) -> bool:
    """尝试取该用户的事务级咨询锁，用于串行化同一用户的记忆提升。

    取不到就返回 False 让调用方跳过——同一用户已经有 worker 在处理，重复处理会
    把同一批对话提升两次。锁随事务结束自动释放，worker 崩溃也不会留下死锁。
    """
    row = conn.execute(
        "SELECT pg_try_advisory_xact_lock(hashtext(%s)) AS locked", (user_id,)
    ).fetchone()
    return bool(row and row["locked"])


def fetch_all(conn: psycopg.Connection, sql: str, params: Any = None) -> list[dict]:
    return conn.execute(sql, params).fetchall()


def fetch_one(conn: psycopg.Connection, sql: str, params: Any = None) -> Optional[dict]:
    return conn.execute(sql, params).fetchone()
