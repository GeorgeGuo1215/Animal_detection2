"""线程安全的 MySQL 连接池，供只读 sql.search 数据源使用。

为何需要
--------
此前每次 ``execute_readonly`` 都会新建并关闭 pymysql 连接——每次工具调用一次 TCP+鉴权握手
（仅 ``vitals.summary`` 就要两次）。并发下会对 MySQL 造成连接风暴，并拉高每轮 Agent 延迟。

本池最多维持 ``PETMIND_MYSQL_POOL_SIZE`` 条存活连接，复用空闲连接，
通过 ``ping(reconnect=True)`` 复活过期连接，池满时在 ``PETMIND_MYSQL_POOL_TIMEOUT`` 内阻塞，
而不是无限开 socket。

线程安全：SQL 工具经 ``asyncio.to_thread`` 运行（见 ``ToolRegistry.call``），
因此借用发生在工作线程。空闲连接放在线程安全的 ``queue.Queue`` 中，创建计数由锁保护。

连接有意以 ``autocommit=True`` 打开：InnoDB 默认 REPEATABLE READ，
从未提交的复用连接会一直读到第一次 SELECT 的快照。Autocommit 让每条语句自成事务，
保证池化复用时读到新数据。
"""
from __future__ import annotations

import queue
import threading
from contextlib import contextmanager
from typing import Iterator, Optional

from .config import MysqlConfig


class MysqlPool:
    """小型、有界、线程安全的 pymysql 连接池。"""

    def __init__(self, cfg: MysqlConfig, *, size: int, borrow_timeout: float) -> None:
        """按配置初始化池容量与借用超时。"""
        self._cfg = cfg
        self._size = max(1, size)
        self._borrow_timeout = max(0.1, borrow_timeout)
        self._idle: "queue.Queue" = queue.Queue(maxsize=self._size)
        self._created = 0
        self._created_lock = threading.Lock()

    # ----------------------------------------------------------------- 建连
    def _new_conn(self):
        """打开一条 autocommit 的 DictCursor 连接。"""
        import pymysql
        from pymysql.cursors import DictCursor

        return pymysql.connect(
            host=self._cfg.host,
            port=self._cfg.port,
            user=self._cfg.user,
            password=self._cfg.password,
            database=self._cfg.database,
            charset="utf8mb4",
            cursorclass=DictCursor,
            autocommit=True,
        )

    # ----------------------------------------------------------------- 获取
    def _acquire(self):
        """获取一条可用连接：优先空闲，否则新建，池满则等待。"""
        # 快路径：有空闲连接则直接复用。
        try:
            conn = self._idle.get_nowait()
        except queue.Empty:
            conn = None

        if conn is None:
            # 未满容量则新建连接...
            with self._created_lock:
                under_capacity = self._created < self._size
                if under_capacity:
                    self._created += 1
            if under_capacity:
                try:
                    return self._new_conn()
                except Exception:
                    # 回滚预留容量，避免泄漏。
                    with self._created_lock:
                        self._created -= 1
                    raise
            # ...否则等待归还。
            try:
                conn = self._idle.get(timeout=self._borrow_timeout)
            except queue.Empty as exc:  # 池已饱和
                raise RuntimeError(
                    "SQL connection pool exhausted "
                    f"(size={self._size}, timeout={self._borrow_timeout}s)"
                ) from exc

        # 交出前校验/复活空闲连接。
        try:
            conn.ping(reconnect=True)
            return conn
        except Exception:
            self._discard(conn)
            # 用新连接替换死连接。
            with self._created_lock:
                self._created += 1
            try:
                return self._new_conn()
            except Exception:
                with self._created_lock:
                    self._created -= 1
                raise

    # ----------------------------------------------------------------- 归还
    def _release(self, conn, *, broken: bool) -> None:
        """归还连接到空闲队列；损坏或队列满则丢弃。"""
        if broken:
            self._discard(conn)
            return
        try:
            self._idle.put_nowait(conn)
        except queue.Full:
            self._discard(conn)

    def _discard(self, conn) -> None:
        """关闭连接并减少已创建计数。"""
        try:
            conn.close()
        except Exception:
            pass
        with self._created_lock:
            self._created = max(0, self._created - 1)

    # ----------------------------------------------------------------- 公开接口
    @contextmanager
    def connection(self) -> Iterator:
        """上下文管理器：借用连接，异常时标记为损坏。"""
        conn = self._acquire()
        broken = False
        try:
            yield conn
        except Exception:
            broken = True
            raise
        finally:
            self._release(conn, broken=broken)

    def close_all(self) -> None:
        """关闭所有空闲连接。"""
        while True:
            try:
                conn = self._idle.get_nowait()
            except queue.Empty:
                break
            try:
                conn.close()
            except Exception:
                pass
        with self._created_lock:
            self._created = 0

    @property
    def size(self) -> int:
        """池的最大容量。"""
        return self._size

    @property
    def in_use(self) -> int:
        """当前正在使用的连接数。"""
        with self._created_lock:
            return self._created - self._idle.qsize()


# --- 模块级单例 -------------------------------------------------
_POOL: Optional[MysqlPool] = None
_POOL_LOCK = threading.Lock()


def get_pool(cfg: MysqlConfig) -> MysqlPool:
    """返回进程级连接池，按首次配置懒加载。"""
    global _POOL  # noqa: PLW0603
    if _POOL is None:
        with _POOL_LOCK:
            if _POOL is None:
                _POOL = MysqlPool(cfg, size=cfg.pool_size, borrow_timeout=cfg.pool_timeout)
    return _POOL


def close_pool() -> None:
    """关闭全部池化连接。应用关闭时（以及测试中）调用。"""
    global _POOL  # noqa: PLW0603
    with _POOL_LOCK:
        if _POOL is not None:
            _POOL.close_all()
            _POOL = None
