"""Thread-safe MySQL connection pool for the read-only sql.search datastore.

Why this exists
---------------
Previously every ``execute_readonly`` call opened a brand-new pymysql
connection and closed it — one TCP + auth handshake per tool invocation
(``vitals.summary`` alone does two). Under concurrency that becomes a
connection storm against MySQL and adds latency to every agent turn.

This pool hands out at most ``PETMIND_MYSQL_POOL_SIZE`` live connections,
reuses idle ones, revives stale ones via ``ping(reconnect=True)`` and blocks
(bounded by ``PETMIND_MYSQL_POOL_TIMEOUT``) once the pool is saturated instead
of unboundedly opening sockets.

Thread-safety: sql tools run through ``asyncio.to_thread`` (see
``ToolRegistry.call``), so borrows happen from worker threads. A
``queue.Queue`` (thread-safe) holds idle connections and a lock guards the
"created" counter.

Connections are opened with ``autocommit=True`` on purpose: InnoDB defaults to
REPEATABLE READ, so a reused connection that never commits would keep serving
the snapshot from its first SELECT. Autocommit makes each statement its own
transaction, guaranteeing fresh reads across pooled reuse.
"""
from __future__ import annotations

import queue
import threading
from contextlib import contextmanager
from typing import Iterator, Optional

from .config import MysqlConfig


class MysqlPool:
    """A small bounded, thread-safe pool of pymysql connections."""

    def __init__(self, cfg: MysqlConfig, *, size: int, borrow_timeout: float) -> None:
        self._cfg = cfg
        self._size = max(1, size)
        self._borrow_timeout = max(0.1, borrow_timeout)
        self._idle: "queue.Queue" = queue.Queue(maxsize=self._size)
        self._created = 0
        self._created_lock = threading.Lock()

    # ----------------------------------------------------------------- connect
    def _new_conn(self):
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

    # ----------------------------------------------------------------- acquire
    def _acquire(self):
        # Fast path: reuse an idle connection if one is available.
        try:
            conn = self._idle.get_nowait()
        except queue.Empty:
            conn = None

        if conn is None:
            # Open a new connection if we are still under capacity...
            with self._created_lock:
                under_capacity = self._created < self._size
                if under_capacity:
                    self._created += 1
            if under_capacity:
                try:
                    return self._new_conn()
                except Exception:
                    # Roll back the reservation so we don't leak capacity.
                    with self._created_lock:
                        self._created -= 1
                    raise
            # ...otherwise wait for one to be returned.
            try:
                conn = self._idle.get(timeout=self._borrow_timeout)
            except queue.Empty as exc:  # pool saturated
                raise RuntimeError(
                    "SQL connection pool exhausted "
                    f"(size={self._size}, timeout={self._borrow_timeout}s)"
                ) from exc

        # Validate / revive an idle connection before handing it out.
        try:
            conn.ping(reconnect=True)
            return conn
        except Exception:
            self._discard(conn)
            # Replace the dead connection with a fresh one.
            with self._created_lock:
                self._created += 1
            try:
                return self._new_conn()
            except Exception:
                with self._created_lock:
                    self._created -= 1
                raise

    # ----------------------------------------------------------------- release
    def _release(self, conn, *, broken: bool) -> None:
        if broken:
            self._discard(conn)
            return
        try:
            self._idle.put_nowait(conn)
        except queue.Full:
            self._discard(conn)

    def _discard(self, conn) -> None:
        try:
            conn.close()
        except Exception:
            pass
        with self._created_lock:
            self._created = max(0, self._created - 1)

    # ----------------------------------------------------------------- public
    @contextmanager
    def connection(self) -> Iterator:
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
        return self._size

    @property
    def in_use(self) -> int:
        with self._created_lock:
            return self._created - self._idle.qsize()


# --- Module-level singleton -------------------------------------------------
_POOL: Optional[MysqlPool] = None
_POOL_LOCK = threading.Lock()


def get_pool(cfg: MysqlConfig) -> MysqlPool:
    """Return the process-wide pool, building it lazily from the first config."""
    global _POOL  # noqa: PLW0603
    if _POOL is None:
        with _POOL_LOCK:
            if _POOL is None:
                _POOL = MysqlPool(cfg, size=cfg.pool_size, borrow_timeout=cfg.pool_timeout)
    return _POOL


def close_pool() -> None:
    """Close all pooled connections. Called on app shutdown (and in tests)."""
    global _POOL  # noqa: PLW0603
    with _POOL_LOCK:
        if _POOL is not None:
            _POOL.close_all()
            _POOL = None
