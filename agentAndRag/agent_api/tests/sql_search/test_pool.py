"""Unit tests for the MySQL connection pool (no live database required).

These use a fake connection object so they exercise the pool's borrow / reuse /
revive / discard bookkeeping deterministically, without touching MySQL.

Run: pytest tests/sql_search/test_pool.py
"""
import itertools
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import pytest

from app.sql_search.config import MysqlConfig
from app.sql_search.pool import MysqlPool


_ids = itertools.count(1)


class FakeConn:
    def __init__(self) -> None:
        self.id = next(_ids)
        self.closed = False
        self.ping_calls = 0
        self.ping_should_fail = False

    def ping(self, reconnect: bool = False) -> None:
        self.ping_calls += 1
        if self.ping_should_fail:
            raise RuntimeError("connection dead")

    def close(self) -> None:
        self.closed = True


def _cfg() -> MysqlConfig:
    return MysqlConfig(
        host="x", port=3306, user="u", password="", database="petmind",
        max_limit=200, debug_sql=False, pool_size=2, pool_timeout=0.3,
    )


def _pool(size: int = 2, timeout: float = 0.3) -> MysqlPool:
    cfg = _cfg()
    pool = MysqlPool(cfg, size=size, borrow_timeout=timeout)
    # Route connection creation through the fake factory.
    pool._new_conn = lambda: FakeConn()  # type: ignore[method-assign]
    return pool


def test_idle_connection_is_reused():
    pool = _pool(size=2)
    with pool.connection() as c1:
        first_id = c1.id
    # Returned to the idle pool; the next borrow should reuse the same object.
    with pool.connection() as c2:
        assert c2.id == first_id
    assert pool.in_use == 0


def test_capacity_bound_and_timeout():
    pool = _pool(size=1, timeout=0.2)
    borrowed = pool._acquire()
    assert pool.in_use == 1
    # Pool saturated: a second borrow must time out rather than open a socket.
    with pytest.raises(RuntimeError):
        pool._acquire()
    # After releasing, the next borrow succeeds and reuses the connection.
    pool._release(borrowed, broken=False)
    reused = pool._acquire()
    assert reused.id == borrowed.id
    pool._release(reused, broken=False)


def test_stale_connection_is_revived():
    pool = _pool(size=1)
    with pool.connection() as c1:
        dead_id = c1.id
    # Mark the idle connection as dead so ping() fails on next borrow.
    idle = pool._idle.queue[0]
    idle.ping_should_fail = True
    with pool.connection() as c2:
        assert c2.id != dead_id  # replaced with a fresh connection
    assert idle.closed is True
    assert pool.in_use == 0


def test_broken_connection_is_discarded():
    pool = _pool(size=2)
    captured = {}
    with pytest.raises(ValueError):
        with pool.connection() as c:
            captured["conn"] = c
            raise ValueError("boom")
    assert captured["conn"].closed is True
    assert pool.in_use == 0
    # A subsequent borrow must be a new connection, not the broken one.
    with pool.connection() as c2:
        assert c2.id != captured["conn"].id


def test_close_all_closes_idle_and_resets():
    pool = _pool(size=2)
    a = pool._acquire()
    b = pool._acquire()
    pool._release(a, broken=False)
    pool._release(b, broken=False)
    pool.close_all()
    assert a.closed is True
    assert b.closed is True
    assert pool.in_use == 0
    assert pool._idle.qsize() == 0
