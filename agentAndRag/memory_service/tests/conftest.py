"""测试夹具。

存储层跑真实 PostgreSQL：SQL 里有 pgvector 距离运算、部分索引、SKIP LOCKED、
JSONB 合并这些东西，用假连接 mock 掉等于什么都没验证。每个用例包在一个事务里，
结束时回滚，互不干扰也不留垃圾。

数据库不可用时本地默认跳过；CI/验收设置 MEMORY_TEST_REQUIRE_DB=1 后会直接失败，
避免把大批 skip 误报成“回归通过”。
"""

from __future__ import annotations

import os
import sys
import uuid
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.config import load_config  # noqa: E402
from memory_service.tests.helpers import FakeEmbedder, RecordingLLM  # noqa: E402


@pytest.fixture(scope="session")
def dsn() -> str:
    return load_config().dsn


@pytest.fixture(scope="session")
def _db_available(dsn: str) -> bool:
    try:
        import psycopg

        with psycopg.connect(dsn, connect_timeout=3) as conn:
            conn.execute("SELECT 1")
        return True
    except Exception as exc:  # noqa: BLE001 - 连不上的原因都一样处理
        if os.getenv("MEMORY_TEST_REQUIRE_DB", "").strip().lower() in {
            "1", "true", "yes", "on",
        }:
            pytest.fail(f"required PostgreSQL test database is unavailable: {exc}", pytrace=False)
        return False


@pytest.fixture
def conn(dsn: str, _db_available: bool):
    """一条包在事务里的连接，用例结束回滚。"""
    if not _db_available:
        pytest.skip(
            "本地 PostgreSQL 不可用；先执行 "
            "python memory_service/scripts/init_local_db.py"
        )

    import psycopg
    from pgvector.psycopg import register_vector
    from psycopg.rows import dict_row

    connection = psycopg.connect(dsn, row_factory=dict_row, autocommit=False)
    register_vector(connection)
    try:
        yield connection
    finally:
        connection.rollback()
        connection.close()


def _scoped_id(prefix: str) -> str:
    """每个用例一个全新 ID。

    不要用 id(conn) 之类的对象地址来派生：CPython 会复用地址，一旦数据库里留下过
    同名用户的已提交数据，用例之间就会串味，表现为难以复现的偶发失败。
    """
    return f"{prefix}_{uuid.uuid4().hex[:16]}"


@pytest.fixture
def user_id(conn) -> str:
    """建一个仅存在于本用例事务中的用户。"""
    uid = _scoped_id("test_user")
    conn.execute(
        'INSERT INTO memory_subjects ("id", "displayName", "source") VALUES (%s, %s, %s)'
        ' ON CONFLICT ("id") DO NOTHING',
        (uid, "pytest", "test"),
    )
    return uid


@pytest.fixture
def other_user_id(conn) -> str:
    """第二个用户，用来验证记忆不会跨用户泄漏。"""
    uid = _scoped_id("test_other")
    conn.execute(
        'INSERT INTO memory_subjects ("id", "displayName", "source") VALUES (%s, %s, %s)'
        ' ON CONFLICT ("id") DO NOTHING',
        (uid, "pytest-other", "test"),
    )
    return uid


@pytest.fixture
def embedder() -> FakeEmbedder:
    return FakeEmbedder()


@pytest.fixture
def llm() -> RecordingLLM:
    return RecordingLLM()
