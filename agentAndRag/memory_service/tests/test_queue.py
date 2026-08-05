"""任务队列与多 worker 并发的测试。

这里不能用回滚夹具：SKIP LOCKED 与咨询锁的行为只有在多条真实连接、真实提交的
前提下才能观察到，单连接事务里怎么测都是假的。用例自己负责建数据和清理。
"""

from __future__ import annotations

import sys
import threading
import uuid
from dataclasses import replace
from datetime import timedelta
from pathlib import Path
from typing import List

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db  # noqa: E402
from memory_service.app.config import load_config  # noqa: E402
from memory_service.app.memory import queue, short_term  # noqa: E402
from memory_service.app.worker import MemoryWorker  # noqa: E402
from memory_service.tests.helpers import (  # noqa: E402
    BASE_TIME,
    FailingLLM,
    FakeEmbedder,
    RecordingLLM,
)


@pytest.fixture
def cfg():
    return replace(
        load_config(),
        short_term_capacity=2,
        mid_term_capacity=5,
        heat_threshold=2.0,
        task_max_attempts=2,
        worker_poll_interval=0.01,
    )


@pytest.fixture
def pool(cfg, _db_available):
    """初始化全局连接池，worker 与队列操作都走它。"""
    if not _db_available:
        pytest.skip("本地 PostgreSQL 不可用")
    db.close_pool()
    db.init_pool(cfg)
    try:
        yield
    finally:
        db.close_pool()


@pytest.fixture
def users(pool) -> List[str]:
    """五个真实提交的用户，用例结束时删掉（记忆与任务靠外键级联清理）。"""
    ids = [f"qtest_{uuid.uuid4().hex[:10]}" for _ in range(5)]
    with db.connection() as conn:
        for uid in ids:
            conn.execute(
                'INSERT INTO "User" ("id", "username", "passwordHash")'
                ' VALUES (%s, %s, %s)',
                (uid, "pytest-queue", "x"),
            )
    yield ids
    with db.connection() as conn:
        conn.execute('DELETE FROM "User" WHERE "id" = ANY(%s)', (ids,))


# ---------------------------------------------------------------- 入队


def test_enqueue_merges_tasks_for_same_user(users):
    user = users[0]
    with db.connection() as conn:
        first = queue.enqueue(conn, user_id=user)
        second = queue.enqueue(conn, user_id=user)
        third = queue.enqueue(conn, user_id=user)

    assert first is not None
    # 用户连聊多轮只留一个待处理任务，worker 一次消化全部积压
    assert second is None and third is None

    with db.connection() as conn:
        row = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "userId" = %s', (user,)
        ).fetchone()
    assert row["n"] == 1


def test_enqueue_keeps_tasks_for_different_users_separate(users):
    with db.connection() as conn:
        ids = [queue.enqueue(conn, user_id=uid) for uid in users]
    assert all(task_id is not None for task_id in ids)
    assert len(set(ids)) == len(users)


def test_claim_marks_task_running_and_counts_attempt(users):
    with db.connection() as conn:
        queue.enqueue(conn, user_id=users[0])

    with db.connection() as conn:
        task = queue.claim(conn)
        assert task is not None
        assert task["userId"] == users[0]
        assert task["attempts"] == 1

    with db.connection() as conn:
        row = conn.execute(
            'SELECT "status" FROM memory_tasks WHERE "id" = %s', (task["id"],)
        ).fetchone()
    assert row["status"] == "running"


def test_claim_returns_none_when_queue_is_empty(users):
    with db.connection() as conn:
        assert queue.claim(conn) is None


def test_complete_removes_task(users):
    with db.connection() as conn:
        task_id = queue.enqueue(conn, user_id=users[0])
    with db.connection() as conn:
        queue.complete(conn, task_id)
    with db.connection() as conn:
        row = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "id" = %s', (task_id,)
        ).fetchone()
    assert row["n"] == 0


# ---------------------------------------------------------------- 并发出队


def test_concurrent_claims_never_hand_out_the_same_task(users):
    """SKIP LOCKED 的核心保证：多 worker 抢队列，每条任务只被取走一次。"""
    with db.connection() as conn:
        for uid in users:
            queue.enqueue(conn, user_id=uid)

    claimed: List[int] = []
    lock = threading.Lock()
    barrier = threading.Barrier(4)

    def grab():
        barrier.wait()
        while True:
            with db.connection() as conn:
                task = queue.claim(conn)
                if task is None:
                    return
                with lock:
                    claimed.append(task["id"])
                queue.complete(conn, task["id"])

    threads = [threading.Thread(target=grab) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=20)

    assert len(claimed) == len(users)
    assert len(set(claimed)) == len(users)


def test_advisory_lock_serialises_the_same_user(users):
    """同一用户的记忆提升必须串行，否则同一批对话会被提升两次。"""
    user = users[0]
    acquired: List[bool] = []
    lock = threading.Lock()
    first_holds = threading.Event()
    release_first = threading.Event()

    def hold_lock():
        with db.connection() as conn:
            got = db.try_user_lock(conn, user)
            with lock:
                acquired.append(got)
            first_holds.set()
            # 锁是事务级的，事务不结束就一直持有
            release_first.wait(timeout=10)

    def contend():
        first_holds.wait(timeout=10)
        with db.connection() as conn:
            got = db.try_user_lock(conn, user)
            with lock:
                acquired.append(got)

    holder = threading.Thread(target=hold_lock)
    rival = threading.Thread(target=contend)
    holder.start()
    rival.start()
    rival.join(timeout=15)
    release_first.set()
    holder.join(timeout=15)

    assert acquired.count(True) == 1
    assert acquired.count(False) == 1


def test_different_users_do_not_block_each_other(users):
    with db.connection() as conn_a, db.connection() as conn_b:
        assert db.try_user_lock(conn_a, users[0]) is True
        assert db.try_user_lock(conn_b, users[1]) is True


# ---------------------------------------------------------------- worker


def _feed(user_id: str, count: int) -> None:
    with db.connection() as conn:
        for i in range(count):
            short_term.append(
                conn,
                user_id=user_id,
                user_input=f"喂养问题{i}",
                agent_response=f"回答{i}",
                created_at=BASE_TIME + timedelta(minutes=i),
            )


def test_worker_consumes_task_and_promotes_memory(users, cfg):
    user = users[0]
    _feed(user, 6)
    with db.connection() as conn:
        queue.enqueue(conn, user_id=user)

    worker = MemoryWorker(cfg, FakeEmbedder(), RecordingLLM(), clock=lambda: BASE_TIME)
    assert worker.run_once() is True

    with db.connection() as conn:
        segments = conn.execute(
            'SELECT count(*) AS n FROM memory_segments WHERE "userId" = %s', (user,)
        ).fetchone()
        pending = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "userId" = %s', (user,)
        ).fetchone()

    assert segments["n"] >= 1
    assert pending["n"] == 0


def test_worker_reports_no_work_on_empty_queue(users, cfg):
    worker = MemoryWorker(cfg, FakeEmbedder(), RecordingLLM())
    assert worker.run_once() is False


def test_failed_task_is_retried_then_marked_failed(users, cfg):
    """LLM 挂掉不该让任务静默消失，也不该无限重试。"""
    user = users[0]
    _feed(user, 6)
    with db.connection() as conn:
        task_id = queue.enqueue(conn, user_id=user)

    worker = MemoryWorker(cfg, FakeEmbedder(), FailingLLM("llm down"), clock=lambda: BASE_TIME)

    assert worker.run_once() is False
    with db.connection() as conn:
        row = conn.execute(
            'SELECT "status", "attempts", "lastError" FROM memory_tasks WHERE "id" = %s',
            (task_id,),
        ).fetchone()
    assert row["status"] == "pending"
    assert row["attempts"] == 1
    assert "llm down" in row["lastError"]

    # 第二次失败达到上限，转入 failed 留待排查
    assert worker.run_once() is False
    with db.connection() as conn:
        row = conn.execute(
            'SELECT "status", "attempts" FROM memory_tasks WHERE "id" = %s', (task_id,)
        ).fetchone()
    assert row["status"] == "failed"
    assert row["attempts"] == cfg.task_max_attempts


def test_failed_task_does_not_lose_short_term_dialogue(users, cfg):
    """提升失败时短期对话必须还在，否则用户的对话就凭空丢了。"""
    user = users[0]
    _feed(user, 6)
    with db.connection() as conn:
        queue.enqueue(conn, user_id=user)

    worker = MemoryWorker(cfg, FakeEmbedder(), FailingLLM(), clock=lambda: BASE_TIME)
    worker.run_once()

    with db.connection() as conn:
        assert short_term.count(conn, user) == 6


def test_stale_running_tasks_are_requeued(users):
    """进程被强杀留下的 running 任务要能被捞回来。"""
    user = users[0]
    with db.connection() as conn:
        task_id = queue.enqueue(conn, user_id=user)
        conn.execute(
            """
            UPDATE memory_tasks
            SET "status" = 'running',
                "lockedAt" = CURRENT_TIMESTAMP - make_interval(hours => 2)
            WHERE "id" = %s
            """,
            (task_id,),
        )

    with db.connection() as conn:
        recovered = queue.requeue_stale_running(conn, older_than_seconds=600)
    assert recovered == 1

    with db.connection() as conn:
        row = conn.execute(
            'SELECT "status" FROM memory_tasks WHERE "id" = %s', (task_id,)
        ).fetchone()
    assert row["status"] == "pending"


def test_multiple_workers_process_distinct_users_without_duplication(users, cfg):
    """多 worker 跑同一批任务：每个用户只被提升一次，不重不漏。"""
    for uid in users:
        _feed(uid, 6)
    with db.connection() as conn:
        for uid in users:
            queue.enqueue(conn, user_id=uid)

    workers = [
        MemoryWorker(cfg, FakeEmbedder(), RecordingLLM(), name=f"w{i}",
                     clock=lambda: BASE_TIME)
        for i in range(3)
    ]
    barrier = threading.Barrier(len(workers))

    def drain(worker):
        barrier.wait()
        idle_rounds = 0
        while idle_rounds < 3:
            if worker.run_once():
                idle_rounds = 0
            else:
                idle_rounds += 1

    threads = [threading.Thread(target=drain, args=(w,)) for w in workers]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=60)

    assert sum(w.processed for w in workers) == len(users)

    with db.connection() as conn:
        remaining = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "userId" = ANY(%s)', (users,)
        ).fetchone()
        # 每个用户的对话都被归档，且没有谁被处理两遍
        for uid in users:
            pages = conn.execute(
                'SELECT count(*) AS n FROM memory_pages WHERE "userId" = %s', (uid,)
            ).fetchone()
            assert pages["n"] == 4
    assert remaining["n"] == 0
