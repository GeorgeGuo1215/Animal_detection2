"""长跑模拟的 CI 轻量版。

scripts/simulate_usage.py 跑 200 轮用来标定参数、出报告，人看指标做判断。这里是
它的守门版本：同一份剧本压到几十轮，只断言那些**一旦破了就说明记忆系统坏了**的
不变量，几秒内跑完，进 CI 没有负担。

复用同一份剧本而不是另写一套，是为了让 CI 挡住的东西和标定时量的东西是同一个系统。

和 test_queue.py 一样不能用回滚夹具：提升与汰换要走真实提交的多连接路径。
"""

from __future__ import annotations

import sys
import uuid
from dataclasses import replace
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db  # noqa: E402
from memory_service.app.config import load_config  # noqa: E402
from memory_service.app.memory import long_term, mid_term, short_term  # noqa: E402
from memory_service.scripts.simulate_usage import (  # noqa: E402
    build_script,
    run_simulation,
)

ROUNDS = 60


@pytest.fixture(scope="module")
def cfg(_db_available):
    """把容量压到能在几十轮内触发汰换的量级。

    线上默认是 50 段，这里给 6：不逼出汰换的话，这个用例就只是在测常规写入，
    而汰换恰恰是最容易悄悄丢数据的那条路径。
    """
    if not _db_available:
        pytest.skip("本地 PostgreSQL 不可用")
    return replace(
        load_config(),
        short_term_capacity=6,
        mid_term_capacity=6,
        knowledge_capacity=40,
        heat_threshold=2.0,
        promotion_batch=3,
        analysis_min_pages=3,
    )


@pytest.fixture(scope="module")
def pool(cfg):
    db.close_pool()
    db.init_pool(cfg)
    try:
        yield
    finally:
        db.close_pool()


@pytest.fixture(scope="module")
def two_users(pool):
    """两个用户：一个跑剧本，一个全程沉默，用来验证记忆不串户。"""
    ids = [f"simtest_{uuid.uuid4().hex[:10]}" for _ in range(2)]
    with db.connection() as conn:
        for uid in ids:
            conn.execute(
                'INSERT INTO "User" ("id", "username", "passwordHash") VALUES (%s, %s, %s)',
                (uid, "pytest-sim", "x"),
            )
    yield ids
    with db.connection() as conn:
        conn.execute('DELETE FROM "User" WHERE "id" = ANY(%s)', (ids,))


@pytest.fixture(scope="module")
def metrics(cfg, two_users):
    """整个模块共跑一次模拟，下面每条断言都看同一份结果。

    模拟本身是只读断言的对象，不会被用例改动；跑一次省下的秒数在 CI 上是实打实的。
    """
    turns = build_script(ROUNDS, seed=20260801)
    return run_simulation(cfg, turns, two_users[0], sample_every=ROUNDS)


def test_eviction_actually_runs(metrics):
    """前提校验：容量没压到位的话，下面几条关于汰换的断言全是空转。"""
    assert metrics.evicted_segments > 0


def test_anchor_facts_survive_the_long_run(metrics):
    """埋在剧本开头的关键事实，跑完几十轮之后必须还召得回来。

    这是判断汰换是否过激最直接的指标：段可以被淘汰，但驱逐前沉淀下来的知识
    要接得住，"猫对鸡肉过敏"这种信息丢了就是事故。
    """
    assert metrics.anchors_total > 0
    assert metrics.anchors_recalled == metrics.anchors_total


def test_evicted_topics_are_still_recallable(metrics):
    """段被淘汰的话题再被问起时，不该什么都召不回来。"""
    if metrics.evicted_topic_queries == 0:
        pytest.skip("本次没有针对被淘汰话题的查询，无从判定误杀")
    miss_rate = metrics.evicted_topic_misses / metrics.evicted_topic_queries
    assert miss_rate <= 0.1


def test_capacities_stay_within_limits(cfg, metrics):
    """各层不能无限膨胀——汰换失效时最先表现为容量越界。"""
    assert metrics.final["short_term"] <= cfg.short_term_capacity
    assert metrics.final["segments"] <= cfg.mid_term_capacity
    assert metrics.final["knowledge"] <= cfg.knowledge_capacity


def test_llm_cost_does_not_track_dialogue_volume(metrics):
    """成本必须与对话量解耦。

    上游是 2N+3，每轮至少两次调用；这里靠批量提升与分析页数门槛把它压到每轮
    一次以下。这条断言破了，说明某个节流开关被改坏了。
    """
    assert metrics.llm_calls / metrics.rounds < 1.0


def test_profile_accumulates_instead_of_overwriting(metrics):
    """画像应以累积为主。反复自我推翻说明提升触发得太频繁。"""
    assert metrics.profile_writes > 0
    assert metrics.profile_overwrites <= metrics.profile_writes * 0.2


def test_pets_do_not_bleed_across_segments(two_users, metrics):
    """两只宠物的话题交替出现几十轮后，段的宠物标签仍要和它每一页对得上。

    这条是看 dump 出来的记忆内容才发现的：指标全绿（锚点存活、误杀率 0、命中率
    100%）的那一版，18 个段里有 7 个混入了另一只宠物的对话，最严重的 51 页里混了
    14 页。指标只能证明信息没丢，证明不了标签贴对了，所以要单独守。
    """
    user_id = two_users[0]
    with db.connection() as conn:
        rows = conn.execute(
            """
            SELECT s."petId" AS seg_pet, p."petId" AS page_pet
            FROM memory_segments s JOIN memory_pages p ON p."segmentId" = s."id"
            WHERE s."userId" = %s
            """,
            (user_id,),
        ).fetchall()

    assert rows
    mismatched = [r for r in rows if r["seg_pet"] != r["page_pet"]]
    assert not mismatched


def test_no_memory_leaks_into_another_user(two_users, metrics):
    """旁观用户全程没说过话，任何一层都不该出现它的记忆。"""
    bystander = two_users[1]
    with db.connection() as conn:
        assert short_term.count(conn, bystander) == 0
        assert mid_term.count_segments(conn, bystander) == 0
        assert long_term.count_knowledge(conn, bystander) == 0
        assert long_term.get_profile(conn, bystander)["profile"] == {}
