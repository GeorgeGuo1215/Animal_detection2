"""存储层测试，跑真实 PostgreSQL。

重点覆盖三类容易出错又无法靠 mock 验证的东西：pgvector 距离运算、
SQL 与 Python 两份热度实现的一致性、以及跨用户隔离。
"""

from __future__ import annotations

import sys
from datetime import timedelta
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.heat import HeatParams, segment_heat  # noqa: E402
from memory_service.app.memory import long_term, mid_term, short_term  # noqa: E402
from memory_service.tests.helpers import BASE_TIME, fake_vector  # noqa: E402


# ---------------------------------------------------------------- 短期记忆


def test_append_and_count(conn, user_id):
    for i in range(3):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"问题{i}",
            agent_response=f"回答{i}",
            created_at=BASE_TIME + timedelta(minutes=i),
        )
    assert short_term.count(conn, user_id) == 3


def test_recent_returns_chronological_order(conn, user_id):
    for i in range(5):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"问题{i}",
            agent_response=f"回答{i}",
            created_at=BASE_TIME + timedelta(minutes=i),
        )
    rows = short_term.recent(conn, user_id, limit=3)
    # 取最近 3 条，但返回时老的在前，便于直接拼进 prompt
    assert [r["userInput"] for r in rows] == ["问题2", "问题3", "问题4"]


def test_drain_overflow_keeps_newest_and_returns_oldest(conn, user_id):
    for i in range(7):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"问题{i}",
            agent_response=f"回答{i}",
            created_at=BASE_TIME + timedelta(minutes=i),
        )
    drained = short_term.drain_overflow(conn, user_id, keep=3)

    assert [r["userInput"] for r in drained] == ["问题0", "问题1", "问题2", "问题3"]
    assert short_term.count(conn, user_id) == 3


def test_drain_returns_empty_when_not_overflowing(conn, user_id):
    short_term.append(conn, user_id=user_id, user_input="只有一条", agent_response="嗯")
    assert short_term.drain_overflow(conn, user_id, keep=10) == []
    assert short_term.count(conn, user_id) == 1


def test_short_term_is_isolated_per_user(conn, user_id, other_user_id):
    short_term.append(conn, user_id=user_id, user_input="我的", agent_response="a")
    short_term.append(conn, user_id=other_user_id, user_input="别人的", agent_response="b")

    assert short_term.count(conn, user_id) == 1
    drained = short_term.drain_overflow(conn, user_id, keep=0)
    assert [r["userInput"] for r in drained] == ["我的"]
    # 汰换只影响自己，别人的数据原封不动
    assert short_term.count(conn, other_user_id) == 1


def test_turn_receipt_survives_short_term_drain(conn, user_id):
    message_id, created = short_term.append_once(
        conn,
        user_id=user_id,
        user_input="去年的疫苗反应",
        agent_response="已记录",
        turn_id="session-a:turn-1",
    )
    assert created is True
    short_term.drain_overflow(conn, user_id, keep=0)

    repeated_id, repeated_created = short_term.append_once(
        conn,
        user_id=user_id,
        user_input="去年的疫苗反应",
        agent_response="已记录",
        turn_id="session-a:turn-1",
    )

    assert repeated_id == message_id
    assert repeated_created is False
    assert short_term.count(conn, user_id) == 0


# ---------------------------------------------------------------- 中期记忆


def _make_segment(conn, user_id, summary, keywords, now=BASE_TIME, pet_id=None):
    return mid_term.create_segment(
        conn,
        user_id=user_id,
        summary=summary,
        keywords=keywords,
        embedding=fake_vector(summary),
        now=now,
        pet_id=pet_id,
    )


def test_create_segment_and_add_pages_updates_page_count(conn, user_id):
    seg = _make_segment(conn, user_id, "关于喂养的对话", ["喂养"])
    mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[
            {"user_input": "喂养问题1", "agent_response": "答1", "embedding": fake_vector("喂养问题1")},
            {"user_input": "喂养问题2", "agent_response": "答2", "embedding": fake_vector("喂养问题2")},
        ],
        now=BASE_TIME,
    )
    row = conn.execute(
        'SELECT "pageCount", "heat" FROM memory_segments WHERE "id" = %s', (seg,)
    ).fetchone()
    assert row["pageCount"] == 2
    assert row["heat"] > 0


def test_sql_heat_matches_python_heat(conn, user_id):
    """SQL 与 Python 两份热度实现必须给出同一个数。

    排序要在数据库里做（时间衰减必须实时计算），但公式定义在 heat.py，
    两边一旦漂移，汰换与提升的行为就会和单测验证过的语义脱节。
    """
    params = HeatParams(alpha=1.3, beta=0.7, gamma=2.1, tau_hours=18.0)
    seg = _make_segment(conn, user_id, "关于疫苗的对话", ["疫苗"])
    conn.execute(
        'UPDATE memory_segments SET "visitCount" = 6, "pageCount" = 14,'
        ' "lastVisitAt" = %s WHERE "id" = %s',
        (BASE_TIME - timedelta(hours=30), seg),
    )

    now = BASE_TIME
    row = conn.execute(
        f'SELECT ({mid_term.HEAT_SQL}) AS h FROM memory_segments WHERE "id" = %(sid)s',
        {**mid_term._heat_params(params, now), "sid": seg},
    ).fetchone()

    expected = segment_heat(6, 14, BASE_TIME - timedelta(hours=30), now, params)
    assert float(row["h"]) == pytest.approx(expected, rel=1e-9)


def test_find_best_segment_merges_same_topic(conn, user_id):
    seg = _make_segment(conn, user_id, "关于喂养的对话", ["喂养"])
    best = mid_term.find_best_segment(
        conn,
        user_id=user_id,
        embedding=fake_vector("喂养相关的新问题"),
        keywords=["喂养"],
        threshold=0.6,
    )
    assert best is not None and best["id"] == seg


def test_find_best_segment_rejects_unrelated_topic(conn, user_id):
    _make_segment(conn, user_id, "关于喂养的对话", ["喂养"])
    best = mid_term.find_best_segment(
        conn,
        user_id=user_id,
        embedding=fake_vector("关于疫苗的对话"),
        keywords=["疫苗"],
        threshold=0.6,
    )
    assert best is None


def test_find_best_segment_never_crosses_users(conn, user_id, other_user_id):
    _make_segment(conn, other_user_id, "关于喂养的对话", ["喂养"])
    best = mid_term.find_best_segment(
        conn,
        user_id=user_id,
        embedding=fake_vector("关于喂养的对话"),
        keywords=["喂养"],
        threshold=0.6,
    )
    assert best is None


def test_search_returns_relevant_pages_and_heats_segment(conn, user_id):
    seg = _make_segment(conn, user_id, "关于皮肤的对话", ["皮肤"])
    mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[
            {"user_input": "皮肤发红怎么办", "agent_response": "先看是否过敏",
             "embedding": fake_vector("皮肤发红怎么办")},
        ],
        now=BASE_TIME,
    )
    before = conn.execute(
        'SELECT "visitCount" FROM memory_segments WHERE "id" = %s', (seg,)
    ).fetchone()["visitCount"]

    results = mid_term.search(
        conn,
        user_id=user_id,
        embedding=fake_vector("皮肤问题"),
        top_k_segments=5,
        top_k_pages=5,
        now=BASE_TIME + timedelta(minutes=10),
    )

    assert len(results) == 1
    assert results[0]["user_input"] == "皮肤发红怎么办"
    after = conn.execute(
        'SELECT "visitCount" FROM memory_segments WHERE "id" = %s', (seg,)
    ).fetchone()["visitCount"]
    # 被检索命中会抬高热度，这是"常被问到的话题更容易沉淀"的机制
    assert after == before + 1


def test_search_does_not_leak_across_users(conn, user_id, other_user_id):
    seg = _make_segment(conn, other_user_id, "关于体重的对话", ["体重"])
    mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=other_user_id,
        pages=[{"user_input": "体重下降", "agent_response": "注意观察",
                "embedding": fake_vector("体重下降")}],
        now=BASE_TIME,
    )
    results = mid_term.search(
        conn,
        user_id=user_id,
        embedding=fake_vector("体重"),
        top_k_segments=5,
        top_k_pages=5,
        now=BASE_TIME,
    )
    assert results == []


def test_hottest_segment_requires_crossing_threshold(conn, user_id):
    seg = _make_segment(conn, user_id, "关于牙齿的对话", ["牙齿"])
    mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[{"user_input": "牙结石", "agent_response": "定期洁牙",
                "embedding": fake_vector("牙结石")}],
        now=BASE_TIME,
    )
    assert mid_term.hottest_segment(
        conn, user_id=user_id, threshold=99.0, now=BASE_TIME
    ) is None
    hot = mid_term.hottest_segment(conn, user_id=user_id, threshold=0.1, now=BASE_TIME)
    assert hot is not None and hot["id"] == seg


def test_hottest_segment_skips_fully_analyzed_segments(conn, user_id):
    """已全部分析过的段不该被反复挑出来烧 LLM。"""
    seg = _make_segment(conn, user_id, "关于驱虫的对话", ["驱虫"])
    page_ids = mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[{"user_input": "驱虫周期", "agent_response": "每月一次",
                "embedding": fake_vector("驱虫周期")}],
        now=BASE_TIME,
    )
    mid_term.mark_analyzed(
        conn, segment_id=seg, page_ids=page_ids, now=BASE_TIME
    )
    assert mid_term.hottest_segment(
        conn, user_id=user_id, threshold=0.1, now=BASE_TIME
    ) is None


def test_mark_analyzed_resets_visits_but_keeps_page_count(conn, user_id):
    """相对上游的关键差异：页数是客观事实，不能跟着访问计数一起清零。"""
    seg = _make_segment(conn, user_id, "关于运动的对话", ["运动"])
    page_ids = mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[{"user_input": "遛狗时长", "agent_response": "每天一小时",
                "embedding": fake_vector("遛狗时长")}],
        now=BASE_TIME,
    )
    mid_term.touch(conn, [seg], now=BASE_TIME)
    mid_term.mark_analyzed(conn, segment_id=seg, page_ids=page_ids, now=BASE_TIME)

    row = conn.execute(
        'SELECT "visitCount", "pageCount", "lastAnalyzedAt" FROM memory_segments'
        ' WHERE "id" = %s',
        (seg,),
    ).fetchone()
    assert row["visitCount"] == 0
    assert row["pageCount"] == 1
    assert row["lastAnalyzedAt"] is not None


def test_overflow_segments_picks_coldest_first(conn, user_id):
    """汰换必须按实时热度，让久未访问的段真正冷下来。"""
    hot = _make_segment(conn, user_id, "关于喂养的对话", ["喂养"])
    cold = _make_segment(conn, user_id, "关于绝育的对话", ["绝育"])
    conn.execute(
        'UPDATE memory_segments SET "visitCount" = 10, "pageCount" = 20,'
        ' "lastVisitAt" = %s WHERE "id" = %s',
        (BASE_TIME, hot),
    )
    conn.execute(
        'UPDATE memory_segments SET "visitCount" = 0, "pageCount" = 1,'
        ' "lastVisitAt" = %s WHERE "id" = %s',
        (BASE_TIME - timedelta(days=60), cold),
    )

    victims = mid_term.overflow_segments(
        conn, user_id=user_id, capacity=1, now=BASE_TIME
    )
    assert [v["id"] for v in victims] == [cold]


def test_delete_segments_cascades_to_pages(conn, user_id):
    seg = _make_segment(conn, user_id, "关于过敏的对话", ["过敏"])
    mid_term.add_pages(
        conn,
        segment_id=seg,
        user_id=user_id,
        pages=[{"user_input": "鸡肉过敏", "agent_response": "换粮",
                "embedding": fake_vector("鸡肉过敏")}],
        now=BASE_TIME,
    )
    mid_term.delete_segments(conn, [seg])
    remaining = conn.execute(
        'SELECT count(*) AS n FROM memory_pages WHERE "segmentId" = %s', (seg,)
    ).fetchone()
    assert remaining["n"] == 0


def test_heat_distribution_reports_percentiles(conn, user_id):
    for name in ("喂养", "疫苗", "皮肤"):
        _make_segment(conn, user_id, f"关于{name}的对话", [name])
    stats = mid_term.heat_distribution(conn, user_id=user_id, now=BASE_TIME)
    assert stats["n"] == 3
    assert stats["max"] >= stats["p90"] >= stats["p50"] >= stats["min"]


# ---------------------------------------------------------------- 长期画像


def test_merge_profile_recurses_into_nested_dicts():
    old = {"petFacts": {"pet_a": {"品种": "布偶"}}}
    new = {"petFacts": {"pet_b": {"品种": "柯基"}}}
    merged = long_term.merge_profile(old, new)
    # 本轮只聊了另一只宠物，不能把先前那只的信息抹掉
    assert merged["petFacts"]["pet_a"]["品种"] == "布偶"
    assert merged["petFacts"]["pet_b"]["品种"] == "柯基"


def test_merge_profile_unions_lists_without_duplicates():
    merged = long_term.merge_profile(
        {"concerns": ["体重", "皮肤"]}, {"concerns": ["皮肤", "牙齿"]}
    )
    assert merged["concerns"] == ["体重", "皮肤", "牙齿"]


def test_merge_profile_ignores_empty_and_placeholder_values():
    old = {"communication": {"style": "偏好通俗"}}
    for placeholder in ({}, {"communication": None}, {"communication": "无"}):
        assert long_term.merge_profile(old, placeholder) == old


def test_update_profile_creates_then_merges(conn, user_id):
    first = long_term.update_profile(
        conn,
        user_id=user_id,
        delta={"concerns": ["体重"]},
        now=BASE_TIME,
    )
    assert first == {"concerns": ["体重"]}

    second = long_term.update_profile(
        conn,
        user_id=user_id,
        delta={"concerns": ["皮肤"], "healthWatch": ["过敏史"]},
        now=BASE_TIME,
    )
    assert second["concerns"] == ["体重", "皮肤"]
    assert second["healthWatch"] == ["过敏史"]

    stored = long_term.get_profile(conn, user_id)
    assert stored["version"] == 2
    assert stored["profile"]["concerns"] == ["体重", "皮肤"]


def test_update_profile_drops_unknown_fields(conn, user_id):
    """LLM 可能自由发挥出别的键，画像结构不该随模型漂移。"""
    result = long_term.update_profile(
        conn,
        user_id=user_id,
        delta={"randomField": "垃圾", "concerns": ["体重"]},
        now=BASE_TIME,
    )
    assert result == {"concerns": ["体重"]}


def test_update_profile_returns_none_when_delta_is_useless(conn, user_id):
    assert long_term.update_profile(
        conn, user_id=user_id, delta={"concerns": []}, now=BASE_TIME
    ) is None
    assert long_term.update_profile(
        conn, user_id=user_id, delta={}, now=BASE_TIME
    ) is None


# ---------------------------------------------------------------- 长期知识


def test_add_knowledge_deduplicates_identical_content(conn, user_id):
    first = long_term.add_knowledge(
        conn, user_id=user_id, content="布偶猫对鸡肉过敏",
        embedding=fake_vector("布偶猫对鸡肉过敏"),
    )
    second = long_term.add_knowledge(
        conn, user_id=user_id, content="布偶猫对鸡肉过敏",
        embedding=fake_vector("布偶猫对鸡肉过敏"),
    )
    assert first is not None
    assert second is None
    assert long_term.count_knowledge(conn, user_id) == 1


def test_same_content_for_different_users_is_not_deduplicated(conn, user_id, other_user_id):
    assert long_term.add_knowledge(conn, user_id=user_id, content="喜欢天然粮") is not None
    assert long_term.add_knowledge(conn, user_id=other_user_id, content="喜欢天然粮") is not None


def test_add_knowledge_skips_empty_and_placeholder(conn, user_id):
    for junk in ("", "   ", "无", "none"):
        assert long_term.add_knowledge(conn, user_id=user_id, content=junk) is None
    assert long_term.count_knowledge(conn, user_id) == 0


def test_search_knowledge_ranks_by_similarity_and_records_hits(conn, user_id):
    long_term.add_knowledge(
        conn, user_id=user_id, content="关于疫苗的记录",
        embedding=fake_vector("关于疫苗的记录"),
    )
    long_term.add_knowledge(
        conn, user_id=user_id, content="关于体重的记录",
        embedding=fake_vector("关于体重的记录"),
    )
    hits = long_term.search_knowledge(
        conn, user_id=user_id, embedding=fake_vector("疫苗"),
        top_k=1, now=BASE_TIME,
    )
    assert len(hits) == 1
    assert hits[0]["content"] == "关于疫苗的记录"

    row = conn.execute(
        'SELECT "hitCount" FROM memory_knowledge WHERE "id" = %s', (hits[0]["id"],)
    ).fetchone()
    assert row["hitCount"] == 1


def test_knowledge_capacity_evicts_never_used_entries_first(conn, user_id):
    """被反复查到的旧知识应当活过从没被用过的新知识。

    上游按纯时间顺序淘汰，会把早期录入但一直有用的关键事实挤掉。
    """
    important = long_term.add_knowledge(
        conn, user_id=user_id, content="布偶猫对鸡肉过敏",
        embedding=fake_vector("布偶猫对鸡肉过敏"), now=BASE_TIME,
    )
    conn.execute(
        'UPDATE memory_knowledge SET "hitCount" = 9 WHERE "id" = %s', (important,)
    )
    for i in range(3):
        long_term.add_knowledge(
            conn, user_id=user_id, content=f"无关紧要的记录{i}",
            now=BASE_TIME + timedelta(days=i + 1),
        )

    removed = long_term.enforce_knowledge_capacity(conn, user_id=user_id, capacity=2)
    assert removed == 2

    survivors = conn.execute(
        'SELECT "content" FROM memory_knowledge WHERE "userId" = %s', (user_id,)
    ).fetchall()
    assert "布偶猫对鸡肉过敏" in [row["content"] for row in survivors]
