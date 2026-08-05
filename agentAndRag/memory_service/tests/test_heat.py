"""热度、相似度与连续性纯函数的单测。

这层不碰数据库也不调 LLM，是整个服务里唯一可以完全确定性验证的部分，
所以边界条件在这里一次性覆盖干净。
"""

from __future__ import annotations

import math
import sys
from datetime import datetime, timedelta
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.heat import (  # noqa: E402
    HeatParams,
    cosine,
    is_continuous,
    jaccard,
    segment_heat,
    time_decay,
    topic_score,
)

NOW = datetime(2026, 8, 1, 12, 0, 0)


# ---------------------------------------------------------------- 时间衰减


def test_time_decay_is_one_at_zero_elapsed():
    assert time_decay(NOW, NOW, tau_hours=24) == pytest.approx(1.0)


def test_time_decay_halves_around_tau_times_ln2():
    """经过 tau*ln2 小时后衰减到一半，验证确实是 exp(-Δt/tau)。"""
    tau = 24.0
    half_life = tau * math.log(2)
    past = NOW - timedelta(hours=half_life)
    assert time_decay(past, NOW, tau_hours=tau) == pytest.approx(0.5, rel=1e-6)


def test_time_decay_is_monotonically_decreasing():
    values = [time_decay(NOW - timedelta(hours=h), NOW, 24) for h in (0, 1, 6, 24, 72)]
    assert values == sorted(values, reverse=True)


def test_future_timestamp_is_clamped_to_one():
    """时钟漂移导致 lastVisitAt 在未来时，衰减因子不能大于 1 把热度撑爆。"""
    future = NOW + timedelta(hours=5)
    assert time_decay(future, NOW, tau_hours=24) == 1.0


def test_non_positive_tau_disables_decay():
    past = NOW - timedelta(days=30)
    assert time_decay(past, NOW, tau_hours=0) == 1.0


# ---------------------------------------------------------------- 热度公式


def test_heat_increases_with_visits_and_pages():
    base = segment_heat(1, 1, NOW, NOW)
    assert segment_heat(5, 1, NOW, NOW) > base
    assert segment_heat(1, 5, NOW, NOW) > base


def test_heat_decays_as_segment_goes_stale():
    fresh = segment_heat(3, 5, NOW, NOW)
    stale = segment_heat(3, 5, NOW - timedelta(days=7), NOW)
    assert stale < fresh


def test_log_compression_prevents_page_count_domination():
    """这是相对上游线性公式的核心改动，必须锁住行为。

    线性公式下 100 页的陈旧段热度是 100+，任何新段都不可能超过它。
    对数压缩后，一个页数少但刚被反复访问的段应当能压过它。
    """
    stale_giant = segment_heat(
        visit_count=0,
        page_count=100,
        last_visit=NOW - timedelta(days=30),
        now=NOW,
    )
    fresh_small = segment_heat(
        visit_count=8,
        page_count=4,
        last_visit=NOW,
        now=NOW,
    )
    assert fresh_small > stale_giant


def test_heat_matches_explicit_formula():
    params = HeatParams(alpha=2.0, beta=0.5, gamma=3.0, tau_hours=12.0)
    last_visit = NOW - timedelta(hours=12)
    expected = (
        2.0 * math.log1p(4)
        + 0.5 * math.log1p(9)
        + 3.0 * math.exp(-1.0)
    )
    assert segment_heat(4, 9, last_visit, NOW, params) == pytest.approx(expected)


@pytest.mark.parametrize("visits,pages", [(-5, 3), (3, -5), (-1, -1)])
def test_negative_counters_are_floored_at_zero(visits, pages):
    """脏数据不应该让 log1p 拿到负数而抛异常。"""
    value = segment_heat(visits, pages, NOW, NOW)
    assert value == pytest.approx(segment_heat(max(0, visits), max(0, pages), NOW, NOW))


def test_brand_new_empty_segment_stays_below_default_threshold():
    """刚建的小段不该立刻触发画像提升，否则每轮对话都要烧一次 LLM。"""
    from memory_service.app.heat import DEFAULT_HEAT_THRESHOLD

    assert segment_heat(0, 3, NOW, NOW) < DEFAULT_HEAT_THRESHOLD


# ---------------------------------------------------------------- 相似度


def test_cosine_of_identical_vectors_is_one():
    assert cosine([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]) == pytest.approx(1.0)


def test_cosine_of_orthogonal_vectors_is_zero():
    assert cosine([1.0, 0.0], [0.0, 1.0]) == pytest.approx(0.0)


def test_cosine_ignores_magnitude():
    assert cosine([1.0, 1.0], [5.0, 5.0]) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "a,b",
    [([], [1.0]), ([1.0], []), ([1.0, 2.0], [1.0]), ([0.0, 0.0], [1.0, 1.0])],
)
def test_cosine_returns_zero_for_degenerate_input(a, b):
    assert cosine(a, b) == 0.0


def test_jaccard_is_case_and_whitespace_insensitive():
    assert jaccard(["Cat", " dog "], ["cat", "DOG"]) == pytest.approx(1.0)


def test_jaccard_partial_overlap():
    assert jaccard(["a", "b"], ["b", "c"]) == pytest.approx(1 / 3)


def test_empty_keyword_sets_score_zero_not_one():
    """两边都没关键词说明没信息，不能当成完全匹配去合并话题段。"""
    assert jaccard([], []) == 0.0
    assert jaccard(["a"], []) == 0.0


def test_topic_score_combines_both_signals():
    assert topic_score(0.5, 0.25, keyword_weight=2.0) == pytest.approx(1.0)


# ---------------------------------------------------------------- 连续性


def test_no_previous_turn_is_not_continuous():
    assert is_continuous(None, NOW, similarity=0.99, max_gap_minutes=30, min_similarity=0.5) is False


def test_continuous_when_recent_and_similar():
    prev = NOW - timedelta(minutes=5)
    assert is_continuous(prev, NOW, 0.8, max_gap_minutes=30, min_similarity=0.5) is True


def test_not_continuous_when_gap_too_large():
    prev = NOW - timedelta(hours=5)
    assert is_continuous(prev, NOW, 0.99, max_gap_minutes=30, min_similarity=0.5) is False


def test_not_continuous_when_topic_shifted():
    prev = NOW - timedelta(minutes=1)
    assert is_continuous(prev, NOW, 0.1, max_gap_minutes=30, min_similarity=0.5) is False


def test_similarity_exactly_at_threshold_counts_as_continuous():
    prev = NOW - timedelta(minutes=1)
    assert is_continuous(prev, NOW, 0.5, max_gap_minutes=30, min_similarity=0.5) is True


def test_out_of_order_timestamps_are_not_continuous():
    """乱序到达的消息不该被当作连续对话串起来。"""
    prev = NOW + timedelta(minutes=10)
    assert is_continuous(prev, NOW, 0.9, max_gap_minutes=30, min_similarity=0.5) is False
