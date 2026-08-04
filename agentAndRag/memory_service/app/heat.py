"""热度、相似度与连续性判断的纯函数层。

这个模块刻意不依赖数据库、不读环境变量、不取系统时间：当前时间一律由调用方
作为参数传入。长跑模拟需要把虚拟时钟推进几周来观察时间衰减，如果这里内部调
用 ``datetime.now()`` 就没法模拟了。

与上游 MemoryOS 的差异见 ``segment_heat`` 的说明。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Iterable, Sequence

# 上游 mid_term.py 的默认值：HEAT_ALPHA/BETA/GAMMA = 1，RECENCY_TAU_HOURS = 24。
DEFAULT_ALPHA = 1.0
DEFAULT_BETA = 1.0
DEFAULT_GAMMA = 1.0
DEFAULT_TAU_HOURS = 24.0

# 触发"中期段提升为长期画像"的热度阈值。
#
# 上游用 5.0，但那是在线性公式下；本模块对访问次数与页数做了对数压缩，
# 同样的段热度数值会小很多，阈值必须同步下调，否则永远触发不了。
# 3.0 的含义大致是：一个 10 页的新段，或者一个 5 页且被检索命中过两次的段。
# 这个值由 scripts/simulate_usage.py 的长跑报告标定。
DEFAULT_HEAT_THRESHOLD = 3.0


@dataclass(frozen=True)
class HeatParams:
    alpha: float = DEFAULT_ALPHA
    beta: float = DEFAULT_BETA
    gamma: float = DEFAULT_GAMMA
    tau_hours: float = DEFAULT_TAU_HOURS


def time_decay(last_visit: datetime, now: datetime, tau_hours: float = DEFAULT_TAU_HOURS) -> float:
    """指数时间衰减，与上游 ``compute_time_decay`` 同式：exp(-Δhours / tau)。

    未来时间戳（时钟漂移或测试构造）钳到 1.0，避免衰减因子大于 1 把热度撑爆。
    """
    if tau_hours <= 0:
        return 1.0
    delta_hours = (now - last_visit).total_seconds() / 3600.0
    if delta_hours <= 0:
        return 1.0
    return math.exp(-delta_hours / tau_hours)


def segment_heat(
    visit_count: int,
    page_count: int,
    last_visit: datetime,
    now: datetime,
    params: HeatParams = HeatParams(),
) -> float:
    """计算话题段热度。

    上游公式是 ``alpha*N_visit + beta*L_interaction + gamma*R_recency``，三项线性
    相加且前两项无上界。实际跑起来的问题是：一个持续增长的长段，页数可以到几十上百，
    直接把另外两项淹没，于是最热的永远是那个最长的段，新话题再怎么被反复访问也挤不
    进来，时间衰减也就失去意义了。

    这里对前两项取 ``log1p`` 压缩，让三项处在可比的量级上：页数从 10 涨到 100 只让
    热度增加约 1.15，而时间衰减项在一天内就能贡献接近 1。这样"最近反复聊的小话题"
    能够压过"很久以前的长话题"，符合热度本来的语义。
    """
    visits = max(0, int(visit_count))
    pages = max(0, int(page_count))
    recency = time_decay(last_visit, now, params.tau_hours)
    return (
        params.alpha * math.log1p(visits)
        + params.beta * math.log1p(pages)
        + params.gamma * recency
    )


def cosine(a: Sequence[float], b: Sequence[float]) -> float:
    """余弦相似度。长度不等或任一为零向量时返回 0。"""
    if not a or not b or len(a) != len(b):
        return 0.0
    dot = 0.0
    norm_a = 0.0
    norm_b = 0.0
    for x, y in zip(a, b):
        dot += x * y
        norm_a += x * x
        norm_b += y * y
    if norm_a <= 0.0 or norm_b <= 0.0:
        return 0.0
    return dot / math.sqrt(norm_a * norm_b)


def jaccard(a: Iterable[str], b: Iterable[str]) -> float:
    """关键词集合的 Jaccard 相似度，两边都空时算 0（无信息而非完全相同）。"""
    set_a = {str(x).strip().lower() for x in a if str(x).strip()}
    set_b = {str(x).strip().lower() for x in b if str(x).strip()}
    if not set_a or not set_b:
        return 0.0
    union = set_a | set_b
    if not union:
        return 0.0
    return len(set_a & set_b) / len(union)


def topic_score(
    semantic_similarity: float,
    keyword_similarity: float,
    keyword_weight: float = 1.0,
) -> float:
    """新对话与已有话题段的综合匹配分，同上游 ``semantic + alpha * jaccard``。

    超过阈值就并入该段，否则新建段。
    """
    return semantic_similarity + keyword_weight * keyword_similarity


def is_continuous(
    prev_time: datetime | None,
    curr_time: datetime,
    similarity: float,
    max_gap_minutes: float,
    min_similarity: float,
) -> bool:
    """判断两轮对话是否属于同一个连续话题。

    上游对每一页都调一次 LLM 来回答这个 true/false 问题，处理 N 条就是 N 次调用，
    是整个提升流程里最大的成本项。实际上"隔了多久"加"语义像不像"这两个信号已经
    足够，所以这里用启发式替代，零 LLM 调用。
    """
    if prev_time is None:
        return False
    gap_minutes = (curr_time - prev_time).total_seconds() / 60.0
    if gap_minutes < 0 or gap_minutes > max_gap_minutes:
        return False
    return similarity >= min_similarity
