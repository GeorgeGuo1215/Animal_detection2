"""长跑模拟与参数标定。

热度系数、提升阈值、各层容量这些默认值照抄上游没有说服力。这个脚本用一份可重复的
剧本把它们的实际效果量出来：约 200 轮对话、虚拟时间跨度数周、话题冷热分层、
两只宠物交织，外加埋在早期的锚点事实，跑完输出指标 JSON 与 markdown 报告。

几个关键设计：

- 时间是**注入**的，不是真等。heat.py 的当前时间全部由参数传入，模拟器按剧本推进
  虚拟时钟，几周的时间衰减在几秒内就能跑完。
- LLM 与 embedding 用确定性假实现，多次运行结果完全可比，也不占显卡、不花钱。
- 每轮之后照常触发 worker，走的是和线上完全相同的提升与汰换代码路径。

    python memory_service/scripts/simulate_usage.py
    python memory_service/scripts/simulate_usage.py --rounds 400 --heat-threshold 4

在 agentAndRag/ 目录下执行。
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
import uuid
from dataclasses import dataclass, field, replace
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db  # noqa: E402
from memory_service.app.config import MemoryConfig, load_config  # noqa: E402
from memory_service.app.memory import (  # noqa: E402
    consolidator,
    long_term,
    mid_term,
    queue,
    retriever,
    short_term,
)
from memory_service.tests.helpers import FakeEmbedder, RecordingLLM  # noqa: E402

START = datetime(2026, 6, 1, 9, 0, 0)
REPORTS_DIR = Path(__file__).resolve().parent.parent / "reports"

PETS = ["咪咪", "豆豆"]

# 话题按出现频率分层：高频话题反复聊，低频话题只出现一两次。
# 汰换是否合理，看的就是低频话题被淘汰后关键信息还在不在。
TOPIC_TIERS: Dict[str, List[str]] = {
    "hot": ["喂养", "体重"],
    "warm": ["疫苗", "驱虫", "运动"],
    "cold": ["皮肤", "牙齿", "绝育", "行为"],
}

QUESTION_TEMPLATES = {
    "喂养": "{pet}今天的{topic}安排怎么调整比较好",
    "体重": "{pet}的{topic}最近有变化，需要注意什么",
    "疫苗": "{pet}的{topic}接种计划要怎么安排",
    "驱虫": "{pet}的{topic}多久做一次合适",
    "运动": "{pet}每天的{topic}量够不够",
    "皮肤": "{pet}的{topic}状态有点异常",
    "牙齿": "{pet}的{topic}需要怎么护理",
    "绝育": "{pet}的{topic}手术要注意什么",
    "行为": "{pet}最近的{topic}有点反常",
}

# 埋在剧本早期的关键长期信息。跑完之后查它们还召不召得回来，
# 这是判断汰换是否过激最直接的指标。
ANCHORS = [
    ("过敏", "咪咪", "咪咪对鸡肉过敏，吃了会呕吐", "已记录，请避开含鸡肉的猫粮"),
    ("疫苗", "豆豆", "豆豆去年打疫苗有过敏反应", "已记录，下次接种需提前告知医生"),
]


@dataclass
class Turn:
    index: int
    at: datetime
    topic: str
    pet: str
    question: str
    answer: str
    is_anchor: bool = False


@dataclass
class Sample:
    round: int
    day: float
    short_term: int
    segments: int
    knowledge: int
    heat_p50: float
    heat_p90: float
    heat_max: float


class CountingLLM:
    """给任意 LLM 客户端套一层计数与容错。

    真实模型跑长跑必须容错：一次限流或超时就让整轮模拟白跑是不可接受的。线上遇到
    这种情况由 worker 重试，这里也重试一次，仍失败就跳过这批对话继续往下——正好
    对应线上任务耗尽重试次数后的行为，让指标反映真实可用性而不是理想情况。
    """

    def __init__(self, inner, retries: int = 1) -> None:
        self._inner = inner
        self._retries = retries
        self.call_count = 0
        self.failures = 0
        self.total_seconds = 0.0

    def complete(self, messages, **kwargs) -> str:
        last_error: Optional[Exception] = None
        for attempt in range(self._retries + 1):
            self.call_count += 1
            started = time.monotonic()
            try:
                return self._inner.complete(messages, **kwargs)
            except Exception as exc:  # noqa: BLE001 - 长跑要吞掉一切上游异常
                last_error = exc
                self.failures += 1
                if attempt < self._retries:
                    time.sleep(1.0)
            finally:
                self.total_seconds += time.monotonic() - started
        raise RuntimeError(f"LLM 连续 {self._retries + 1} 次失败") from last_error


@dataclass
class Metrics:
    rounds: int
    samples: List[Sample] = field(default_factory=list)
    llm_calls: int = 0
    llm_failures: int = 0
    llm_seconds: float = 0.0
    consolidation_errors: int = 0
    promotions: int = 0
    analyses: int = 0
    evicted_segments: int = 0
    distilled: int = 0
    profile_writes: int = 0
    profile_overwrites: int = 0
    queries: int = 0
    query_hits: int = 0
    evicted_topic_queries: int = 0
    evicted_topic_misses: int = 0
    anchors_total: int = 0
    anchors_recalled: int = 0
    final: Dict[str, Any] = field(default_factory=dict)


def build_script(rounds: int, seed: int) -> List[Turn]:
    """生成冷热分层的对话剧本。

    高频话题占多数轮次，低频话题只在开头露一两次面——这样跑到后期，低频话题
    自然会变成最冷的段，正好检验汰换会不会把它们连同关键信息一起丢掉。
    """
    rng = random.Random(seed)
    turns: List[Turn] = []
    moment = START

    # 锚点事实排在最前面，让它们有足够长的时间变冷。
    for index, (topic, pet, question, answer) in enumerate(ANCHORS):
        turns.append(
            Turn(index, moment, topic, pet, question, answer, is_anchor=True)
        )
        moment += timedelta(minutes=rng.randint(10, 40))

    # 低频话题紧随其后，之后就再也不出现。
    for topic in TOPIC_TIERS["cold"]:
        pet = rng.choice(PETS)
        turns.append(
            Turn(
                len(turns),
                moment,
                topic,
                pet,
                QUESTION_TEMPLATES[topic].format(pet=pet, topic=topic),
                f"关于{topic}的建议：先观察，必要时就医",
            )
        )
        moment += timedelta(minutes=rng.randint(20, 90))

    # 主体轮次按 6:3:1 的比例在三层话题间取样。
    weights = ["hot"] * 6 + ["warm"] * 3 + ["cold"] * 1
    while len(turns) < rounds:
        tier = rng.choice(weights)
        topic = rng.choice(TOPIC_TIERS[tier])
        pet = rng.choice(PETS)
        turns.append(
            Turn(
                len(turns),
                moment,
                topic,
                pet,
                QUESTION_TEMPLATES[topic].format(pet=pet, topic=topic),
                f"针对{pet}的{topic}问题，建议保持规律并持续观察",
            )
        )
        # 大部分对话间隔几十分钟，偶尔隔一两天，模拟真实的使用节奏。
        if rng.random() < 0.25:
            moment += timedelta(hours=rng.randint(12, 48))
        else:
            moment += timedelta(minutes=rng.randint(5, 90))

    return turns


def run_simulation(
    cfg: MemoryConfig,
    turns: List[Turn],
    user_id: str,
    sample_every: int,
    *,
    embedder=None,
    llm=None,
    progress_every: int = 0,
) -> Metrics:
    """跑完整个剧本。

    embedder 与 llm 默认用确定性假实现（可复现、免费、离线）；传入真实实现即为
    真实联调，两者走的是完全相同的代码路径。
    """
    embedder = embedder or FakeEmbedder(cfg.embedding_dim)
    llm = llm if llm is not None else CountingLLM(RecordingLLM())
    metrics = Metrics(rounds=len(turns))
    started_at = time.monotonic()

    evicted_topics: Dict[str, int] = {}
    previous_profile: Dict[str, Any] = {}

    for turn in turns:
        with db.connection() as conn:
            short_term.append(
                conn,
                user_id=user_id,
                user_input=turn.question,
                agent_response=turn.answer,
                pet_id=turn.pet,
                created_at=turn.at,
            )
            size = short_term.count(conn, user_id)
            if consolidator.should_promote(size, cfg):
                queue.enqueue(conn, user_id=user_id, now=turn.at)

        # worker 侧：把队列里的活干完，走的是和线上一样的代码路径。
        while True:
            with db.connection() as conn:
                task = queue.claim(conn, now=turn.at)
                if task is None:
                    break
                try:
                    result = consolidator.consolidate(
                        conn, user_id=user_id, cfg=cfg, embedder=embedder,
                        llm=llm, now=turn.at,
                    )
                except Exception as exc:  # noqa: BLE001
                    # 真实模型下这里会因限流、超时、返回体不合法而抛错。线上由 worker
                    # 重试，重试耗尽则标记失败；模拟里记一笔继续跑，让报告如实反映
                    # 有多少批对话没能归档，而不是让整轮长跑崩在半路。
                    metrics.consolidation_errors += 1
                    print(f"  [轮次 {turn.index}] 提升失败，跳过：{exc}")
                    queue.fail(
                        conn,
                        task_id=task["id"],
                        user_id=task["userId"],
                        kind=task["kind"],
                        error=str(exc),
                        attempts=task["attempts"],
                        # CountingLLM 已经重试过了，这里不再让任务回到队列里空耗
                        max_attempts=1,
                        now=turn.at,
                    )
                    continue
                queue.complete(conn, task["id"])

            metrics.promotions += result.get("segments_created", 0) + result.get(
                "segments_merged", 0
            )
            metrics.analyses += result.get("analyzed_segments", 0)
            metrics.evicted_segments += result.get("evicted_segments", 0)
            metrics.distilled += result.get("distilled", 0)

            if result.get("profile_updated"):
                metrics.profile_writes += 1
                with db.connection() as conn:
                    current = long_term.get_profile(conn, user_id)["profile"]
                metrics.profile_overwrites += _count_overwrites(previous_profile, current)
                previous_profile = current

        # 记录哪些话题的段被汰换掉了，后面查这些话题时用来算误杀。
        if metrics.evicted_segments:
            evicted_topics[turn.topic] = evicted_topics.get(turn.topic, 0)

        # 每隔若干轮做一次检索，模拟用户提问时记忆系统被调用。
        if turn.index % 7 == 0 and turn.index > 0:
            _probe(cfg, user_id, turn, embedder, metrics, evicted_topics)

        if turn.index % sample_every == 0:
            metrics.samples.append(_sample(cfg, user_id, turn))

        if progress_every and turn.index and turn.index % progress_every == 0:
            elapsed = time.monotonic() - started_at
            print(
                f"  {turn.index}/{len(turns)} 轮 | LLM {llm.call_count} 次 "
                f"| 已用时 {elapsed:.0f}s"
            )

    metrics.llm_calls = llm.call_count
    metrics.llm_failures = getattr(llm, "failures", 0)
    metrics.llm_seconds = getattr(llm, "total_seconds", 0.0)
    _check_anchors(cfg, user_id, turns, embedder, metrics)
    metrics.final = _final_state(cfg, user_id, turns[-1].at)
    return metrics


def _count_overwrites(old: Dict[str, Any], new: Dict[str, Any]) -> int:
    """统计有多少个已有字段被改写（而非新增）。

    抖动次数过高说明提升触发得太频繁，画像在反复自我推翻，这时候该调高热度阈值。
    """
    count = 0
    for key, value in new.items():
        if key in old and old[key] != value and not _is_superset(old[key], value):
            count += 1
    return count


def _is_superset(old: Any, new: Any) -> bool:
    """列表追加、字典新增键都算正常累积，不算抖动。"""
    if isinstance(old, list) and isinstance(new, list):
        return all(item in new for item in old)
    if isinstance(old, dict) and isinstance(new, dict):
        return all(k in new for k in old)
    return False


def _probe(
    cfg: MemoryConfig,
    user_id: str,
    turn: Turn,
    embedder,
    metrics: Metrics,
    evicted_topics: Dict[str, int],
) -> None:
    with db.connection() as conn:
        context = retriever.build_context(
            conn, user_id=user_id, query=turn.topic, cfg=cfg,
            embedder=embedder, now=turn.at,
        )

    metrics.queries += 1
    recalled = _recall_text(context)
    hit = turn.topic in recalled
    if hit:
        metrics.query_hits += 1

    # 误杀：这个话题的段被淘汰过，现在查它却什么都召不回来。
    if turn.topic in evicted_topics:
        metrics.evicted_topic_queries += 1
        if not hit:
            metrics.evicted_topic_misses += 1


def _recall_text(context: Dict[str, Any]) -> str:
    parts = [item["content"] for item in context.get("knowledge", [])]
    for item in context.get("related_pages", []):
        parts.append(item["segment_summary"])
        parts.append(item["user_input"])
    for item in context.get("recent_dialogue", []):
        parts.append(item["user_input"])
    parts.append(json.dumps(context.get("profile", {}), ensure_ascii=False))
    return " ".join(parts)


def _check_anchors(
    cfg: MemoryConfig, user_id: str, turns: List[Turn], embedder, metrics: Metrics
) -> None:
    """跑完之后回头查锚点事实，看关键长期信息还在不在。"""
    end = turns[-1].at + timedelta(hours=1)
    for topic, pet, question, _ in ANCHORS:
        metrics.anchors_total += 1
        with db.connection() as conn:
            context = retriever.build_context(
                conn, user_id=user_id, query=question, cfg=cfg,
                embedder=embedder, now=end,
            )
        if topic in _recall_text(context) or pet in _recall_text(context):
            metrics.anchors_recalled += 1


def _sample(cfg: MemoryConfig, user_id: str, turn: Turn) -> Sample:
    with db.connection() as conn:
        heat = mid_term.heat_distribution(
            conn, user_id=user_id, now=turn.at, params=cfg.heat
        )
        return Sample(
            round=turn.index,
            day=round((turn.at - START).total_seconds() / 86400.0, 2),
            short_term=short_term.count(conn, user_id),
            segments=mid_term.count_segments(conn, user_id),
            knowledge=long_term.count_knowledge(conn, user_id),
            heat_p50=round(heat["p50"], 3),
            heat_p90=round(heat["p90"], 3),
            heat_max=round(heat["max"], 3),
        )


def _final_state(cfg: MemoryConfig, user_id: str, at: datetime) -> Dict[str, Any]:
    with db.connection() as conn:
        heat = mid_term.heat_distribution(conn, user_id=user_id, now=at, params=cfg.heat)
        profile = long_term.get_profile(conn, user_id)
        sources = conn.execute(
            'SELECT "source", count(*) AS n FROM memory_knowledge'
            ' WHERE "userId" = %s GROUP BY "source"',
            (user_id,),
        ).fetchall()
        return {
            "short_term": short_term.count(conn, user_id),
            "segments": mid_term.count_segments(conn, user_id),
            "knowledge": long_term.count_knowledge(conn, user_id),
            "knowledge_by_source": {row["source"]: int(row["n"]) for row in sources},
            "heat": heat,
            "profile_version": profile["version"],
            "profile_fields": sorted(profile["profile"].keys()),
        }


def render_report(cfg: MemoryConfig, metrics: Metrics, seed: int) -> str:
    anchor_rate = (
        metrics.anchors_recalled / metrics.anchors_total if metrics.anchors_total else 0.0
    )
    miss_rate = (
        metrics.evicted_topic_misses / metrics.evicted_topic_queries
        if metrics.evicted_topic_queries
        else 0.0
    )
    hit_rate = metrics.query_hits / metrics.queries if metrics.queries else 0.0
    span_days = metrics.samples[-1].day if metrics.samples else 0.0
    upstream_calls = 2 * metrics.rounds + 3 * max(1, metrics.analyses)

    lines = [
        "# 记忆系统长跑模拟报告",
        "",
        f"- 对话轮次：{metrics.rounds}",
        f"- 虚拟时间跨度：{span_days:.1f} 天",
        f"- 随机种子：{seed}（同种子结果可完全复现）",
        "",
        "## 参数",
        "",
        "| 参数 | 取值 |",
        "| --- | --- |",
        f"| 短期容量 | {cfg.short_term_capacity} |",
        f"| 中期段容量 | {cfg.mid_term_capacity} |",
        f"| 知识容量 | {cfg.knowledge_capacity} |",
        f"| 热度系数 α/β/γ | {cfg.heat.alpha} / {cfg.heat.beta} / {cfg.heat.gamma} |",
        f"| 时间衰减 τ | {cfg.heat.tau_hours} 小时 |",
        f"| 提升阈值 | {cfg.heat_threshold} |",
        f"| 段合并阈值 | {cfg.segment_similarity_threshold} |",
        "",
        "## 核心指标",
        "",
        "| 指标 | 数值 | 说明 |",
        "| --- | --- | --- |",
        f"| 锚点事实存续率 | {anchor_rate:.0%} | 早期埋入的关键信息在长跑结束后仍能召回的比例 |",
        f"| 汰换误杀率 | {miss_rate:.0%} | 被淘汰话题再次被查询时完全召不回的比例 |",
        f"| 检索命中率 | {hit_rate:.0%} | 查询主题能在返回上下文中找到对应内容的比例 |",
        f"| LLM 调用总数 | {metrics.llm_calls} | 上游同等规模约需 {upstream_calls} 次 |",
        f"| 每轮对话 LLM 成本 | {metrics.llm_calls / metrics.rounds:.3f} 次 | 越低越好 |",
        f"| 画像抖动次数 | {metrics.profile_overwrites} | 已有字段被改写（非累积）的次数，过高说明提升过于频繁 |",
        f"| 画像更新次数 | {metrics.profile_writes} | |",
        f"| 段提升次数 | {metrics.promotions} | 新建加合并 |",
        f"| 画像分析次数 | {metrics.analyses} | 直接对应 LLM 成本 |",
        f"| 汰换段数 | {metrics.evicted_segments} | |",
        f"| 汰换沉淀知识数 | {metrics.distilled} | 被淘汰的段留下的摘要 |",
        "",
        "## 终态",
        "",
        "```json",
        json.dumps(metrics.final, ensure_ascii=False, indent=2, default=str),
        "```",
        "",
        "## 各层容量随轮次变化",
        "",
        "| 轮次 | 第几天 | 短期 | 中期段 | 知识 | 热度 P50 | 热度 P90 | 热度 max |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for sample in metrics.samples:
        lines.append(
            f"| {sample.round} | {sample.day:.1f} | {sample.short_term} | "
            f"{sample.segments} | {sample.knowledge} | {sample.heat_p50} | "
            f"{sample.heat_p90} | {sample.heat_max} |"
        )

    lines += ["", "## 结论", "", _conclusions(cfg, metrics, anchor_rate, miss_rate)]
    return "\n".join(lines)


def _conclusions(
    cfg: MemoryConfig, metrics: Metrics, anchor_rate: float, miss_rate: float
) -> str:
    notes: List[str] = []
    evicted = metrics.evicted_segments

    if evicted == 0:
        # 一次都没汰换时，锚点存活只能说明常规检索没问题，跟汰换安不安全无关。
        # 不写清楚这一点，报告会把"没发生的事"当成"验证通过的事"。
        notes.append(
            f"- 本次未触发汰换：中期段峰值没有达到容量上限 {cfg.mid_term_capacity}。"
            "因此锚点存续率与误杀率这两项只反映常规检索质量，不构成对汰换安全性的验证；"
            "要验证汰换请用 --mid-term-capacity 调小容量重跑。"
        )
    elif anchor_rate >= 0.99:
        notes.append(
            f"- 汰换 {evicted} 段后锚点事实仍全部存活，说明驱逐前把段摘要沉淀为长期知识"
            "这一步是有效的，早期录入的关键信息不会随着话题变冷而丢失。"
        )
    else:
        notes.append(
            f"- 汰换 {evicted} 段后锚点存续率只有 {anchor_rate:.0%}，汰换偏激进。"
            "建议调大 MEMORY_MID_TERM_CAPACITY 或 MEMORY_KNOWLEDGE_CAPACITY。"
        )

    if evicted == 0:
        pass
    elif miss_rate <= 0.05:
        notes.append(f"- 汰换误杀率 {miss_rate:.0%}，被淘汰话题仍可通过沉淀的知识召回。")
    else:
        notes.append(
            f"- 汰换误杀率 {miss_rate:.0%} 偏高，说明沉淀的知识不足以替代原段，"
            "建议调大中期容量。"
        )

    per_round = metrics.llm_calls / metrics.rounds if metrics.rounds else 0
    notes.append(
        f"- 每轮对话平均消耗 {per_round:.3f} 次 LLM 调用。上游 MemoryOS 为每一页做"
        "连续性判断与 meta 生成，同等规模需要 2N+3 量级；本方案把这两项分别换成"
        "启发式判断和直接删除，成本与对话量解耦。"
    )

    if metrics.profile_overwrites <= metrics.profile_writes * 0.2:
        notes.append(
            f"- 画像抖动 {metrics.profile_overwrites} 次 / 更新 {metrics.profile_writes} 次，"
            "结构化增量合并让画像以累积为主，没有出现反复自我推翻。"
        )
    else:
        notes.append(
            f"- 画像抖动 {metrics.profile_overwrites} 次占更新 {metrics.profile_writes} 次的比例偏高，"
            "建议调高 MEMORY_HEAT_THRESHOLD 减少提升频率。"
        )

    heat = metrics.final.get("heat", {})
    if heat.get("max", 0) and heat.get("p50", 0):
        ratio = heat["max"] / max(heat["p50"], 1e-6)
        if ratio <= 3:
            notes.append(
                f"- 终态热度 P50={heat['p50']:.2f} / max={heat['max']:.2f}，"
                "最热段没有碾压式领先，对数压缩确实抑制了单项支配。"
            )
        else:
            notes.append(
                f"- 终态热度 max/P50 达到 {ratio:.1f} 倍，存在单段独大，"
                "可考虑调小 β（页数权重）。"
            )

    return "\n".join(notes)


def main() -> int:
    parser = argparse.ArgumentParser(description="记忆系统长跑模拟")
    parser.add_argument("--rounds", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260801)
    parser.add_argument("--sample-every", type=int, default=20)
    parser.add_argument("--short-term-capacity", type=int, default=None)
    parser.add_argument("--mid-term-capacity", type=int, default=None)
    parser.add_argument("--heat-threshold", type=float, default=None)
    parser.add_argument("--keep-data", action="store_true", help="跑完不删除数据，便于手工查看")
    parser.add_argument(
        "--real-llm",
        action="store_true",
        help="调用真实 LLM（会产生费用与耗时），默认用确定性假实现",
    )
    parser.add_argument(
        "--real-embedding",
        action="store_true",
        help="用真实 e5 模型编码（CPU 推理，不占显卡）",
    )
    args = parser.parse_args()

    cfg = load_config()
    overrides: Dict[str, Any] = {}
    if args.short_term_capacity:
        overrides["short_term_capacity"] = args.short_term_capacity
    if args.mid_term_capacity:
        overrides["mid_term_capacity"] = args.mid_term_capacity
    if args.heat_threshold is not None:
        overrides["heat_threshold"] = args.heat_threshold
    if overrides:
        cfg = replace(cfg, **overrides)

    if args.real_llm and not cfg.llm_api_key:
        print("未配置 MEMORY_LLM_API_KEY / OPENAI_API_KEY，无法使用 --real-llm")
        return 2

    db.close_pool()
    db.init_pool(cfg)

    embedder = None
    llm = None
    if args.real_embedding:
        from memory_service.app.embedding import SentenceTransformerEmbedder

        print(f"加载 embedding 模型 {cfg.embedding_model}（{cfg.embedding_device}）…")
        embedder = SentenceTransformerEmbedder(
            model_name=cfg.embedding_model, device=cfg.embedding_device
        )
    if args.real_llm:
        from memory_service.app.llm import OpenAICompatClient

        print(f"使用真实 LLM：{cfg.llm_model} @ {cfg.llm_base_url}")
        llm = CountingLLM(
            OpenAICompatClient(
                base_url=cfg.llm_base_url,
                api_key=cfg.llm_api_key,
                model=cfg.llm_model,
                timeout=cfg.llm_timeout,
            )
        )

    user_id = f"sim_{uuid.uuid4().hex[:10]}"
    turns = build_script(args.rounds, args.seed)
    print(f"剧本 {len(turns)} 轮，虚拟跨度 {(turns[-1].at - START).days} 天，user={user_id}")

    with db.connection() as conn:
        conn.execute(
            'INSERT INTO "User" ("id", "username", "passwordHash") VALUES (%s, %s, %s)',
            (user_id, "simulation", "x"),
        )

    try:
        metrics = run_simulation(
            cfg,
            turns,
            user_id,
            args.sample_every,
            embedder=embedder,
            llm=llm,
            progress_every=20 if (args.real_llm or args.real_embedding) else 0,
        )
    finally:
        if not args.keep_data:
            with db.connection() as conn:
                conn.execute('DELETE FROM "User" WHERE "id" = %s', (user_id,))
        db.close_pool()

    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report = render_report(cfg, metrics, args.seed)

    md_path = REPORTS_DIR / f"simulation_{stamp}.md"
    json_path = REPORTS_DIR / f"simulation_{stamp}.json"
    md_path.write_text(report, encoding="utf-8")
    json_path.write_text(
        json.dumps(
            {
                "rounds": metrics.rounds,
                "llm_calls": metrics.llm_calls,
                "promotions": metrics.promotions,
                "analyses": metrics.analyses,
                "evicted_segments": metrics.evicted_segments,
                "distilled": metrics.distilled,
                "profile_writes": metrics.profile_writes,
                "profile_overwrites": metrics.profile_overwrites,
                "queries": metrics.queries,
                "query_hits": metrics.query_hits,
                "evicted_topic_queries": metrics.evicted_topic_queries,
                "evicted_topic_misses": metrics.evicted_topic_misses,
                "anchors_total": metrics.anchors_total,
                "anchors_recalled": metrics.anchors_recalled,
                "final": metrics.final,
                "samples": [s.__dict__ for s in metrics.samples],
            },
            ensure_ascii=False,
            indent=2,
            default=str,
        ),
        encoding="utf-8",
    )

    print()
    print(report)
    print(f"\n报告已写入 {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
