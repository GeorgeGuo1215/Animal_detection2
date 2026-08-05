"""提升流程与检索的测试。

覆盖短期溢出 → 中期成段 → 热度越阈 → 画像/知识生成 → 汰换 → 检索召回这条主链，
并把"LLM 调用次数不随对话条数增长"作为硬约束锁住。
"""

from __future__ import annotations

import sys
from dataclasses import replace
from datetime import timedelta
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.config import load_config  # noqa: E402
from memory_service.app.db import to_vector_param  # noqa: E402
from memory_service.app.memory import (  # noqa: E402
    consolidator,
    long_term,
    mid_term,
    retriever,
    short_term,
)
from memory_service.tests.helpers import BASE_TIME, RecordingLLM  # noqa: E402


@pytest.fixture
def cfg():
    """小容量配置，让溢出、提升与汰换在几轮对话内就能被触发。"""
    return replace(
        load_config(),
        short_term_capacity=2,
        mid_term_capacity=3,
        knowledge_capacity=10,
        heat_threshold=2.0,
        promotion_batch=2,
        analysis_min_pages=3,
    )


def _feed(conn, user_id, turns, start=BASE_TIME, pet_id=None):
    """灌入若干轮对话，每轮间隔 1 分钟。"""
    for index, (question, answer) in enumerate(turns):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=question,
            agent_response=answer,
            pet_id=pet_id,
            created_at=start + timedelta(minutes=index),
        )


# ---------------------------------------------------------------- 短期 → 中期


def test_short_term_overflow_becomes_segment(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [(f"喂养问题{i}", f"喂养回答{i}") for i in range(5)])

    result = consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    assert result["promoted"] == 3
    assert result["segments_created"] == 1
    # 容量内的最新两条仍留在短期
    assert short_term.count(conn, user_id) == 2


def test_nothing_happens_before_overflow(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [("喂养问题", "回答")])
    result = consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    assert result["promoted"] == 0
    # 没有溢出就不该发生任何 LLM 调用
    assert llm.call_count == 0


def test_same_topic_merges_into_existing_segment(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(5)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    later = BASE_TIME + timedelta(hours=2)
    _feed(conn, user_id, [(f"又是喂养问题{i}", f"回答{i}") for i in range(5)], start=later)
    result = consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=later
    )

    assert result["segments_merged"] >= 1
    assert mid_term.count_segments(conn, user_id) == 1


def test_merged_segment_summary_reflects_new_content(conn, user_id, cfg, embedder, llm):
    """段吸收新对话后，摘要要跟着变。

    这是真实模型跑 200 轮暴露的问题：段摘要建段时写一次就再不更新，最后出现摘要写着
    "鸡肉过敏"、段里实际装着 33 页体重与喂养的情况。摘要既是检索第一层的召回依据，
    又是汰换时沉淀成长期知识的原文，失真的代价不小。
    """
    # 阈值压到 0 强制合并：这里要验的是"合并之后摘要有没有跟着走"，
    # 而不是"会不会合并"（后者由 test_same_topic_merges_into_existing_segment 覆盖）
    always_merge = replace(cfg, segment_similarity_threshold=0.0)

    class DriftingLLM(RecordingLLM):
        """每次归档给出不同措辞的摘要，模拟真实模型的实际表现。"""

        def __init__(self):
            super().__init__()
            self.round = 0

        def complete(self, messages, **kwargs):
            super().complete(messages, **kwargs)
            self.round += 1
            return (
                f'{{"summaries": [{{"theme": "话题{self.round}",'
                f' "content": "第{self.round}批对话的摘要",'
                f' "keywords": ["词{self.round}"], "turns": [1, 2, 3, 4, 5]}}]}}'
            )

    drifting = DriftingLLM()
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(5)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=always_merge, embedder=embedder,
        llm=drifting, now=BASE_TIME,
    )
    before = conn.execute(
        'SELECT "summary", "keywords", "summaryEmbedding" FROM memory_segments'
        ' WHERE "userId" = %s',
        (user_id,),
    ).fetchone()

    later = BASE_TIME + timedelta(hours=2)
    _feed(conn, user_id, [(f"又是喂养问题{i}", f"回答{i}") for i in range(5)], start=later)
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=always_merge, embedder=embedder,
        llm=drifting, now=later,
    )
    after = conn.execute(
        'SELECT "summary", "keywords", "summaryEmbedding" FROM memory_segments'
        ' WHERE "userId" = %s',
        (user_id,),
    ).fetchone()

    assert after["summary"] != before["summary"]
    # 旧话题不能被新话题顶掉，段代表的是它装过的全部内容
    assert before["summary"] in after["summary"]
    # 关键词取并集，段涵盖的话题变宽了才检索得到
    assert set(before["keywords"]) <= set(after["keywords"])
    # 向量要基于合并后的摘要重算，否则检索仍按旧面貌召回
    assert to_vector_param(after["summaryEmbedding"]) != to_vector_param(
        before["summaryEmbedding"]
    )


def test_segment_summary_stays_bounded_after_many_merges(conn, user_id, cfg, embedder, llm):
    """反复合并不能让摘要无限膨胀。

    有界拼接是"直接替换会丢历史、无限追加会撑爆、再调 LLM 融合会破坏固定三次调用
    预算"之间的折中，这条用例锁住"有界"这一半。
    """
    for round_index in range(6):
        at = BASE_TIME + timedelta(hours=2 * round_index)
        _feed(
            conn,
            user_id,
            [(f"喂养问题{round_index}_{i}", f"回答{i}") for i in range(5)],
            start=at,
        )
        consolidator.promote_short_to_mid(
            conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=at
        )

    row = conn.execute(
        'SELECT "summary", "keywords" FROM memory_segments WHERE "userId" = %s',
        (user_id,),
    ).fetchone()
    assert len(row["summary"].split(mid_term.SUMMARY_SEPARATOR)) <= mid_term.MAX_SUMMARY_PARTS
    assert len(row["keywords"]) <= mid_term.MAX_KEYWORDS


def test_pet_name_is_stripped_from_keywords(conn, user_id, cfg, embedder, llm):
    """关键词不该带宠物标识。

    宠物维度已由 petId 承载，关键词再带一遍，同一只宠物的所有话题就都共享这个词，
    Jaccard 被系统性抬高——真实模型跑 200 轮时，喂养/体重/疫苗因此全被判成同一话题。
    提示词里写了这条约束，但模型不保证遵守，所以代码要兜底，这条用例锁的是兜底。
    """

    class PetNameLLM(RecordingLLM):
        def complete(self, messages, **kwargs):
            super().complete(messages, **kwargs)
            return (
                '{"summaries": [{"theme": "体重管理", "content": "咪咪的体重变化",'
                ' "keywords": ["咪咪", "体重"], "turns": [1, 2, 3, 4, 5]}]}'
            )

    _feed(
        conn,
        user_id,
        [(f"体重问题{i}", f"回答{i}") for i in range(5)],
        pet_id="咪咪",
    )
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=PetNameLLM(), now=BASE_TIME
    )

    row = conn.execute(
        'SELECT "keywords" FROM memory_segments WHERE "userId" = %s', (user_id,)
    ).fetchone()
    assert "咪咪" not in row["keywords"]
    assert "体重" in row["keywords"]


def test_single_page_topic_still_gets_analysed(conn, user_id, cfg, embedder, llm):
    """只有一页的冷门话题也必须被分析。

    真实模型跑 200 轮暴露过这条：话题段做细之后，"去年打疫苗过敏"独占一个 1 页的段，
    热度达标却凑不满当时 5 页的分析门槛，于是全库 37% 的对话从未被分析，过敏史一条
    都没沉淀下来。宠物医疗场景里低频恰恰是重要信息的特征，这条用例防止门槛被调回去。
    """
    segment_id = mid_term.create_segment(
        conn,
        user_id=user_id,
        summary="豆豆的疫苗过敏史",
        keywords=["疫苗", "过敏"],
        embedding=embedder.embed_query("豆豆的疫苗过敏史"),
        now=BASE_TIME,
        pet_id="豆豆",
    )
    mid_term.add_pages(
        conn,
        segment_id=segment_id,
        user_id=user_id,
        pages=[{"user_input": "豆豆去年打疫苗过敏", "agent_response": "已记录"}],
        now=BASE_TIME,
    )

    # 热度阈值压到 0，把"页数门槛"单独隔离出来：这里要验的就是它
    hot = mid_term.hottest_segment(
        conn,
        user_id=user_id,
        threshold=0.0,
        now=BASE_TIME,
        params=cfg.heat,
        min_unanalyzed=1,
    )
    assert hot is not None, "单页段必须够得着分析，否则关键信息永远沉淀不下来"
    assert hot["id"] == segment_id

    # 门槛调回 5 就会把它挡在门外——这正是当初丢掉过敏史的原因
    blocked = mid_term.hottest_segment(
        conn,
        user_id=user_id,
        threshold=0.0,
        now=BASE_TIME,
        params=cfg.heat,
        min_unanalyzed=5,
    )
    assert blocked is None


def test_different_topics_produce_separate_segments(conn, user_id, cfg, embedder, llm):
    _feed(
        conn,
        user_id,
        [
            ("喂养问题一", "回答"),
            ("喂养问题二", "回答"),
            ("疫苗问题一", "回答"),
            ("疫苗问题二", "回答"),
            ("皮肤问题一", "回答"),
        ],
    )
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    assert mid_term.count_segments(conn, user_id) >= 2


def _segment_page_pets(conn, user_id):
    """段的宠物标签与它每一页的宠物标签配对，用来断言两者一致。"""
    return conn.execute(
        """
        SELECT s."petId" AS seg_pet, p."petId" AS page_pet
        FROM memory_segments s JOIN memory_pages p ON p."segmentId" = s."id"
        WHERE s."userId" = %s
        """,
        (user_id,),
    ).fetchall()


def test_same_topic_for_different_pets_does_not_share_a_segment(
    conn, user_id, cfg, embedder, llm
):
    """两只宠物聊同一个话题，必须落在各自的段里。

    LLM 的多主题分组只按话题走，"咪咪的喂养"和"豆豆的喂养"会被归进同一组；若不按
    宠物再拆一次，段的 petId 只能取组内第一个非空值，另一只宠物的页就被贴错了标签。
    """
    for index, pet in enumerate(["咪咪", "豆豆", "咪咪", "豆豆", "咪咪", "豆豆"]):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"{pet}的喂养安排怎么调整",
            agent_response="建议保持规律",
            pet_id=pet,
            created_at=BASE_TIME + timedelta(minutes=index),
        )

    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    pairs = _segment_page_pets(conn, user_id)
    assert pairs
    assert all(row["seg_pet"] == row["page_pet"] for row in pairs)
    assert {row["seg_pet"] for row in pairs} == {"咪咪", "豆豆"}


def test_untagged_dialogue_does_not_join_a_pet_segment(conn, user_id, cfg, embedder, llm):
    """没带宠物的对话不该被并进某只宠物的段，否则段摘要沉淀成知识时会张冠李戴。"""
    _feed(conn, user_id, [(f"喂养问题{i}", "回答") for i in range(5)], pet_id="咪咪")
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    later = BASE_TIME + timedelta(hours=2)
    _feed(conn, user_id, [(f"喂养问题{i}", "回答") for i in range(5)], start=later)
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=later
    )

    pairs = _segment_page_pets(conn, user_id)
    assert all(row["seg_pet"] == row["page_pet"] for row in pairs)
    assert {row["seg_pet"] for row in pairs} == {"咪咪", None}


def test_no_dialogue_is_lost_when_llm_omits_turn_numbers(conn, user_id, cfg, embedder):
    """模型漏标序号是常态，漏掉的对话必须被兜底收进某个段。"""

    class SloppyLLM(RecordingLLM):
        def _summaries(self, prompt):
            return '{"summaries": [{"theme": "喂养", "content": "摘要", "keywords": ["喂养"], "turns": [1]}]}'

    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(5)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=SloppyLLM(), now=BASE_TIME
    )

    pages = conn.execute(
        'SELECT count(*) AS n FROM memory_pages WHERE "userId" = %s', (user_id,)
    ).fetchone()
    assert pages["n"] == 3


def test_malformed_llm_output_still_preserves_dialogue(conn, user_id, cfg, embedder):
    """模型返回的不是 JSON 时，对话不能凭空消失。"""

    class BrokenLLM(RecordingLLM):
        def complete(self, messages, *, temperature=0.3, max_tokens=1024):
            self.calls.append({"prompt": ""})
            return "抱歉，我无法完成这个请求。"

    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(5)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=BrokenLLM(), now=BASE_TIME
    )

    pages = conn.execute(
        'SELECT count(*) AS n FROM memory_pages WHERE "userId" = %s', (user_id,)
    ).fetchone()
    assert pages["n"] == 3


# ---------------------------------------------------------------- 中期 → 长期


def test_cold_segment_does_not_trigger_llm(conn, user_id, cfg, embedder, llm):
    """热度没到阈值就一次 LLM 都不该发生，这是热度机制省成本的地方。"""
    hot_threshold_cfg = replace(cfg, heat_threshold=99.0)
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(5)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=hot_threshold_cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    before = llm.call_count

    result = consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=hot_threshold_cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    assert result["analyzed_segments"] == 0
    assert llm.call_count == before


def test_hot_segment_produces_profile_and_knowledge(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(8)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    result = consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    assert result["analyzed_segments"] == 1
    assert result["profile_updated"] is True
    assert result["knowledge_added"] >= 1

    profile = long_term.get_profile(conn, user_id)["profile"]
    assert "communication" in profile


def test_analyzed_pages_are_not_reprocessed(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(8)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    calls_after_first = llm.call_count

    second = consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    assert second["analyzed_segments"] == 0
    assert llm.call_count == calls_after_first


# ---------------------------------------------------------------- LLM 调用预算


@pytest.mark.parametrize("dialogue_count", [8, 16, 32])
def test_llm_calls_stay_at_three_regardless_of_volume(
    conn, user_id, cfg, embedder, dialogue_count
):
    """一轮提升恒定 3 次 LLM 调用，与这批对话有多少条无关。

    上游是每页一次连续性判断加一次 meta 生成，处理 N 条要 2N+3 次；本方案把这两项
    分别换成启发式判断和直接删除，剩下的三次是多主题摘要、画像分析、知识抽取。
    这是整套重写里最直接的性能收益，必须随对话量变化反复验证。
    """
    llm = RecordingLLM()
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(dialogue_count)])

    result = consolidator.consolidate(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    assert llm.call_count == 3
    assert result["llm_calls"] == 3


def test_promotion_is_batched_to_amortise_llm_cost(conn, user_id, cfg, embedder):
    """攒够一批才提升：溢出一条时不该派活，否则每轮对话都要付一次 LLM。"""
    _feed(conn, user_id, [(f"喂养问题{i}", "回答") for i in range(cfg.short_term_capacity + 1)])
    assert consolidator.should_promote(short_term.count(conn, user_id), cfg) is False

    _feed(
        conn,
        user_id,
        [(f"补充问题{i}", "回答") for i in range(cfg.promotion_batch)],
        start=BASE_TIME + timedelta(hours=1),
    )
    assert consolidator.should_promote(short_term.count(conn, user_id), cfg) is True


def test_small_batch_of_new_pages_does_not_retrigger_analysis(conn, user_id, cfg, embedder):
    """热段每加一页就重新分析一次的话，画像会被反复重写，成本回到与对话量成正比。"""
    llm = RecordingLLM()
    _feed(conn, user_id, [(f"喂养问题{i}", "回答") for i in range(8)])
    consolidator.consolidate(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    calls_after_first = llm.call_count

    # 只新增不足 analysis_min_pages 的对话，不该触发第二次画像分析
    later = BASE_TIME + timedelta(hours=1)
    _feed(conn, user_id, [("喂养再问一句", "回答")], start=later)
    result = consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=later
    )

    assert result["analyzed_segments"] == 0
    assert llm.call_count == calls_after_first


# ---------------------------------------------------------------- 汰换


def test_eviction_distills_summary_into_knowledge(conn, user_id, cfg, embedder, llm):
    """被淘汰的段不能凭空消失，摘要要沉淀成长期知识。"""
    tiny = replace(cfg, mid_term_capacity=1)
    for index, topic in enumerate(("喂养", "疫苗", "皮肤")):
        moment = BASE_TIME + timedelta(days=index)
        _feed(
            conn,
            user_id,
            [(f"{topic}问题{i}", "回答") for i in range(4)],
            start=moment,
        )
        consolidator.promote_short_to_mid(
            conn, user_id=user_id, cfg=tiny, embedder=embedder, llm=llm, now=moment
        )

    now = BASE_TIME + timedelta(days=2)
    result = consolidator.evict(conn, user_id=user_id, cfg=tiny, now=now)

    assert result["evicted_segments"] >= 1
    assert result["distilled"] >= 1
    assert mid_term.count_segments(conn, user_id) == 1

    sources = conn.execute(
        'SELECT DISTINCT "source" FROM memory_knowledge WHERE "userId" = %s', (user_id,)
    ).fetchall()
    assert "evicted_segment" in [row["source"] for row in sources]


def test_eviction_keeps_the_hottest_segment(conn, user_id, cfg, embedder, llm):
    tiny = replace(cfg, mid_term_capacity=1)
    _feed(conn, user_id, [(f"喂养问题{i}", "回答") for i in range(4)])
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=tiny, embedder=embedder, llm=llm, now=BASE_TIME
    )
    # 清掉容量内残留的那几轮，模拟这一天的对话已经全部归档完毕。
    # 否则它们会在十天后连同新对话一起被提升，把喂养段的 lastVisitAt 一并刷新——
    # 那是符合预期的行为（对话确实是那时才归档的），但会让这个用例测不到热度衰减。
    conn.execute('DELETE FROM memory_short_term WHERE "userId" = %s', (user_id,))

    later = BASE_TIME + timedelta(days=10)
    _feed(conn, user_id, [(f"疫苗问题{i}", "回答") for i in range(4)], start=later)
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=tiny, embedder=embedder, llm=llm, now=later
    )

    consolidator.evict(conn, user_id=user_id, cfg=tiny, now=later)

    survivors = conn.execute(
        'SELECT "summary" FROM memory_segments WHERE "userId" = %s', (user_id,)
    ).fetchall()
    assert len(survivors) == 1
    # 十天前的话题已经凉透，留下的应当是刚聊过的那个
    assert "疫苗" in survivors[0]["summary"]


# ---------------------------------------------------------------- 检索


def test_retrieval_recalls_relevant_history(conn, user_id, cfg, embedder, llm):
    _feed(
        conn,
        user_id,
        [("我家猫皮肤发红", "可能是过敏")] + [(f"喂养问题{i}", "回答") for i in range(5)],
    )
    consolidator.consolidate(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    context = retriever.build_context(
        conn,
        user_id=user_id,
        query="皮肤问题怎么处理",
        cfg=cfg,
        embedder=embedder,
        now=BASE_TIME + timedelta(hours=1),
    )

    recalled = " ".join(item["user_input"] for item in context["related_pages"])
    assert "皮肤" in recalled


def test_retrieval_includes_profile_and_recent_dialogue(conn, user_id, cfg, embedder, llm):
    _feed(conn, user_id, [(f"喂养问题{i}", f"回答{i}") for i in range(8)])
    consolidator.consolidate(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    context = retriever.build_context(
        conn, user_id=user_id, query="喂养建议", cfg=cfg, embedder=embedder, now=BASE_TIME
    )

    assert context["profile"]
    assert context["recent_dialogue"]
    assert retriever.format_context(context)


def test_retrieval_never_returns_other_users_memory(
    conn, user_id, other_user_id, cfg, embedder, llm
):
    _feed(conn, other_user_id, [(f"皮肤问题{i}", "别人的回答") for i in range(6)])
    consolidator.consolidate(
        conn, user_id=other_user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    context = retriever.build_context(
        conn, user_id=user_id, query="皮肤问题", cfg=cfg, embedder=embedder, now=BASE_TIME
    )

    assert context["related_pages"] == []
    assert context["knowledge"] == []
    assert context["profile"] == {}
