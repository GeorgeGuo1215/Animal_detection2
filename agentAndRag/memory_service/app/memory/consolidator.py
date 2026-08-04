"""记忆提升：短期 → 中期 → 长期，以及汰换。

每跑一轮完整提升，LLM 调用次数是固定的 3 次：

1. 多主题摘要，把一批溢出的对话按话题分组
2. 用户画像增量分析
3. 长期知识抽取

上游 MemoryOS 处理 N 条对话需要 2N+3 次——每页一次连续性判断、每页一次 meta_info
生成。连续性判断在这里由 heat.is_continuous 用"时间间隔 + 向量相似度"替代，
meta_info 直接砍掉（它产出的内容在检索链路上几乎没有被消费）。

后两次调用还有个前提：只有当某个话题段的热度越过阈值时才发生。冷清的话题不会
触发画像分析，这是热度机制真正省下成本的地方。
"""

from __future__ import annotations

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence

import psycopg

from .. import prompts
from ..config import MemoryConfig
from ..heat import cosine, is_continuous
from ..llm import LLMClient, as_string_list, parse_json_object
from . import long_term, mid_term, short_term

logger = logging.getLogger(__name__)

# 单次提升最多处理多少页，防止长期沉睡的用户突然回来时一次性灌爆提示词。
_MAX_PAGES_PER_ANALYSIS = 30


def should_promote(short_term_size: int, cfg: MemoryConfig) -> bool:
    """判断是否值得派一次提升任务。

    只超出一条就跑一次的话，每轮对话都要付一次摘要 LLM 调用。攒够 promotion_batch
    条再跑，同样的对话量只需要 1/batch 的调用；代价仅仅是短期队列多留几条还没归档
    的对话——而它们本来就会出现在检索上下文里，用户体验上没有差别。
    """
    return short_term_size >= cfg.short_term_capacity + cfg.promotion_batch


def _turn_text(turn: Dict[str, Any]) -> str:
    return f"{turn.get('userInput') or turn.get('user_input') or ''}\n" \
           f"{turn.get('agentResponse') or turn.get('agent_response') or ''}"


def _normalize_turn(turn: Dict[str, Any]) -> Dict[str, Any]:
    """把数据库行统一成提示词与存储层都能直接用的形状。"""
    return {
        "id": turn.get("id"),
        "user_input": turn.get("userInput") or turn.get("user_input") or "",
        "agent_response": turn.get("agentResponse") or turn.get("agent_response") or "",
        "pet_id": turn.get("petId") or turn.get("pet_id"),
        "created_at": turn.get("createdAt") or turn.get("created_at"),
    }


def _link_continuous_turns(
    turns: List[Dict[str, Any]],
    vectors: List[List[float]],
    cfg: MemoryConfig,
) -> None:
    """给连续的对话之间连上 prevPageId。

    判断依据是"间隔够近 + 语义够像"，零 LLM 调用。链本身用于检索命中某一页时
    能顺着找回上下文，判断错了最多是少串一条链，不影响正确性。
    """
    for index in range(1, len(turns)):
        prev, curr = turns[index - 1], turns[index]
        if not prev.get("created_at") or not curr.get("created_at"):
            continue
        similarity = cosine(vectors[index - 1], vectors[index])
        if is_continuous(
            prev["created_at"],
            curr["created_at"],
            similarity,
            cfg.continuity_gap_minutes,
            cfg.continuity_similarity,
        ):
            curr["prev_page_id"] = prev["id"]


def _assign_turns(
    summaries: List[Dict[str, Any]], total: int
) -> List[Dict[str, Any]]:
    """把 LLM 给出的 turns 序号映射成实际的对话下标，并保证不丢对话。

    模型经常漏标几轮或给出越界序号。任何没被认领的对话都并进第一个主题——
    宁可归错话题，也不能让它凭空消失。
    """
    claimed: set[int] = set()
    groups: List[Dict[str, Any]] = []

    for item in summaries:
        indexes: List[int] = []
        for raw in item.get("turns") or []:
            try:
                index = int(raw) - 1
            except (TypeError, ValueError):
                continue
            if 0 <= index < total and index not in claimed:
                claimed.add(index)
                indexes.append(index)
        groups.append(
            {
                "theme": str(item.get("theme") or "").strip(),
                "content": str(item.get("content") or "").strip(),
                "keywords": as_string_list(item.get("keywords")),
                "indexes": indexes,
            }
        )

    orphans = [i for i in range(total) if i not in claimed]
    if orphans:
        if groups:
            groups[0]["indexes"] = sorted(groups[0]["indexes"] + orphans)
        else:
            groups.append(
                {"theme": "近期对话", "content": "", "keywords": [], "indexes": orphans}
            )

    return [g for g in groups if g["indexes"]]


def _clean_keywords(keywords: Sequence[str], pet_id: Optional[str]) -> List[str]:
    """剔除等同于宠物标识的关键词。

    宠物维度已经由 petId 承载，关键词再带一遍会让同一只宠物的所有话题共享一个词，
    Jaccard 被系统性抬高——真实模型跑 200 轮时，"喂养""体重""疫苗"因此全被判成
    同一话题，18 个段塌成 4 个。提示词里也写了这条约束，但模型不保证遵守，代码得兜底。
    """
    if not pet_id:
        return [k for k in keywords if k]
    lowered = pet_id.strip().lower()
    cleaned = [k for k in keywords if k and k.strip().lower() != lowered]
    # 全被剔掉说明模型只给了宠物名，那还不如留着原样，至少不是空的
    return cleaned or [k for k in keywords if k]


def _split_by_pet(
    groups: List[Dict[str, Any]], turns: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    """把每个话题组再按宠物拆开，一个组内只留同一只宠物的对话。

    LLM 的多主题分组只按话题走，"咪咪的体重"和"豆豆的体重"会被归进同一组。
    若直接建段，段的 petId 只能取组内第一个非空值，另一只宠物的页就被贴上了错误
    的标签——长跑里 18 个段有 7 个混入了别的宠物，最严重的一个 51 页里混了 14 页。

    后果不在页级检索（页有自己的 petId），而在段级：按宠物过滤段会漏，段摘要被汰换
    时沉淀成知识还会把另一只宠物的信息张冠李戴。

    拆分是纯 Python 的分桶，不额外调用 LLM。没有 petId 的对话单独成桶，不与任何
    具名宠物混在一起。
    """
    result: List[Dict[str, Any]] = []
    for group in groups:
        buckets: Dict[Optional[str], List[int]] = {}
        for index in group["indexes"]:
            buckets.setdefault(turns[index].get("pet_id"), []).append(index)
        if len(buckets) <= 1:
            result.append(group)
            continue
        # 保持原有顺序，让"先出现的宠物"先建段，结果可复现
        for pet_id in sorted(buckets, key=lambda p: buckets[p][0]):
            result.append({**group, "indexes": buckets[pet_id]})
    return result


def promote_short_to_mid(
    conn: psycopg.Connection,
    *,
    user_id: str,
    cfg: MemoryConfig,
    embedder,
    llm: LLMClient,
    now: datetime,
) -> Dict[str, Any]:
    """把溢出的短期对话按话题并入中期记忆。消耗 1 次 LLM 调用。"""
    drained = short_term.drain_overflow(conn, user_id, keep=cfg.short_term_capacity)
    if not drained:
        return {"promoted": 0, "segments_created": 0, "segments_merged": 0, "llm_calls": 0}

    turns = [_normalize_turn(row) for row in drained]
    vectors = embedder.embed_documents([_turn_text(t) for t in turns])
    _link_continuous_turns(turns, vectors, cfg)

    raw = llm.complete(prompts.summary_messages(turns), temperature=0.2, max_tokens=1500)
    summaries = parse_json_object(raw).get("summaries") or []
    if not isinstance(summaries, list):
        summaries = []
    groups = _split_by_pet(_assign_turns(summaries, len(turns)), turns)

    created = 0
    merged = 0
    for group in groups:
        summary_text = group["content"] or group["theme"] or "近期对话"
        # 拆分后组内 petId 已经唯一，取第一条即可代表整组
        pet_id = turns[group["indexes"][0]].get("pet_id")
        keywords = _clean_keywords(
            group["keywords"] or ([group["theme"]] if group["theme"] else []), pet_id
        )
        summary_embedding = embedder.embed_query(f"{group['theme']} {summary_text}")

        best = mid_term.find_best_segment(
            conn,
            user_id=user_id,
            embedding=summary_embedding,
            keywords=keywords,
            threshold=cfg.segment_similarity_threshold,
            keyword_weight=cfg.keyword_weight,
            pet_id=pet_id,
        )
        if best:
            segment_id = best["id"]
            merged += 1
            # 段吸收了新话题，摘要与向量要跟着走，否则它代表的内容会越来越名不副实
            mid_term.refresh_summary(
                conn,
                segment_id=segment_id,
                summary=summary_text,
                keywords=keywords,
                now=now,
                embed=embedder.embed_query,
            )
        else:
            segment_id = mid_term.create_segment(
                conn,
                user_id=user_id,
                summary=summary_text,
                keywords=keywords,
                embedding=summary_embedding,
                now=now,
                pet_id=pet_id,
                params=cfg.heat,
            )
            created += 1

        mid_term.add_pages(
            conn,
            segment_id=segment_id,
            user_id=user_id,
            pages=[
                {
                    "id": turns[i]["id"],
                    "user_input": turns[i]["user_input"],
                    "agent_response": turns[i]["agent_response"],
                    "embedding": vectors[i],
                    "pet_id": turns[i].get("pet_id"),
                    "prev_page_id": turns[i].get("prev_page_id"),
                    "created_at": turns[i].get("created_at"),
                }
                for i in group["indexes"]
            ],
            now=now,
            params=cfg.heat,
        )

    return {
        "promoted": len(turns),
        "segments_created": created,
        "segments_merged": merged,
        "llm_calls": 1,
    }


def promote_mid_to_long(
    conn: psycopg.Connection,
    *,
    user_id: str,
    cfg: MemoryConfig,
    embedder,
    llm: LLMClient,
    now: datetime,
) -> Dict[str, Any]:
    """热度越阈的话题段提升为长期画像与知识。消耗 2 次 LLM 调用。

    热度没到阈值时一次调用都不发生——这是整套热度机制的成本意义所在。
    """
    segment = mid_term.hottest_segment(
        conn,
        user_id=user_id,
        threshold=cfg.heat_threshold,
        now=now,
        params=cfg.heat,
        min_unanalyzed=cfg.analysis_min_pages,
    )
    if not segment:
        return {"analyzed_segments": 0, "knowledge_added": 0, "profile_updated": False, "llm_calls": 0}

    pages = mid_term.unanalyzed_pages(conn, segment["id"], limit=_MAX_PAGES_PER_ANALYSIS)
    if not pages:
        return {"analyzed_segments": 0, "knowledge_added": 0, "profile_updated": False, "llm_calls": 0}

    turns = [_normalize_turn(row) for row in pages]

    profile_raw = llm.complete(prompts.profile_messages(turns), temperature=0.2, max_tokens=800)
    delta = parse_json_object(profile_raw)
    updated = long_term.update_profile(
        conn, user_id=user_id, delta=delta, now=now
    ) is not None

    knowledge_raw = llm.complete(prompts.knowledge_messages(turns), temperature=0.2, max_tokens=800)
    facts = as_string_list(parse_json_object(knowledge_raw).get("facts"))

    added = 0
    if facts:
        vectors = embedder.embed_documents(facts)
        for fact, vector in zip(facts, vectors):
            if long_term.add_knowledge(
                conn,
                user_id=user_id,
                content=fact,
                embedding=vector,
                pet_id=segment.get("petId"),
                source="extraction",
                now=now,
            ):
                added += 1

    mid_term.mark_analyzed(
        conn,
        segment_id=segment["id"],
        page_ids=[row["id"] for row in pages],
        now=now,
        params=cfg.heat,
    )

    return {
        "analyzed_segments": 1,
        "knowledge_added": added,
        "profile_updated": updated,
        "llm_calls": 2,
    }


def evict(
    conn: psycopg.Connection,
    *,
    user_id: str,
    cfg: MemoryConfig,
    now: datetime,
) -> Dict[str, Any]:
    """汰换超容量的最冷话题段，不消耗 LLM 调用。

    被淘汰的段不是直接丢弃：它的摘要先沉淀成一条长期知识。段的原始对话页确实
    没了，但"曾经聊过什么"这个信息保留下来，后续检索仍有机会命中。这是相对上游
    直接 pop 掉最冷条目的关键改进——否则用户三个月前提过的过敏史会随着话题变冷
    彻底消失。
    """
    victims = mid_term.overflow_segments(
        conn,
        user_id=user_id,
        capacity=cfg.mid_term_capacity,
        now=now,
        params=cfg.heat,
    )
    if not victims:
        removed_knowledge = long_term.enforce_knowledge_capacity(
            conn, user_id=user_id, capacity=cfg.knowledge_capacity
        )
        return {"evicted_segments": 0, "distilled": 0, "evicted_knowledge": removed_knowledge}

    distilled = 0
    for victim in victims:
        summary = (victim.get("summary") or "").strip()
        if not summary:
            continue
        # 复用段摘要已有的向量，省掉一次编码。
        if long_term.add_knowledge(
            conn,
            user_id=user_id,
            content=summary,
            embedding=victim.get("summaryEmbedding"),
            pet_id=victim.get("petId"),
            source="evicted_segment",
            now=now,
        ):
            distilled += 1

    mid_term.delete_segments(conn, [v["id"] for v in victims])
    removed_knowledge = long_term.enforce_knowledge_capacity(
        conn, user_id=user_id, capacity=cfg.knowledge_capacity
    )

    return {
        "evicted_segments": len(victims),
        "distilled": distilled,
        "evicted_knowledge": removed_knowledge,
    }


def consolidate(
    conn: psycopg.Connection,
    *,
    user_id: str,
    cfg: MemoryConfig,
    embedder,
    llm: LLMClient,
    now: Optional[datetime] = None,
) -> Dict[str, Any]:
    """完整的一轮提升：短→中、中→长、汰换。

    now 可注入，长跑模拟靠它推进虚拟时钟来观察时间衰减的实际效果。
    """
    moment = now or datetime.now()
    to_mid = promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=moment
    )
    to_long = promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=moment
    )
    evicted = evict(conn, user_id=user_id, cfg=cfg, now=moment)

    return {
        **to_mid,
        **to_long,
        **evicted,
        "llm_calls": to_mid["llm_calls"] + to_long["llm_calls"],
    }
