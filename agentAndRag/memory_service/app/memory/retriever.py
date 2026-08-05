"""跨三层的上下文检索。

只负责把相关记忆吐出来，不生成回答。上游 MemoryOS 的 ``get_response()`` 把 LLM 应答
也包进去了，那是 agent 的职责，记忆系统插手只会让两边的提示词互相打架。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional

import psycopg

from ..config import MemoryConfig
from . import long_term, mid_term, short_term


def build_context(
    conn: psycopg.Connection,
    *,
    user_id: str,
    query: str,
    cfg: MemoryConfig,
    embedder,
    now: datetime,
    pet_id: Optional[str] = None,
) -> Dict[str, Any]:
    """组装一次检索的完整上下文。

    检索本身会抬高命中话题段的热度，也会给命中的知识条目计数——被反复问到的内容
    因此更不容易被汰换掉。
    """
    query_vector = embedder.embed_query(query) if query else []

    profile = long_term.get_profile(conn, user_id)

    knowledge: List[Dict[str, Any]] = []
    related: List[Dict[str, Any]] = []
    if query_vector:
        knowledge = long_term.search_knowledge(
            conn,
            user_id=user_id,
            embedding=query_vector,
            top_k=cfg.top_k_knowledge,
            now=now,
            pet_id=pet_id,
        )
        related = mid_term.search(
            conn,
            user_id=user_id,
            embedding=query_vector,
            top_k_segments=cfg.top_k_segments,
            top_k_pages=cfg.top_k_pages,
            now=now,
            params=cfg.heat,
            pet_id=pet_id,
        )

    recent = short_term.recent(conn, user_id, limit=cfg.short_term_context_size)

    return {
        "user_id": user_id,
        "profile": profile["profile"],
        "profile_version": profile["version"],
        "knowledge": knowledge,
        "related_pages": related,
        "recent_dialogue": [
            {
                "user_input": row["userInput"],
                "agent_response": row["agentResponse"],
                "created_at": row["createdAt"],
            }
            for row in recent
        ],
    }


def format_context(context: Dict[str, Any]) -> str:
    """把检索结果拼成可直接塞进 system prompt 的中文文本块。

    调用方也可以拿结构化结果自己排版，这里只是提供一个开箱可用的默认形态。
    """
    blocks: List[str] = []

    profile = context.get("profile") or {}
    if profile:
        lines = []
        for key, label in (
            ("communication", "沟通偏好"),
            ("concerns", "长期关注"),
            ("petFacts", "宠物信息"),
            ("healthWatch", "健康关注点"),
        ):
            value = profile.get(key)
            if value:
                lines.append(f"- {label}: {_render(value)}")
        if lines:
            blocks.append("【用户画像】\n" + "\n".join(lines))

    knowledge = context.get("knowledge") or []
    if knowledge:
        blocks.append(
            "【长期记忆】\n"
            + "\n".join(f"- {item['content']}" for item in knowledge)
        )

    related = context.get("related_pages") or []
    if related:
        blocks.append(
            "【相关历史对话】\n"
            + "\n".join(
                f"- 用户: {item['user_input']}\n  助手: {item['agent_response']}"
                for item in related
            )
        )

    recent = context.get("recent_dialogue") or []
    if recent:
        blocks.append(
            "【近期对话】\n"
            + "\n".join(
                f"- 用户: {_clip(item.get('user_input'), 800)}\n"
                f"  助手: {_clip(item.get('agent_response'), 420)}"
                for item in recent
            )
        )

    return "\n\n".join(blocks)


def _clip(value: Any, limit: int) -> str:
    text = str(value or "").strip()
    if len(text) <= limit:
        return text
    return text[: max(0, limit - 1)].rstrip() + "…"


def _render(value: Any) -> str:
    if isinstance(value, dict):
        return "; ".join(f"{k}: {_render(v)}" for k, v in value.items())
    if isinstance(value, (list, tuple)):
        return "、".join(_render(v) for v in value)
    return str(value)
