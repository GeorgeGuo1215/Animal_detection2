"""把某个用户的记忆内容原样打出来。

指标报告回答的是"系统在不在工作"，这个脚本回答"它到底记住了什么"——摘要写得对不对、
画像有没有跑偏、被汰换的话题留下了什么，这些只能靠眼睛看。

配合模拟脚本的 --keep-data 使用：

    python memory_service/scripts/simulate_usage.py --rounds 200 --keep-data
    python memory_service/scripts/dump_memory.py            # 不给 user-id 就挑记忆最多的那个

在 agentAndRag/ 目录下执行。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db  # noqa: E402
from memory_service.app.config import load_config  # noqa: E402


def pick_user(conn) -> Optional[str]:
    """挑记忆最多的用户。调试时库里通常只有一个跑过剧本的用户。"""
    row = conn.execute(
        """
        SELECT "userId", count(*) AS n
        FROM memory_pages
        GROUP BY "userId"
        ORDER BY n DESC
        LIMIT 1
        """
    ).fetchone()
    return row["userId"] if row else None


def dump(conn, user_id: str, pages_per_segment: int) -> None:
    print(f"用户 {user_id}")

    profile = conn.execute(
        'SELECT "profile", "version" FROM memory_profiles WHERE "userId" = %s',
        (user_id,),
    ).fetchone()

    print("\n" + "=" * 70)
    print("长期记忆 · 用户画像")
    print("=" * 70)
    if profile:
        print(f"（第 {profile['version']} 次更新）")
        print(json.dumps(profile["profile"], ensure_ascii=False, indent=2))
    else:
        print("（尚未生成）")

    knowledge = conn.execute(
        """
        SELECT "content", "source", "hitCount", "petId"
        FROM memory_knowledge WHERE "userId" = %s
        ORDER BY "hitCount" DESC, "createdAt"
        """,
        (user_id,),
    ).fetchall()

    print("\n" + "=" * 70)
    print(f"长期记忆 · 知识条目（{len(knowledge)} 条）")
    print("=" * 70)
    for item in knowledge:
        # source 区分是 LLM 抽取的，还是话题段被汰换时沉淀下来的
        origin = "汰换沉淀" if item["source"] == "evicted_segment" else "对话抽取"
        pet = f" [{item['petId']}]" if item["petId"] else ""
        print(f"· {item['content']}{pet}（{origin}，命中 {item['hitCount']} 次）")

    segments = conn.execute(
        """
        SELECT "id", "summary", "keywords", "petId", "visitCount", "pageCount", "heat"
        FROM memory_segments WHERE "userId" = %s
        ORDER BY "heat" DESC
        """,
        (user_id,),
    ).fetchall()

    print("\n" + "=" * 70)
    print(f"中期记忆 · 话题段（{len(segments)} 个，按热度降序）")
    print("=" * 70)
    for seg in segments:
        pet = f" [{seg['petId']}]" if seg["petId"] else ""
        print(
            f"\n热度 {seg['heat']:.2f} | 访问 {seg['visitCount']} 次 | "
            f"{seg['pageCount']} 页{pet}"
        )
        print(f"  摘要：{seg['summary']}")
        print(f"  关键词：{'、'.join(seg['keywords']) or '（无）'}")

        if pages_per_segment <= 0:
            continue
        pages = conn.execute(
            """
            SELECT "userInput", "analyzed" FROM memory_pages
            WHERE "segmentId" = %s ORDER BY "createdAt" LIMIT %s
            """,
            (seg["id"], pages_per_segment),
        ).fetchall()
        for page in pages:
            mark = "✓" if page["analyzed"] else "·"
            print(f"  {mark} {page['userInput']}")

    recent = conn.execute(
        """
        SELECT "userInput", "createdAt" FROM memory_short_term
        WHERE "userId" = %s ORDER BY "createdAt" DESC, "id" DESC
        """,
        (user_id,),
    ).fetchall()

    print("\n" + "=" * 70)
    print(f"短期记忆 · 尚未归档的对话（{len(recent)} 条，新→旧）")
    print("=" * 70)
    for turn in recent:
        print(f"· {turn['userInput']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="打印某用户的记忆内容")
    parser.add_argument("--user-id", default=None, help="不给则挑记忆最多的用户")
    parser.add_argument("--pages", type=int, default=2, help="每个段展示几页原始对话，0 为不展示")
    args = parser.parse_args()

    db.close_pool()
    db.init_pool(load_config())
    try:
        with db.connection() as conn:
            user_id = args.user_id or pick_user(conn)
            if not user_id:
                print("库里没有任何记忆数据。先跑："
                      "python memory_service/scripts/simulate_usage.py --keep-data")
                return 1
            dump(conn, user_id, args.pages)
    finally:
        db.close_pool()
    return 0


if __name__ == "__main__":
    sys.exit(main())
