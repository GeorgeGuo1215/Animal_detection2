"""Deterministic source graph for every derived memory item."""
from __future__ import annotations

from typing import Any, Iterable

import psycopg
from psycopg.types.json import Jsonb


def link(
    conn: psycopg.Connection,
    *,
    user_id: str,
    source_type: str,
    source_id: str,
    target_type: str,
    target_id: str,
    generation_tag: str,
    metadata: dict[str, Any] | None = None,
) -> None:
    conn.execute(
        """
        INSERT INTO memory_derivations
            ("userId", "sourceType", "sourceId", "targetType", "targetId",
             "generationTag", "metadata")
        VALUES (%s, %s, %s, %s, %s, %s, %s)
        ON CONFLICT DO NOTHING
        """,
        (user_id, source_type, source_id, target_type, target_id, generation_tag, Jsonb(metadata or {})),
    )


def link_many(
    conn: psycopg.Connection,
    *,
    user_id: str,
    sources: Iterable[tuple[str, str]],
    target_type: str,
    target_id: str,
    generation_tag: str,
    metadata: dict[str, Any] | None = None,
) -> None:
    for source_type, source_id in sources:
        link(conn, user_id=user_id, source_type=source_type, source_id=source_id,
             target_type=target_type, target_id=target_id,
             generation_tag=generation_tag, metadata=metadata)


def target_source_count(
    conn: psycopg.Connection, *, user_id: str, target_type: str, target_id: str
) -> int:
    row = conn.execute(
        'SELECT count(*) AS n FROM memory_derivations '
        'WHERE "userId"=%s AND "targetType"=%s AND "targetId"=%s',
        (user_id, target_type, target_id),
    ).fetchone()
    return int(row["n"]) if row else 0


def remove_source(
    conn: psycopg.Connection, *, user_id: str, source_type: str, source_id: str
) -> list[dict[str, Any]]:
    rows = conn.execute(
        'DELETE FROM memory_derivations WHERE "userId"=%s AND "sourceType"=%s '
        'AND "sourceId"=%s RETURNING "targetType", "targetId", "generationTag"',
        (user_id, source_type, source_id),
    ).fetchall()
    return [dict(row) for row in rows]


def remove_target(
    conn: psycopg.Connection, *, user_id: str, target_type: str, target_id: str
) -> None:
    conn.execute(
        'DELETE FROM memory_derivations WHERE "userId"=%s AND "targetType"=%s AND "targetId"=%s',
        (user_id, target_type, target_id),
    )


def target_segment_sources(
    conn: psycopg.Connection, *, user_id: str, target_type: str, target_id: str
) -> list[str]:
    rows = conn.execute(
        'SELECT DISTINCT "sourceId" FROM memory_derivations '
        'WHERE "userId"=%s AND "sourceType"=\'segment\' '
        'AND "targetType"=%s AND "targetId"=%s',
        (user_id, target_type, target_id),
    ).fetchall()
    return [str(row["sourceId"]) for row in rows]


def delete_orphan_segment_sources(
    conn: psycopg.Connection, *, user_id: str, segment_ids: Iterable[str]
) -> int:
    """Delete source segments no longer supporting any retained long-term item.

    A segment shared by another knowledge/profile target remains intact. Pages
    belonging to a truly orphaned segment are removed by the database cascade.
    """
    deleted = 0
    for segment_id in set(segment_ids):
        remaining = conn.execute(
            'SELECT 1 FROM memory_derivations WHERE "userId"=%s '
            'AND "sourceType"=\'segment\' AND "sourceId"=%s LIMIT 1',
            (user_id, segment_id),
        ).fetchone()
        if remaining:
            continue
        row = conn.execute(
            'DELETE FROM memory_segments WHERE "userId"=%s AND "id"=%s RETURNING "id"',
            (user_id, segment_id),
        ).fetchone()
        deleted += int(row is not None)
    return deleted
