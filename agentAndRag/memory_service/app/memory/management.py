"""User-visible memory listing and provenance-based physical deletion."""
from __future__ import annotations

from typing import Any
import hashlib
import json

import psycopg

from . import provenance

_SNAPSHOT_TABLES = (
    "memory_ingest_receipts",
    "memory_short_term",
    "memory_segments",
    "memory_pages",
    "memory_profiles",
    "memory_knowledge",
    "memory_tasks",
    "memory_derivations",
)


def _snapshot_checksum(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def export_snapshot(conn: psycopg.Connection, *, user_id: str) -> dict[str, Any]:
    subject = conn.execute(
        'SELECT to_jsonb(subject) AS data FROM memory_subjects AS subject WHERE "id"=%s',
        (user_id,),
    ).fetchone()
    if not subject:
        raise KeyError(user_id)
    tables: dict[str, list[dict[str, Any]]] = {}
    for table in _SNAPSHOT_TABLES:
        row = conn.execute(
            f'SELECT COALESCE(jsonb_agg(to_jsonb(item)), \'[]\'::jsonb) AS data '
            f'FROM (SELECT * FROM {table} WHERE "userId"=%s ORDER BY 1) AS item',
            (user_id,),
        ).fetchone()
        tables[table] = list(row["data"] or [])
    payload = {
        "schema_version": 2,
        "user_id": user_id,
        "subject": subject["data"],
        "tables": tables,
    }
    return {**payload, "checksum": _snapshot_checksum(payload)}


def restore_snapshot(
    conn: psycopg.Connection, *, user_id: str, snapshot: dict[str, Any]
) -> dict[str, int]:
    payload = {key: value for key, value in snapshot.items() if key != "checksum"}
    if snapshot.get("schema_version") != 2 or snapshot.get("user_id") != user_id:
        raise ValueError("snapshot schema or user does not match")
    if snapshot.get("checksum") != _snapshot_checksum(payload):
        raise ValueError("snapshot checksum mismatch")
    subject = dict(snapshot.get("subject") or {})
    if subject.get("id") != user_id:
        raise ValueError("snapshot subject does not match")
    tables = snapshot.get("tables")
    if not isinstance(tables, dict) or set(tables) != set(_SNAPSHOT_TABLES):
        raise ValueError("snapshot table set is invalid")

    clear_all(conn, user_id=user_id)
    conn.execute(
        'UPDATE memory_subjects SET "displayName"=%s, "source"=%s, "metadata"=%s::jsonb, '
        '"updatedAt"=CURRENT_TIMESTAMP WHERE "id"=%s',
        (subject.get("displayName"), subject.get("source") or "external",
         json.dumps(subject.get("metadata") or {}, ensure_ascii=False), user_id),
    )
    restored: dict[str, int] = {}
    for table in _SNAPSHOT_TABLES:
        rows = tables.get(table)
        if not isinstance(rows, list):
            raise ValueError(f"snapshot table {table} is invalid")
        allowed_columns = {
            str(column["column_name"])
            for column in conn.execute(
                "SELECT column_name FROM information_schema.columns "
                "WHERE table_schema='public' AND table_name=%s",
                (table,),
            ).fetchall()
        }
        restored[table] = 0
        for raw in rows:
            if not isinstance(raw, dict) or raw.get("userId") != user_id:
                raise ValueError(f"snapshot row ownership mismatch in {table}")
            row = dict(raw)
            if set(row) != allowed_columns:
                raise ValueError(f"snapshot columns are invalid in {table}")
            if table == "memory_tasks" and row.get("status") == "running":
                row["status"], row["lockedAt"] = "pending", None
            columns = list(row)
            quoted_columns = ", ".join(f'"{name}"' for name in columns)
            placeholders = ", ".join(["%s"] * len(columns))
            values = []
            for name in columns:
                value = row[name]
                if name in {"metadata", "payload", "profile"} and isinstance(value, (dict, list)):
                    value = json.dumps(value, ensure_ascii=False)
                values.append(value)
            conn.execute(
                f'INSERT INTO {table} ({quoted_columns}) VALUES ({placeholders})', values
            )
            restored[table] += 1
    return restored


def _flatten_profile(value: Any, prefix: str = "") -> list[tuple[str, Any]]:
    if isinstance(value, dict):
        rows: list[tuple[str, Any]] = []
        for key, child in value.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            rows.extend(_flatten_profile(child, path))
        return rows
    return [(prefix, value)] if prefix else []


def list_items(conn: psycopg.Connection, *, user_id: str, limit: int = 100) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    # Keep room for consolidated memories so a user with a long recent history
    # can still see both groups in the management UI.
    short_term_limit = max(1, min(50, limit))
    recent = conn.execute(
        'SELECT "id", "userInput", "createdAt" FROM memory_short_term '
        'WHERE "userId"=%s ORDER BY "createdAt" DESC, "id" DESC LIMIT %s',
        (user_id, short_term_limit),
    ).fetchall()
    items.extend({
        "id": f"short_term:{row['id']}",
        "type": "short_term",
        "label": "近期对话",
        "content": row["userInput"],
        "created_at": row["createdAt"],
        "source_count": 0,
        "generation_tags": [],
    } for row in recent)
    profile = conn.execute(
        'SELECT "profile", "updatedAt" FROM memory_profiles WHERE "userId"=%s', (user_id,)
    ).fetchone()
    if profile:
        for path, value in _flatten_profile(profile["profile"] or {}):
            origin = conn.execute(
                'SELECT count(DISTINCT ("sourceType", "sourceId")) AS source_count, '
                'array_agg(DISTINCT "generationTag") AS tags FROM memory_derivations '
                'WHERE "userId"=%s AND "targetType"=\'profile\' AND "targetId"=%s',
                (user_id, path),
            ).fetchone()
            items.append({"id": f"profile:{path}", "type": "profile", "label": path,
                          "content": value, "created_at": profile["updatedAt"],
                          "source_count": int(origin["source_count"] or 0),
                          "generation_tags": origin["tags"] or ["legacy_import"]})
    knowledge = conn.execute(
        'SELECT "id", "content", "source", "createdAt" FROM memory_knowledge '
        'WHERE "userId"=%s ORDER BY "createdAt" DESC LIMIT %s',
        (user_id, max(1, min(limit, 200))),
    ).fetchall()
    for row in knowledge:
        origin = conn.execute(
            'SELECT count(DISTINCT ("sourceType", "sourceId")) AS source_count, '
            'array_agg(DISTINCT "generationTag") AS tags FROM memory_derivations '
            'WHERE "userId"=%s AND "targetType"=\'knowledge\' AND "targetId"=%s',
            (user_id, row["id"]),
        ).fetchone()
        items.append({"id": f"knowledge:{row['id']}", "type": "knowledge",
                      "label": row["source"], "content": row["content"],
                      "created_at": row["createdAt"],
                      "source_count": int(origin["source_count"] or 0),
                      "generation_tags": origin["tags"] or [row["source"] or "legacy_import"]})
    return items[:limit]


def _delete_profile_path(conn: psycopg.Connection, user_id: str, path: str) -> None:
    # jsonb #- accepts a text[] path and physically removes the selected leaf.
    parts = [part for part in path.split(".") if part]
    if not parts:
        return
    conn.execute(
        'UPDATE memory_profiles SET "profile" = "profile" #- %s::text[], '
        '"version"="version"+1, "updatedAt"=CURRENT_TIMESTAMP WHERE "userId"=%s',
        (parts, user_id),
    )


def delete_item(conn: psycopg.Connection, *, user_id: str, item_id: str) -> dict[str, Any]:
    kind, separator, raw_id = item_id.partition(":")
    if not separator or kind not in {"short_term", "knowledge", "profile"} or not raw_id:
        raise KeyError(item_id)
    if kind == "short_term":
        row = conn.execute(
            'DELETE FROM memory_short_term WHERE "userId"=%s AND "id"=%s RETURNING "id"',
            (user_id, raw_id),
        ).fetchone()
        if not row:
            raise KeyError(item_id)
        return {"deleted": 1}
    source_segments = provenance.target_segment_sources(
        conn, user_id=user_id, target_type=kind, target_id=raw_id
    )
    if kind == "knowledge":
        row = conn.execute(
            'DELETE FROM memory_knowledge WHERE "userId"=%s AND "id"=%s RETURNING "id"',
            (user_id, raw_id),
        ).fetchone()
        if not row:
            raise KeyError(item_id)
        provenance.remove_target(conn, user_id=user_id, target_type="knowledge", target_id=raw_id)
        return {
            "deleted": 1,
            "segments_deleted": provenance.delete_orphan_segment_sources(
                conn, user_id=user_id, segment_ids=source_segments
            ),
        }
    if kind == "profile":
        _delete_profile_path(conn, user_id, raw_id)
        provenance.remove_target(conn, user_id=user_id, target_type="profile", target_id=raw_id)
        return {
            "deleted": 1,
            "segments_deleted": provenance.delete_orphan_segment_sources(
                conn, user_id=user_id, segment_ids=source_segments
            ),
        }


def clear_all(conn: psycopg.Connection, *, user_id: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for table in ("memory_tasks", "memory_derivations", "memory_knowledge", "memory_profiles",
                  "memory_pages", "memory_segments", "memory_short_term", "memory_ingest_receipts"):
        cur = conn.execute(f'DELETE FROM {table} WHERE "userId"=%s', (user_id,))
        counts[table] = cur.rowcount
    return counts


def clear_scope(
    conn: psycopg.Connection, *, user_id: str, scope: str
) -> dict[str, int]:
    """Clear exactly one user-visible memory layer.

    Consolidated layers keep their existing provenance semantics: source
    segments are removed only when no retained knowledge/profile item uses
    them. Clearing recent memory never removes already consolidated memory.
    """
    if scope == "short_term":
        # Cancel pending consolidation for removed recent memories, while
        # retaining ingest receipts so historical turns cannot be reinserted.
        tasks = conn.execute(
            'DELETE FROM memory_tasks WHERE "userId"=%s', (user_id,)
        ).rowcount
        deleted = conn.execute(
            'DELETE FROM memory_short_term WHERE "userId"=%s', (user_id,)
        ).rowcount
        return {"items": deleted, "tasks": tasks, "segments": 0}

    if scope == "knowledge":
        ids = [str(row["id"]) for row in conn.execute(
            'SELECT "id" FROM memory_knowledge WHERE "userId"=%s', (user_id,)
        ).fetchall()]
        segments = 0
        for item_id in ids:
            result = delete_item(
                conn, user_id=user_id, item_id=f"knowledge:{item_id}"
            )
            segments += int(result.get("segments_deleted", 0))
        return {"items": len(ids), "tasks": 0, "segments": segments}

    if scope == "profile":
        row = conn.execute(
            'SELECT "profile" FROM memory_profiles WHERE "userId"=%s', (user_id,)
        ).fetchone()
        paths = [path for path, _ in _flatten_profile(row["profile"] or {})] if row else []
        segments = 0
        for path in paths:
            result = delete_item(
                conn, user_id=user_id, item_id=f"profile:{path}"
            )
            segments += int(result.get("segments_deleted", 0))
        conn.execute('DELETE FROM memory_profiles WHERE "userId"=%s', (user_id,))
        return {"items": len(paths), "tasks": 0, "segments": segments}

    raise ValueError(f"unsupported memory scope: {scope}")
