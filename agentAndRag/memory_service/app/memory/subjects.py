"""Memory-owned user subjects mapped from upstream authenticated identities."""
from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, Optional

import psycopg
from psycopg.types.json import Jsonb


def ensure(
    conn: psycopg.Connection,
    *,
    user_id: str,
    display_name: Optional[str] = None,
    source: str = "external",
    metadata: Optional[Dict[str, Any]] = None,
    now: Optional[datetime] = None,
) -> Dict[str, Any]:
    """Create or refresh a stable memory subject without touching PetHealth User."""
    row = conn.execute(
        """
        INSERT INTO memory_subjects
            ("id", "displayName", "source", "metadata", "createdAt", "updatedAt")
        VALUES (%(user_id)s, %(display_name)s, %(source)s, %(metadata)s,
                COALESCE(%(now)s, CURRENT_TIMESTAMP), COALESCE(%(now)s, CURRENT_TIMESTAMP))
        ON CONFLICT ("id") DO UPDATE SET
            "displayName" = COALESCE(EXCLUDED."displayName", memory_subjects."displayName"),
            "source" = CASE
                WHEN memory_subjects."source" = 'legacy' THEN EXCLUDED."source"
                ELSE memory_subjects."source"
            END,
            "metadata" = memory_subjects."metadata" || EXCLUDED."metadata",
            "updatedAt" = COALESCE(EXCLUDED."updatedAt", CURRENT_TIMESTAMP)
        RETURNING "id", "displayName", "source", "metadata", "createdAt", "updatedAt"
        """,
        {
            "user_id": user_id.strip(),
            "display_name": display_name.strip() if display_name else None,
            "source": (source or "external").strip(),
            "metadata": Jsonb(metadata or {}),
            "now": now,
        },
    ).fetchone()
    return dict(row)


def get(conn: psycopg.Connection, user_id: str) -> Optional[Dict[str, Any]]:
    row = conn.execute(
        'SELECT "id", "displayName", "source", "metadata", "createdAt", "updatedAt" '
        'FROM memory_subjects WHERE "id" = %s',
        (user_id,),
    ).fetchone()
    return dict(row) if row else None
