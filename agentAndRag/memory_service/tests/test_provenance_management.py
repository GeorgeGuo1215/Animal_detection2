from __future__ import annotations

from dataclasses import replace
from datetime import timedelta

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from memory_service.app.config import load_config
from memory_service.app.routers import build_router
from memory_service.app.memory import consolidator, management, provenance, short_term
from memory_service.tests.helpers import BASE_TIME


def _config():
    return replace(
        load_config(),
        short_term_capacity=1,
        promotion_batch=1,
        heat_threshold=0.0,
        analysis_min_pages=1,
    )


def test_management_routes_fail_closed_without_a_service_token():
    app = FastAPI()
    app.include_router(build_router(replace(_config(), management_token="")))
    with TestClient(app) as client:
        assert client.get("/v1/memory/manage/user-1").status_code == 503

    protected = FastAPI()
    protected.include_router(build_router(replace(_config(), management_token="m" * 32)))
    with TestClient(protected) as client:
        assert client.get("/v1/memory/manage/user-1").status_code == 401


def test_long_term_dependencies_point_to_mid_term_segments_not_turns(
    conn, user_id, embedder, llm
):
    turn_ids = ["run-source-a", "run-source-b", "run-source-c"]
    for index, turn_id in enumerate(turn_ids):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"猫咪皮肤问题{index}",
            agent_response=f"皮肤回答{index}",
            turn_id=turn_id,
            created_at=BASE_TIME + timedelta(minutes=index),
        )

    cfg = _config()
    promoted = consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    assert promoted["promoted"] == 2

    columns = {row["column_name"] for row in conn.execute(
        "SELECT column_name FROM information_schema.columns "
        "WHERE table_schema='public' AND table_name='memory_pages'"
    ).fetchall()}
    assert "sourceTurnId" not in columns

    consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    edges = conn.execute(
        'SELECT "sourceType", "sourceId", "targetType", "generationTag" '
        'FROM memory_derivations WHERE "userId"=%s',
        (user_id,),
    ).fetchall()
    assert edges
    assert {row["sourceType"] for row in edges} == {"segment"}
    assert {row["targetType"] for row in edges} <= {"profile", "knowledge"}
    segment_ids = {row["id"] for row in conn.execute(
        'SELECT "id" FROM memory_segments WHERE "userId"=%s', (user_id,)
    ).fetchall()}
    assert {row["sourceId"] for row in edges} <= segment_ids
    assert all(row["generationTag"] for row in edges)

    listed = management.list_items(conn, user_id=user_id)
    assert listed
    long_items = [item for item in listed if item["type"] != "short_term"]
    assert long_items
    assert all(item["source_count"] >= 1 for item in long_items)
    assert all(item["generation_tags"] for item in long_items)


def test_deleting_long_term_item_keeps_mid_term_memory_unchanged(
    conn, user_id, embedder, llm
):
    for index, turn_id in enumerate(("run-delete-me", "run-keep-me", "run-remain-short")):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"猫咪过敏问题{index}",
            agent_response=f"过敏回答{index}",
            turn_id=turn_id,
            created_at=BASE_TIME + timedelta(minutes=index),
        )
    cfg = _config()
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )

    before = conn.execute(
        'SELECT (SELECT count(*) FROM memory_segments WHERE "userId"=%s) AS segments, '
        '(SELECT count(*) FROM memory_pages WHERE "userId"=%s) AS pages',
        (user_id, user_id),
    ).fetchone()
    item = next(
        row for row in management.list_items(conn, user_id=user_id)
        if row["type"] == "knowledge"
    )
    result = management.delete_item(conn, user_id=user_id, item_id=item["id"])
    assert result["deleted"] == 1
    after = conn.execute(
        'SELECT (SELECT count(*) FROM memory_segments WHERE "userId"=%s) AS segments, '
        '(SELECT count(*) FROM memory_pages WHERE "userId"=%s) AS pages',
        (user_id, user_id),
    ).fetchone()
    assert after == before
    assert conn.execute(
        'SELECT 1 FROM memory_derivations WHERE "userId"=%s '
        'AND "targetType"=\'knowledge\' AND "targetId"=%s',
        (user_id, item["id"].split(":", 1)[1]),
    ).fetchone() is None
    assert conn.execute(
        'SELECT count(*) AS n FROM memory_pages WHERE "userId"=%s AND "analyzed"=true',
        (user_id,),
    ).fetchone()["n"] > 0
    with pytest.raises(KeyError):
        management.delete_item(conn, user_id=user_id, item_id="turn:run-delete-me")


def test_deleting_long_term_item_cascades_only_orphaned_mid_term_sources(conn, user_id):
    for segment_id in ("segment-exclusive", "segment-shared"):
        conn.execute(
            'INSERT INTO memory_segments ("id", "userId", "summary") VALUES (%s, %s, %s)',
            (segment_id, user_id, segment_id),
        )
        conn.execute(
            'INSERT INTO memory_pages '
            '("id", "segmentId", "userId", "userInput", "agentResponse", "analyzed") '
            'VALUES (%s, %s, %s, %s, %s, true)',
            (f"page-{segment_id}", segment_id, user_id, "input", "response"),
        )
    conn.execute(
        'INSERT INTO memory_knowledge ("id", "userId", "content") VALUES '
        '(%s, %s, %s), (%s, %s, %s)',
        ("knowledge-delete", user_id, "delete me", "knowledge-keep", user_id, "keep me"),
    )
    provenance.link_many(
        conn,
        user_id=user_id,
        sources=(("segment", "segment-exclusive"), ("segment", "segment-shared")),
        target_type="knowledge",
        target_id="knowledge-delete",
        generation_tag="test-delete",
    )
    provenance.link(
        conn,
        user_id=user_id,
        source_type="segment",
        source_id="segment-shared",
        target_type="knowledge",
        target_id="knowledge-keep",
        generation_tag="test-keep",
    )

    result = management.delete_item(
        conn, user_id=user_id, item_id="knowledge:knowledge-delete"
    )

    assert result == {"deleted": 1, "segments_deleted": 1}
    segment_ids = {row["id"] for row in conn.execute(
        'SELECT "id" FROM memory_segments WHERE "userId"=%s', (user_id,)
    ).fetchall()}
    assert "segment-exclusive" not in segment_ids
    assert "segment-shared" in segment_ids
    assert conn.execute(
        'SELECT 1 FROM memory_pages WHERE "id"=%s', ("page-segment-exclusive",)
    ).fetchone() is None
    assert conn.execute(
        'SELECT 1 FROM memory_knowledge WHERE "id"=%s', ("knowledge-keep",)
    ).fetchone() is not None


def test_short_term_items_are_visible_and_can_be_deleted(conn, user_id):
    short_id = short_term.append(
        conn,
        user_id=user_id,
        user_input="团团今天抓挠次数增加",
        agent_response="请记录皮损变化",
        turn_id="short-visible-turn",
        created_at=BASE_TIME,
    )

    listed = management.list_items(conn, user_id=user_id)
    item = next(row for row in listed if row["id"] == f"short_term:{short_id}")
    assert item["type"] == "short_term"
    assert item["content"] == "团团今天抓挠次数增加"

    assert management.delete_item(
        conn, user_id=user_id, item_id=item["id"]
    ) == {"deleted": 1}
    assert conn.execute(
        'SELECT 1 FROM memory_short_term WHERE "id"=%s', (short_id,)
    ).fetchone() is None


def test_scoped_clear_keeps_other_memory_layers(conn, user_id):
    short_term.append(
        conn, user_id=user_id, user_input="近期问题", agent_response="近期回答",
        turn_id="scope-short", created_at=BASE_TIME,
    )
    for segment_id in ("scope-knowledge-segment", "scope-profile-segment"):
        conn.execute(
            'INSERT INTO memory_segments ("id", "userId", "summary") VALUES (%s, %s, %s)',
            (segment_id, user_id, segment_id),
        )
    conn.execute(
        'INSERT INTO memory_knowledge ("id", "userId", "content") VALUES (%s, %s, %s)',
        ("scope-knowledge", user_id, "长期事实"),
    )
    conn.execute(
        'INSERT INTO memory_profiles ("userId", "profile") VALUES (%s, %s::jsonb)',
        (user_id, '{"preferences":{"style":"简洁"}}'),
    )
    provenance.link(
        conn, user_id=user_id, source_type="segment", source_id="scope-knowledge-segment",
        target_type="knowledge", target_id="scope-knowledge", generation_tag="scope-test",
    )
    provenance.link(
        conn, user_id=user_id, source_type="segment", source_id="scope-profile-segment",
        target_type="profile", target_id="preferences.style", generation_tag="scope-test",
    )

    assert management.clear_scope(conn, user_id=user_id, scope="short_term")["items"] == 1
    assert conn.execute(
        'SELECT 1 FROM memory_knowledge WHERE "userId"=%s', (user_id,)
    ).fetchone()
    assert conn.execute(
        'SELECT 1 FROM memory_profiles WHERE "userId"=%s', (user_id,)
    ).fetchone()

    assert management.clear_scope(conn, user_id=user_id, scope="knowledge")["items"] == 1
    assert conn.execute(
        'SELECT 1 FROM memory_knowledge WHERE "userId"=%s', (user_id,)
    ).fetchone() is None
    assert conn.execute(
        'SELECT 1 FROM memory_segments WHERE "id"=%s', ("scope-knowledge-segment",)
    ).fetchone() is None
    assert conn.execute(
        'SELECT 1 FROM memory_profiles WHERE "userId"=%s', (user_id,)
    ).fetchone()

    assert management.clear_scope(conn, user_id=user_id, scope="profile")["items"] == 1
    assert conn.execute(
        'SELECT 1 FROM memory_profiles WHERE "userId"=%s', (user_id,)
    ).fetchone() is None
    assert conn.execute(
        'SELECT 1 FROM memory_segments WHERE "id"=%s', ("scope-profile-segment",)
    ).fetchone() is None


def test_versioned_snapshot_restores_all_memory_layers(conn, user_id, embedder, llm):
    for index in range(4):
        short_term.append(
            conn,
            user_id=user_id,
            user_input=f"猫咪喂养记录{index}",
            agent_response=f"喂养建议{index}",
            turn_id=f"backup-run-{index}",
            created_at=BASE_TIME + timedelta(minutes=index),
        )
    cfg = _config()
    consolidator.promote_short_to_mid(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    consolidator.promote_mid_to_long(
        conn, user_id=user_id, cfg=cfg, embedder=embedder, llm=llm, now=BASE_TIME
    )
    snapshot = management.export_snapshot(conn, user_id=user_id)
    expected = {name: len(rows) for name, rows in snapshot["tables"].items()}
    assert snapshot["checksum"]

    management.clear_all(conn, user_id=user_id)
    assert not management.list_items(conn, user_id=user_id)
    restored = management.restore_snapshot(conn, user_id=user_id, snapshot=snapshot)
    assert restored == expected
    assert management.export_snapshot(conn, user_id=user_id)["tables"] == snapshot["tables"]

    corrupted = dict(snapshot)
    corrupted["checksum"] = "0" * 64
    try:
        management.restore_snapshot(conn, user_id=user_id, snapshot=corrupted)
    except ValueError as exc:
        assert "checksum" in str(exc)
    else:
        raise AssertionError("corrupted snapshot must be rejected")
