"""HTTP 接口测试。

跑真实的 FastAPI 应用与真实数据库，只把 embedding 与 LLM 换成假实现——
接口层最容易出错的地方是参数透传和响应结构，这两样 mock 掉数据库就测不出来了。
"""

from __future__ import annotations

import sys
import uuid
from dataclasses import replace
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db, embedding, llm as llm_module  # noqa: E402
from memory_service.app.config import load_config  # noqa: E402
from memory_service.app.main import create_app  # noqa: E402
from memory_service.app.memory import consolidator, queue  # noqa: E402
from memory_service.tests.helpers import (  # noqa: E402
    BASE_TIME,
    FakeEmbedder,
    RecordingLLM,
)


@pytest.fixture
def cfg():
    return replace(
        load_config(),
        short_term_capacity=3,
        heat_threshold=2.0,
        promotion_batch=2,
        analysis_min_pages=3,
    )


@pytest.fixture
def client(cfg, _db_available):
    """启好的应用实例，不带后台 worker（提升由用例显式触发，便于断言）。"""
    if not _db_available:
        pytest.skip("本地 PostgreSQL 不可用")

    embedding.set_embedder(FakeEmbedder())
    llm_module.set_llm(RecordingLLM())
    db.close_pool()

    app = create_app(cfg, with_workers=False)
    with TestClient(app) as test_client:
        yield test_client

    db.close_pool()
    embedding.set_embedder(None)
    llm_module.set_llm(None)


@pytest.fixture
def api_user(client) -> str:
    uid = f"apitest_{uuid.uuid4().hex[:10]}"
    with db.connection() as conn:
        conn.execute(
            'INSERT INTO "User" ("id", "username", "passwordHash") VALUES (%s, %s, %s)',
            (uid, "pytest-api", "x"),
        )
    yield uid
    with db.connection() as conn:
        conn.execute('DELETE FROM "User" WHERE "id" = %s', (uid,))


def _post_message(client, user_id, text="喂养问题", answer="回答"):
    return client.post(
        "/v1/memory/messages",
        json={"user_id": user_id, "user_input": text, "agent_response": answer},
    )


# ---------------------------------------------------------------- 写入


def test_write_message_returns_immediately(client, api_user):
    response = _post_message(client, api_user)
    assert response.status_code == 200

    body = response.json()
    assert body["short_term_size"] == 1
    # 没溢出就不该派活给 worker
    assert body["queued"] is False


def test_overflow_enqueues_a_task(client, api_user, cfg):
    # 攒够 capacity + promotion_batch 条才派活，之前只是单纯堆在短期队列里
    for i in range(cfg.short_term_capacity + cfg.promotion_batch):
        response = _post_message(client, api_user, f"喂养问题{i}")
        assert response.status_code == 200

    assert response.json()["queued"] is True

    with db.connection() as conn:
        row = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "userId" = %s', (api_user,)
        ).fetchone()
    assert row["n"] == 1


def test_repeated_overflow_merges_into_one_task(client, api_user, cfg):
    """用户连聊很多轮只应产生一个待处理任务，队列不能被写入频率撑爆。"""
    for i in range(cfg.short_term_capacity + 6):
        _post_message(client, api_user, f"喂养问题{i}")

    with db.connection() as conn:
        row = conn.execute(
            'SELECT count(*) AS n FROM memory_tasks WHERE "userId" = %s', (api_user,)
        ).fetchone()
    assert row["n"] == 1


def test_unknown_user_returns_404(client):
    response = _post_message(client, "user-that-does-not-exist")
    assert response.status_code == 404


def test_missing_required_field_is_rejected(client, api_user):
    response = client.post(
        "/v1/memory/messages", json={"user_id": api_user, "agent_response": "只有回复"}
    )
    assert response.status_code == 422


def test_blank_user_id_is_rejected(client):
    response = client.post(
        "/v1/memory/messages",
        json={"user_id": "", "user_input": "问题", "agent_response": "回答"},
    )
    assert response.status_code == 422


# ---------------------------------------------------------------- 检索


def test_context_returns_all_three_layers(client, api_user, cfg):
    for i in range(8):
        _post_message(client, api_user, f"皮肤问题{i}")

    with db.connection() as conn:
        consolidator.consolidate(
            conn,
            user_id=api_user,
            cfg=cfg,
            embedder=embedding.get_embedder(),
            llm=llm_module.get_llm(),
            now=BASE_TIME,
        )

    response = client.post(
        "/v1/memory/context",
        json={"user_id": api_user, "query": "皮肤怎么办"},
    )
    assert response.status_code == 200

    body = response.json()
    assert body["user_id"] == api_user
    assert body["profile"]
    assert body["related_pages"]
    assert body["recent_dialogue"]
    assert body["text"]


def test_context_can_skip_the_rendered_text(client, api_user):
    _post_message(client, api_user)
    response = client.post(
        "/v1/memory/context",
        json={"user_id": api_user, "query": "喂养", "include_text": False},
    )
    assert response.json()["text"] is None


def test_context_for_user_without_memory_is_empty_not_error(client, api_user):
    response = client.post(
        "/v1/memory/context", json={"user_id": api_user, "query": "随便问问"}
    )
    assert response.status_code == 200

    body = response.json()
    assert body["profile"] == {}
    assert body["knowledge"] == []
    assert body["related_pages"] == []


# ---------------------------------------------------------------- 画像与统计


def test_profile_endpoint_returns_empty_for_new_user(client, api_user):
    response = client.get(f"/v1/memory/profile/{api_user}")
    assert response.status_code == 200
    assert response.json() == {
        "user_id": api_user,
        "profile": {},
        "version": 0,
        "updated_at": None,
    }


def test_profile_endpoint_reflects_consolidation(client, api_user, cfg):
    for i in range(8):
        _post_message(client, api_user, f"喂养问题{i}")
    with db.connection() as conn:
        consolidator.consolidate(
            conn,
            user_id=api_user,
            cfg=cfg,
            embedder=embedding.get_embedder(),
            llm=llm_module.get_llm(),
            now=BASE_TIME,
        )

    body = client.get(f"/v1/memory/profile/{api_user}").json()
    assert body["version"] >= 1
    assert "communication" in body["profile"]


def test_stats_endpoint_reports_each_layer(client, api_user, cfg):
    for i in range(8):
        _post_message(client, api_user, f"疫苗问题{i}")
    with db.connection() as conn:
        consolidator.consolidate(
            conn,
            user_id=api_user,
            cfg=cfg,
            embedder=embedding.get_embedder(),
            llm=llm_module.get_llm(),
            now=BASE_TIME,
        )

    body = client.get(f"/v1/memory/stats/{api_user}").json()
    assert body["short_term"] == cfg.short_term_capacity
    assert body["segments"] >= 1
    assert body["heat"]["max"] >= body["heat"]["p50"]


def test_stats_for_unknown_user_is_all_zero(client):
    body = client.get("/v1/memory/stats/nobody").json()
    assert body["short_term"] == 0
    assert body["segments"] == 0
    assert body["heat"]["n"] == 0


# ---------------------------------------------------------------- 运维


def test_health_reports_database_and_queue(client):
    body = client.get("/health").json()
    assert body["status"] == "ok"
    assert body["database"] == "ok"
    assert isinstance(body["queue"], dict)
