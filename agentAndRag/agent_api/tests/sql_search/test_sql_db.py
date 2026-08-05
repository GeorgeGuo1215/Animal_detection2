"""Integration tests against the local petmind MySQL (localhost:3306).

Skipped automatically if the database is not reachable. These assertions reflect
the current seed data (cat_001 / dog_001) and validate multi-table sql.search,
mandatory animal scope, and the vitals.summary aggregation tool.

Run: pytest tests/sql_search/test_sql_db.py
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import pytest

from app.sql_search.config import load_mysql_config
from app.sql_search.tool import sql_search_tool
from app.sql_search.vitals_summary import vitals_summary_tool
from app.sql_search.animal_profile import fetch_animal_profile
from app.context.request_context import set_request_animal_id


def _db_available() -> bool:
    try:
        import pymysql

        cfg = load_mysql_config()
        conn = pymysql.connect(
            host=cfg.host,
            port=cfg.port,
            user=cfg.user,
            password=cfg.password,
            database=cfg.database,
            connect_timeout=3,
        )
        conn.close()
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _db_available(), reason="petmind MySQL not reachable")


def _clear_scope():
    set_request_animal_id(explicit=None)


def test_sql_search_animals_scoped():
    set_request_animal_id(explicit="cat_001")
    try:
        res = sql_search_tool(table="animals")
    finally:
        _clear_scope()
    assert res["ok"] is True
    assert res["scoped_animal_id"] == "cat_001"
    assert len(res["rows"]) == 1
    assert res["rows"][0]["species"] == "cat"


def test_sql_search_requires_animal_id():
    _clear_scope()
    res = sql_search_tool(table="animals")
    assert res["ok"] is False
    assert res["error"] == "ANIMAL_ID_REQUIRED"


def test_sql_search_scope_cannot_be_bypassed():
    set_request_animal_id(explicit="cat_001")
    try:
        res = sql_search_tool(
            table="animals",
            where=[{"column": "animal_id", "op": "eq", "value": "dog_001"}],
        )
    finally:
        _clear_scope()
    assert res["ok"] is True
    # User-supplied animal_id is dropped; only the scoped pet is ever returned.
    for row in res["rows"]:
        assert row["animal_id"] == "cat_001"


def test_sql_search_sensor_events_table():
    set_request_animal_id(explicit="dog_001")
    try:
        res = sql_search_tool(table="sensor_events", limit=5)
    finally:
        _clear_scope()
    assert res["ok"] is True
    assert res["table"] == "sensor_events"
    for row in res["rows"]:
        assert row["animal_id"] == "dog_001"


def test_vitals_summary_dog001():
    set_request_animal_id(explicit="dog_001")
    try:
        res = vitals_summary_tool()
    finally:
        _clear_scope()
    assert res["ok"] is True
    assert res["animal_id"] == "dog_001"
    v = res["vitals"]
    assert v["sample_count"] == 7
    assert v["hr_bpm"]["min"] == 83
    assert v["hr_bpm"]["max"] == 98
    assert v["hr_bpm"]["min"] <= v["hr_bpm"]["avg"] <= v["hr_bpm"]["max"]
    assert res["temperature"]["sample_count"] == 52


def test_vitals_summary_requires_animal_id():
    _clear_scope()
    res = vitals_summary_tool()
    assert res["ok"] is False
    assert res["error"] == "ANIMAL_ID_REQUIRED"


def test_fetch_animal_profile():
    assert fetch_animal_profile("cat_001")["species"] == "cat"
    assert fetch_animal_profile("does_not_exist") is None
    assert fetch_animal_profile(None) is None
    assert fetch_animal_profile("") is None


def test_pool_reuses_connections_across_calls():
    """Many sequential queries must not open more than pool_size connections."""
    from app.sql_search.pool import close_pool, get_pool

    cfg = load_mysql_config()
    close_pool()  # start from a clean pool for a deterministic count
    pool = get_pool(cfg)

    set_request_animal_id(explicit="dog_001")
    try:
        for _ in range(12):
            res = sql_search_tool(table="animals")
            assert res["ok"] is True
    finally:
        _clear_scope()

    # All connections returned; total opened bounded by the configured size.
    assert pool.in_use == 0
    assert pool._created <= cfg.pool_size
    close_pool()
