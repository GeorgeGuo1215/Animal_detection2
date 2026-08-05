"""Pure unit tests for the multi-table sql.search whitelist, compiler and scope.

No DB / no LLM. Run: pytest tests/sql_search/test_sql_unit.py
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import pytest

from app.sql_search.schema_catalog import ALLOWED_TABLES, validate_columns, validate_table
from app.sql_search.query_compiler import compile_select
from app.sql_search.tool import _merge_animal_scope
from app.sql_search.animal_profile import species_label


def test_whitelist_has_new_tables():
    assert "animals" in ALLOWED_TABLES
    assert "sensor_events" in ALLOWED_TABLES
    assert "daily_reports" in ALLOWED_TABLES


def test_validate_table_rejects_timeseries_and_unknown():
    # Time-series tables have no animal_id column and must stay out of sql.search.
    for bad in ["vitals_samples", "temp_samples", "accel_samples", "devices", "users"]:
        with pytest.raises(ValueError):
            validate_table(bad)


def test_validate_columns_animals():
    cols = validate_columns("animals", ["species", "breed"])
    assert cols == ["species", "breed"]
    with pytest.raises(ValueError):
        validate_columns("animals", ["password"])


def test_compile_select_animals_with_scope():
    where = _merge_animal_scope("cat_001", [{"column": "species", "op": "eq", "value": "cat"}])
    sql, params = compile_select(
        table="animals",
        columns=["animal_id", "species"],
        where=where,
        order_by=None,
        limit=10,
    )
    assert "FROM `animals`" in sql
    assert "`animal_id` = %s" in sql
    assert params[0] == "cat_001"
    assert params[-1] == 10


def test_merge_scope_drops_user_supplied_animal_id():
    where = _merge_animal_scope(
        "cat_001",
        [{"column": "animal_id", "op": "eq", "value": "dog_999"}],
    )
    assert len(where) == 1
    assert where[0]["column"] == "animal_id"
    assert where[0]["value"] == "cat_001"


def test_species_label_mapping():
    assert species_label({"species": "cat"}) is not None
    assert species_label({"species": "dog"}) is not None
    assert species_label({"species": "cat"}) != species_label({"species": "dog"})
    assert species_label({"species": "other"}) is None
    assert species_label({"species": "weird_value"}) is None
    assert species_label(None) is None
