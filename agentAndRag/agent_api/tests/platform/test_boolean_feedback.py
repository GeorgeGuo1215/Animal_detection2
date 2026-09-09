from __future__ import annotations

import importlib.util
from pathlib import Path

from alembic.migration import MigrationContext
from alembic.operations import Operations
import pytest
import sqlalchemy as sa

from agent_api.app.platform.message_feedback import good_to_rating, rating_to_good
from agent_api.app.platform.models import Message


@pytest.mark.parametrize("rating,value", [("up", True), ("down", False), (None, None)])
def test_wire_roundtrip_and_single_stored_column(rating, value):
    assert rating_to_good(rating) is value
    assert good_to_rating(value) == rating
    message = Message(feedback_is_good=value)
    assert message.feedback_rating == rating
    assert "feedback_rating" not in Message.__table__.columns
    assert Message.__table__.c.feedback_is_good.nullable


@pytest.mark.parametrize("value", [0, 1, "false", "up", [], {}])
def test_boolean_contract_rejects_coerced_values(value):
    with pytest.raises(ValueError):
        good_to_rating(value)


def feedback_migration():
    path = Path(__file__).resolve().parents[2] / "alembic/versions/20260909_0006_boolean_feedback.py"
    spec = importlib.util.spec_from_file_location("boolean_feedback_migration", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sqlite_migration_preserves_three_states_and_reverses(tmp_path):
    engine = sa.create_engine(f"sqlite:///{tmp_path / 'migration.db'}")
    with engine.begin() as conn:
        conn.execute(sa.text("CREATE TABLE platform_messages (id INTEGER PRIMARY KEY, feedback_rating VARCHAR(8), feedback_updated_at TEXT)"))
        conn.execute(sa.text("INSERT INTO platform_messages VALUES (1,'up','2026-09-09'),(2,'down','2026-09-08'),(3,NULL,NULL)"))
        module = feedback_migration()
        module.op = Operations(MigrationContext.configure(conn))
        module.upgrade()
        rows = conn.execute(sa.text("SELECT feedback_is_good,feedback_updated_at FROM platform_messages ORDER BY id")).all()
        assert rows == [(1, '2026-09-09'), (0, '2026-09-08'), (None, None)]
        assert "feedback_rating" not in {c['name'] for c in sa.inspect(conn).get_columns('platform_messages')}
        module.downgrade()
        assert conn.execute(sa.text("SELECT feedback_rating FROM platform_messages ORDER BY id")).scalars().all() == ['up', 'down', None]
    engine.dispose()


def test_invalid_history_aborts_before_ddl(tmp_path):
    engine = sa.create_engine(f"sqlite:///{tmp_path / 'invalid.db'}")
    with engine.begin() as conn:
        conn.execute(sa.text("CREATE TABLE platform_messages (feedback_rating VARCHAR(8))"))
        conn.execute(sa.text("INSERT INTO platform_messages VALUES ('invalid')"))
        module = feedback_migration()
        module.op = Operations(MigrationContext.configure(conn))
        with pytest.raises(ValueError, match="historical"):
            module.upgrade()
        assert {c['name'] for c in sa.inspect(conn).get_columns('platform_messages')} == {'feedback_rating'}
    engine.dispose()
