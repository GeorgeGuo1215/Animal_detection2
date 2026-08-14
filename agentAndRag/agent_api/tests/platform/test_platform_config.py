from __future__ import annotations

import pytest

from agent_api.app.platform.config import get_platform_settings, reset_platform_settings_cache


def test_production_requires_postgres_and_redis(monkeypatch):
    monkeypatch.setenv("AGENT_PLATFORM_ENV", "production")
    monkeypatch.setenv("AGENT_PLATFORM_DB_URL", "sqlite+aiosqlite:///unsafe.db")
    monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "x" * 40)
    monkeypatch.setenv("AGENT_PLATFORM_AUTO_CREATE_SCHEMA", "0")
    monkeypatch.setenv("AGENT_PLATFORM_EXPOSE_DEV_TOKENS", "0")
    reset_platform_settings_cache()
    with pytest.raises(RuntimeError, match="PostgreSQL"):
        get_platform_settings()
    reset_platform_settings_cache()
