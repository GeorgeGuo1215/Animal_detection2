from __future__ import annotations

import pytest

from agent_api.app.platform.config import get_platform_settings, reset_platform_settings_cache


def test_production_requires_postgres_and_redis(monkeypatch):
    """验证生产环境必须配置 Postgres 和 Redis。"""
    monkeypatch.setenv("AGENT_PLATFORM_ENV", "production")
    monkeypatch.setenv("AGENT_PLATFORM_DB_URL", "sqlite+aiosqlite:///unsafe.db")
    monkeypatch.setenv("AGENT_PLATFORM_JWT_SECRET", "x" * 40)
    monkeypatch.setenv("AGENT_PLATFORM_AUTO_CREATE_SCHEMA", "0")
    monkeypatch.setenv("AGENT_PLATFORM_EXPOSE_DEV_TOKENS", "0")
    reset_platform_settings_cache()
    with pytest.raises(RuntimeError, match="PostgreSQL"):
        get_platform_settings()
    reset_platform_settings_cache()


@pytest.mark.parametrize(
    ("name", "value", "message"),
    [
        ("AGENT_PLATFORM_COOKIE_SECURE", "0", "refresh cookies"),
        ("AGENT_PLATFORM_FRONTEND_ORIGIN", "http://agent.example.com", "HTTPS"),
        ("AGENT_WORKER_TOKEN", "short", "AGENT_WORKER_TOKEN"),
        ("MEMORY_MANAGEMENT_TOKEN", "short", "MEMORY_MANAGEMENT_TOKEN"),
        ("AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET", "short", "webhook secret"),
        ("AGENT_ALLOW_INSECURE_DEFAULT_KEY", "1", "default API key"),
    ],
)
def test_production_rejects_insecure_service_boundaries(monkeypatch, name, value, message):
    """验证生产环境拒绝不安全的服务边界配置。"""
    secure = {
        "AGENT_PLATFORM_ENV": "production",
        "AGENT_PLATFORM_DB_URL": "postgresql+asyncpg://user:pass@127.0.0.1/db",
        "AGENT_PLATFORM_REDIS_URL": "redis://127.0.0.1:6379/0",
        "AGENT_PLATFORM_AUTO_CREATE_SCHEMA": "0",
        "AGENT_PLATFORM_EXPOSE_DEV_TOKENS": "0",
        "AGENT_PLATFORM_JWT_SECRET": "j" * 32,
        "AGENT_PLATFORM_COOKIE_SECURE": "1",
        "AGENT_PLATFORM_FRONTEND_ORIGIN": "https://agent.example.com",
        "AGENT_WORKER_TOKEN": "w" * 32,
        "MEMORY_MANAGEMENT_TOKEN": "m" * 32,
        "AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET": "p" * 32,
    }
    for key, item in secure.items():
        monkeypatch.setenv(key, item)
    monkeypatch.setenv(name, value)
    reset_platform_settings_cache()
    with pytest.raises(RuntimeError, match=message):
        get_platform_settings()
    reset_platform_settings_cache()
