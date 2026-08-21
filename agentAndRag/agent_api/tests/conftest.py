from __future__ import annotations

import pytest

from agent_api.app.platform.config import reset_platform_settings_cache


@pytest.fixture(autouse=True)
def _isolate_platform_environment(monkeypatch):
    """自动夹具：把平台环境固定为 development，避免本机/生产配置污染单元测试。"""
    monkeypatch.setenv("AGENT_PLATFORM_ENV", "development")
    monkeypatch.setenv("AGENT_PLATFORM_AUTO_CREATE_SCHEMA", "1")
    reset_platform_settings_cache()
    yield
    reset_platform_settings_cache()
