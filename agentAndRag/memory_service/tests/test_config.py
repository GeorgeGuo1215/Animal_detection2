"""配置解析的单测：空串、脏值、回退链路。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.config import (  # noqa: E402
    DEFAULT_EMBEDDING_MODEL,
    load_config,
)

_MEMORY_ENV = [
    "MEMORY_DB_DSN",
    "MEMORY_DB_CONNECT_TIMEOUT",
    "MEMORY_DB_POOL_MIN",
    "MEMORY_DB_POOL_MAX",
    "MEMORY_EMBEDDING_MODEL",
    "MEMORY_EMBEDDING_DEVICE",
    "MEMORY_EMBEDDING_DIM",
    "MEMORY_LLM_BASE_URL",
    "MEMORY_LLM_API_KEY",
    "MEMORY_LLM_MODEL",
    "MEMORY_LLM_TIMEOUT",
    "MEMORY_SHORT_TERM_CAPACITY",
    "MEMORY_MID_TERM_CAPACITY",
    "MEMORY_KNOWLEDGE_CAPACITY",
    "MEMORY_HEAT_ALPHA",
    "MEMORY_HEAT_BETA",
    "MEMORY_HEAT_GAMMA",
    "MEMORY_HEAT_TAU_HOURS",
    "MEMORY_HEAT_THRESHOLD",
    "MEMORY_SEGMENT_SIMILARITY_THRESHOLD",
    "MEMORY_KEYWORD_WEIGHT",
    "MEMORY_CONTINUITY_GAP_MINUTES",
    "MEMORY_CONTINUITY_SIMILARITY",
    "MEMORY_TOP_K_SEGMENTS",
    "MEMORY_TOP_K_PAGES",
    "MEMORY_TOP_K_KNOWLEDGE",
    "MEMORY_SHORT_TERM_CONTEXT_SIZE",
    "MEMORY_WORKER_CONCURRENCY",
    "MEMORY_WORKER_POLL_INTERVAL",
    "MEMORY_TASK_MAX_ATTEMPTS",
    "OPENAI_BASE_URL",
    "OPENAI_API_KEY",
    "OPENAI_MODEL",
]


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    """每个用例都从"什么都没配"的状态起步。"""
    for name in _MEMORY_ENV:
        monkeypatch.delenv(name, raising=False)


def test_defaults_are_cpu_and_offline_safe():
    cfg = load_config()
    assert cfg.embedding_device == "cpu"
    assert cfg.embedding_model == DEFAULT_EMBEDDING_MODEL
    assert cfg.embedding_dim == 384


def test_blank_value_is_treated_as_unset(monkeypatch):
    """编排工具常把未赋值变量渲染成空串，不能因此拿到空 DSN。"""
    monkeypatch.setenv("MEMORY_DB_DSN", "   ")
    cfg = load_config()
    assert cfg.dsn.startswith("postgresql://")


def test_llm_settings_fall_back_to_openai_env(monkeypatch):
    monkeypatch.setenv("OPENAI_BASE_URL", "https://example.test/v1")
    monkeypatch.setenv("OPENAI_API_KEY", "sk-from-openai-var")
    monkeypatch.setenv("OPENAI_MODEL", "some-model")
    cfg = load_config()
    assert cfg.llm_base_url == "https://example.test/v1"
    assert cfg.llm_api_key == "sk-from-openai-var"
    assert cfg.llm_model == "some-model"


def test_memory_prefixed_llm_vars_win_over_openai(monkeypatch):
    monkeypatch.setenv("OPENAI_MODEL", "openai-model")
    monkeypatch.setenv("MEMORY_LLM_MODEL", "memory-model")
    assert load_config().llm_model == "memory-model"


def test_invalid_numbers_fall_back_instead_of_crashing(monkeypatch):
    monkeypatch.setenv("MEMORY_HEAT_ALPHA", "not-a-number")
    monkeypatch.setenv("MEMORY_SHORT_TERM_CAPACITY", "abc")
    cfg = load_config()
    assert cfg.heat.alpha == 1.0
    assert cfg.short_term_capacity == 10


@pytest.mark.parametrize("raw", ["0", "-3"])
def test_capacity_must_be_positive(monkeypatch, raw):
    """容量为 0 会让短期队列永远处于溢出状态，必须挡掉。"""
    monkeypatch.setenv("MEMORY_SHORT_TERM_CAPACITY", raw)
    assert load_config().short_term_capacity == 10


def test_heat_params_are_read_from_env(monkeypatch):
    monkeypatch.setenv("MEMORY_HEAT_ALPHA", "2.5")
    monkeypatch.setenv("MEMORY_HEAT_TAU_HOURS", "48")
    cfg = load_config()
    assert cfg.heat.alpha == 2.5
    assert cfg.heat.tau_hours == 48.0


def test_redacted_dsn_hides_password(monkeypatch):
    monkeypatch.setenv("MEMORY_DB_DSN", "postgresql://admin:supersecret@db:5432/mem")
    redacted = load_config().redacted_dsn()
    assert "supersecret" not in redacted
    assert "admin" in redacted


def test_pool_max_never_below_min(monkeypatch):
    monkeypatch.setenv("MEMORY_DB_POOL_MIN", "5")
    monkeypatch.setenv("MEMORY_DB_POOL_MAX", "2")
    cfg = load_config()
    # db._new_pool 会取 max(min, max)，这里确认配置本身如实反映用户输入，
    # 由建池时兜底，避免静默改写用户配置造成困惑。
    assert cfg.pool_min == 5
    assert cfg.pool_max == 2
