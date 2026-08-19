from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import app.middleware.auth as auth


MISSING_KEYS_PATH = Path(__file__).with_name("_missing_auth_test_keys.txt")


def test_missing_key_configuration_fails_fast(monkeypatch):
    monkeypatch.setattr(auth, "_keys_file_path", lambda: MISSING_KEYS_PATH)
    monkeypatch.delenv("AGENT_API_KEYS", raising=False)
    monkeypatch.delenv("AGENT_ALLOW_INSECURE_DEFAULT_KEY", raising=False)
    monkeypatch.delenv("AGENT_DISABLE_AUTH", raising=False)
    monkeypatch.setenv("AGENT_LEGACY_API_KEYS_ENABLED", "1")

    with pytest.raises(RuntimeError, match="API key configuration is missing"):
        auth.load_api_keys()

    assert not MISSING_KEYS_PATH.exists()


def test_environment_keys_work_without_writing_a_file(monkeypatch):
    monkeypatch.setattr(auth, "_keys_file_path", lambda: MISSING_KEYS_PATH)
    monkeypatch.setenv("AGENT_API_KEYS", "key-one,key-two")
    monkeypatch.delenv("AGENT_ALLOW_INSECURE_DEFAULT_KEY", raising=False)
    monkeypatch.delenv("AGENT_DISABLE_AUTH", raising=False)
    monkeypatch.setenv("AGENT_LEGACY_API_KEYS_ENABLED", "1")

    auth.load_api_keys()

    assert auth.is_valid_key("key-one")
    assert auth.is_valid_key("key-two")
    assert not MISSING_KEYS_PATH.exists()


def test_disabled_legacy_keys_do_not_require_or_load_key_file(monkeypatch, tmp_path):
    key_file = tmp_path / "keys.txt"
    key_file.write_text("legacy-secret\n", encoding="utf-8")
    monkeypatch.setattr(auth, "_keys_file_path", lambda: key_file)
    monkeypatch.setenv("AGENT_LEGACY_API_KEYS_ENABLED", "0")
    monkeypatch.delenv("AGENT_API_KEYS", raising=False)
    monkeypatch.delenv("AGENT_DISABLE_AUTH", raising=False)

    auth.load_api_keys()

    assert not auth.is_valid_key("legacy-secret")
    assert auth._VALID_KEYS == set()
