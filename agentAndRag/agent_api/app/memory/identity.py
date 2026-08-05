"""Stable identity mapping for browser-only Chat-MoE memory subjects."""
from __future__ import annotations

import hashlib
import unicodedata


def normalize_test_username(username: str) -> str:
    """Normalize a tester-supplied name without changing its display spelling."""
    return " ".join(unicodedata.normalize("NFKC", str(username or "")).strip().split())


def chat_moe_memory_user_id(username: str) -> str:
    """Map a test username to a stable, namespaced, non-plaintext subject id."""
    normalized = normalize_test_username(username)
    if not normalized:
        raise ValueError("username is required")
    digest = hashlib.sha256(normalized.casefold().encode("utf-8")).hexdigest()[:32]
    return f"chatmoe:{digest}"
