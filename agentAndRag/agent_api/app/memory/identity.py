"""仅浏览器 Chat-MoE 记忆主体的稳定身份映射。"""
from __future__ import annotations

import hashlib
import unicodedata


def normalize_test_username(username: str) -> str:
    """规范化测试者输入的用户名，不改变其展示用拼写。"""
    return " ".join(unicodedata.normalize("NFKC", str(username or "")).strip().split())


def chat_moe_memory_user_id(username: str) -> str:
    """将测试用户名映射为稳定、带命名空间、非明文的 subject id。"""
    normalized = normalize_test_username(username)
    if not normalized:
        raise ValueError("username is required")
    digest = hashlib.sha256(normalized.casefold().encode("utf-8")).hexdigest()[:32]
    return f"chatmoe:{digest}"
