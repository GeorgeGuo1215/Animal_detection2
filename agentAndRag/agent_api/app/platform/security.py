from __future__ import annotations

import hashlib
import hmac
import secrets
import time
from datetime import datetime, timedelta, timezone
from typing import Any

import jwt

from .config import get_platform_settings


def normalize_email(email: str) -> str:
    """规范化邮箱（去空白、小写），并做基本格式校验。

    Raises:
        ValueError: 长度超限或缺少有效 ``@`` 分隔时抛出。
    """
    value = str(email or "").strip().casefold()
    if len(value) > 320 or "@" not in value or value.startswith("@") or value.endswith("@"):
        raise ValueError("invalid email address")
    return value


def hash_secret(value: str) -> str:
    """对密钥、API Key 等敏感字符串做 SHA-256 十六进制摘要。"""
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def secure_token(bytes_count: int = 32) -> str:
    """生成 URL 安全的随机令牌，``bytes_count`` 为底层随机字节数。"""
    return secrets.token_urlsafe(bytes_count)


def hash_password(password: str) -> str:
    """使用 Argon2 哈希用户密码。

    Raises:
        ValueError: 密码长度不在 10–200 字符时抛出。
        RuntimeError: 未安装 ``argon2-cffi`` 时抛出。
    """
    if len(password) < 10 or len(password) > 200:
        raise ValueError("password must contain 10-200 characters")
    try:
        from argon2 import PasswordHasher
    except ImportError as exc:
        raise RuntimeError("argon2-cffi is required for platform password hashing") from exc
    return PasswordHasher(time_cost=3, memory_cost=65536, parallelism=4).hash(password)


def verify_password(password_hash: str, password: str) -> bool:
    """校验明文密码是否匹配 Argon2 哈希；哈希无效或不匹配时返回 False。"""
    try:
        from argon2 import PasswordHasher
        from argon2.exceptions import InvalidHashError, VerifyMismatchError

        return PasswordHasher().verify(password_hash, password)
    except (VerifyMismatchError, InvalidHashError):
        return False


def create_access_token(*, user_id: str, role: str, token_version: int) -> str:
    """签发短时 JWT access token，载荷含用户 ID、角色与 token 版本。"""
    settings = get_platform_settings()
    now = datetime.now(timezone.utc)
    payload = {
        "sub": user_id,
        "role": role,
        "ver": token_version,
        "jti": secure_token(12),
        "iat": now,
        "nbf": now,
        "exp": now + timedelta(seconds=settings.access_ttl_seconds),
        "iss": settings.jwt_issuer,
        "aud": settings.jwt_audience,
    }
    return jwt.encode(payload, settings.jwt_secret, algorithm=settings.jwt_algorithm)


def decode_access_token(token: str) -> dict[str, Any]:
    """校验并解码 JWT access token，返回载荷字典。"""
    settings = get_platform_settings()
    return jwt.decode(
        token,
        settings.jwt_secret,
        algorithms=[settings.jwt_algorithm],
        issuer=settings.jwt_issuer,
        audience=settings.jwt_audience,
        options={"require": ["sub", "role", "ver", "jti", "exp", "iat"]},
    )


def create_api_key() -> tuple[str, str, str]:
    """生成平台 API Key。

    Returns:
        ``(明文密钥, 前缀, SHA-256 摘要)``，明文仅在创建时返回一次。
    """
    prefix = secrets.token_hex(5)
    raw = f"pm_live_{prefix}_{secure_token(32)}"
    return raw, prefix, hash_secret(raw)


def verify_webhook_signature(*, body: bytes, timestamp: str, signature: str) -> bool:
    """校验支付 Webhook 的 HMAC-SHA256 签名；时间戳偏离超过 5 分钟视为无效。"""
    settings = get_platform_settings()
    try:
        ts = int(timestamp)
    except ValueError:
        return False
    if abs(int(time.time()) - ts) > 300:
        return False
    expected = hmac.new(
        settings.payment_webhook_secret.encode("utf-8"),
        timestamp.encode("ascii") + b"." + body,
        hashlib.sha256,
    ).hexdigest()
    return hmac.compare_digest(expected, signature)
