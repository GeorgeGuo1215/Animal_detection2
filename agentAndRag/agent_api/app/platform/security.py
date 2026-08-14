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
    value = str(email or "").strip().casefold()
    if len(value) > 320 or "@" not in value or value.startswith("@") or value.endswith("@"):
        raise ValueError("invalid email address")
    return value


def hash_secret(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def secure_token(bytes_count: int = 32) -> str:
    return secrets.token_urlsafe(bytes_count)


def hash_password(password: str) -> str:
    if len(password) < 10 or len(password) > 200:
        raise ValueError("password must contain 10-200 characters")
    try:
        from argon2 import PasswordHasher
    except ImportError as exc:
        raise RuntimeError("argon2-cffi is required for platform password hashing") from exc
    return PasswordHasher(time_cost=3, memory_cost=65536, parallelism=4).hash(password)


def verify_password(password_hash: str, password: str) -> bool:
    try:
        from argon2 import PasswordHasher
        from argon2.exceptions import InvalidHashError, VerifyMismatchError

        return PasswordHasher().verify(password_hash, password)
    except (VerifyMismatchError, InvalidHashError):
        return False


def create_access_token(*, user_id: str, role: str, token_version: int) -> str:
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
    prefix = secrets.token_hex(5)
    raw = f"pm_live_{prefix}_{secure_token(32)}"
    return raw, prefix, hash_secret(raw)


def verify_webhook_signature(*, body: bytes, timestamp: str, signature: str) -> bool:
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
