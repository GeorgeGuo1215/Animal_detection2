from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path


def _bool(name: str, default: bool) -> bool:
    """读取环境变量并解析为布尔值。

    未设置时返回 ``default``；``0`` / ``false`` / ``no`` / ``off``（大小写不敏感）视为 False。
    """
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off"}


def _int(name: str, default: int, minimum: int = 1) -> int:
    """读取环境变量并解析为不小于 ``minimum`` 的整数；非法值回退到 ``default``。"""
    try:
        return max(minimum, int(os.getenv(name, "") or default))
    except ValueError:
        return default


def _default_sqlite_url() -> str:
    """返回开发环境默认的 SQLite 异步连接 URL。"""
    root = Path(__file__).resolve().parents[3]
    path = (root / "agent_api_logs" / "petmind_platform.db").as_posix()
    return f"sqlite+aiosqlite:///{path}"


@dataclass(frozen=True)
class PlatformSettings:
    enabled: bool
    environment: str
    database_url: str
    redis_url: str
    auto_create_schema: bool
    jwt_secret: str
    jwt_algorithm: str
    jwt_issuer: str
    jwt_audience: str
    access_ttl_seconds: int
    refresh_ttl_seconds: int
    refresh_reuse_grace_seconds: int
    invite_ttl_seconds: int
    reset_ttl_seconds: int
    cookie_secure: bool
    expose_dev_tokens: bool
    rate_limit_per_minute: int
    rate_limit_burst: int
    sync_timeout_seconds: int
    conversation_retention_days: int
    conversation_delete_grace_days: int
    payment_webhook_secret: str
    frontend_origin: str
    allowed_hosts: tuple[str, ...]
    database_pool_size: int
    database_max_overflow: int

    @property
    def production(self) -> bool:
        """当前是否运行在 production 环境。"""
        return self.environment == "production"

    def validate(self) -> None:
        """校验生产环境安全约束；平台未启用时直接返回。

        Raises:
            RuntimeError: 生产环境缺少 PostgreSQL、Redis、JWT/Webhook 密钥强度，
                或仍使用开发期不安全配置时抛出。
        """
        if not self.enabled:
            return
        if self.production:
            if not self.database_url.startswith(("postgresql+asyncpg://", "postgresql://")):
                raise RuntimeError("Production Agent platform requires PostgreSQL/asyncpg")
            if not self.redis_url:
                raise RuntimeError("Production Agent platform requires AGENT_PLATFORM_REDIS_URL")
            if len(self.jwt_secret.encode("utf-8")) < 32:
                raise RuntimeError("AGENT_PLATFORM_JWT_SECRET must contain at least 32 bytes")
            if self.auto_create_schema:
                raise RuntimeError("Production schema must be managed by Alembic, not create_all")
            if self.expose_dev_tokens:
                raise RuntimeError("Development token exposure is forbidden in production")
            if _bool("AGENT_ALLOW_INSECURE_DEFAULT_KEY", False):
                raise RuntimeError("Insecure default API key is forbidden in production")
            if not self.cookie_secure:
                raise RuntimeError("Production refresh cookies require AGENT_PLATFORM_COOKIE_SECURE=1")
            if not self.frontend_origin.startswith("https://"):
                raise RuntimeError("Production frontend origin must use HTTPS")
            worker_secret = os.getenv("AGENT_WORKER_TOKEN", "")
            if len(worker_secret.encode("utf-8")) < 32 or worker_secret == self.jwt_secret:
                raise RuntimeError("Production AGENT_WORKER_TOKEN must be an independent 32-byte secret")
            memory_secret = os.getenv("MEMORY_MANAGEMENT_TOKEN", "")
            if _bool("AGENT_MEMORY_ENABLED", True) and len(memory_secret.encode("utf-8")) < 32:
                raise RuntimeError("Production MEMORY_MANAGEMENT_TOKEN must contain at least 32 bytes")
            if len(self.payment_webhook_secret.encode("utf-8")) < 32 or self.payment_webhook_secret == "development-webhook-secret":
                raise RuntimeError("Production payment webhook secret must contain at least 32 bytes")


@lru_cache(maxsize=1)
def get_platform_settings() -> PlatformSettings:
    """从环境变量加载并缓存平台配置，加载后立即执行校验。"""
    environment = (os.getenv("AGENT_PLATFORM_ENV") or "development").strip().lower()
    settings = PlatformSettings(
        enabled=_bool("AGENT_PLATFORM_ENABLED", True),
        environment=environment,
        database_url=(os.getenv("AGENT_PLATFORM_DB_URL") or _default_sqlite_url()).strip(),
        redis_url=(os.getenv("AGENT_PLATFORM_REDIS_URL") or "").strip(),
        auto_create_schema=_bool("AGENT_PLATFORM_AUTO_CREATE_SCHEMA", environment != "production"),
        jwt_secret=os.getenv("AGENT_PLATFORM_JWT_SECRET", "petmind-development-secret-change-me"),
        jwt_algorithm=os.getenv("AGENT_PLATFORM_JWT_ALGORITHM", "HS256"),
        jwt_issuer=os.getenv("AGENT_PLATFORM_JWT_ISSUER", "petmind-agent"),
        jwt_audience=os.getenv("AGENT_PLATFORM_JWT_AUDIENCE", "petmind-agent-web"),
        access_ttl_seconds=_int("AGENT_PLATFORM_ACCESS_TTL_SEC", 15 * 60),
        refresh_ttl_seconds=_int("AGENT_PLATFORM_REFRESH_TTL_SEC", 30 * 24 * 3600),
        refresh_reuse_grace_seconds=_int("AGENT_PLATFORM_REFRESH_REUSE_GRACE_SEC", 5, minimum=0),
        invite_ttl_seconds=_int("AGENT_PLATFORM_INVITE_TTL_SEC", 7 * 24 * 3600),
        reset_ttl_seconds=_int("AGENT_PLATFORM_RESET_TTL_SEC", 30 * 60),
        cookie_secure=_bool("AGENT_PLATFORM_COOKIE_SECURE", environment == "production"),
        expose_dev_tokens=_bool("AGENT_PLATFORM_EXPOSE_DEV_TOKENS", environment != "production"),
        rate_limit_per_minute=_int("AGENT_PLATFORM_RATE_LIMIT", 60),
        rate_limit_burst=_int("AGENT_PLATFORM_RATE_BURST", 20),
        sync_timeout_seconds=min(120, _int("AGENT_PLATFORM_SYNC_TIMEOUT_SEC", 120)),
        conversation_retention_days=_int("AGENT_PLATFORM_CONVERSATION_RETENTION_DAYS", 365),
        conversation_delete_grace_days=_int("AGENT_PLATFORM_DELETE_GRACE_DAYS", 30),
        payment_webhook_secret=os.getenv("AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET", "development-webhook-secret"),
        frontend_origin=os.getenv("AGENT_PLATFORM_FRONTEND_ORIGIN", "http://localhost:5173"),
        allowed_hosts=tuple(
            host.strip()
            for host in os.getenv("AGENT_PLATFORM_ALLOWED_HOSTS", "localhost,127.0.0.1").split(",")
            if host.strip()
        ),
        database_pool_size=_int("AGENT_PLATFORM_DB_POOL_SIZE", 10),
        database_max_overflow=_int("AGENT_PLATFORM_DB_MAX_OVERFLOW", 20, minimum=0),
    )
    settings.validate()
    return settings


def reset_platform_settings_cache() -> None:
    """清空 ``get_platform_settings`` 的 LRU 缓存，便于测试重新加载配置。"""
    get_platform_settings.cache_clear()
