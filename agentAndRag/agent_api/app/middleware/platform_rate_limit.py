from __future__ import annotations

import hashlib
import re
import time

from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware

from ..platform.config import get_platform_settings


async def _rate_identity_and_plan(request: Request) -> tuple[str, int, int]:
    """根据 Token/套餐解析限流身份及每分钟配额、突发量。"""
    settings = get_platform_settings()
    host = request.client.host if request.client else "unknown"
    authorization = request.headers.get("authorization", "")
    token = authorization[7:].strip() if authorization.lower().startswith("bearer ") else request.headers.get("x-api-key", "").strip()
    user_id = ""
    key_id = ""
    try:
        if token.startswith("pm_live_"):
            from sqlalchemy import select

            from ..platform.database import platform_session
            from ..platform.models import ApiKey
            from ..platform.security import hash_secret

            async with platform_session() as session:
                key = await session.scalar(select(ApiKey).where(ApiKey.key_hash == hash_secret(token)))
                if key is not None:
                    user_id, key_id = key.user_id, key.id
        elif token:
            from ..platform.security import decode_access_token

            user_id = str(decode_access_token(token).get("sub") or "")
    except Exception:  # noqa: BLE001
        pass

    per_minute = settings.rate_limit_per_minute
    burst = settings.rate_limit_burst
    if user_id:
        try:
            from sqlalchemy import or_, select

            from ..platform.database import platform_session
            from ..platform.models import Plan, Subscription, utcnow

            async with platform_session() as session:
                features = await session.scalar(
                    select(Plan.features)
                    .join(Subscription, Subscription.plan_code == Plan.code)
                    .where(
                        Subscription.user_id == user_id,
                        Subscription.status == "active",
                        or_(Subscription.expires_at.is_(None), Subscription.expires_at > utcnow()),
                    )
                    .order_by(Subscription.expires_at.desc())
                    .limit(1)
                )
            if isinstance(features, dict):
                per_minute = min(10_000, max(1, int(features.get("rate_limit_per_minute", per_minute))))
                burst = min(10_000, max(1, int(features.get("rate_limit_burst", burst))))
        except Exception:  # noqa: BLE001
            pass
    fingerprint = key_id or user_id or (hashlib.sha256(token.encode()).hexdigest()[:16] if token else "anonymous")
    return f"{host}:{fingerprint}", per_minute, burst

_TOKEN_BUCKET = """
local key = KEYS[1]
local now = tonumber(ARGV[1])
local rate = tonumber(ARGV[2])
local burst = tonumber(ARGV[3])
local requested = 1
local values = redis.call('HMGET', key, 'tokens', 'updated')
local tokens = tonumber(values[1]) or burst
local updated = tonumber(values[2]) or now
tokens = math.min(burst, tokens + math.max(0, now - updated) * rate)
local allowed = tokens >= requested
if allowed then tokens = tokens - requested end
redis.call('HMSET', key, 'tokens', tokens, 'updated', now)
redis.call('EXPIRE', key, math.ceil(burst / rate) + 60)
return {allowed and 1 or 0, math.floor(tokens), math.ceil(math.max(0, requested - tokens) / rate)}
"""


class PlatformRateLimitMiddleware(BaseHTTPMiddleware):
    """基于 Redis 的全局令牌桶；开发环境可回退到进程内计数。"""

    def __init__(self, app) -> None:
        """初始化 Redis 客户端占位与本地回退桶。"""
        super().__init__(app)
        self._redis = None
        self._local: dict[str, tuple[float, float]] = {}
        self._fixed_local: dict[str, tuple[int, float]] = {}

    async def _fixed_window(self, key: str, limit: int, seconds: int) -> tuple[bool, int]:
        """固定窗口计数；优先 Redis，生产环境 Redis 失败则报错。"""
        settings = get_platform_settings()
        if settings.redis_url:
            try:
                if self._redis is None:
                    from redis.asyncio import Redis

                    self._redis = Redis.from_url(settings.redis_url, decode_responses=True)
                value = int(await self._redis.incr(key))
                if value == 1:
                    await self._redis.expire(key, seconds)
                ttl = int(await self._redis.ttl(key))
                return value <= limit, max(1, ttl)
            except Exception:
                if settings.production:
                    raise RuntimeError("Redis rate limiter unavailable")
        now = time.monotonic()
        count, expires = self._fixed_local.get(key, (0, now + seconds))
        if now >= expires:
            count, expires = 0, now + seconds
        count += 1
        self._fixed_local[key] = (count, expires)
        return count <= limit, max(1, int(expires - now))

    async def _check(self, key: str, per_minute: int, burst: int) -> tuple[bool, int, int]:
        """令牌桶检查，返回 (是否允许, 剩余令牌, 重试秒数)。"""
        settings = get_platform_settings()
        rate = per_minute / 60.0
        if settings.redis_url:
            try:
                if self._redis is None:
                    from redis.asyncio import Redis

                    self._redis = Redis.from_url(settings.redis_url, decode_responses=True)
                result = await self._redis.eval(_TOKEN_BUCKET, 1, key, time.time(), rate, burst)
                return bool(int(result[0])), int(result[1]), max(1, int(result[2]))
            except Exception:
                if settings.production:
                    raise RuntimeError("Redis rate limiter unavailable")
        now = time.monotonic()
        tokens, updated = self._local.get(key, (float(burst), now))
        tokens = min(float(burst), tokens + max(0.0, now - updated) * rate)
        allowed = tokens >= 1
        if allowed:
            tokens -= 1
        self._local[key] = (tokens, now)
        retry = max(1, int((1 - tokens) / rate)) if not allowed else 1
        return allowed, int(tokens), retry

    async def dispatch(self, request: Request, call_next):
        """对 /api/v1/ 与 /v1/ 做套餐限流及敏感路由固定窗口保护。"""
        if request.method == "OPTIONS" or not request.url.path.startswith(("/api/v1/", "/v1/")):
            return await call_next(request)
        identity, per_minute, burst = await _rate_identity_and_plan(request)
        route = re.sub(r"/[0-9a-fA-F-]{16,}(?=/|$)", "/:id", request.url.path)
        if route.startswith("/api/v1/me/memories/"):
            route = "/api/v1/me/memories/:item"
        sensitive: tuple[int, int, int] | None = None
        if request.method == "POST" and route == "/api/v1/activation-codes/redeem":
            sensitive = (5, 600, 20)
        elif request.method == "DELETE" and route.startswith("/api/v1/me/memories/"):
            sensitive = (5, 60, 30)
        elif request.method == "DELETE" and route == "/api/v1/me/memories":
            # 用户一次访问可能清空三个可见记忆层。
            sensitive = (6, 3600, 20)
        if sensitive:
            limit, seconds, ip_limit = sensitive
            host = request.client.host if request.client else "unknown"
            window = int(time.time()) // seconds
            user_key = hashlib.sha256(f"{identity}:{route}:{window}".encode()).hexdigest()[:32]
            ip_key = hashlib.sha256(f"{host}:{route}:{window}".encode()).hexdigest()[:32]
            try:
                user_allowed, user_retry = await self._fixed_window(f"petmind:sensitive:user:{user_key}", limit, seconds)
                ip_allowed, ip_retry = await self._fixed_window(f"petmind:sensitive:ip:{ip_key}", ip_limit, seconds)
            except RuntimeError:
                return JSONResponse(status_code=503, content={"code": "rate_limiter_unavailable", "message": "Request protection service is unavailable", "request_id": getattr(request.state, "request_id", ""), "details": None}, headers={"Retry-After": "5"})
            if not user_allowed or not ip_allowed:
                retry = max(user_retry if not user_allowed else 0, ip_retry if not ip_allowed else 0, 1)
                return JSONResponse(status_code=429, content={"code": "rate_limited", "message": "Too many requests", "request_id": getattr(request.state, "request_id", ""), "details": {"retry_after": retry}}, headers={"Retry-After": str(retry)})
        digest = hashlib.sha256(f"{identity}:{request.method}:{route}".encode()).hexdigest()[:32]
        try:
            allowed, remaining, retry = await self._check(f"petmind:rate:{digest}", per_minute, burst)
        except RuntimeError:
            return JSONResponse(
                status_code=503,
                content={
                    "code": "rate_limiter_unavailable",
                    "message": "Request protection service is unavailable",
                    "request_id": getattr(request.state, "request_id", ""),
                    "details": None,
                },
                headers={"Retry-After": "5"},
            )
        headers = {
            "X-RateLimit-Limit": str(burst),
            "X-RateLimit-Remaining": str(max(0, remaining)),
        }
        if not allowed:
            headers["Retry-After"] = str(retry)
            return JSONResponse(
                status_code=429,
                content={
                    "code": "rate_limited",
                    "message": "Too many requests",
                    "request_id": getattr(request.state, "request_id", ""),
                    "details": {"retry_after": retry},
                },
                headers=headers,
            )
        response = await call_next(request)
        response.headers.update(headers)
        return response
