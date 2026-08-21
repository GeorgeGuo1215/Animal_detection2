"""
按 API key 节流的令牌桶限流中间件。

无需外部依赖，使用进程内内存令牌桶。
"""
from __future__ import annotations

import asyncio
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Optional

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import JSONResponse


@dataclass
class _Bucket:
    """单个令牌桶的剩余令牌数与上次补充时间。"""

    tokens: float
    last_refill: float


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    按 key 的令牌桶限流。

    Parameters
    ----------
    rate : float
        每个 *period* 秒允许的请求数。
    period : float
        时间窗口秒数（默认 60，即每分钟 rate 次）。
    burst : int | None
        最大突发量，默认等于 *rate*。
    """

    def __init__(self, app, *, rate: float = 30, period: float = 60, burst: Optional[int] = None):
        """初始化限流参数与豁免路径。"""
        super().__init__(app)
        self.rate = rate
        self.period = period
        self.burst = float(burst if burst is not None else rate)
        self._buckets: Dict[str, _Bucket] = defaultdict(
            lambda: _Bucket(tokens=self.burst, last_refill=time.monotonic())
        )
        self._lock = asyncio.Lock()
        self._exempt = {
            "/health",
            "/ready",
            "/docs",
            "/openapi.json",
            "/redoc",
        }

    def _extract_key(self, request: Request) -> str:
        """从 Bearer / X-API-Key 或客户端 IP 提取限流键。"""
        auth = request.headers.get("authorization", "")
        if auth.lower().startswith("bearer "):
            return auth[7:].strip()[:32]
        api_key = request.headers.get("x-api-key", "")
        if api_key:
            return api_key.strip()[:32]
        client = request.client
        return client.host if client else "unknown"

    async def dispatch(self, request: Request, call_next):
        """豁免路径直接放行，否则按令牌桶扣减；不足则返回 429。"""
        if (
            request.url.path in self._exempt
            or request.url.path.startswith(("/api/v1/", "/v1/"))
            or request.method == "OPTIONS"
        ):
            return await call_next(request)

        key = self._extract_key(request)
        async with self._lock:
            bucket = self._buckets[key]
            now = time.monotonic()
            elapsed = now - bucket.last_refill
            bucket.tokens = min(self.burst, bucket.tokens + elapsed * (self.rate / self.period))
            bucket.last_refill = now

            if bucket.tokens < 1.0:
                retry_after = (1.0 - bucket.tokens) / (self.rate / self.period)
                return JSONResponse(
                    status_code=429,
                    content={"error": {"message": "Rate limit exceeded", "retry_after_seconds": round(retry_after, 1)}},
                    headers={"Retry-After": str(int(retry_after) + 1)},
                )
            bucket.tokens -= 1.0

        return await call_next(request)
