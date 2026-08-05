"""
Token-bucket rate limiter middleware for per-API-key throttling.

No external dependency required -- uses a simple in-memory token bucket.
"""
from __future__ import annotations

import asyncio
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Optional

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import JSONResponse


@dataclass
class _Bucket:
    tokens: float
    last_refill: float


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Per-key token-bucket rate limiter.

    Parameters
    ----------
    rate : float
        Number of requests allowed per *period* seconds.
    period : float
        Time window in seconds (default 60 → rate requests per minute).
    burst : int | None
        Maximum burst size.  Defaults to *rate*.
    """

    def __init__(self, app, *, rate: float = 30, period: float = 60, burst: Optional[int] = None):
        super().__init__(app)
        self.rate = rate
        self.period = period
        self.burst = float(burst if burst is not None else rate)
        self._buckets: Dict[str, _Bucket] = defaultdict(
            lambda: _Bucket(tokens=self.burst, last_refill=time.monotonic())
        )
        self._lock = asyncio.Lock()
        self._exempt = {"/health", "/docs", "/openapi.json", "/redoc"}

    def _extract_key(self, request: Request) -> str:
        auth = request.headers.get("authorization", "")
        if auth.lower().startswith("bearer "):
            return auth[7:].strip()[:32]
        api_key = request.headers.get("x-api-key", "")
        if api_key:
            return api_key.strip()[:32]
        client = request.client
        return client.host if client else "unknown"

    async def dispatch(self, request: Request, call_next):
        if request.url.path in self._exempt or request.method == "OPTIONS":
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
