"""Shared HTTP quota service. No model/resource slots are acquired here."""
from collections import Counter, OrderedDict
import hashlib
import os
import time
import uuid

from fastapi import Request
from fastapi.responses import JSONResponse
from sqlalchemy import or_, select

from ..platform.config import get_platform_settings
from ..platform.database import platform_session
from ..platform.dependencies import Principal, get_current_principal
from ..platform.models import Plan, Subscription, utcnow
from .backend import Bucket, Decision, RedisBuckets
from .policies import Policy, configured_limits


class HttpLimits:
    def __init__(self, *, backend=None, limits=None, prefix: str | None = None, clock=time.monotonic):
        settings = get_platform_settings()
        self.backend = backend or RedisBuckets(settings.redis_url, production=settings.production)
        self.limits = limits or configured_limits()
        self.prefix = prefix or os.getenv("AGENT_HTTP_RATE_PREFIX", "petmind:http:v1")
        self.clock = clock
        self.plans: OrderedDict[str, tuple[float, tuple[int, int]]] = OrderedDict()
        self.metrics = Counter()

    def key(self, identity: str, group: str) -> str:
        return f"{self.prefix}:{group}:{hashlib.sha256(identity.encode()).hexdigest()}"

    def bucket(self, identity: str, group: str, values=None) -> Bucket:
        rate, capacity = values or self.limits[group]
        return Bucket(self.key(identity, group), rate / 60.0, capacity)

    async def plan_limits(self, user_id: str) -> tuple[int, int]:
        now = self.clock()
        cached = self.plans.get(user_id)
        if cached is not None and cached[0] > now:
            self.plans.move_to_end(user_id)
            return cached[1]
        async with platform_session() as session:
            features = await session.scalar(select(Plan.features).join(
                Subscription, Subscription.plan_code == Plan.code
            ).where(Subscription.user_id == user_id, Subscription.status == "active",
                    or_(Subscription.expires_at.is_(None), Subscription.expires_at > utcnow()))
              .order_by(Subscription.expires_at.desc()).limit(1))
        rate, burst = self.limits["generation"]
        if isinstance(features, dict):
            try:
                rate = min(10000, max(1, int(features.get("rate_limit_per_minute", rate))))
                burst = min(10000, max(1, int(features.get("rate_limit_burst", burst))))
            except (ValueError, TypeError):
                rate, burst = self.limits["generation"]
        for key in list(self.plans):
            if self.plans[key][0] <= now:
                del self.plans[key]
        self.plans[user_id] = (now + 30, (rate, burst))
        self.plans.move_to_end(user_id)
        while len(self.plans) > 1024:
            self.plans.popitem(last=False)
        return rate, burst

    async def check(self, request: Request, group: str, buckets: list[Bucket]):
        try:
            decision = await self.backend.check(buckets)
        except RuntimeError:
            self.metrics[group, "unavailable"] += 1
            return failure(request, 503, "rate_limiter_unavailable", 5), None
        self.metrics[group, "allowed" if decision.allowed else "rejected"] += 1
        if not decision.allowed:
            return failure(request, 429, "rate_limited", decision.retry_after, decision), decision
        return None, decision

    async def for_route(self, request: Request, policy: Policy, template: str):
        host = request.client.host if request.client else "unknown"
        principal = getattr(request.state, "platform_principal", None)
        if policy.identity == "user":
            # The normal dependency subsequently reuses this validated principal.
            async with platform_session() as session:
                principal = await get_current_principal(request, session)
        if isinstance(principal, Principal) and policy.identity != "public":
            identity = "user:" + principal.user_id
        elif policy.identity == "legacy" and getattr(request.state, "http_legacy_identity", None):
            identity = request.state.http_legacy_identity
        else:
            identity = "ip:" + host  # Unverified credentials never create fresh quotas.
        values = await self.plan_limits(principal.user_id) if (
            policy.group == "generation" and isinstance(principal, Principal)
        ) else None
        buckets = [self.bucket(identity, policy.group, values)]
        if policy.extra:
            buckets.append(self.bucket(identity, policy.extra))
        if policy.sensitive:
            limit, seconds, ip_limit = policy.sensitive
            buckets.extend([
                Bucket(self.key(identity, "sensitive:user:" + template), 0, limit, seconds),
                Bucket(self.key("ip:" + host, "sensitive:ip:" + template), 0, ip_limit, seconds),
            ])
        return await self.check(request, policy.extra or policy.group, buckets)

    def snapshot(self):
        return {"policies": {k: {"per_minute": v[0], "burst": v[1]} for k, v in self.limits.items()},
                "counters": {f"{group}.{outcome}": count for (group, outcome), count in self.metrics.items()},
                "counter_scope": "process", "quota_scope": "redis" if self.backend.url else "process-development",
                "plan_cache_entries": len(self.plans)}

    async def close(self):
        await self.backend.close()
        self.plans.clear()


def quota_headers(decision: Decision) -> dict[str, str]:
    return {"X-RateLimit-Limit": str(decision.limit), "X-RateLimit-Remaining": str(decision.remaining)}


def failure(request: Request, status: int, code: str, retry: int, decision=None):
    message = "请求过于频繁，请稍后重试" if status == 429 else "请求保护服务暂不可用，请稍后重试"
    headers = {"Retry-After": str(retry), **(quota_headers(decision) if decision else {})}
    request_id = getattr(request.state, "request_id", None) or uuid.uuid4().hex
    headers["X-Request-Id"] = request_id
    if request.url.path.startswith("/api/v1/"):
        content = {"code": code, "message": message, "request_id": request_id,
                   "details": {"retry_after": retry}}
    else:
        content = {"error": {"message": message, "type": code, "code": code, "retry_after_seconds": retry}}
    return JSONResponse(status_code=status, content=content, headers=headers)
