"""Atomic distributed buckets plus bounded, testable development fallback."""
import asyncio
from collections import OrderedDict
from dataclasses import dataclass
import json
import math
import time


@dataclass(frozen=True)
class Bucket:
    key: str
    rate: float
    capacity: int
    window: int = 0


@dataclass(frozen=True)
class Decision:
    allowed: bool
    remaining: int
    retry_after: int
    limit: int


# All clocks and expiry boundaries come from Redis. Multiple user budgets are
# checked and debited together: rejecting a sub-policy does not consume its parent.
SCRIPT = """
local clock = redis.call('TIME')
local now = tonumber(clock[1]) + tonumber(clock[2]) / 1000000
local cfg = cjson.decode(ARGV[1])
local states = {}
local allowed, retry, chosen, remaining = true, 0, 1, math.huge
for i, key in ipairs(KEYS) do
  local c = cfg[i]
  local values = redis.call('HMGET', key, 'tokens', 'updated')
  local tokens = tonumber(values[1]) or c.capacity
  local updated = tonumber(values[2]) or now
  local ttl, wait
  if c.window > 0 then
    local start = math.floor(now / c.window) * c.window
    if updated ~= start then tokens = c.capacity end
    updated = start
    ttl = math.ceil(start + c.window - now)
    wait = ttl
  else
    tokens = math.min(c.capacity, tokens + math.max(0, now - updated) * c.rate)
    updated = now
    ttl = math.ceil(c.capacity / c.rate) + 60
    wait = math.ceil(math.max(0, 1 - tokens) / c.rate)
  end
  states[i] = {tokens, updated, ttl}
  if tokens < 1 then allowed = false; retry = math.max(retry, wait) end
  if tokens < remaining then remaining = tokens; chosen = i end
end
for i, key in ipairs(KEYS) do
  local s = states[i]
  local tokens = s[1] - (allowed and 1 or 0)
  redis.call('HSET', key, 'tokens', tokens, 'updated', s[2])
  redis.call('EXPIRE', key, math.max(1, s[3]))
end
return {allowed and 1 or 0, math.floor(math.max(0, remaining - (allowed and 1 or 0))),
        math.max(1, retry), cfg[chosen].capacity}
"""


class LocalBuckets:
    def __init__(self, *, maximum: int = 10000, clock=time.monotonic):
        self.maximum, self.clock = maximum, clock
        self.items: OrderedDict[str, tuple[float, float, float]] = OrderedDict()
        self.lock = asyncio.Lock()
        self.next_prune = 0.0

    async def check(self, buckets: list[Bucket]) -> Decision:
        async with self.lock:
            now = self.clock()
            # Amortize the bounded sweep; hot requests do not scan 10,000 keys.
            if now >= self.next_prune:
                for key in list(self.items):
                    if self.items[key][2] <= now:
                        del self.items[key]
                self.next_prune = now + 1
            states = []
            retry, remaining, limit = 0, math.inf, 0
            for b in buckets:
                stored = self.items.get(b.key)
                tokens, updated, _ = stored if stored and stored[2] > now else (float(b.capacity), now, now)
                if b.window:
                    start = math.floor(now / b.window) * b.window
                    if updated != start:
                        tokens = float(b.capacity)
                    updated, expires = start, start + b.window
                    wait = math.ceil(expires - now)
                else:
                    tokens = min(b.capacity, tokens + max(0, now - updated) * b.rate)
                    updated, expires = now, now + math.ceil(b.capacity / b.rate) + 60
                    wait = math.ceil(max(0, 1 - tokens) / b.rate)
                if tokens < 1:
                    retry = max(retry, wait)
                if tokens < remaining:
                    remaining, limit = tokens, b.capacity
                states.append((b.key, tokens, updated, expires))
            allowed = remaining >= 1
            for key, tokens, updated, expires in states:
                self.items[key] = (tokens - int(allowed), updated, expires)
                self.items.move_to_end(key)
            while len(self.items) > self.maximum:
                self.items.popitem(last=False)
            return Decision(allowed, max(0, math.floor(remaining) - int(allowed)), max(1, retry), limit)


class RedisBuckets:
    def __init__(self, url: str, *, production: bool, maximum: int = 10000):
        self.url, self.production = url, production
        self.client = None
        self.script = None
        self.local = LocalBuckets(maximum=maximum)

    async def check(self, buckets: list[Bucket]) -> Decision:
        if self.url:
            try:
                if self.client is None:
                    from redis.asyncio import Redis
                    self.client = Redis.from_url(self.url, decode_responses=True,
                                                 socket_connect_timeout=1, socket_timeout=1)
                if self.script is None:
                    self.script = self.client.register_script(SCRIPT)
                result = await self.script(keys=[b.key for b in buckets], args=[json.dumps([
                    {"rate": b.rate, "capacity": b.capacity, "window": b.window} for b in buckets
                ])])
                return Decision(bool(result[0]), int(result[1]), int(result[2]), int(result[3]))
            except Exception as exc:
                if self.production:
                    raise RuntimeError("HTTP rate limiter unavailable") from exc
        elif self.production:
            raise RuntimeError("HTTP rate limiter requires Redis")
        return await self.local.check(buckets)

    async def close(self) -> None:
        if self.client is not None:
            await self.client.aclose()
            self.client = None
        self.script = None
        self.local.items.clear()
