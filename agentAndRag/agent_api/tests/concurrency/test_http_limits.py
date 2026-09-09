import asyncio
from contextlib import asynccontextmanager
from unittest.mock import AsyncMock

from fastapi import FastAPI
from fastapi.routing import APIRoute
import httpx
import pytest

from agent_api.app.http_limits.backend import Bucket, LocalBuckets, RedisBuckets
from agent_api.app.http_limits.integration import install_http_limits
from agent_api.app.http_limits.policies import DEFAULTS, Policy, policy_for
from agent_api.app.http_limits.service import HttpLimits


def test_plan_cache_bounded_expiring_and_only_generation_reads_plans(monkeypatch):
    from agent_api.app.http_limits import service as module
    from agent_api.app.platform.dependencies import Principal
    from starlette.requests import Request
    query = AsyncMock(return_value={"rate_limit_per_minute": 42, "rate_limit_burst": 8})
    class Session:
        scalar = query
    @asynccontextmanager
    async def sessions():
        yield Session()
    monkeypatch.setattr(module, 'platform_session', sessions)
    async def scenario():
        clock = [0.0]
        limiter = HttpLimits(backend=RedisBuckets('', production=False), clock=lambda: clock[0])
        assert await limiter.plan_limits('u') == (42, 8)
        assert await limiter.plan_limits('u') == (42, 8)
        assert query.await_count == 1
        clock[0] = 30
        await limiter.plan_limits('u')
        assert query.await_count == 2
        for i in range(1100):
            await limiter.plan_limits(str(i))
        assert len(limiter.plans) == 1024
        request = Request({'type': 'http', 'path': '/', 'headers': [], 'client': ('127.0.0.1', 1)})
        request.state.platform_principal = Principal('u', 'u@example.invalid', 'VET', frozenset(), 'jwt')
        before = query.await_count
        for group in ('read', 'write', 'control'):
            await limiter.for_route(request, Policy(group), '/')
        assert query.await_count == before
        await limiter.close()
        assert not limiter.plans
    asyncio.run(scenario())


def test_refill_multi_budget_atomicity_and_retry():
    async def scenario():
        clock = [100.0]
        store = LocalBuckets(clock=lambda: clock[0])
        parent = Bucket('write:user', 2, 20)
        child = Bucket('feedback:user', .5, 1)
        assert (await store.check([parent, child])).allowed
        decision = await store.check([parent, child])
        assert not decision.allowed and decision.retry_after == 2
        assert store.items[parent.key][0] == 19  # A rejection never debits its parent.
        clock[0] += 1
        assert (await store.check([parent, child])).retry_after == 1
        clock[0] += 1
        assert (await store.check([parent, child])).allowed
    asyncio.run(scenario())


def test_fixed_window_expiry_and_bounded_fallback():
    async def scenario():
        clock = [59.0]
        store = LocalBuckets(maximum=2, clock=lambda: clock[0])
        b = Bucket('sensitive', 0, 1, 60)
        assert (await store.check([b])).allowed
        assert (await store.check([b])).retry_after == 1
        clock[0] = 60
        assert (await store.check([b])).allowed
        for key in ('a', 'b', 'c'):
            await store.check([Bucket(key, 1, 1)])
        assert len(store.items) == 2
        clock[0] += 500
        await store.check([Bucket('new', 1, 1)])
        assert list(store.items) == ['new']
    asyncio.run(scenario())


def test_concurrent_local_burst_has_no_over_admission():
    async def scenario():
        store = LocalBuckets(clock=lambda: 100)
        result = await asyncio.gather(*(store.check([Bucket('user', 1, 6)]) for _ in range(50)))
        assert sum(r.allowed for r in result) == 6
    asyncio.run(scenario())


def test_all_production_routes_have_explicit_policy():
    from agent_api.app.main import app
    for route in app.routes:
        if isinstance(route, APIRoute):
            for method in route.methods:
                policy_for(method, route.path)
    with pytest.raises(ValueError, match="no rate policy"):
        policy_for("POST", "/api/v1/new-unclassified-route")
    feedback = policy_for("PUT", "/api/v1/messages/{message_id}/feedback")
    assert feedback == Policy("write", extra="feedback")
    assert policy_for("DELETE", "/api/v1/runs/{run_id}").group == "control"


def test_unverified_tokens_cannot_create_new_public_buckets():
    async def scenario():
        app = FastAPI()
        @app.post('/api/v1/auth/login')
        async def login():
            return {'ok': True}
        limits = dict(DEFAULTS, auth=(60, 2))
        service = HttpLimits(backend=RedisBuckets('', production=False), limits=limits)
        install_http_limits(app, service)
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
            codes = []
            for token in ('forged-a', 'forged-b', 'forged-c'):
                response = await client.post('/api/v1/auth/login', headers={'Authorization': 'Bearer ' + token})
                codes.append(response.status_code)
            assert codes == [200, 200, 429]
            assert response.headers['Retry-After'] == '1'
            assert response.json()['code'] == 'rate_limited'
            assert response.headers['X-RateLimit-Limit'] == '2'
        await service.close()
    asyncio.run(scenario())


def test_redis_failure_is_closed_in_production_and_headers_are_stable():
    class Broken:
        async def eval(self, *args):
            raise OSError('offline')
        async def aclose(self):
            self.closed = True
    async def scenario():
        backend = RedisBuckets('redis://unused', production=True)
        backend.client = Broken()
        app = FastAPI()
        @app.get('/api/v1/plans')
        async def plans():
            raise AssertionError('must not enter endpoint')
        @app.get('/health')
        async def health():
            return {'ok': True}
        service = install_http_limits(app, HttpLimits(backend=backend))
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
            response = await client.get('/api/v1/plans')
            assert response.status_code == 503 and response.headers['Retry-After'] == '5'
            assert response.json()['code'] == 'rate_limiter_unavailable'
            assert (await client.get('/health')).status_code == 200
        broken = backend.client
        await service.close()
        assert broken.closed and backend.client is None
    asyncio.run(scenario())


def test_sse_counts_connections_and_resume_not_chunks(monkeypatch):
    from fastapi.responses import StreamingResponse
    from agent_api.app.http_limits import service as module
    from agent_api.app.platform.dependencies import Principal
    principal = Principal('stream-user', 'stream@example.invalid', 'VET', frozenset(), 'jwt')
    monkeypatch.setattr(module, 'get_current_principal', AsyncMock(return_value=principal))
    async def scenario():
        app = FastAPI()
        @app.get('/api/v1/runs/{run_id}/events')
        async def events(run_id: str):
            async def chunks():
                for i in range(20):
                    yield f'id: {i}\ndata: synthetic\n\n'
            return StreamingResponse(chunks(), media_type='text/event-stream')
        backend = RedisBuckets('', production=False)
        backend.local.clock = lambda: 100.0
        limiter = install_http_limits(app, HttpLimits(backend=backend, limits=dict(DEFAULTS, control=(120, 2))))
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
            first = await client.get('/api/v1/runs/one/events')
            assert first.status_code == 200 and first.text.count('data:') == 20
            assert first.headers['X-RateLimit-Remaining'] == '1'
            resumed = await client.get('/api/v1/runs/one/events', headers={'Last-Event-ID': '10'})
            assert resumed.status_code == 200 and resumed.headers['X-RateLimit-Remaining'] == '0'
            assert (await client.get('/api/v1/runs/two/events')).status_code == 429
        await limiter.close()
    asyncio.run(scenario())
