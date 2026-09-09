"""One IP admission middleware and explicit post-auth route quotas, SSE-safe."""
from functools import wraps

from fastapi import Request
from fastapi.routing import APIRoute, request_response

from .policies import EXEMPT, policy_for
from .service import HttpLimits, quota_headers


class HttpRateLimitMiddleware:
    def __init__(self, app, *, service: HttpLimits):
        self.app, self.service = app, service

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http" or scope["method"] == "OPTIONS" or scope["path"] in EXEMPT:
            return await self.app(scope, receive, send)
        request = Request(scope, receive=receive)
        host = request.client.host if request.client else "unknown"
        rejection, _ = await self.service.check(request, "ip", [self.service.bucket("ip:" + host, "ip")])
        if rejection is not None:
            return await rejection(scope, receive, send)
        await self.app(scope, receive, send)


def _guard(handler, policy, template, service):
    @wraps(handler)
    async def guarded(request: Request):
        rejection, decision = await service.for_route(request, policy, template)
        if rejection is not None:
            return rejection
        response = await handler(request)
        response.headers.update(quota_headers(decision))
        return response
    return guarded


def install_http_limits(app, service: HttpLimits | None = None, *, add_middleware: bool = True):
    service = service or HttpLimits()
    app.state.http_limits = service
    for route in app.routes:
        if not isinstance(route, APIRoute):
            continue
        if getattr(route, "http_limits_installed", False):
            raise ValueError("HTTP quota guard installed twice")
        policies = {method: policy_for(method, route.path) for method in route.methods}
        original = route.get_route_handler()
        # All declared methods remain explicit; no parameterized URL guessing.
        async def guarded(request, handlers={method: _guard(original, policy, route.path, service)
                                            if policy else original for method, policy in policies.items()}):
            return await handlers[request.method](request)
        route.app = request_response(guarded)
        route.http_limits_installed = True
    if add_middleware:
        app.add_middleware(HttpRateLimitMiddleware, service=service)
    return service
