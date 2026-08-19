from __future__ import annotations

import re
import time
import uuid

from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware


class PlatformRequestMiddleware(BaseHTTPMiddleware):
    """Attach request IDs and enforce a bounded body before JSON parsing."""

    def __init__(self, app, *, max_body_bytes: int = 1_048_576) -> None:
        super().__init__(app)
        self.max_body_bytes = max_body_bytes
        self.snapshot_body_bytes = 8 * 1024 * 1024

    async def dispatch(self, request: Request, call_next):
        request_id = (request.headers.get("x-request-id") or uuid.uuid4().hex)[:64]
        request.state.request_id = request_id
        if request.url.path.startswith("/api/v1/"):
            limit = self.max_body_bytes
            if request.method == "POST" and re.fullmatch(
                r"/api/v1/admin/users/[^/]+/data-snapshot/restore-file",
                request.url.path,
            ):
                limit = self.snapshot_body_bytes
            raw_length = request.headers.get("content-length")
            too_large = bool(raw_length and raw_length.isdigit() and int(raw_length) > limit)
            if not too_large and request.method in {"POST", "PUT", "PATCH", "DELETE"}:
                too_large = len(await request.body()) > limit
            if too_large:
                return JSONResponse(
                    status_code=413,
                    content={
                        "code": "request_too_large",
                        "message": "Request body exceeds the platform limit",
                        "request_id": request_id,
                        "details": {"max_bytes": limit},
                    },
                    headers={"X-Request-Id": request_id},
                )
        started = time.perf_counter()
        response = await call_next(request)
        response.headers["X-Request-Id"] = request_id
        response.headers["X-Response-Time-Ms"] = f"{(time.perf_counter() - started) * 1000:.1f}"
        return response
