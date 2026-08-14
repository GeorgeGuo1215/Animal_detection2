"""
API Key authentication middleware.

Keys are loaded from keys.txt (one key per line) on startup.
Clients must send header: Authorization: Bearer <api-key>
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Set

from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware


_VALID_KEYS: Set[str] = set()


def _keys_file_path() -> Path:
    """Return path to keys.txt under agent_api/."""
    return Path(__file__).resolve().parents[2] / "keys.txt"


def load_api_keys() -> None:
    """Load API keys from keys.txt into memory. Called on startup."""
    global _VALID_KEYS
    path = _keys_file_path()
    env_keys = {
        key.strip()
        for key in os.getenv("AGENT_API_KEYS", "").split(",")
        if key.strip()
    }
    if not path.exists():
        if os.getenv("AGENT_ALLOW_INSECURE_DEFAULT_KEY", "0") == "1":
            default_key = "sk-petmind-default-key-2026"
            path.write_text(
                f"# INSECURE development key; replace before deployment\n{default_key}\n",
                encoding="utf-8",
            )
            print(f"[auth] Created explicitly enabled insecure development key at {path}")
        elif not env_keys and os.getenv("AGENT_DISABLE_AUTH", "0") != "1":
            raise RuntimeError(
                f"API key configuration is missing: create {path} or set AGENT_API_KEYS. "
                "For local-only development, explicitly set AGENT_ALLOW_INSECURE_DEFAULT_KEY=1."
            )
    
    keys = set(env_keys)
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                keys.add(line)
    if not keys and os.getenv("AGENT_DISABLE_AUTH", "0") != "1":
        raise RuntimeError("Authentication is enabled but no API keys were configured.")
    _VALID_KEYS = keys
    print(f"[auth] Loaded {len(_VALID_KEYS)} API key(s) from {path}")


def is_valid_key(key: str) -> bool:
    """Check if the given key is valid."""
    legacy_enabled = os.getenv("AGENT_LEGACY_API_KEYS_ENABLED", "1").strip().lower() not in {"0", "false", "no", "off"}
    return legacy_enabled and key in _VALID_KEYS


async def _record_legacy_key_use(request: Request, key: str) -> None:
    try:
        from ..platform.database import platform_session
        from ..platform.services import audit
        from ..platform.security import hash_secret

        async with platform_session() as session:
            await audit(
                session,
                action="legacy_api_key.used",
                resource_type="api_key",
                ip_address=request.client.host if request.client else None,
                detail={"key_fingerprint": hash_secret(key)[:16], "path": request.url.path},
            )
            await session.commit()
    except Exception:  # noqa: BLE001
        # Compatibility authentication must not fail only because audit storage
        # is unavailable; production monitoring still observes this condition.
        pass


def get_api_key_from_request(request: Request) -> str | None:
    """Extract API key from Authorization header (Bearer token) or X-API-Key header."""
    auth = request.headers.get("Authorization", "")
    if auth.startswith("Bearer "):
        return auth[7:].strip()
    return request.headers.get("X-API-Key", "").strip() or None


# Paths that don't require authentication
_PUBLIC_PATHS = {
    "/health",
    "/ready",
    "/docs",
    "/openapi.json",
    "/redoc",
    "/chat",
    "/chat-moe",
    "/chat-moe/sessions",
    "/chat-moe/completions",
    "/admin",
    "/qa/feedback",
}


def _path_allows_anonymous(path: str) -> bool:
    """Paths that skip API key (browser / BLE ingest to n8n, integration debug)."""
    if path in _PUBLIC_PATHS:
        return True
    if path == "/integration/ingest":
        return True
    if path.startswith("/integration/debug/"):
        return True
    # Platform routes perform JWT/API-key authentication in route dependencies.
    # Keeping them out of the legacy keys.txt gate is required for browser JWTs
    # and for the intentionally public plan/login endpoints.
    if path.startswith("/api/v1/"):
        return True
    return False


class APIKeyAuthMiddleware(BaseHTTPMiddleware):
    """Middleware to enforce API key authentication."""

    async def dispatch(self, request: Request, call_next):
        # Skip auth for public paths and OPTIONS (CORS preflight)
        if _path_allows_anonymous(request.url.path) or request.method == "OPTIONS":
            return await call_next(request)
        
        # Skip auth if disabled via env
        if os.getenv("AGENT_DISABLE_AUTH", "0") == "1":
            return await call_next(request)
        
        key = get_api_key_from_request(request)
        if not key:
            return JSONResponse(
                status_code=401,
                content={"error": {"message": "Missing API key. Use Authorization: Bearer <key> header.", "type": "auth_error"}},
            )
        if not is_valid_key(key):
            # New database-backed API keys share the OpenAI-compatible routes
            # during the legacy-key migration window.
            if key.startswith("pm_live_"):
                from ..platform.dependencies import authenticate_platform_api_key

                principal = await authenticate_platform_api_key(key)
                if principal is not None:
                    required_scope = None
                    if request.url.path == "/v1/models":
                        required_scope = "models:read"
                    elif request.url.path == "/v1/chat/completions":
                        required_scope = "chat:write"
                    else:
                        return JSONResponse(status_code=403, content={"error": {"message": "Database API keys are limited to OpenAI-compatible routes", "type": "auth_error"}})
                    if required_scope not in principal.scopes:
                        return JSONResponse(status_code=403, content={"error": {"message": f"Missing API key scope: {required_scope}", "type": "auth_error"}})
                    request.state.platform_principal = principal
                    request.state.platform_user_id = principal.user_id
                    return await call_next(request)
            return JSONResponse(
                status_code=401,
                content={"error": {"message": "Invalid API key.", "type": "auth_error"}},
            )
        await _record_legacy_key_use(request, key)
        return await call_next(request)
