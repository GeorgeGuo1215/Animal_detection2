"""
API Key authentication middleware.

Keys are loaded from keys.txt (one key per line) on startup.
Clients must send header: Authorization: Bearer <api-key>
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Set

from fastapi import HTTPException, Request
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
    return key in _VALID_KEYS


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
            raise HTTPException(
                status_code=401,
                detail={"error": {"message": "Missing API key. Use Authorization: Bearer <key> header.", "type": "auth_error"}},
            )
        if not is_valid_key(key):
            raise HTTPException(
                status_code=401,
                detail={"error": {"message": "Invalid API key.", "type": "auth_error"}},
            )
        return await call_next(request)
