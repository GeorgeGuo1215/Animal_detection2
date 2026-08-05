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
    """Return path to keys.txt (next to main.py's parent dir)."""
    return Path(__file__).resolve().parents[1] / "keys.txt"


def load_api_keys() -> None:
    """Load API keys from keys.txt into memory. Called on startup."""
    global _VALID_KEYS
    path = _keys_file_path()
    if not path.exists():
        # Create default keys.txt with one placeholder key
        default_key = "sk-pethealthai-default-key-2026"
        path.write_text(f"# API keys (one per line, lines starting with # are ignored)\n{default_key}\n", encoding="utf-8")
        print(f"[auth] Created default keys.txt with key: {default_key}")
    
    keys = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            keys.add(line)
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
_PUBLIC_PATHS = {"/health", "/docs", "/openapi.json", "/redoc"}


class APIKeyAuthMiddleware(BaseHTTPMiddleware):
    """Middleware to enforce API key authentication."""

    async def dispatch(self, request: Request, call_next):
        # Skip auth for public paths and OPTIONS (CORS preflight)
        if request.url.path in _PUBLIC_PATHS or request.method == "OPTIONS":
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
