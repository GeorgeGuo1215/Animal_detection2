"""
API Key 鉴权中间件。

启动时从 keys.txt（每行一个 key）加载密钥。
客户端需发送：Authorization: Bearer <api-key>
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
    """返回 agent_api/ 下 keys.txt 的路径。"""
    return Path(__file__).resolve().parents[2] / "keys.txt"


def load_api_keys() -> None:
    """从 keys.txt 加载 API key 到内存。启动时调用。"""
    global _VALID_KEYS
    legacy_enabled = os.getenv("AGENT_LEGACY_API_KEYS_ENABLED", "1").strip().lower() not in {
        "0", "false", "no", "off",
    }
    if not legacy_enabled:
        _VALID_KEYS = set()
        print("[auth] Legacy file/environment API keys are disabled.")
        return
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
    """检查给定 key 是否有效。"""
    legacy_enabled = os.getenv("AGENT_LEGACY_API_KEYS_ENABLED", "1").strip().lower() not in {"0", "false", "no", "off"}
    return legacy_enabled and key in _VALID_KEYS


async def _record_legacy_key_use(request: Request, key: str) -> None:
    """记录旧版 API key 使用审计；存储失败不影响鉴权。"""
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
        # 兼容鉴权不得因审计存储不可用而失败；生产监控仍可观察到该情况。
        pass


def get_api_key_from_request(request: Request) -> str | None:
    """从 Authorization Bearer 或 X-API-Key 头提取 API key。"""
    auth = request.headers.get("Authorization", "")
    if auth.startswith("Bearer "):
        return auth[7:].strip()
    return request.headers.get("X-API-Key", "").strip() or None


# 不需要鉴权的路径
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
    """判断路由是否绕过旧版 API-key 门禁。"""
    if path in _PUBLIC_PATHS:
        return True
    # 平台路由在依赖中自行做 JWT/API-key 鉴权。
    # 必须排除在旧版 keys.txt 门禁之外，以便浏览器 JWT 以及公开的套餐/登录端点可用。
    if path.startswith("/api/v1/"):
        return True
    return False


class APIKeyAuthMiddleware(BaseHTTPMiddleware):
    """强制校验 API key 的中间件。"""

    async def dispatch(self, request: Request, call_next):
        """公开路径、OPTIONS 与关闭鉴权时放行，否则校验 Bearer/平台 API key。"""
        # 公开路径与 OPTIONS（CORS 预检）跳过鉴权
        if _path_allows_anonymous(request.url.path) or request.method == "OPTIONS":
            return await call_next(request)
        
        # 环境变量关闭鉴权时跳过
        if os.getenv("AGENT_DISABLE_AUTH", "0") == "1":
            return await call_next(request)
        
        key = get_api_key_from_request(request)
        if not key:
            return JSONResponse(
                status_code=401,
                content={"error": {"message": "Missing API key. Use Authorization: Bearer <key> header.", "type": "auth_error"}},
            )
        if not is_valid_key(key):
            # 迁移期内，数据库 API key 可共用 OpenAI 兼容路由。
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
        import hashlib
        request.state.http_legacy_identity = "legacy:" + hashlib.sha256(key.encode()).hexdigest()
        await _record_legacy_key_use(request, key)
        return await call_next(request)
