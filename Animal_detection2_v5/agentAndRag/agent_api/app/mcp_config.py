from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class McpServerConfig:
    name: str
    transport: str = "stdio"
    command: Optional[str] = None
    args: List[str] = field(default_factory=list)
    env: Dict[str, str] = field(default_factory=dict)
    url: Optional[str] = None
    enabled: bool = True
    cwd: Optional[str] = None


def _repo_root() -> Path:
    # agent_api/app/mcp_config.py -> repo root is parents[2]
    return Path(__file__).resolve().parents[2]


def _default_config_path() -> Path:
    return _repo_root() / "agent_api" / "mcp_servers.json"


def _load_json_text() -> Optional[str]:
    inline = os.getenv("MCP_SERVER_JSON")
    if inline and inline.strip():
        return inline

    path = os.getenv("MCP_SERVER_CONFIG")
    config_path = Path(path) if path and path.strip() else _default_config_path()
    if config_path.exists():
        return config_path.read_text(encoding="utf-8")
    return None


def _coerce_servers(obj: Any) -> List[Dict[str, Any]]:
    if isinstance(obj, dict) and "servers" in obj:
        obj = obj["servers"]
    if isinstance(obj, list):
        return [o for o in obj if isinstance(o, dict)]
    return []


def load_mcp_servers() -> List[McpServerConfig]:
    text = _load_json_text()
    if not text:
        return []
    try:
        obj = json.loads(text)
    except Exception:
        return []

    servers: List[McpServerConfig] = []
    for raw in _coerce_servers(obj):
        name = str(raw.get("name") or "").strip()
        if not name:
            continue
        transport = str(raw.get("transport") or "stdio").strip().lower()
        enabled = raw.get("enabled")
        if enabled is None:
            enabled = not bool(raw.get("disabled"))
        if not enabled:
            continue

        raw_cwd = raw.get("cwd")
        if not raw_cwd:
            raw_cwd = str(_repo_root())

        cfg = McpServerConfig(
            name=name,
            transport=transport,
            command=raw.get("command"),
            args=list(raw.get("args") or []),
            env=dict(raw.get("env") or {}),
            url=raw.get("url"),
            enabled=bool(enabled),
            cwd=raw_cwd,
        )
        servers.append(cfg)
    return servers
