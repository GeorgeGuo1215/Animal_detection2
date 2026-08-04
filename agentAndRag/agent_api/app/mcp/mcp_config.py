from __future__ import annotations

import json
import os
import shutil
import sys
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
    # agent_api/app/mcp/mcp_config.py -> agentAndRag repo root
    return Path(__file__).resolve().parents[3]


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


def _resolve_stdio_command(command: Optional[str]) -> str:
    """
    Pick a Python executable for MCP stdio servers.

    - If `command` is missing, empty, or points to a non-existent file (e.g. Linux path in JSON on Windows),
      fall back to ``sys.executable`` (current uvicorn / conda env).
    - If `command` is ``python`` / ``python3``, resolve via PATH; otherwise keep a valid file path.
    """
    if not command or not str(command).strip():
        return sys.executable
    raw = str(command).strip().strip('"').strip("'")
    if os.path.isfile(raw):
        return raw
    # Broken abs path from another OS / machine
    if raw.startswith("/") or (len(raw) > 2 and raw[1] == ":" and not os.path.isfile(raw)):
        return sys.executable
    w = shutil.which(raw.split()[0]) if raw else None
    if w:
        return w
    return sys.executable


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

        cmd = _resolve_stdio_command(raw.get("command"))
        cfg = McpServerConfig(
            name=name,
            transport=transport,
            command=cmd,
            args=list(raw.get("args") or []),
            env=dict(raw.get("env") or {}),
            url=raw.get("url"),
            enabled=bool(enabled),
            cwd=raw_cwd,
        )
        servers.append(cfg)
    return servers
