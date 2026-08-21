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
    """单个 MCP 服务器的传输、命令、环境与开关配置。"""

    name: str
    transport: str = "stdio"
    command: Optional[str] = None
    args: List[str] = field(default_factory=list)
    env: Dict[str, str] = field(default_factory=dict)
    url: Optional[str] = None
    enabled: bool = True
    cwd: Optional[str] = None


def _repo_root() -> Path:
    """定位 agentAndRag 仓库根目录。"""
    # agent_api/app/mcp/mcp_config.py -> agentAndRag 仓库根
    return Path(__file__).resolve().parents[3]


def _default_config_path() -> Path:
    """默认 MCP 服务器配置文件路径。"""
    return _repo_root() / "agent_api" / "mcp_servers.json"


def _load_json_text() -> Optional[str]:
    """从 MCP_SERVER_JSON 或配置文件读取 JSON 文本。"""
    inline = os.getenv("MCP_SERVER_JSON")
    if inline and inline.strip():
        return inline

    path = os.getenv("MCP_SERVER_CONFIG")
    config_path = Path(path) if path and path.strip() else _default_config_path()
    if config_path.exists():
        return config_path.read_text(encoding="utf-8")
    return None


def _coerce_servers(obj: Any) -> List[Dict[str, Any]]:
    """将 JSON 对象规范为服务器配置字典列表。"""
    if isinstance(obj, dict) and "servers" in obj:
        obj = obj["servers"]
    if isinstance(obj, list):
        return [o for o in obj if isinstance(o, dict)]
    return []


def _resolve_stdio_command(command: Optional[str]) -> str:
    """
    为 MCP stdio 服务器选定 Python 可执行文件。

    - `command` 缺失、为空或指向不存在的文件（例如 JSON 里是 Linux 路径而当前是 Windows）时，回退到 ``sys.executable``（当前 uvicorn / conda 环境）。
    - `command` 为 ``python`` / ``python3`` 时走 PATH 解析；否则保留有效文件路径。
    """
    if not command or not str(command).strip():
        return sys.executable
    raw = str(command).strip().strip('"').strip("'")
    if os.path.isfile(raw):
        return raw
    # 来自其他操作系统/机器的无效绝对路径
    if raw.startswith("/") or (len(raw) > 2 and raw[1] == ":" and not os.path.isfile(raw)):
        return sys.executable
    w = shutil.which(raw.split()[0]) if raw else None
    if w:
        return w
    return sys.executable


def load_mcp_servers() -> List[McpServerConfig]:
    """加载已启用的 MCP 服务器配置列表；解析失败则返回空列表。"""
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
