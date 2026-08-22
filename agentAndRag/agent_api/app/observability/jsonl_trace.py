from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional


def new_trace_id() -> str:
    """生成十六进制 trace id。"""
    return uuid.uuid4().hex


def _default_trace_dir() -> Path:
    """默认轨迹目录：仓库根下的 agent_api_logs/。"""
    # 统一落在仓库根下的 agent_api_logs/
    here = Path(__file__).resolve()
    repo_root = here.parents[3]
    return repo_root / "agent_api_logs"


def write_trace(
    trace_id: str,
    *,
    tool: str,
    request: Dict[str, Any],
    response: Dict[str, Any],
    error: Optional[str] = None,
) -> None:
    """将一次工具调用轨迹追加写入 JSONL。"""
    trace_dir = Path(os.getenv("AGENT_TRACE_DIR", str(_default_trace_dir())))
    trace_dir.mkdir(parents=True, exist_ok=True)
    path = trace_dir / "trace.jsonl"
    rec = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "trace_id": trace_id,
        "tool": tool,
        "request": request,
        "response": response,
        "error": error,
    }
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
