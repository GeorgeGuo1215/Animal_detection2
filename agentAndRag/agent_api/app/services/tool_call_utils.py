from __future__ import annotations

import json
from typing import Any, Dict


def canonical_tool_call(tool_name: str, arguments: Dict[str, Any]) -> str:
    """返回用于精确去重的稳定工具调用身份串。"""
    try:
        payload = json.dumps(
            arguments or {}, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
    except (TypeError, ValueError):
        payload = repr(arguments)
    return f"{tool_name}:{payload}"


__all__ = ["canonical_tool_call"]
