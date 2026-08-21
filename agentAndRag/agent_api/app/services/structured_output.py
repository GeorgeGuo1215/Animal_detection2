"""解析结构化 LLM 输出的小型辅助函数。"""
from __future__ import annotations

import json
from typing import Any, Dict, Optional, Tuple


def safe_json_loads(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    """尽力从模型输出中提取第一个 JSON 对象。"""
    value = (text or "").strip()
    if not value:
        return None, "empty response"
    try:
        parsed = json.loads(value)
        return (parsed, "") if isinstance(parsed, dict) else (None, "JSON root is not an object")
    except Exception:
        pass

    left = value.find("{")
    right = value.rfind("}")
    if left >= 0 and right > left:
        try:
            parsed = json.loads(value[left : right + 1])
            return (parsed, "") if isinstance(parsed, dict) else (None, "JSON root is not an object")
        except Exception as exc:  # noqa: BLE001
            return None, f"json parse failed: {exc}"
    return None, "json object not found"
