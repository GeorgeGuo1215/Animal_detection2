"""Small helpers for parsing structured LLM output."""
from __future__ import annotations

import json
from typing import Any, Dict, Optional, Tuple


def safe_json_loads(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    """Best-effort extraction of the first JSON object from model output."""
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
