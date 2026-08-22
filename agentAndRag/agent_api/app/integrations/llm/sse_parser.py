"""OpenAI 兼容 SSE 行解析；网络传输与协议解析分离。"""

from __future__ import annotations

import json
from typing import Dict, Optional


def parse_chat_sse_line(line: str) -> Optional[Dict[str, Optional[str]]]:
    """解析单行 data 事件；心跳、损坏 JSON 与 [DONE] 返回 ``None``。"""
    if not line.startswith("data: "):
        return None
    data = line[6:]
    if data == "[DONE]":
        return None
    try:
        chunk = json.loads(data)
    except json.JSONDecodeError:
        return None
    choice = (chunk.get("choices") or [{}])[0]
    delta = choice.get("delta") or {}
    content = delta.get("content")
    finish_reason = choice.get("finish_reason")
    if not content and not finish_reason:
        return None
    return {
        "content": str(content) if content else None,
        "finish_reason": str(finish_reason) if finish_reason else None,
    }
