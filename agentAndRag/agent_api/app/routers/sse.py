from __future__ import annotations

import time
from typing import Any, Dict, Optional

from ..schemas.openai_schemas import (
    ChatCompletionChunk,
    ChatCompletionChunkChoice,
    ChatCompletionChunkDelta,
)


SSE_DONE = "data: [DONE]\n\n"
SSE_RESPONSE_HEADERS = {
    "Cache-Control": "no-cache",
    "Connection": "keep-alive",
    "X-Accel-Buffering": "no",
}


def openai_sse_chunk(
    *,
    request_id: str,
    model: str,
    created: Optional[int] = None,
    content: str = "",
    status: Optional[str] = None,
    detail: Optional[Dict[str, Any]] = None,
    finish: Optional[str] = None,
) -> str:
    """构造 OpenAI 兼容的 SSE data 行。"""
    chunk = ChatCompletionChunk(
        id=request_id,
        created=int(created if created is not None else time.time()),
        model=model,
        choices=[ChatCompletionChunkChoice(
            delta=ChatCompletionChunkDelta(content=content if content else None),
            finish_reason=finish,
        )],
        agent_status=status,
        agent_detail=detail,
    )
    return f"data: {chunk.model_dump_json()}\n\n"
