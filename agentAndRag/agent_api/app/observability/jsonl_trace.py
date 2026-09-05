from __future__ import annotations

import asyncio
import json
import logging
import os
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)
_queue: asyncio.Queue | None = None
_task: asyncio.Task | None = None
_loop: asyncio.AbstractEventLoop | None = None
_dropped = 0


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
    """Enqueue sampled metadata only; never persist questions, answers or credentials."""
    global _queue, _task, _loop, _dropped
    try:
        rate = min(1.0, max(0.0, float(os.getenv("AGENT_TRACE_SAMPLE_RATE", "0.1"))))
    except ValueError:
        rate = 0.1
    try:
        identity = uuid.UUID(trace_id).hex
    except (ValueError, AttributeError):
        identity = new_trace_id()
    if not error and int(identity[:8], 16) / 0x100000000 >= rate:
        return
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        _dropped += 1
        return
    if _queue is None or _loop is not loop or _task is None or _task.done():
        _loop, _queue = loop, asyncio.Queue(maxsize=256)
        _task = asyncio.create_task(_consume(_queue), name="diagnostic-jsonl")
    known = tool if tool in {"v1.chat.completions.stream", "v1.chat.completions.moe"} else "other"
    record = {"ts": time.time(), "trace_id": identity, "tool": known,
              "request": _metadata(request), "response": _metadata(response), "failed": bool(error)}
    try:
        _queue.put_nowait(record)
    except asyncio.QueueFull:
        _dropped += 1
        if _dropped % 100 == 1:
            logger.warning("diagnostic queue full dropped=%s", _dropped)


def _metadata(value: dict) -> dict[str, int]:
    return {
        "query_chars": len(str(value.get("query") or "")),
        "message_count": len(value.get("messages") or []) if isinstance(value.get("messages"), list) else 0,
        "answer_chars": len(str(value.get("answer") or "")),
        "tool_count": len(value.get("tools_called") or []) if isinstance(value.get("tools_called"), list) else 0,
    }


def _append(records: list[dict]) -> None:
    folder = Path(os.getenv("AGENT_TRACE_DIR", str(_default_trace_dir())))
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / "trace.jsonl").open("a", encoding="utf-8") as handle:
        handle.write("".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records))


async def _consume(queue: asyncio.Queue) -> None:
    global _dropped
    while True:
        records = [await queue.get()]
        while len(records) < 32:
            try:
                records.append(queue.get_nowait())
            except asyncio.QueueEmpty:
                break
        try:
            await asyncio.to_thread(_append, records)
        except Exception:
            _dropped += len(records)
            logger.exception("diagnostic trace write failed")
        finally:
            for _ in records:
                queue.task_done()


async def close_trace_writer() -> None:
    global _task, _queue, _loop
    if _queue is not None and _task is not None:
        try:
            await asyncio.wait_for(_queue.join(), timeout=5)
        except TimeoutError:
            logger.warning("diagnostic writer shutdown timed out queued=%s", _queue.qsize())
        _task.cancel()
        await asyncio.gather(_task, return_exceptions=True)
    _task, _queue, _loop = None, None, None


def trace_metrics() -> dict[str, int]:
    return {"dropped": _dropped, "queued": _queue.qsize() if _queue else 0}
