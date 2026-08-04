from __future__ import annotations

import asyncio
import logging
import os
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

from ...concurrency import ResourceBusyError
from ..tool_call_utils import canonical_tool_call
from ...tools.rag_query import is_english_rag_query
from ...tools.tool_registry import ToolRegistry

logger = logging.getLogger(__name__)


def _log_rag_drain_failure(task: "asyncio.Task[None]") -> None:
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        logger.error("rag drain task failed", exc_info=exc)


@dataclass(frozen=True)
class ToolRequest:
    expert: str
    tool_name: str
    arguments: Dict[str, Any]
    request_id: str = ""

    def with_request_id(self) -> "ToolRequest":
        if self.request_id:
            return self
        return ToolRequest(
            expert=self.expert,
            tool_name=self.tool_name,
            arguments=dict(self.arguments),
            request_id=uuid.uuid4().hex,
        )


@dataclass(frozen=True)
class ToolResult:
    request_id: str
    expert: str
    tool_name: str
    arguments: Dict[str, Any]
    result: Any
    ok: bool
    latency_ms: float
    error: str = ""
    shared: bool = False


@dataclass
class _QueuedRagCall:
    key: str
    representative: ToolRequest
    future: "asyncio.Future[Tuple[Any, bool, str, float]]"
    timeout_s: Optional[float]
    subscribers: int = 1


class ToolBroker:
    """Request-scoped tool executor shared by independent expert sessions."""

    def __init__(
        self,
        *,
        registry: ToolRegistry,
        allowed_tools: Optional[Iterable[str]],
        rag_call_timeout_s: Optional[float] = None,
    ) -> None:
        self.registry = registry
        self.allowed_tools = None if allowed_tools is None else frozenset(allowed_tools)
        configured_timeout = (
            float(rag_call_timeout_s)
            if rag_call_timeout_s is not None
            else float(os.getenv("MOE_RAG_BROKER_TIMEOUT_SEC", "120"))
        )
        self.rag_call_timeout_s = max(0.001, configured_timeout)
        self._rag_queue: Deque[_QueuedRagCall] = deque()
        self._rag_pending: Dict[str, _QueuedRagCall] = {}
        self._rag_drain_lock = asyncio.Lock()
        self._rag_drain_task: Optional["asyncio.Task[None]"] = None

    def is_allowed(self, tool_name: str) -> bool:
        if self.allowed_tools is not None and tool_name not in self.allowed_tools:
            return False
        return self.registry.get(tool_name) is not None

    async def execute_batch(
        self,
        requests: List[ToolRequest],
        *,
        timeouts: Optional[Dict[str, float]] = None,
    ) -> Dict[str, ToolResult]:
        normalized = [request.with_request_id() for request in requests]
        grouped: Dict[str, List[ToolRequest]] = {}
        immediate: Dict[str, ToolResult] = {}

        for request in normalized:
            if not self.is_allowed(request.tool_name):
                immediate[request.request_id] = ToolResult(
                    request_id=request.request_id,
                    expert=request.expert,
                    tool_name=request.tool_name,
                    arguments=dict(request.arguments),
                    result={"code": "TOOL_NOT_ALLOWED", "tool_name": request.tool_name},
                    ok=False,
                    latency_ms=0.0,
                    error=f"Tool not allowed: {request.tool_name}",
                )
                continue
            if request.tool_name == "rag.search" and not is_english_rag_query(
                request.arguments.get("query")
            ):
                immediate[request.request_id] = ToolResult(
                    request_id=request.request_id,
                    expert=request.expert,
                    tool_name=request.tool_name,
                    arguments=dict(request.arguments),
                    result={
                        "code": "RAG_QUERY_MUST_BE_ENGLISH",
                        "tool_name": request.tool_name,
                        "message": "rag.search arguments.query must be written in English.",
                    },
                    ok=False,
                    latency_ms=0.0,
                    error="rag.search query must be written in English",
                )
                continue
            key = canonical_tool_call(request.tool_name, request.arguments)
            grouped.setdefault(key, []).append(request)

        async def _execute_direct(group: List[ToolRequest]) -> List[ToolResult]:
            representative = group[0]
            started = time.perf_counter()
            group_timeout: Optional[float] = None
            if timeouts is not None:
                values = [max(0.0, float(timeouts.get(request.request_id, 0.0))) for request in group]
                group_timeout = max(values) if values else 0.0

            try:
                if group_timeout is None:
                    result = await self.registry.call(representative.tool_name, representative.arguments)
                else:
                    result = await asyncio.wait_for(
                        self.registry.call(representative.tool_name, representative.arguments),
                        timeout=group_timeout,
                    )
                ok, error = True, ""
            except asyncio.TimeoutError:
                result = {
                    "code": "EXPERT_TOOL_TIMEOUT",
                    "tool_name": representative.tool_name,
                    "timeout_s": group_timeout,
                }
                ok, error = False, f"Tool timed out after {group_timeout:.3f}s"
            except ResourceBusyError as exc:
                result, ok, error = exc.as_dict(), False, str(exc)
            except Exception as exc:  # noqa: BLE001
                result, ok, error = {"error": str(exc)}, False, str(exc)
            latency_ms = round((time.perf_counter() - started) * 1000.0, 1)
            shared = len(group) > 1
            return [
                ToolResult(
                    request_id=request.request_id,
                    expert=request.expert,
                    tool_name=request.tool_name,
                    arguments=dict(request.arguments),
                    result=result,
                    ok=ok,
                    latency_ms=latency_ms,
                    error=error,
                    shared=shared,
                )
                for request in group
            ]

        async def _await_rag(group: List[ToolRequest], key: str) -> List[ToolResult]:
            loop = asyncio.get_running_loop()
            queued = self._rag_pending.get(key)
            if queued is None:
                queued = _QueuedRagCall(
                    key=key,
                    representative=group[0],
                    future=loop.create_future(),
                    timeout_s=self.rag_call_timeout_s,
                    subscribers=len(group),
                )
                self._rag_pending[key] = queued
                self._rag_queue.append(queued)
            else:
                queued.subscribers += len(group)

            self._ensure_rag_drain()
            results: List[ToolResult] = []
            for request in group:
                request_timeout = None if timeouts is None else max(
                    0.0, float(timeouts.get(request.request_id, 0.0))
                )
                try:
                    if request_timeout is None:
                        result, ok, error, latency_ms = await asyncio.shield(queued.future)
                    else:
                        result, ok, error, latency_ms = await asyncio.wait_for(
                            asyncio.shield(queued.future), timeout=request_timeout
                        )
                except asyncio.TimeoutError:
                    result = {
                        "code": "EXPERT_TOOL_TIMEOUT",
                        "tool_name": request.tool_name,
                        "timeout_s": request_timeout,
                    }
                    ok, error, latency_ms = False, f"Tool timed out after {request_timeout:.3f}s", 0.0
                results.append(
                    ToolResult(
                        request_id=request.request_id,
                        expert=request.expert,
                        tool_name=request.tool_name,
                        arguments=dict(request.arguments),
                        result=result,
                        ok=ok,
                        latency_ms=latency_ms,
                        error=error,
                        shared=queued.subscribers > 1,
                    )
                )
            return results

        tasks = []
        for key, group in grouped.items():
            if group[0].tool_name == "rag.search":
                tasks.append(_await_rag(group, key))
            else:
                tasks.append(_execute_direct(group))
        executed = await asyncio.gather(*tasks)
        output = dict(immediate)
        for group_results in executed:
            for result in group_results:
                output[result.request_id] = result
        return output

    def _ensure_rag_drain(self) -> None:
        if self._rag_drain_task is None or self._rag_drain_task.done():
            self._rag_drain_task = asyncio.create_task(self._drain_rag_queue())
            self._rag_drain_task.add_done_callback(_log_rag_drain_failure)

    async def _drain_rag_queue(self) -> None:
        async with self._rag_drain_lock:
            while self._rag_queue:
                queued = self._rag_queue.popleft()
                started = time.perf_counter()
                result: Any = {"error": "rag drain aborted before completion"}
                ok, error = False, "rag drain aborted before completion"
                try:
                    call = self.registry.call(
                        queued.representative.tool_name,
                        queued.representative.arguments,
                    )
                    if queued.timeout_s is None:
                        result = await call
                    else:
                        result = await asyncio.wait_for(call, timeout=queued.timeout_s)
                    ok, error = True, ""
                except asyncio.TimeoutError:
                    result = {
                        "code": "EXPERT_TOOL_TIMEOUT",
                        "tool_name": queued.representative.tool_name,
                        "timeout_s": queued.timeout_s,
                    }
                    ok, error = False, f"Tool timed out after {queued.timeout_s or 0.0:.3f}s"
                except ResourceBusyError as exc:
                    result, ok, error = exc.as_dict(), False, str(exc)
                except Exception as exc:  # noqa: BLE001
                    result, ok, error = {"error": str(exc)}, False, str(exc)
                finally:
                    # Subscribers block on this future; resolving it must survive any
                    # failure above, otherwise every waiting expert hangs to its timeout.
                    latency_ms = round((time.perf_counter() - started) * 1000.0, 1)
                    if not queued.future.done():
                        queued.future.set_result((result, ok, error, latency_ms))
                    self._rag_pending.pop(queued.key, None)


__all__ = ["ToolBroker", "ToolRequest", "ToolResult"]
