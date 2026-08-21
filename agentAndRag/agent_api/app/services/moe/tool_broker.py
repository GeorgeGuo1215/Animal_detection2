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
    """记录 RAG 队列排空任务失败。"""
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        logger.error("rag drain task failed", exc_info=exc)


@dataclass(frozen=True)
class ToolRequest:
    """一次工具调用请求。"""
    expert: str
    tool_name: str
    arguments: Dict[str, Any]
    request_id: str = ""

    def with_request_id(self) -> "ToolRequest":
        """若无 request_id 则补生成。"""
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
    """一次工具调用结果。"""
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
    """排队中的 RAG 调用（可多专家共享）。"""
    key: str
    representative: ToolRequest
    future: "asyncio.Future[Tuple[Any, bool, str, float]]"
    timeout_s: Optional[float]
    subscribers: int = 1


class ToolBroker:
    """请求级工具执行器，供独立专家会话共享。"""

    def __init__(
        self,
        *,
        registry: ToolRegistry,
        allowed_tools: Optional[Iterable[str]],
        rag_call_timeout_s: Optional[float] = None,
    ) -> None:
        """绑定注册表、允许工具与 RAG 超时。"""
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
        """判断工具是否允许且已注册。"""
        if self.allowed_tools is not None and tool_name not in self.allowed_tools:
            return False
        return self.registry.get(tool_name) is not None

    async def execute_batch(
        self,
        requests: List[ToolRequest],
        *,
        timeouts: Optional[Dict[str, float]] = None,
    ) -> Dict[str, ToolResult]:
        """批量执行工具请求，并按“工具名 + 规范化参数”共享完全相同的调用。

        非 RAG 工具可并发执行；RAG 调用进入请求级串行队列以保护模型资源。每个原始
        request_id 都会得到独立 ToolResult，并保留未允许、非英文查询、超时和繁忙错误。
        """
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
            """调用一组完全相同的非 RAG 请求一次，并复制结果给各 request_id。

            组超时取订阅者预算最大值；超时、资源繁忙和普通异常均转成 ToolResult，
            不让单个工具异常取消同批其他任务。
            """
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
            """订阅或创建指定去重键的 RAG Future，并按请求超时独立等待结果。"""
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
        """确保 RAG 排空任务在运行。"""
        if self._rag_drain_task is None or self._rag_drain_task.done():
            self._rag_drain_task = asyncio.create_task(self._drain_rag_queue())
            self._rag_drain_task.add_done_callback(_log_rag_drain_failure)

    async def _drain_rag_queue(self) -> None:
        """在单一排空锁内串行调用 RAG，并完成所有共享订阅者的 Future。

        无论成功、超时、资源繁忙或异常，都会移除 pending 键并向等待者交付结构化结果，
        防止队列残留导致后续同查询永久等待。
        """
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
