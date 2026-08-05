from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.tool_broker import ToolBroker, ToolRequest
from app.tools.tool_registry import ToolRegistry, ToolSpec


def test_different_rag_requests_are_not_deduplicated_and_run_serially():
    calls = []
    active = 0
    max_active = 0
    registry = ToolRegistry()

    async def rag(**kwargs):
        nonlocal active, max_active
        active += 1
        max_active = max(max_active, active)
        calls.append(dict(kwargs))
        await asyncio.sleep(0.01)
        active -= 1
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])
    requests = [
        ToolRequest("clinical", "rag.search", {"query": "a"}, "one"),
        ToolRequest("pharmacy", "rag.search", {"query": "b"}, "two"),
    ]

    results = asyncio.run(broker.execute_batch(requests))

    assert len(calls) == 2
    assert max_active == 1
    assert set(results) == {"one", "two"}
    assert all(not result.shared for result in results.values())


def test_rag_requests_are_serialized_across_concurrent_batches():
    active = 0
    max_active = 0
    registry = ToolRegistry()

    async def rag(**kwargs):
        nonlocal active, max_active
        active += 1
        max_active = max(max_active, active)
        await asyncio.sleep(0.01)
        active -= 1
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    async def run_batches():
        return await asyncio.gather(
            broker.execute_batch([
                ToolRequest("clinical", "rag.search", {"query": "feline obstruction"}, "one")
            ]),
            broker.execute_batch([
                ToolRequest("pharmacy", "rag.search", {"query": "feline analgesia"}, "two")
            ]),
        )

    asyncio.run(run_batches())

    assert max_active == 1


def test_rag_requests_follow_fifo_submission_order_across_batches():
    calls = []
    first_started = asyncio.Event()
    release_first = asyncio.Event()
    registry = ToolRegistry()

    async def rag(**kwargs):
        query = kwargs["query"]
        calls.append(query)
        if query == "first":
            first_started.set()
            await release_first.wait()
        return {"query": query}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    async def run_batches():
        first = asyncio.create_task(broker.execute_batch([
            ToolRequest("clinical", "rag.search", {"query": "first"}, "one")
        ]))
        await first_started.wait()
        second = asyncio.create_task(broker.execute_batch([
            ToolRequest("pharmacy", "rag.search", {"query": "second"}, "two")
        ]))
        await asyncio.sleep(0)
        release_first.set()
        await asyncio.gather(first, second)

    asyncio.run(run_batches())

    assert calls == ["first", "second"]


def test_identical_inflight_rag_requests_share_one_execution_across_batches():
    calls = []
    started = asyncio.Event()
    release = asyncio.Event()
    registry = ToolRegistry()

    async def rag(**kwargs):
        calls.append(dict(kwargs))
        started.set()
        await release.wait()
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    async def run_batches():
        first = asyncio.create_task(broker.execute_batch([
            ToolRequest("clinical", "rag.search", {"query": "same evidence"}, "one")
        ]))
        await started.wait()
        second = asyncio.create_task(broker.execute_batch([
            ToolRequest("pharmacy", "rag.search", {"query": "same evidence"}, "two")
        ]))
        await asyncio.sleep(0)
        release.set()
        first_result, second_result = await asyncio.gather(first, second)
        return first_result["one"], second_result["two"]

    first_result, second_result = asyncio.run(run_batches())

    assert len(calls) == 1
    assert first_result.shared is True
    assert second_result.shared is True


def test_one_expert_timeout_does_not_cancel_shared_rag_for_another_expert():
    calls = 0
    registry = ToolRegistry()

    async def rag(**kwargs):
        nonlocal calls
        calls += 1
        await asyncio.sleep(0.03)
        return {"hits": [{"text": "evidence"}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(
        registry=registry, allowed_tools=["rag.search"], rag_call_timeout_s=0.1,
    )

    async def run_batches():
        first, second = await asyncio.gather(
            broker.execute_batch(
                [ToolRequest("clinical", "rag.search", {"query": "same"}, "one")],
                timeouts={"one": 0.01},
            ),
            broker.execute_batch(
                [ToolRequest("pharmacy", "rag.search", {"query": "same"}, "two")],
                timeouts={"two": 0.1},
            ),
        )
        return first["one"], second["two"]

    first_result, second_result = asyncio.run(run_batches())

    assert calls == 1
    assert first_result.ok is False
    assert first_result.result["code"] == "EXPERT_TOOL_TIMEOUT"
    assert second_result.ok is True
    assert second_result.result["hits"]


def test_failed_rag_call_clears_pending_slot_for_later_identical_request():
    attempts = 0
    registry = ToolRegistry()

    async def rag(**kwargs):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("index unavailable")
        return {"hits": [{"text": "evidence"}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    async def run_batches():
        failed = await broker.execute_batch([
            ToolRequest("clinical", "rag.search", {"query": "same evidence"}, "one")
        ])
        retried = await broker.execute_batch([
            ToolRequest("pharmacy", "rag.search", {"query": "same evidence"}, "two")
        ])
        return failed["one"], retried["two"]

    failed_result, retried_result = asyncio.run(run_batches())

    assert failed_result.ok is False
    assert "index unavailable" in failed_result.error
    assert retried_result.ok is True
    assert retried_result.result["hits"]
    assert attempts == 2


def test_drain_failure_still_resolves_remaining_queued_rag_calls():
    registry = ToolRegistry()
    started = asyncio.Event()
    release = asyncio.Event()

    async def rag(**kwargs):
        if kwargs["query"] == "broken":
            started.set()
            await release.wait()
            raise RuntimeError("boom")
        return {"hits": [{"text": kwargs["query"]}]}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    async def run_batches():
        first = asyncio.create_task(broker.execute_batch([
            ToolRequest("clinical", "rag.search", {"query": "broken"}, "one")
        ]))
        await started.wait()
        second = asyncio.create_task(broker.execute_batch([
            ToolRequest("pharmacy", "rag.search", {"query": "healthy"}, "two")
        ]))
        await asyncio.sleep(0)
        release.set()
        failed, followup = await asyncio.gather(first, second)
        return failed["one"], followup["two"]

    failed_result, followup_result = asyncio.run(run_batches())

    assert failed_result.ok is False
    assert followup_result.ok is True
    assert followup_result.result["hits"][0]["text"] == "healthy"


def test_non_english_rag_query_is_rejected_without_execution():
    calls = []
    registry = ToolRegistry()

    async def rag(**kwargs):
        calls.append(dict(kwargs))
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=["rag.search"])

    results = asyncio.run(broker.execute_batch([
        ToolRequest("clinical", "rag.search", {"query": "猫尿道梗阻指南"}, "one")
    ]))

    assert calls == []
    assert results["one"].ok is False
    assert results["one"].result["code"] == "RAG_QUERY_MUST_BE_ENGLISH"


def test_direct_rag_tool_rejects_non_english_query_before_loading_index():
    from app.tools.rag_tools import rag_search_tool

    try:
        rag_search_tool(query="猫尿道梗阻指南")
    except ValueError as exc:
        assert "must be written in English" in str(exc)
    else:
        raise AssertionError("Expected a non-English RAG query to be rejected")


def test_disallowed_tool_is_rejected_without_execution():
    calls = []
    registry = ToolRegistry()

    async def rag(**kwargs):
        calls.append(dict(kwargs))
        return {"hits": []}

    registry.register(ToolSpec("rag.search", "rag", {"type": "object"}, rag))
    broker = ToolBroker(registry=registry, allowed_tools=[])

    results = asyncio.run(broker.execute_batch([
        ToolRequest("clinical", "rag.search", {"query": "a"}, "one")
    ]))

    assert calls == []
    assert results["one"].ok is False
    assert results["one"].result["code"] == "TOOL_NOT_ALLOWED"
