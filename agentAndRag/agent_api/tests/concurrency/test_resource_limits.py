from __future__ import annotations

import asyncio
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

_AGENT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_AGENT_ROOT))

from app.concurrency import AsyncResourceLimiter, ResourceBusyError, SyncResourceLimiter  # noqa: E402
from app.llm.llm_client_stream import AsyncOpenAIStreamClient  # noqa: E402


def test_async_limiter_caps_concurrency() -> None:
    async def _run() -> tuple[int, dict]:
        limiter = AsyncResourceLimiter("test", 2)
        active = 0
        max_active = 0

        async def _worker() -> None:
            nonlocal active, max_active
            async with limiter.slot(timeout_s=1):
                active += 1
                max_active = max(max_active, active)
                await asyncio.sleep(0.02)
                active -= 1

        await asyncio.gather(*(_worker() for _ in range(6)))
        return max_active, limiter.snapshot()

    max_active, snapshot = asyncio.run(_run())
    assert max_active == 2
    assert snapshot["active"] == 0
    assert snapshot["waiting"] == 0
    assert snapshot["acquired"] == 6


def test_async_limiter_timeout_and_waiter_cancellation_release_capacity() -> None:
    async def _run() -> None:
        limiter = AsyncResourceLimiter("llm", 1)
        release = asyncio.Event()
        entered = asyncio.Event()

        async def _holder() -> None:
            async with limiter.slot(timeout_s=1):
                entered.set()
                await release.wait()

        holder = asyncio.create_task(_holder())
        await entered.wait()

        with pytest.raises(ResourceBusyError) as timeout:
            async with limiter.slot(timeout_s=0.01):
                pass
        assert timeout.value.resource == "llm"

        waiter = asyncio.create_task(_wait_for_slot(limiter))
        await asyncio.sleep(0)
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        assert limiter.snapshot()["waiting"] == 0

        release.set()
        await holder
        async with limiter.slot(timeout_s=0.1):
            assert limiter.snapshot()["active"] == 1
        assert limiter.snapshot()["active"] == 0

    asyncio.run(_run())


async def _wait_for_slot(limiter: AsyncResourceLimiter) -> None:
    async with limiter.slot(timeout_s=1):
        pass


def test_sync_limiter_caps_thread_concurrency() -> None:
    limiter = SyncResourceLimiter("rag", 2)
    state_lock = threading.Lock()
    active = 0
    max_active = 0

    def _worker() -> None:
        nonlocal active, max_active
        with limiter.slot(timeout_s=1):
            with state_lock:
                active += 1
                max_active = max(max_active, active)
            time.sleep(0.02)
            with state_lock:
                active -= 1

    with ThreadPoolExecutor(max_workers=6) as pool:
        list(pool.map(lambda _: _worker(), range(6)))

    snapshot = limiter.snapshot()
    assert max_active == 2
    assert snapshot["active"] == 0
    assert snapshot["waiting"] == 0
    assert snapshot["acquired"] == 6


def test_sync_limiter_times_out_without_leaking_waiter() -> None:
    limiter = SyncResourceLimiter("rag", 1)
    with limiter.slot(timeout_s=1):
        with pytest.raises(ResourceBusyError):
            with limiter.slot(timeout_s=0.01):
                pass
    snapshot = limiter.snapshot()
    assert snapshot["active"] == 0
    assert snapshot["waiting"] == 0
    assert snapshot["rejected"] == 1


def test_async_and_sync_callers_share_one_counter() -> None:
    async def _run() -> None:
        limiter = AsyncResourceLimiter("llm", 1)
        async with limiter.slot(timeout_s=1):
            def _sync_contender() -> None:
                with pytest.raises(ResourceBusyError):
                    with limiter.sync_slot(timeout_s=0.01):
                        pass

            await asyncio.to_thread(_sync_contender)
            assert limiter.snapshot()["active"] == 1
        assert limiter.snapshot()["active"] == 0

    asyncio.run(_run())


def test_stream_close_releases_llm_slot(monkeypatch: pytest.MonkeyPatch) -> None:
    import app.llm.llm_client_stream as stream_module

    class _FakeResponse:
        def raise_for_status(self) -> None:
            return None

        async def aiter_lines(self):
            yield 'data: {"choices":[{"delta":{"content":"first"}}]}'

    class _FakeStreamContext:
        async def __aenter__(self):
            return _FakeResponse()

        async def __aexit__(self, exc_type, exc, tb):
            return False

    class _FakeClient:
        def stream(self, *args, **kwargs):
            return _FakeStreamContext()

    class _Limits:
        acquire_timeout_s = 1.0
        llm = AsyncResourceLimiter("llm", 1)

    limits = _Limits()
    monkeypatch.setattr(stream_module, "get_resource_limits", lambda: limits)

    async def _run() -> None:
        client = AsyncOpenAIStreamClient.__new__(AsyncOpenAIStreamClient)
        client.base_url = "https://example.test"
        client.api_key = "test"
        client.model = "test-model"
        client._client = _FakeClient()  # type: ignore[assignment]
        stream = client.chat_stream(messages=[{"role": "user", "content": "hi"}])
        assert await stream.__anext__() == "first"
        assert limits.llm.snapshot()["active"] == 1
        await asyncio.wait_for(stream.aclose(), timeout=1)
        assert limits.llm.snapshot()["active"] == 0

    asyncio.run(_run())
