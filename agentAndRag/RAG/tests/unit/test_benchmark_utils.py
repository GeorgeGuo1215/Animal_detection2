from __future__ import annotations

import json

import pytest

from RAG.maintenance.benchmarks.run_capacity_benchmark import _percentile, _summary, _write_json_atomic


pytestmark = pytest.mark.unit


def test_percentile_and_summary() -> None:
    assert _percentile([], 0.95) == 0.0
    assert _percentile([1.0, 2.0, 3.0], 0.5) == 2.0
    summary = _summary([10.0, 20.0, 30.0, 40.0])
    assert summary["count"] == 4
    assert summary["mean_ms"] == 25.0
    assert summary["max_ms"] == 40.0


def test_atomic_json_writer_replaces_complete_document(tmp_path) -> None:
    target = tmp_path / "result.json"
    _write_json_atomic(target, {"status": "partial"})
    _write_json_atomic(target, {"status": "complete", "rows": [1, 2]})
    assert json.loads(target.read_text(encoding="utf-8")) == {
        "status": "complete",
        "rows": [1, 2],
    }
    assert not target.with_suffix(".json.tmp").exists()
