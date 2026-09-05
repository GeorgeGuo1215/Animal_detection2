from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


pytestmark = [pytest.mark.regression, pytest.mark.slow]


def _cases() -> list[dict]:
    path = Path(__file__).resolve().parents[1] / "fixtures" / "retrieval_regression_cases.json"
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("case", _cases(), ids=lambda case: case["id"])
def test_target_book_appears_in_top_five(case: dict) -> None:
    if os.getenv("RUN_RAG_REGRESSION") != "1":
        pytest.skip("set RUN_RAG_REGRESSION=1 to load production models and indexes")

    from agent_api.app.tools.rag_tools import rag_search_tool

    result = rag_search_tool(
        query=case["query"],
        category=case["category"],
        top_k=5,
        rerank=True,
        rerank_candidates=10,
        expand_neighbors=1,
    )
    hits = result["hits"]
    assert hits, case
    assert {hit.get("category") for hit in hits} == {case["category"]}
    actual_books = {str(hit.get("book_id") or "") for hit in hits}
    assert actual_books.intersection(case["expected_books"]), {
        "case": case,
        "actual_books": sorted(actual_books),
    }
