"""Category-scoped RAG search precision vs full index.

Metrics asserted:
- pharmacy query: top-5 in-category source rate under pharmacy.* > full-index rate
- empty category returns [] without error
- category hits only come from allowed source_paths

Run (conda RAG):
  pytest agent_api/tests/rag/test_category_search.py -q
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

_AGENT_ROOT = Path(__file__).resolve().parents[2]
_REPO_ROOT = _AGENT_ROOT.parent  # agentAndRag
sys.path.insert(0, str(_AGENT_ROOT))
sys.path.insert(0, str(_REPO_ROOT))

from app.tools.rag_tools import rag_search_tool  # noqa: E402
from RAG.simple_rag.category_index import resolve_category_index_dirs  # noqa: E402

TAXONOMY = _REPO_ROOT / "RAG" / "data" / "category_taxonomy.json"


@pytest.fixture(scope="module")
def taxonomy() -> dict:
    if not TAXONOMY.exists():
        pytest.skip("category_taxonomy.json missing; run split_index_by_category first")
    return json.loads(TAXONOMY.read_text(encoding="utf-8"))


def _source_paths_for_categories(taxonomy: dict, patterns: list[str]) -> set[str]:
    dirs = resolve_category_index_dirs(repo_root=_REPO_ROOT, category=patterns)
    ids = {d.name for d in dirs}
    paths: set[str] = set()
    for c in taxonomy["categories"]:
        if c["id"] in ids:
            paths.update(c.get("source_paths") or [])
    return paths


def _in_category_rate(hits: list, allowed_paths: set[str]) -> float:
    if not hits:
        return 0.0
    n = 0
    for h in hits:
        sp = h.get("source_path") or ""
        if sp in allowed_paths:
            n += 1
    return n / float(len(hits))


def test_pharmacy_category_improves_top5_precision(taxonomy: dict):
    """Drug-handbook query: pharmacy.* top5 precision should beat full index."""
    query = "Papich veterinary drug dosage contraindication toxicity dog cat"
    pharmacy_paths = _source_paths_for_categories(taxonomy, ["pharmacy.*"])
    assert pharmacy_paths, "pharmacy categories must have books"

    device = os.getenv("AGENT_WARMUP_DEVICE") or None
    full = rag_search_tool(
        query=query,
        top_k=5,
        multi_route=False,
        rerank=False,
        expand_neighbors=0,
        device=device,
    )
    scoped = rag_search_tool(
        query=query,
        top_k=5,
        category=["pharmacy.*"],
        multi_route=False,
        rerank=False,
        expand_neighbors=0,
        device=device,
    )
    full_hits = full.get("hits") or []
    scoped_hits = scoped.get("hits") or []
    assert scoped_hits, "pharmacy category search returned no hits"

    full_rate = _in_category_rate(full_hits, pharmacy_paths)
    scoped_rate = _in_category_rate(scoped_hits, pharmacy_paths)

    # Scoped must be perfect (all hits from pharmacy books)
    assert scoped_rate == 1.0, f"scoped in-category rate={scoped_rate}, hits={scoped_hits}"
    # And strictly better than (or equal only if full already perfect) full index
    assert scoped_rate > full_rate or full_rate == 1.0, (
        f"expected category precision lift: scoped={scoped_rate} full={full_rate}"
    )
    # Persist metric for docs
    metrics = {
        "query": query,
        "top_k": 5,
        "full_in_category_rate": full_rate,
        "scoped_in_category_rate": scoped_rate,
        "lift": scoped_rate - full_rate,
        "full_sources": [h.get("source_path") for h in full_hits],
        "scoped_sources": [h.get("source_path") for h in scoped_hits],
    }
    out = _AGENT_ROOT / "tests" / "rag" / "_last_precision_metrics.json"
    out.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    print("PRECISION_METRICS", json.dumps(metrics, ensure_ascii=False))


def test_empty_category_returns_no_hits(taxonomy: dict):
    device = os.getenv("AGENT_WARMUP_DEVICE") or None
    out = rag_search_tool(
        query="dog calorie requirement AAFCO",
        top_k=5,
        category="nutrition.placeholder",
        multi_route=False,
        rerank=False,
        expand_neighbors=0,
        device=device,
    )
    assert out.get("hits") == []
    assert out.get("params", {}).get("category") == "nutrition.placeholder"


def test_behavior_hits_restricted(taxonomy: dict):
    device = os.getenv("AGENT_WARMUP_DEVICE") or None
    allowed = _source_paths_for_categories(taxonomy, ["behavior.*"])
    out = rag_search_tool(
        query="dog separation anxiety behavior training",
        top_k=5,
        category="behavior.*",
        multi_route=False,
        rerank=False,
        expand_neighbors=0,
        device=device,
    )
    hits = out.get("hits") or []
    assert hits
    for h in hits:
        assert h.get("source_path") in allowed


def test_resolve_wildcard_clinical(taxonomy: dict):
    dirs = resolve_category_index_dirs(repo_root=_REPO_ROOT, category="clinical.*")
    names = {d.name for d in dirs}
    assert "clinical.internal_medicine" in names
    assert "clinical.surgery" in names
    assert "pharmacy.papich" not in names
