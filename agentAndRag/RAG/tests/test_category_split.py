"""Validate category taxonomy + sliced indexes under rag_index_e5_by_cat."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from RAG.simple_rag.vector_store import NumpyVectorStore

RAG_ROOT = Path(__file__).resolve().parents[1]
TAXONOMY = RAG_ROOT / "data" / "category_taxonomy.json"
CAT_ROOT = RAG_ROOT / "data" / "rag_index_e5_by_cat"


@pytest.fixture(scope="module")
def taxonomy() -> dict:
    if not TAXONOMY.exists():
        pytest.skip(f"taxonomy missing: {TAXONOMY} (run split_index_by_category.py first)")
    return json.loads(TAXONOMY.read_text(encoding="utf-8"))


def test_taxonomy_has_categories_and_no_unmatched(taxonomy: dict):
    cats = taxonomy.get("categories") or []
    assert len(cats) >= 40
    assert int(taxonomy.get("matched_books") or 0) >= 60
    assert not taxonomy.get("unmatched_books"), taxonomy.get("unmatched_books")


def test_every_category_has_store_files(taxonomy: dict):
    for c in taxonomy["categories"]:
        d = Path(c["index_dir"])
        assert d.exists(), c["id"]
        assert (d / "store_config.json").exists()
        assert (d / "meta.jsonl").exists()
        assert (d / "embeddings.npy").exists()
        emb = np.load(d / "embeddings.npy")
        assert emb.ndim == 2
        assert emb.shape[0] == int(c["chunk_count"])
        assert emb.shape[1] == int(taxonomy.get("dim") or 384)


def test_empty_placeholder_loads(taxonomy: dict):
    empty = [c for c in taxonomy["categories"] if int(c.get("chunk_count") or 0) == 0]
    assert empty, "expected at least one empty placeholder category"
    for c in empty[:3]:
        st = NumpyVectorStore(Path(c["index_dir"]))
        st.load()
        assert st.size == 0
        assert st.search(np.zeros(st.config.dim, dtype=np.float32), top_k=5) == []


def test_book_in_multiple_categories_duplicated(taxonomy: dict):
    """Atlas of Normal Radiographic Anatomy (027) is in anatomy + imaging."""
    anatomy = next(c for c in taxonomy["categories"] if c["id"] == "basic.anatomy")
    imaging = next(c for c in taxonomy["categories"] if c["id"] == "diagnostics.imaging")
    a_paths = set(anatomy.get("source_paths") or [])
    i_paths = set(imaging.get("source_paths") or [])
    overlap = a_paths & i_paths
    assert overlap, "expected shared book paths between anatomy and imaging"
    assert int(anatomy["chunk_count"]) > 0
    assert int(imaging["chunk_count"]) > 0


def test_pharmacy_and_behavior_nonempty(taxonomy: dict):
    by_id = {c["id"]: c for c in taxonomy["categories"]}
    assert by_id["pharmacy.papich"]["chunk_count"] > 0
    assert by_id["pharmacy.applied_pharmacology"]["chunk_count"] > 0
    assert by_id["behavior.dog_cat_problems"]["chunk_count"] > 0
    assert by_id["behavior.feline_welfare"]["chunk_count"] > 0
    assert by_id["nutrition.placeholder"]["chunk_count"] == 0
