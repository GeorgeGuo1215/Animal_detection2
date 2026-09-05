from __future__ import annotations

import numpy as np
import pytest

from RAG.simple_rag.context_utils import build_neighbor_contexts, build_source_index
from RAG.simple_rag.query_rewrite import NoRewrite
from RAG.simple_rag.retrieval import BM25Retriever, MultiRouteRetriever, RetrievedChunk


pytestmark = pytest.mark.unit


class _StaticRetriever:
    def __init__(self, hits: list[RetrievedChunk]) -> None:
        self.hits = hits

    def retrieve(self, query: str, *, top_k: int) -> list[RetrievedChunk]:
        return self.hits[:top_k]


def test_vector_store_sort_and_dimension_validation(tiny_store) -> None:
    hits = tiny_store.search(np.asarray([0.9, 0.1, 0.0], dtype=np.float32), top_k=2)
    assert [meta["chunk_id"] for meta, _ in hits] == ["a", "b"]
    with pytest.raises(ValueError):
        tiny_store.search(np.ones(2, dtype=np.float32), top_k=1)


def test_bm25_chinese_english_and_empty() -> None:
    metas = [
        {"chunk_id": "u", "text": "猫 urinary obstruction 排尿困难"},
        {"chunk_id": "c", "text": "canine cardiac auscultation"},
    ]
    retriever = BM25Retriever(metas=metas)
    assert retriever.retrieve("urinary obstruction", top_k=1)[0].chunk_id == "u"
    assert retriever.retrieve("", top_k=3) == []


def test_multiroute_rrf_deduplicates_chunks() -> None:
    a = RetrievedChunk("a", 0.9, {"chunk_id": "a"})
    b = RetrievedChunk("b", 0.8, {"chunk_id": "b"})
    retriever = MultiRouteRetriever(
        retrievers=[("dense", _StaticRetriever([a, b])), ("bm25", _StaticRetriever([a]))],
        rewriter=NoRewrite(),
        top_k_per_route=2,
    )
    out = retriever.retrieve("query", top_k=2)
    assert [item.chunk_id for item in out] == ["a", "b"]


def test_neighbor_context_merges_windows_without_crossing_sources() -> None:
    metas = [
        {"source_path": "a.mmd", "chunk_index": i, "text": f"a-{i}"} for i in range(4)
    ] + [{"source_path": "b.mmd", "chunk_index": 0, "text": "b-0"}]
    source_index = build_source_index(metas)
    contexts = build_neighbor_contexts(
        metas=metas,
        hits=[
            {"source_path": "a.mmd", "chunk_index": 1, "score": 0.9},
            {"source_path": "a.mmd", "chunk_index": 2, "score": 0.8},
        ],
        neighbor_n=1,
        _source_index=source_index,
    )
    assert len(contexts) == 1
    assert contexts[0]["n_chunks"] == 4
    assert "b-0" not in contexts[0]["text"]
