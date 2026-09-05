from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from RAG.simple_rag.config import RagConfig
from RAG.simple_rag.embeddings import EmbeddingResult
from RAG.simple_rag.vector_store import NumpyVectorStore, StoreConfig


pytestmark = pytest.mark.integration


class _FakeEmbedder:
    model_name_or_path = "fake"

    def embed_queries(self, queries, **kwargs) -> EmbeddingResult:
        count = len(list(queries))
        return EmbeddingResult(
            vectors=np.tile(np.asarray([[1.0, 0.0]], dtype=np.float32), (count, 1)),
            dim=2,
        )


def test_agent_rag_search_uses_explicit_index(monkeypatch, tmp_path: Path) -> None:
    from agent_api.app.tools import rag_tools

    index = tmp_path / "index"
    store = NumpyVectorStore(index)
    store.init_new(StoreConfig(dim=2))
    store.add(
        np.asarray([[1.0, 0.0]], dtype=np.float32),
        [{"chunk_id": "u1", "source_path": "books/052.mmd", "chunk_index": 0, "text": "feline urinary obstruction emergency"}],
    )
    cfg = RagConfig(raw_dir=tmp_path / "raw", index_dir=index, embedding_model="fake")
    monkeypatch.setattr(rag_tools, "default_config", lambda repo_root: cfg)
    monkeypatch.setattr(rag_tools, "resolve_default_category_index_dirs", lambda **kwargs: [])
    monkeypatch.setattr(rag_tools, "resolve_embedding_model_id", lambda model, repo_root: model)
    monkeypatch.setattr(rag_tools, "resolve_rerank_model_id", lambda model, repo_root: model)
    monkeypatch.setattr(rag_tools, "_get_embedder", lambda model, device: _FakeEmbedder())
    rag_tools._invalidate_index_cache(index)

    result = rag_tools.rag_search_tool.__wrapped__(
        query="feline urinary obstruction",
        index_dir=str(index),
        embedding_model="fake",
        rerank=False,
        expand_neighbors=0,
    )
    assert result["hits"][0]["book_id"] is None
    assert result["hits"][0]["source_path"] == "books/052.mmd"
    assert result["params"]["index_dir"] == str(index)


def test_agent_rag_reindex_invalidates_cached_store(monkeypatch, tmp_path: Path) -> None:
    from agent_api.app.tools import rag_tools

    index = tmp_path / "index"
    cfg = RagConfig(raw_dir=tmp_path / "raw", index_dir=index, embedding_model="fake")
    invalidated: list[Path] = []
    monkeypatch.setattr(rag_tools, "default_config", lambda repo_root: cfg)
    monkeypatch.setattr(rag_tools, "resolve_embedding_model_id", lambda model, repo_root: model)
    monkeypatch.setattr(
        rag_tools,
        "build_or_update_index",
        lambda cfg, **kwargs: {"index_dir": str(cfg.index_dir), "added_chunks": 1},
    )
    monkeypatch.setattr(rag_tools, "_invalidate_index_cache", lambda path: invalidated.append(path))
    result = rag_tools.rag_reindex_tool.__wrapped__(index_dir=str(index), embedding_model="fake")
    assert result["added_chunks"] == 1
    assert invalidated == [index]
