from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from RAG.simple_rag.config import RagConfig
from RAG.simple_rag.embeddings import EmbeddingResult
from RAG.simple_rag.pipeline import build_or_update_index, search
from RAG.simple_rag.vector_store import NumpyVectorStore


pytestmark = pytest.mark.integration


class _FakeEmbedder:
    def __init__(self, model_name_or_path: str, device=None) -> None:
        self.model_name_or_path = model_name_or_path

    @staticmethod
    def _vectors(texts: list[str]) -> np.ndarray:
        rows = []
        for text in texts:
            normalized = text.lower()
            vector = np.asarray(
                [
                    normalized.count("urinary") + normalized.count("排尿"),
                    normalized.count("cardiac") + normalized.count("心脏"),
                    normalized.count("nutrition") + normalized.count("营养"),
                ],
                dtype=np.float32,
            )
            if not np.any(vector):
                vector[0] = 0.01
            vector /= np.linalg.norm(vector)
            rows.append(vector)
        return np.vstack(rows)

    def embed_texts(self, texts, **kwargs) -> EmbeddingResult:
        vectors = self._vectors(list(texts))
        return EmbeddingResult(vectors=vectors, dim=vectors.shape[1])

    def embed_queries(self, queries, **kwargs) -> EmbeddingResult:
        vectors = self._vectors(list(queries))
        return EmbeddingResult(vectors=vectors, dim=vectors.shape[1])


def _write_book(path: Path, sentence: str) -> None:
    path.write_text("\n\n".join([sentence] * 12), encoding="utf-8")


def test_mmd_to_index_query_and_idempotent_increment(monkeypatch, tmp_path: Path) -> None:
    from RAG.simple_rag import pipeline

    monkeypatch.setattr(pipeline, "Embedder", _FakeEmbedder)
    raw = tmp_path / "raw"
    raw.mkdir()
    _write_book(raw / "052.mmd", "feline urinary obstruction requires emergency stabilization")
    _write_book(raw / "045.mmd", "canine cardiac disease requires echocardiography")
    cfg = RagConfig(
        raw_dir=raw,
        index_dir=tmp_path / "index",
        embedding_model="fake",
        chunk_words=30,
        chunk_overlap_words=5,
        min_chunk_words=5,
    )

    first = build_or_update_index(cfg, batch_size=4, device="cpu")
    second = build_or_update_index(cfg, batch_size=4, device="cpu")
    assert first["added_chunks"] > 0
    assert second["added_chunks"] == 0
    assert second["skipped_chunks"] == second["total_chunks"]

    hits = search(cfg, "cardiac echocardiography", top_k=2, device="cpu")
    assert Path(hits[0]["source_path"]).name == "045.mmd"


def test_corrupt_store_is_rejected(tmp_path: Path) -> None:
    index = tmp_path / "broken"
    index.mkdir()
    (index / "store_config.json").write_text('{"dim": 3}', encoding="utf-8")
    (index / "meta.jsonl").write_text(json.dumps({"chunk_id": "x"}) + "\n", encoding="utf-8")
    np.save(index / "embeddings.npy", np.zeros((2, 3), dtype=np.float32))
    with pytest.raises(ValueError, match="embeddings/meta"):
        NumpyVectorStore(index).load()
