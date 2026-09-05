from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

from RAG.simple_rag.vector_store import NumpyVectorStore, StoreConfig


AGENT_ROOT = Path(__file__).resolve().parents[2]
if str(AGENT_ROOT) not in sys.path:
    sys.path.insert(0, str(AGENT_ROOT))


@pytest.fixture
def tiny_store(tmp_path: Path) -> NumpyVectorStore:
    store = NumpyVectorStore(tmp_path / "index")
    store.init_new(StoreConfig(dim=3))
    vectors = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    metas = [
        {"chunk_id": "a", "source_path": "books/a.mmd", "chunk_index": 0, "text": "feline urinary obstruction"},
        {"chunk_id": "b", "source_path": "books/b.mmd", "chunk_index": 0, "text": "canine cardiac disease"},
        {"chunk_id": "c", "source_path": "books/c.mmd", "chunk_index": 0, "text": "nutrition feeding plan"},
    ]
    store.add(vectors, metas)
    return store
