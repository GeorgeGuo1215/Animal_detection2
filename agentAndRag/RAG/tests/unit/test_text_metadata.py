from __future__ import annotations

from pathlib import Path

import pytest

from RAG.simple_rag.metadata import chunk_to_meta
from RAG.simple_rag.text_utils import chunk_text, cleanup_mmd_text, word_count


pytestmark = pytest.mark.unit


def test_cleanup_and_mixed_language_count() -> None:
    cleaned = cleanup_mmd_text("<b>猫犬</b> ![scan](x.png)\r\n\r\n treatment 123")
    assert "<b>" not in cleaned
    assert "scan" not in cleaned
    assert word_count(cleaned) == 4


def test_chunk_metadata_is_stable() -> None:
    text = "。".join(["猫出现排尿困难需要尽快进行临床评估"] * 30)
    chunks = chunk_text(
        source_path=Path("books/052.mmd"),
        source_sha1="abc",
        clean_text=text,
        chunk_words=80,
        chunk_overlap_words=10,
        min_chunk_words=10,
    )
    assert len(chunks) >= 2
    meta = chunk_to_meta(chunks[0])
    assert meta["chunk_id"] == chunks[0].chunk_id
    assert meta["source_path"].endswith("052.mmd")
    assert meta["n_words"] == word_count(meta["text"])


def test_chunk_parameter_boundary() -> None:
    with pytest.raises(AssertionError):
        chunk_text(
            source_path=Path("bad.mmd"),
            source_sha1="bad",
            clean_text="text",
            chunk_words=10,
            chunk_overlap_words=10,
            min_chunk_words=1,
        )
