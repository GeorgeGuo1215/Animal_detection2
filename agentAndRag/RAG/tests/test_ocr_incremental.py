"""单元测试：OCR 书籍发现 / book_id 解析 / 增量 append 辅助（不依赖 GPU）。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

_RAG = Path(__file__).resolve().parents[1]
_AGENTANDRAG = _RAG.parent
_OCR_HF = _AGENTANDRAG / "DeepSeek-OCR-master" / "DeepSeek-OCR-hf"
for p in (str(_AGENTANDRAG), str(_OCR_HF)):
    if p not in sys.path:
        sys.path.insert(0, p)

from batch_ocr_to_vllm_layout import PAGE_SPLIT, discover_books  # noqa: E402
from RAG.simple_rag.vector_store import NumpyVectorStore, StoreConfig  # noqa: E402
from RAG.simple_rag.text_utils import chunk_text, word_count  # noqa: E402
from RAG.tools.append_books_to_category_indexes import (  # noqa: E402
    book_id_from_source,
)
from RAG.tools.copy_ocr_mmd_to_raw import copy_books  # noqa: E402
from RAG.tools.rebuild_category_indexes import (  # noqa: E402
    CatalogBook,
    resolve_book_sources,
)


def test_discover_books_subdir_and_bare(tmp_path: Path):
    (tmp_path / "089").mkdir()
    (tmp_path / "089" / "b.pdf").write_bytes(b"%PDF-b")
    (tmp_path / "089" / "a.pdf").write_bytes(b"%PDF-a")
    (tmp_path / "080.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "batch" / "084" / "volume-1").mkdir(parents=True)
    (tmp_path / "batch" / "084" / "volume-1" / "guide.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "batch" / "084" / "volume-2").mkdir(parents=True)
    (tmp_path / "batch" / "084" / "volume-2" / "duplicate.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "ignore.txt").write_text("x", encoding="utf-8")
    books = discover_books(tmp_path)
    ids = [b["id"] for b in books]
    assert ids == ["080", "084", "089"]
    assert [p.name for p in books[1]["pdfs"]] == ["guide.pdf"]
    # 子文件夹多 PDF 按文件名排序
    names = [p.name for p in books[2]["pdfs"]]
    assert names == ["a.pdf", "b.pdf"]


def test_page_split_join():
    pages = ["page one", "page two"]
    text = PAGE_SPLIT.join(pages)
    assert "<--- Page Split --->" in text
    assert text.count("Page Split") == 1


def test_book_id_from_source():
    assert book_id_from_source(r"C:\data\raw\089.mmd") == "089"
    assert book_id_from_source("/x/068/068.mmd") == "068"
    assert book_id_from_source("nope.md") is None


def test_copy_ocr_mmd_to_raw(tmp_path: Path):
    ocr = tmp_path / "ocr"
    raw = tmp_path / "raw"
    d = ocr / "089"
    d.mkdir(parents=True)
    (d / "089.mmd").write_text("hello", encoding="utf-8")
    (ocr / "080").mkdir()
    # 未完成书：无 mmd
    done = copy_books(ocr_root=ocr, raw_dir=raw, only={"089", "080"})
    assert done == ["089"]
    assert (raw / "089.mmd").read_text(encoding="utf-8") == "hello"


def test_vector_store_append_skips_dup_chunk_id(tmp_path: Path):
    store = NumpyVectorStore(tmp_path / "idx")
    store.init_new(StoreConfig(dim=4))
    v = np.ones((1, 4), dtype=np.float32)
    v /= np.linalg.norm(v)
    meta = {"chunk_id": "c1", "source_path": "089.mmd", "text": "a"}
    assert store.add(v, [meta]) == 1
    assert store.add(v, [meta]) == 0
    assert store.size == 1


def test_semantic_chunker_aggregates_short_paragraphs_and_counts_chinese():
    text = "\n\n".join(f"第{i}段猫犬临床观察显示需要持续复查。" for i in range(30))
    chunks = chunk_text(
        source_path=Path("books/999.mmd"),
        source_sha1="abc",
        clean_text=text,
        chunk_words=120,
        chunk_overlap_words=20,
        min_chunk_words=20,
    )
    assert 2 <= len(chunks) < 10
    assert all(chunk.n_words >= 20 for chunk in chunks)
    assert word_count("猫犬abc treatment") == 4


def test_source_resolution_never_uses_unrelated_numeric_prefix(tmp_path: Path):
    wrong = tmp_path / "84 old toxoplasmosis"
    wrong.mkdir()
    (wrong / "84 old toxoplasmosis.mmd").write_text("wrong book", encoding="utf-8")
    books = [
        CatalogBook(
            book_id="084",
            title="AAHA指南系列",
            mmd="AAHA指南系列",
            source_name="",
            category_ids=["guidelines.general"],
        )
    ]
    resolved, missing = resolve_book_sources(source_root=tmp_path, books=books)
    assert not resolved
    assert [row["book_id"] for row in missing] == ["084"]
