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
from RAG.tools.append_books_to_category_indexes import (  # noqa: E402
    book_id_from_source,
)
from RAG.tools.copy_ocr_mmd_to_raw import copy_books  # noqa: E402


def test_discover_books_subdir_and_bare(tmp_path: Path):
    (tmp_path / "089").mkdir()
    (tmp_path / "089" / "b.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "089" / "a.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "080.pdf").write_bytes(b"%PDF-1.4")
    (tmp_path / "ignore.txt").write_text("x", encoding="utf-8")
    books = discover_books(tmp_path)
    ids = [b["id"] for b in books]
    assert ids == ["080", "089"]
    # 子文件夹多 PDF 按文件名排序
    names = [p.name for p in books[1]["pdfs"]]
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
