"""将全量索引中「新增书」的 chunks 增量追加到涉及的 by_cat 子库（旧流程）。

用法（conda RAG）:
  # 先: ingest 只含新增 mmd 的 raw（或 raw 里已有新增文件且 ingest 跳过旧 chunk）
  python -m RAG.maintenance.indexing.append_books_to_category_indexes --book-ids 089,080
  python -m RAG.maintenance.indexing.append_books_to_category_indexes --xlsx PATH --book-ids 068,069

新资料优先使用 ``rebuild_category_indexes``，它包含规范源解析、去重、分类
审计和逐书验证。本脚本只保留给已经进入主索引的单书追加场景。
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Set

import numpy as np

from RAG.simple_rag.vector_store import NumpyVectorStore, StoreConfig
from RAG.maintenance.indexing.split_index_by_category import (
    parse_taxonomy_from_rows,
    read_xlsx_rows,
)


def _repo_rag() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_xlsx() -> Path:
    return _repo_rag() / "data" / "veterinary_materials_classification_2.0.xlsx"


def book_id_from_source(source_path: str) -> Optional[str]:
    name = Path(source_path).name
    m = re.match(r"^(\d{3})\.mmd$", name, re.I)
    if m:
        return m.group(1)
    # 兜底：路径中 /068/068.mmd
    parts = Path(source_path).parts
    for p in reversed(parts):
        if re.fullmatch(r"\d{3}", p):
            return p
    return None


def categories_for_books(xlsx: Path, book_ids: Set[str]) -> Dict[str, Set[str]]:
    """book_id -> set(category_id)。"""
    rows = read_xlsx_rows(xlsx)
    cats = parse_taxonomy_from_rows(rows)
    out: Dict[str, Set[str]] = {bid: set() for bid in book_ids}
    for cid, spec in cats.items():
        for b in spec.books:
            bid = (b.book_id or "").strip()
            if bid in out:
                out[bid].add(cid)
            # 也匹配 mmd 文件名
            mmd = (b.mmd or "").strip()
            m = re.match(r"^(\d{3})", mmd)
            if m and m.group(1) in out:
                out[m.group(1)].add(cid)
    return out


def append_books(
    *,
    src_index: Path,
    out_root: Path,
    xlsx: Path,
    book_ids: Set[str],
) -> dict:
    store = NumpyVectorStore(src_index)
    if not store.exists():
        raise FileNotFoundError(f"全量索引不存在: {src_index}")
    store.load()
    assert store._emb is not None

    mapping = categories_for_books(xlsx, book_ids)
    # 收集每个 category 要追加的行号
    cat_rows: Dict[str, List[int]] = {}
    matched_books: Set[str] = set()
    for i, meta in enumerate(store._meta):
        sp = str(meta.get("source_path") or "")
        bid = book_id_from_source(sp)
        if not bid or bid not in book_ids:
            continue
        matched_books.add(bid)
        for cid in mapping.get(bid) or set():
            cat_rows.setdefault(cid, []).append(i)

    stats = {
        "book_ids": sorted(book_ids),
        "matched_books": sorted(matched_books),
        "categories": {},
        "unmapped_books": sorted(book_ids - matched_books),
    }

    for cid, rows in sorted(cat_rows.items()):
        cat_dir = out_root / cid
        cat_dir.mkdir(parents=True, exist_ok=True)
        cat_store = NumpyVectorStore(cat_dir)
        if cat_store.exists():
            cat_store.load()
        else:
            cat_store.init_new(StoreConfig(dim=store.config.dim, metric=store.config.metric))

        vecs = store._emb[np.array(rows, dtype=np.int64)]
        metas = [store._meta[i] for i in rows]
        n = cat_store.add(vecs, metas)
        stats["categories"][cid] = {"candidate_rows": len(rows), "added": n, "size": cat_store.size}

    return stats


def main() -> None:
    p = argparse.ArgumentParser(description="增量追加新书到 by_cat 子库")
    p.add_argument("--src-index", type=str, default=str(_repo_rag() / "data" / "rag_index_e5"))
    p.add_argument("--out-root", type=str, default=str(_repo_rag() / "data" / "rag_index_e5_by_cat"))
    p.add_argument("--xlsx", type=str, default=str(_default_xlsx()))
    p.add_argument("--book-ids", type=str, required=True, help="逗号分隔，如 068,069,089")
    args = p.parse_args()
    book_ids = {x.strip() for x in args.book_ids.split(",") if x.strip()}
    stats = append_books(
        src_index=Path(args.src_index),
        out_root=Path(args.out_root),
        xlsx=Path(args.xlsx),
        book_ids=book_ids,
    )
    print(json.dumps(stats, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
