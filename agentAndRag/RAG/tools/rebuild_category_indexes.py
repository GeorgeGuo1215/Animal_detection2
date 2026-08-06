"""按分类表从 OCR MMD 全量重建主索引与二级分类索引。

该脚本只选择每本书的一个规范源文件：优先 ``<book_id>/<book_id>.mmd``，
其次使用分类表 F 列记录的原始文件名。不会导入 ``*_det.mmd``，也不会用
数字前缀猜测文件，避免把 084 误配到旧目录名以 ``84`` 开头的弓形虫书籍。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections import OrderedDict, defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.text_utils import chunk_text, cleanup_mmd_text, read_text_lossy
from RAG.tools.split_index_by_category import (
    CategorySpec,
    _norm,
    parse_taxonomy_from_rows,
    read_xlsx_rows,
)


@dataclass
class CatalogBook:
    book_id: str
    title: str
    mmd: str
    source_name: str
    category_ids: List[str] = field(default_factory=list)
    source_path: Optional[Path] = None


def _rag_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_xlsx() -> Path:
    return _rag_root() / "data" / "veterinary_materials_classification_2.0.xlsx"


def _source_files(source_root: Path) -> List[Path]:
    return sorted(
        p
        for p in source_root.rglob("*.mmd")
        if p.is_file() and not p.name.lower().endswith("_det.mmd")
    )


def _unique_match(index: Dict[str, List[Path]], value: str) -> Optional[Path]:
    key = _norm(value)
    matches = index.get(key) or []
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        hashes = {hashlib.sha1(p.read_bytes()).hexdigest() for p in matches}
        if len(hashes) == 1:
            return sorted(matches)[0]
    return None


def resolve_book_sources(
    *, source_root: Path, books: Iterable[CatalogBook]
) -> tuple[List[CatalogBook], List[dict]]:
    files = _source_files(source_root)
    by_name: Dict[str, List[Path]] = defaultdict(list)
    by_parent: Dict[str, List[Path]] = defaultdict(list)
    for path in files:
        by_name[_norm(path.name)].append(path)
        by_name[_norm(path.stem)].append(path)
        by_parent[_norm(path.parent.name)].append(path)

    resolved: List[CatalogBook] = []
    missing: List[dict] = []
    for book in books:
        exact = source_root / book.book_id / f"{book.book_id}.mmd"
        path: Optional[Path] = exact if exact.is_file() else None
        if path is None and book.source_name:
            path = _unique_match(by_name, book.source_name) or _unique_match(
                by_parent, Path(book.source_name).stem
            )
        if path is None and book.mmd and not re.fullmatch(r"\d{3}\.mmd", book.mmd, re.I):
            path = _unique_match(by_name, book.mmd) or _unique_match(by_parent, book.mmd)
        if path is None and len(_norm(book.title)) >= 20:
            path = _unique_match(by_name, book.title) or _unique_match(by_parent, book.title)
        book.source_path = path
        if path is None:
            missing.append(
                {
                    "book_id": book.book_id,
                    "title": book.title,
                    "mmd": book.mmd,
                    "source_name": book.source_name,
                    "category_ids": book.category_ids,
                }
            )
        else:
            resolved.append(book)
    return resolved, missing


def load_catalog(xlsx: Path) -> tuple[OrderedDict[str, CategorySpec], List[CatalogBook]]:
    categories = parse_taxonomy_from_rows(read_xlsx_rows(xlsx))
    books: OrderedDict[str, CatalogBook] = OrderedDict()
    for category_id, spec in categories.items():
        for entry in spec.books:
            if not entry.book_id.isdigit():
                continue
            book_id = entry.book_id.zfill(3)
            book = books.get(book_id)
            if book is None:
                book = CatalogBook(
                    book_id=book_id,
                    title=entry.title,
                    mmd=entry.mmd,
                    source_name=entry.source_name,
                )
                books[book_id] = book
            if category_id not in book.category_ids:
                book.category_ids.append(category_id)
    return categories, list(books.values())


def _write_store(index_dir: Path, vectors: np.ndarray, metas: Sequence[dict]) -> None:
    index_dir.mkdir(parents=True, exist_ok=True)
    dim = int(vectors.shape[1])
    (index_dir / "store_config.json").write_text(
        json.dumps({"dim": dim, "metric": "cosine_dot"}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    np.save(index_dir / "embeddings.npy", vectors.astype(np.float32, copy=False))
    with (index_dir / "meta.jsonl").open("w", encoding="utf-8") as handle:
        for meta in metas:
            handle.write(json.dumps(meta, ensure_ascii=False) + "\n")


def _assert_empty_output(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f"输出目录非空，为防止覆盖已停止: {path}")


def rebuild_indexes(
    *,
    xlsx: Path,
    source_root: Path,
    full_index_out: Path,
    category_root_out: Path,
    taxonomy_out: Path,
    runtime_full_index: str,
    runtime_category_root: str,
    embedding_model: str,
    device: Optional[str],
    batch_size: int,
    chunk_units: int,
    overlap_units: int,
    min_units: int,
    allowed_missing: set[str],
    excluded_books: Dict[str, str],
) -> dict:
    _assert_empty_output(full_index_out)
    _assert_empty_output(category_root_out)
    categories, catalog = load_catalog(xlsx)
    resolved, missing = resolve_book_sources(source_root=source_root, books=catalog)
    unexpected_missing = [m for m in missing if m["book_id"] not in allowed_missing]
    if unexpected_missing:
        raise FileNotFoundError(
            "分类表书籍缺少规范 MMD: " + ", ".join(m["book_id"] for m in unexpected_missing)
        )
    excluded = []
    kept: List[CatalogBook] = []
    for book in resolved:
        reason = excluded_books.get(book.book_id)
        if reason is None:
            kept.append(book)
        else:
            excluded.append(
                {
                    "book_id": book.book_id,
                    "title": book.title,
                    "reason": reason,
                    "source_file": book.source_path.name if book.source_path else None,
                    "category_ids": book.category_ids,
                }
            )
    resolved = kept
    resolved_source_count = len(resolved)

    # 同一份 OCR 内容在分类表中可能以不同书号重复出现。相同分类内保留
    # 第一条作为规范书号；跨分类重复通常意味着源文件错配，必须显式排除。
    deduplicated: List[dict] = []
    canonical_by_sha1: Dict[str, CatalogBook] = {}
    unique_books: List[CatalogBook] = []
    for book in resolved:
        assert book.source_path is not None
        source_sha1 = hashlib.sha1(book.source_path.read_bytes()).hexdigest()
        canonical = canonical_by_sha1.get(source_sha1)
        if canonical is None:
            canonical_by_sha1[source_sha1] = book
            unique_books.append(book)
            continue
        if set(book.category_ids) != set(canonical.category_ids):
            raise ValueError(
                "相同 MMD 被分到不同分类，请核对或用 --exclude-book 排除错配项: "
                f"{canonical.book_id}({canonical.category_ids}) vs "
                f"{book.book_id}({book.category_ids})"
            )
        deduplicated.append(
            {
                "book_id": book.book_id,
                "alias_of": canonical.book_id,
                "title": book.title,
                "reason": "exact_source_sha1_duplicate",
                "source_sha1": source_sha1,
                "category_ids": book.category_ids,
            }
        )
    resolved = unique_books

    embedder = Embedder(embedding_model, device=device)
    all_vectors: List[np.ndarray] = []
    all_metas: List[dict] = []
    cat_vectors: Dict[str, List[np.ndarray]] = defaultdict(list)
    cat_metas: Dict[str, List[dict]] = defaultdict(list)
    book_stats: List[dict] = []
    dim = 0

    for number, book in enumerate(resolved, start=1):
        assert book.source_path is not None
        raw_text, source_sha1 = read_text_lossy(book.source_path)
        clean_text = cleanup_mmd_text(raw_text)
        logical_path = Path("books") / f"{book.book_id}.mmd"
        chunks = chunk_text(
            source_path=logical_path,
            source_sha1=source_sha1,
            clean_text=clean_text,
            chunk_words=chunk_units,
            chunk_overlap_words=overlap_units,
            min_chunk_words=min_units,
        )
        if not chunks:
            raise ValueError(f"书籍清洗/切分后没有正文: {book.book_id} {book.source_path}")
        texts = [chunk.text for chunk in chunks]
        vectors = embedder.embed_texts(
            texts, batch_size=batch_size, normalize=True, show_progress=False
        ).vectors
        dim = int(vectors.shape[1])
        metas: List[dict] = []
        for chunk in chunks:
            meta = asdict(chunk)
            meta.update(
                {
                    "chunk_id": hashlib.sha1(
                        f"semantic-v2:{book.book_id}:{source_sha1}:{chunk.chunk_index}".encode()
                    ).hexdigest(),
                    "source_path": f"books/{book.book_id}.mmd",
                    "source_file": book.source_path.name,
                    "book_id": book.book_id,
                    "book_title": book.title,
                    "category_ids": book.category_ids,
                    "n_units": chunk.n_words,
                    "chunking_version": "semantic-v2",
                }
            )
            metas.append(meta)
        all_vectors.append(vectors)
        all_metas.extend(metas)
        for category_id in book.category_ids:
            cat_vectors[category_id].append(vectors)
            cat_metas[category_id].extend(metas)
        book_stats.append(
            {
                "book_id": book.book_id,
                "title": book.title,
                "source_file": book.source_path.name,
                "source_sha1": source_sha1,
                "source_bytes": book.source_path.stat().st_size,
                "chunk_count": len(chunks),
                "category_ids": book.category_ids,
            }
        )
        print(
            f"[{number:02d}/{len(resolved):02d}] {book.book_id}: "
            f"chunks={len(chunks)} categories={','.join(book.category_ids)}",
            flush=True,
        )

    if not all_vectors:
        raise ValueError("没有可入库书籍")
    full_vectors = np.concatenate(all_vectors, axis=0).astype(np.float32, copy=False)
    _write_store(full_index_out, full_vectors, all_metas)

    category_counts: Dict[str, int] = {}
    for category_id in categories:
        vectors_list = cat_vectors.get(category_id) or []
        metas = cat_metas.get(category_id) or []
        vectors = (
            np.concatenate(vectors_list, axis=0).astype(np.float32, copy=False)
            if vectors_list
            else np.zeros((0, dim), dtype=np.float32)
        )
        _write_store(category_root_out / category_id, vectors, metas)
        category_counts[category_id] = len(metas)

    by_id = {book.book_id: book for book in catalog}
    resolved_ids = {book.book_id for book in resolved}
    taxonomy_categories: List[dict] = []
    for category_id, spec in categories.items():
        category_books = []
        for entry in spec.books:
            if not entry.book_id.isdigit():
                continue
            book_id = entry.book_id.zfill(3)
            if any(x["book_id"] == book_id for x in category_books):
                continue
            book = by_id[book_id]
            category_books.append(
                {
                    "book_id": book_id,
                    "mmd": book.mmd,
                    "title": book.title,
                    "source_path": f"books/{book_id}.mmd" if book_id in resolved_ids else None,
                }
            )
        taxonomy_categories.append(
            {
                "id": category_id,
                "zh_l1": spec.zh_l1,
                "zh_l2": spec.zh_l2,
                "book_ids": [book["book_id"] for book in category_books],
                "books": category_books,
                "source_paths": [
                    book["source_path"] for book in category_books if book["source_path"]
                ],
                "chunk_count": category_counts[category_id],
                "index_dir": f"{runtime_category_root.rstrip('/')}/{category_id}",
            }
        )

    report = {
        "version": 2,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "classification_file": xlsx.name,
        "embedding_model": embedding_model,
        "dim": dim,
        "chunking": {
            "version": "semantic-v2",
            "target_units": chunk_units,
            "overlap_units": overlap_units,
            "min_units": min_units,
            "unit": "English word or CJK character",
        },
        "source_index": runtime_full_index,
        "out_root": runtime_category_root,
        "matched_books": len(resolved),
        "resolved_source_records": resolved_source_count,
        "unmatched_books": missing,
        "excluded_books": excluded,
        "deduplicated_books": deduplicated,
        "total_chunks": len(all_metas),
        "books": book_stats,
        "categories": taxonomy_categories,
    }
    taxonomy_out.parent.mkdir(parents=True, exist_ok=True)
    taxonomy_out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="按分类表全量重建 RAG 主索引与分类索引")
    parser.add_argument("--xlsx", default=str(_default_xlsx()))
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--full-index-out", required=True)
    parser.add_argument("--category-root-out", required=True)
    parser.add_argument("--taxonomy-out", required=True)
    parser.add_argument("--runtime-full-index", default="RAG/data/rag_index_e5")
    parser.add_argument("--runtime-category-root", default="RAG/data/rag_index_e5_by_cat")
    parser.add_argument("--embedding-model", default="intfloat/multilingual-e5-small")
    parser.add_argument("--device", default=None)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--chunk-units", type=int, default=380)
    parser.add_argument("--overlap-units", type=int, default=60)
    parser.add_argument("--min-units", type=int, default=30)
    parser.add_argument("--allow-missing", default="")
    parser.add_argument(
        "--exclude-book",
        action="append",
        default=[],
        metavar="ID=REASON",
        help="排除已解析书籍并写入 taxonomy，可重复使用",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    excluded_books: Dict[str, str] = {}
    for value in args.exclude_book:
        book_id, sep, reason = value.partition("=")
        if not sep or not book_id.strip() or not reason.strip():
            parser.error("--exclude-book 格式必须为 ID=REASON")
        excluded_books[book_id.strip().zfill(3)] = reason.strip()
    report = rebuild_indexes(
        xlsx=Path(args.xlsx),
        source_root=Path(args.source_root),
        full_index_out=Path(args.full_index_out),
        category_root_out=Path(args.category_root_out),
        taxonomy_out=Path(args.taxonomy_out),
        runtime_full_index=args.runtime_full_index.replace("\\", "/"),
        runtime_category_root=args.runtime_category_root.replace("\\", "/"),
        embedding_model=args.embedding_model,
        device=args.device,
        batch_size=args.batch_size,
        chunk_units=args.chunk_units,
        overlap_units=args.overlap_units,
        min_units=args.min_units,
        allowed_missing={x.strip().zfill(3) for x in args.allow_missing.split(",") if x.strip()},
        excluded_books=excluded_books,
    )
    print(
        json.dumps(
            {
                "matched_books": report["matched_books"],
                "missing_books": [x["book_id"] for x in report["unmatched_books"]],
                "excluded_books": [x["book_id"] for x in report["excluded_books"]],
                "deduplicated_books": [x["book_id"] for x in report["deduplicated_books"]],
                "total_chunks": report["total_chunks"],
                "categories": len(report["categories"]),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
