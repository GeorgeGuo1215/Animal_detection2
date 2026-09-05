"""逐书抽取原文片段，验证分类索引的召回与隔离。"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.vector_store import NumpyVectorStore


def _excerpt(text: str, max_chars: int = 700) -> str:
    clean = " ".join((text or "").split())
    # 取书籍中部 chunk 的开头，而不是 chunk 中段；sentence-transformers
    # 对超出模型 token 上限的 passage 会从尾部截断，开头才一定参与向量编码。
    return clean[:max_chars]


def validate(
    *,
    taxonomy_path: Path,
    category_root: Path,
    embedding_model: str,
    device: Optional[str],
    batch_size: int,
    top_k: int,
) -> dict:
    taxonomy = json.loads(taxonomy_path.read_text(encoding="utf-8"))
    categories = {c["id"]: c for c in taxonomy["categories"]}
    assigned: dict[str, set[str]] = defaultdict(set)
    titles: dict[str, str] = {}
    for category_id, category in categories.items():
        for book in category.get("books") or []:
            if not book.get("source_path"):
                continue
            book_id = str(book["book_id"]).zfill(3)
            assigned[book_id].add(category_id)
            titles.setdefault(book_id, str(book.get("title") or ""))

    stores: dict[str, NumpyVectorStore] = {}
    contamination: list[dict] = []
    membership_missing: list[dict] = []
    source_meta: dict[str, tuple[str, dict]] = {}
    for category_id, category in categories.items():
        store = NumpyVectorStore(category_root / category_id)
        store.load()
        stores[category_id] = store
        present = defaultdict(int)
        for meta in store._meta:  # noqa: SLF001 - validation intentionally inspects persisted rows
            book_id = str(meta.get("book_id") or "").zfill(3)
            present[book_id] += 1
            if category_id not in assigned.get(book_id, set()):
                contamination.append(
                    {"category_id": category_id, "book_id": book_id, "chunk_id": meta.get("chunk_id")}
                )
            source_meta.setdefault(book_id, (category_id, meta))
        for book in category.get("books") or []:
            if book.get("source_path") and not present.get(str(book["book_id"]).zfill(3)):
                membership_missing.append(
                    {"category_id": category_id, "book_id": str(book["book_id"]).zfill(3)}
                )

    embedder = Embedder(embedding_model, device=device)
    grouped: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for book_id in sorted(assigned):
        # 选择规模最小的已分配分类进行检索，减少同类大部头书之间的干扰。
        category_id = min(assigned[book_id], key=lambda cid: stores[cid].size)
        candidates = [m for m in stores[category_id]._meta if str(m.get("book_id")) == book_id]  # noqa: SLF001
        meta = candidates[len(candidates) // 2]
        grouped[category_id].append((book_id, _excerpt(str(meta.get("text") or ""))))

    retrieval_rows: list[dict] = []
    for category_id, queries in sorted(grouped.items()):
        store = stores[category_id]
        assert store._emb is not None  # noqa: SLF001
        query_vectors = embedder.embed_queries(
            [query for _, query in queries], batch_size=batch_size, normalize=True
        ).vectors
        scores = store._emb @ query_vectors.T  # noqa: SLF001
        k = min(top_k, store.size)
        for column, (book_id, query) in enumerate(queries):
            order = np.argpartition(-scores[:, column], kth=k - 1)[:k]
            order = order[np.argsort(-scores[order, column])]
            top = [
                {
                    "rank": rank,
                    "book_id": str(store._meta[int(row)].get("book_id") or ""),  # noqa: SLF001
                    "book_title": str(store._meta[int(row)].get("book_title") or ""),  # noqa: SLF001
                    "score": float(scores[int(row), column]),
                    "chunk_id": store._meta[int(row)].get("chunk_id"),  # noqa: SLF001
                }
                for rank, row in enumerate(order, start=1)
            ]
            ranks = [hit["rank"] for hit in top if hit["book_id"] == book_id]
            retrieval_rows.append(
                {
                    "book_id": book_id,
                    "book_title": titles.get(book_id, ""),
                    "category_id": category_id,
                    "excerpt": query,
                    "hit": bool(ranks),
                    "expected_book_rank": ranks[0] if ranks else None,
                    "top_hits": top,
                }
            )

    failures = [row for row in retrieval_rows if not row["hit"]]
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "taxonomy": str(taxonomy_path),
        "category_root": str(category_root),
        "embedding_model": embedding_model,
        "top_k": top_k,
        "summary": {
            "books_expected": len(assigned),
            "books_tested": len(retrieval_rows),
            "books_hit": len(retrieval_rows) - len(failures),
            "top1_hits": sum(row["expected_book_rank"] == 1 for row in retrieval_rows),
            "retrieval_failures": len(failures),
            "membership_missing": len(membership_missing),
            "category_contamination": len(contamination),
        },
        "membership_missing": membership_missing,
        "category_contamination": contamination,
        "books": retrieval_rows,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="逐书验证分类索引召回")
    parser.add_argument("--taxonomy", required=True)
    parser.add_argument("--category-root", required=True)
    parser.add_argument("--embedding-model", default="intfloat/multilingual-e5-small")
    parser.add_argument("--device", default=None)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = validate(
        taxonomy_path=Path(args.taxonomy),
        category_root=Path(args.category_root),
        embedding_model=args.embedding_model,
        device=args.device,
        batch_size=args.batch_size,
        top_k=args.top_k,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    summary = report["summary"]
    return 0 if not any(
        summary[key]
        for key in ("retrieval_failures", "membership_missing", "category_contamination")
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
