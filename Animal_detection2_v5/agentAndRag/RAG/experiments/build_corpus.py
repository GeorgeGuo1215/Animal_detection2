from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
# 允许直接运行：python RAG/experiments/build_corpus.py
# - experiments/ 用于导入 common.py
# - RAG/ 用于导入 simple_rag/
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1]))

from common import book_id_from_source_path, write_jsonl  # noqa: E402
from simple_rag.config import RagConfig, default_config  # noqa: E402
from simple_rag.text_utils import (  # noqa: E402
    chunk_text,
    cleanup_mmd_text,
    iter_mmd_files,
    read_text_lossy,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="从 RAG/data/raw/**/*.mmd 构建评测语料 corpus.jsonl（chunk 粒度）")
    parser.add_argument("--raw-dir", type=str, default=None)
    parser.add_argument("--out", type=str, default="RAG/experiments/out/corpus.jsonl")
    parser.add_argument("--chunk-words", type=int, default=800)
    parser.add_argument("--chunk-overlap-words", type=int, default=150)
    parser.add_argument("--min-chunk-words", type=int, default=40)
    parser.add_argument("--limit-books", type=int, default=None)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    cfg0 = default_config(repo_root)
    cfg = RagConfig(
        raw_dir=Path(args.raw_dir) if args.raw_dir else cfg0.raw_dir,
        index_dir=cfg0.index_dir,
        embedding_model=cfg0.embedding_model,
        chunk_words=args.chunk_words,
        chunk_overlap_words=args.chunk_overlap_words,
        min_chunk_words=args.min_chunk_words,
    )

    files = list(iter_mmd_files(cfg.raw_dir))
    if args.limit_books is not None:
        files = files[: int(args.limit_books)]

    rows = []
    for fp in files:
        text, sha1 = read_text_lossy(fp)
        clean = cleanup_mmd_text(text)
        chunks = chunk_text(
            source_path=fp,
            source_sha1=sha1,
            clean_text=clean,
            chunk_words=cfg.chunk_words,
            chunk_overlap_words=cfg.chunk_overlap_words,
            min_chunk_words=cfg.min_chunk_words,
        )
        for ch in chunks:
            src = str(fp)
            rows.append(
                {
                    "chunk_id": ch.chunk_id,
                    "book_id": book_id_from_source_path(src),
                    "source_path": src,
                    "source_sha1": sha1,
                    "chunk_index": ch.chunk_index,
                    "n_words": ch.n_words,
                    "text": ch.text,
                }
            )

    write_jsonl(Path(args.out), rows)
    print(f"写入 {len(rows)} chunks → {args.out}")


if __name__ == "__main__":
    main()


