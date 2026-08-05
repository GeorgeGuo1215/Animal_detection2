from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Iterable, List, Optional

from tqdm import tqdm

from .config import RagConfig
from .embeddings import Embedder
from .text_utils import chunk_text, cleanup_mmd_text, iter_mmd_files, read_text_lossy
from .vector_store import NumpyVectorStore, StoreConfig
from .vector_index import chunk_to_meta


def build_or_update_index(
    cfg: RagConfig,
    *,
    limit_books: Optional[int] = None,
    batch_size: int = 32,
    device: Optional[str] = None,
) -> dict:
    """
    扫描 cfg.raw_dir 下所有 .mmd，分块 + embedding + 建立/增量更新索引。

    返回统计信息 dict（方便 CLI 打印）。
    """
    cfg.index_dir.mkdir(parents=True, exist_ok=True)

    files = list(iter_mmd_files(cfg.raw_dir))
    if limit_books is not None:
        files = files[: int(limit_books)]

    store = NumpyVectorStore(cfg.index_dir)
    embedder = Embedder(cfg.embedding_model, device=device)

    total_files = len(files)
    total_chunks = 0
    added_chunks = 0
    skipped_chunks = 0

    # 若 store 不存在：先用第一批向量的 dim 初始化（延迟到 flush）
    if store.exists():
        store.load()

    pending_texts: List[str] = []
    pending_metas: List[dict] = []
    pending_count = 0
    initialized = store.exists()

    def flush() -> None:
        nonlocal added_chunks, skipped_chunks, pending_texts, pending_metas, pending_count, initialized
        if not pending_texts:
            return
        emb = embedder.embed_texts(
            pending_texts, batch_size=batch_size, normalize=True, show_progress=False
        )
        if not initialized:
            store.init_new(StoreConfig(dim=emb.dim))
            initialized = True
        before = store.size
        n_add = store.add(emb.vectors, pending_metas)
        after = store.size
        added_chunks += n_add
        skipped_chunks += (len(pending_metas) - n_add)
        pending_texts = []
        pending_metas = []
        pending_count = after - before

    for path in tqdm(files, desc="入库 .mmd", unit="本"):
        text, sha1 = read_text_lossy(path)
        clean = cleanup_mmd_text(text)
        chunks = chunk_text(
            source_path=path,
            source_sha1=sha1,
            clean_text=clean,
            chunk_words=cfg.chunk_words,
            chunk_overlap_words=cfg.chunk_overlap_words,
            min_chunk_words=cfg.min_chunk_words,
        )
        total_chunks += len(chunks)

        for ch in chunks:
            pending_texts.append(ch.text)
            pending_metas.append(chunk_to_meta(ch))
            pending_count += 1
            if pending_count >= batch_size:
                flush()

    flush()

    return {
        "raw_dir": str(cfg.raw_dir),
        "index_dir": str(cfg.index_dir),
        "embedding_model": cfg.embedding_model,
        "total_files": total_files,
        "total_chunks": total_chunks,
        "added_chunks": added_chunks,
        "skipped_chunks": skipped_chunks,
        "index_size": store.size if initialized else 0,
        "chunking": {
            "chunk_words": cfg.chunk_words,
            "chunk_overlap_words": cfg.chunk_overlap_words,
            "min_chunk_words": cfg.min_chunk_words,
        },
    }


def search(
    cfg: RagConfig,
    query: str,
    *,
    top_k: int = 5,
    device: Optional[str] = None,
) -> List[dict]:
    store = NumpyVectorStore(cfg.index_dir)
    if not store.exists():
        raise FileNotFoundError(f"索引不存在：{cfg.index_dir}（请先运行 ingest）")
    store.load()

    embedder = Embedder(cfg.embedding_model, device=device)
    q_emb = embedder.embed_queries([query], batch_size=1, normalize=True).vectors[0]
    hits = store.search(q_emb, top_k=top_k)
    return [
        {
            "score": float(score),
            "source_path": meta.get("source_path"),
            "chunk_index": meta.get("chunk_index"),
            "n_words": meta.get("n_words"),
            "text": meta.get("text"),
            "chunk_id": meta.get("chunk_id"),
        }
        for meta, score in hits
    ]


