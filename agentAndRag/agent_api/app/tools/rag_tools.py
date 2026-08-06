from __future__ import annotations

import hashlib
import os
from collections import OrderedDict
from functools import wraps
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union
import threading

from ..concurrency import get_resource_limits

_RAG_DEVICE: Optional[str] = os.getenv("AGENT_WARMUP_DEVICE") or None

from RAG.simple_rag.category_index import resolve_category_index_dirs
from RAG.simple_rag.config import RagConfig, default_config
from RAG.simple_rag.context_utils import build_neighbor_contexts, build_source_index
from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.pipeline import build_or_update_index
from RAG.simple_rag.query_rewrite import LLMRewriter, NoRewrite, TemplateRewriter
from RAG.simple_rag.retrieval import BM25Retriever, MultiRouteRetriever, RetrievedChunk
from RAG.simple_rag.reranker import CrossEncoderReranker
from RAG.simple_rag.vector_store import NumpyVectorStore

from RAG.query import overlap_score

from ..hf_local_model import resolve_embedding_model_id, resolve_rerank_model_id
from .rag_query import require_english_rag_query

_LOCK = threading.RLock()
_STORE_CACHE: Dict[str, NumpyVectorStore] = {}
_EMBEDDER_CACHE: Dict[tuple[str, Optional[str]], Embedder] = {}
_BM25_CACHE: Dict[str, BM25Retriever] = {}
_RERANKER_CACHE: Dict[tuple[str, Optional[str]], CrossEncoderReranker] = {}
_SOURCE_INDEX_CACHE: Dict[str, Dict[str, Dict[int, str]]] = {}

_QUERY_EMB_CACHE_SIZE = 128
_QUERY_EMB_CACHE: OrderedDict[str, Any] = OrderedDict()


def _query_emb_key(query: str, model: str) -> str:
    return hashlib.md5(f"{model}||{query}".encode()).hexdigest()


def _get_store(index_dir: Path) -> NumpyVectorStore:
    key = str(index_dir.resolve())
    with _LOCK:
        st = _STORE_CACHE.get(key)
        if st is not None:
            return st
        st = NumpyVectorStore(index_dir)
        st.load()
        _STORE_CACHE[key] = st
        return st


def _get_embedder(embedding_model: str, device: Optional[str]) -> Embedder:
    resolved = resolve_embedding_model_id(embedding_model, _repo_root())
    key = (resolved, device)
    with _LOCK:
        em = _EMBEDDER_CACHE.get(key)
        if em is not None:
            return em
        em = Embedder(resolved, device=device)
        _EMBEDDER_CACHE[key] = em
        return em


def _get_bm25(index_dir: Path) -> BM25Retriever:
    key = str(index_dir.resolve())
    with _LOCK:
        bm = _BM25_CACHE.get(key)
        if bm is not None:
            return bm
        st = _get_store(index_dir)
        bm = BM25Retriever(metas=st._meta)  # noqa: SLF001
        _BM25_CACHE[key] = bm
        return bm


def _get_reranker(rerank_model: str, device: Optional[str]) -> CrossEncoderReranker:
    resolved = resolve_rerank_model_id(rerank_model, _repo_root())
    key = (resolved, device)
    with _LOCK:
        rr = _RERANKER_CACHE.get(key)
        if rr is not None:
            return rr
        rr = CrossEncoderReranker(resolved, device=device)
        _RERANKER_CACHE[key] = rr
        return rr


def _get_source_index(index_dir: Path) -> Dict[str, Dict[int, str]]:
    key = str(index_dir.resolve())
    with _LOCK:
        si = _SOURCE_INDEX_CACHE.get(key)
        if si is not None:
            return si
        st = _get_store(index_dir)
        si = build_source_index(st._meta)  # noqa: SLF001
        _SOURCE_INDEX_CACHE[key] = si
        return si


def _embed_query_cached(embedder: Embedder, query: str, model_name: str):
    """LRU cache for query embeddings to avoid recomputing identical queries."""
    ck = _query_emb_key(query, model_name)
    with _LOCK:
        if ck in _QUERY_EMB_CACHE:
            _QUERY_EMB_CACHE.move_to_end(ck)
            return _QUERY_EMB_CACHE[ck]
    vec = embedder.embed_queries([query], batch_size=1, normalize=True).vectors[0]
    with _LOCK:
        _QUERY_EMB_CACHE[ck] = vec
        if len(_QUERY_EMB_CACHE) > _QUERY_EMB_CACHE_SIZE:
            _QUERY_EMB_CACHE.popitem(last=False)
    return vec


def _invalidate_index_cache(index_dir: Path) -> None:
    key = str(index_dir.resolve())
    with _LOCK:
        _STORE_CACHE.pop(key, None)
        _BM25_CACHE.pop(key, None)
        _SOURCE_INDEX_CACHE.pop(key, None)
        _QUERY_EMB_CACHE.clear()


def warmup_rag_cache(
    *,
    index_dir: Path,
    embedding_model: str = "intfloat/multilingual-e5-small",
    device: Optional[str] = None,
    enable_bm25: bool = True,
    enable_reranker: bool = False,
    rerank_model: str = "BAAI/bge-reranker-large",
) -> Dict[str, Any]:
    st = _get_store(index_dir)
    _get_embedder(embedding_model, device)
    _get_source_index(index_dir)
    if enable_bm25:
        _get_bm25(index_dir)
    if enable_reranker:
        _get_reranker(rerank_model, device)
    return {
        "index_dir": str(index_dir),
        "embedding_model": embedding_model,
        "device": device,
        "enable_bm25": bool(enable_bm25),
        "enable_reranker": bool(enable_reranker),
        "rerank_model": rerank_model if enable_reranker else None,
        "index_size": int(st.size),
    }


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _with_rag_limit(handler):
    @wraps(handler)
    def _wrapped(*args: Any, **kwargs: Any) -> Dict[str, Any]:
        limits = get_resource_limits()
        with limits.rag.slot(timeout_s=limits.acquire_timeout_s):
            return handler(*args, **kwargs)

    return _wrapped


class _LimitedSyncRewriter:
    def __init__(self, delegate: Any) -> None:
        self.delegate = delegate

    def rewrite(self, query: str) -> List[str]:
        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            return self.delegate.rewrite(query)


class _CachedDenseRetriever:
    def __init__(self, *, store: NumpyVectorStore, embedder: Embedder) -> None:
        self.store = store
        self.embedder = embedder

    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]:
        q = (query or "").strip()
        if not q:
            return []
        q_emb = _embed_query_cached(self.embedder, q, self.embedder.model_name_or_path)
        hits = self.store.search(q_emb, top_k=int(top_k))
        return [RetrievedChunk(chunk_id=str(m.get("chunk_id")), score=float(s), meta=m) for m, s in hits]


def _build_cached_multiroute(
    *,
    index_dir: Path,
    embedding_model: str,
    device: Optional[str],
    rewrite: str,
    rewrite_base_url: Optional[str],
    rewrite_api_key: Optional[str],
    rewrite_model: Optional[str],
    rewrite_max_out: int,
    rewrite_timeout_s: float,
) -> MultiRouteRetriever:
    st = _get_store(index_dir)
    em = _get_embedder(embedding_model, device)
    dense = _CachedDenseRetriever(store=st, embedder=em)
    bm25 = _get_bm25(index_dir)
    if rewrite == "none":
        rewriter = NoRewrite()
    elif rewrite == "llm":
        rewriter = _LimitedSyncRewriter(
            LLMRewriter(
                base_url=rewrite_base_url,
                api_key=rewrite_api_key,
                model=rewrite_model,
                max_out=int(rewrite_max_out),
                timeout_s=float(rewrite_timeout_s),
            )
        )
    else:
        rewriter = TemplateRewriter(max_out=int(rewrite_max_out))
    return MultiRouteRetriever(retrievers=[("dense", dense), ("bm25", bm25)], rewriter=rewriter, top_k_per_route=20)


def _hit_from_meta(meta: dict, score: float, *, category: Optional[str], index_dir: Path) -> Dict[str, Any]:
    return {
        "score": float(score),
        "source_path": meta.get("source_path"),
        "source_file": meta.get("source_file"),
        "book_id": meta.get("book_id"),
        "book_title": meta.get("book_title"),
        "chunk_index": meta.get("chunk_index"),
        "n_words": meta.get("n_words"),
        "n_units": meta.get("n_units"),
        "chunking_version": meta.get("chunking_version"),
        "text": meta.get("text"),
        "chunk_id": meta.get("chunk_id"),
        "category": category,
        "_index_dir": str(index_dir),
    }


def _retrieve_from_index(
    *,
    index_dir: Path,
    query: str,
    retrieve_k: int,
    embedding_model: str,
    device: Optional[str],
    multi_route: bool,
    rewrite: str,
    rewrite_base_url: Optional[str],
    rewrite_api_key: Optional[str],
    rewrite_model: Optional[str],
    rewrite_max_out: int,
    rewrite_timeout_s: float,
    category: Optional[str] = None,
) -> List[Dict[str, Any]]:
    if not index_dir.exists():
        return []
    # Empty placeholder stores: still loadable, size==0 → no hits
    try:
        st = _get_store(index_dir)
    except Exception:  # noqa: BLE001
        return []
    if st.size == 0:
        return []

    if not multi_route:
        em = _get_embedder(embedding_model, device)
        q_emb = _embed_query_cached(em, query, embedding_model)
        raw_hits = st.search(q_emb, top_k=retrieve_k)
        return [_hit_from_meta(meta, score, category=category, index_dir=index_dir) for meta, score in raw_hits]

    mr = _build_cached_multiroute(
        index_dir=index_dir,
        embedding_model=embedding_model,
        device=device,
        rewrite=rewrite,
        rewrite_base_url=rewrite_base_url,
        rewrite_api_key=rewrite_api_key,
        rewrite_model=rewrite_model,
        rewrite_max_out=int(rewrite_max_out),
        rewrite_timeout_s=float(rewrite_timeout_s),
    )
    return [
        _hit_from_meta(h.meta, h.score, category=category, index_dir=index_dir)
        for h in mr.retrieve(query, top_k=retrieve_k)
    ]


def _merge_hits_by_score(hits: List[Dict[str, Any]], *, top_k: int) -> List[Dict[str, Any]]:
    """Deduplicate by chunk_id (keep best score) then take top_k."""
    best: Dict[str, Dict[str, Any]] = {}
    orphans: List[Dict[str, Any]] = []
    for h in hits:
        cid = h.get("chunk_id")
        if not isinstance(cid, str) or not cid:
            orphans.append(h)
            continue
        prev = best.get(cid)
        if prev is None or float(h.get("score") or 0) > float(prev.get("score") or 0):
            best[cid] = h
    merged = list(best.values()) + orphans
    merged.sort(key=lambda x: float(x.get("score") or 0), reverse=True)
    return merged[: max(int(top_k), 0)]


def _expand_neighbors_multi(hits: List[Dict[str, Any]], *, neighbor_n: int) -> List[dict]:
    by_dir: Dict[str, List[Dict[str, Any]]] = {}
    for h in hits:
        d = str(h.get("_index_dir") or "")
        by_dir.setdefault(d, []).append(h)
    contexts: List[dict] = []
    for d, group in by_dir.items():
        if not d:
            continue
        store = _get_store(Path(d))
        src_idx = _get_source_index(Path(d))
        contexts.extend(
            build_neighbor_contexts(
                metas=store._meta,  # noqa: SLF001
                hits=group,
                neighbor_n=int(neighbor_n),
                _source_index=src_idx,
            )
        )
    return contexts


@_with_rag_limit
def rag_search_tool(
    *,
    query: str,
    top_k: int = 5,
    index_dir: Optional[str] = None,
    category: Optional[Union[str, Sequence[str]]] = None,
    embedding_model: str = "intfloat/multilingual-e5-small",
    device: Optional[str] = _RAG_DEVICE,
    multi_route: bool = False,
    rewrite: str = "template",
    rewrite_base_url: Optional[str] = None,
    rewrite_api_key: Optional[str] = None,
    rewrite_model: Optional[str] = None,
    rewrite_max_out: int = 5,
    rewrite_timeout_s: float = 60.0,
    rerank: bool = False,
    rerank_model: str = "BAAI/bge-reranker-large",
    rerank_candidates: int = 10,
    rerank_batch_size: int = 32,
    rerank_keep_topn: int = 0,
    rerank_filter_overlap: float = 0.15,
    expand_neighbors: int = 1,
    per_text_max_chars: int = 5000,
    include_hits_text: bool = True,
    include_contexts_text: bool = True,
) -> Dict[str, Any]:
    query = require_english_rag_query(query)
    repo_root = _repo_root()
    cfg0 = default_config(repo_root)
    resolved_emb = resolve_embedding_model_id(embedding_model, repo_root)
    resolved_rr = resolve_rerank_model_id(rerank_model, repo_root)

    cat_dirs = resolve_category_index_dirs(repo_root=repo_root, category=category)
    # Explicit index_dir wins over category when both provided without category dirs
    if cat_dirs:
        search_targets: List[tuple[Optional[str], Path]] = []
        # recover category id from dir name
        for d in cat_dirs:
            search_targets.append((d.name, d))
        primary_index = cat_dirs[0]
    else:
        primary_index = Path(index_dir) if index_dir else cfg0.index_dir
        search_targets = [(None, primary_index)]

    cfg = RagConfig(
        raw_dir=cfg0.raw_dir,
        index_dir=primary_index,
        embedding_model=resolved_emb,
        chunk_words=cfg0.chunk_words,
        chunk_overlap_words=cfg0.chunk_overlap_words,
        min_chunk_words=cfg0.min_chunk_words,
    )

    retrieve_k = int(top_k)
    if rerank:
        retrieve_k = max(retrieve_k, int(rerank_candidates))
    # When merging multiple category stores, over-fetch per store then merge
    per_store_k = retrieve_k if len(search_targets) == 1 else max(retrieve_k, int(top_k))

    hits: List[Dict[str, Any]] = []
    for cat_id, idir in search_targets:
        hits.extend(
            _retrieve_from_index(
                index_dir=idir,
                query=query,
                retrieve_k=per_store_k,
                embedding_model=cfg.embedding_model,
                device=device,
                multi_route=multi_route,
                rewrite=rewrite,
                rewrite_base_url=rewrite_base_url,
                rewrite_api_key=rewrite_api_key,
                rewrite_model=rewrite_model,
                rewrite_max_out=int(rewrite_max_out),
                rewrite_timeout_s=float(rewrite_timeout_s),
                category=cat_id,
            )
        )
    hits = _merge_hits_by_score(hits, top_k=retrieve_k)

    _rerank_skip_thr = float(os.getenv("RAG_RERANK_SKIP_THRESHOLD", "0.85"))
    dense_top_score = hits[0].get("score", 0.0) if hits else 0.0
    should_rerank = rerank and hits and dense_top_score < _rerank_skip_thr

    if should_rerank:
        # Pre-filter: remove bottom 25% of candidates to reduce reranker workload
        if len(hits) > 4:
            cutoff = max(int(len(hits) * 0.75), int(top_k))
            hits_sorted = sorted(hits, key=lambda h: h.get("score", 0), reverse=True)
            hits = hits_sorted[:cutoff]

        passages = [(h.get("text") or "").strip() for h in hits]
        rr = _get_reranker(resolved_rr, device)
        order = rr.rerank(query=query, passages=passages, top_k=int(top_k), batch_size=int(rerank_batch_size))
        new_hits: List[dict] = []
        for r in order:
            h = dict(hits[int(r.index)])
            h["score_retrieval"] = float(h.get("score") or 0.0)
            h["score"] = float(r.score)
            h["score_rerank"] = float(r.score)
            new_hits.append(h)
        hits = new_hits

        thr = float(rerank_filter_overlap or 0.0)
        if thr > 0.0:
            kept = []
            for h in hits:
                ov = overlap_score(query, (h.get("text") or ""))
                hh = dict(h)
                hh["overlap"] = float(ov)
                if ov >= thr:
                    kept.append(hh)
            if kept:
                hits = kept

        topn = int(rerank_keep_topn or 0)
        if topn > 0 and len(hits) > topn:
            hits = hits[:topn]
    else:
        hits = hits[: int(top_k)]

    contexts: List[dict] = []
    if int(expand_neighbors) > 0 and hits:
        contexts = _expand_neighbors_multi(hits, neighbor_n=int(expand_neighbors))

    def _clip(s: str) -> str:
        s = (s or "").strip()
        if per_text_max_chars > 0 and len(s) > per_text_max_chars:
            return s[:per_text_max_chars] + "\n...(truncated)..."
        return s

    # Strip internal field before return
    for h in hits:
        h.pop("_index_dir", None)

    if not include_hits_text:
        for h in hits:
            h.pop("text", None)
    else:
        for h in hits:
            if "text" in h:
                h["text"] = _clip(h["text"])

    if not include_contexts_text:
        for c in contexts:
            c.pop("text", None)
    else:
        for c in contexts:
            if "text" in c:
                c["text"] = _clip(c["text"])

    return {
        "query": query,
        "params": {
            "top_k": int(top_k),
            "multi_route": bool(multi_route),
            "rewrite": rewrite,
            "rewrite_model": rewrite_model,
            "rewrite_max_out": int(rewrite_max_out),
            "rerank": bool(rerank),
            "expand_neighbors": int(expand_neighbors),
            "index_dir": str(cfg.index_dir),
            "category": list(category) if isinstance(category, (list, tuple)) else category,
            "embedding_model": cfg.embedding_model,
        },
        "hits": hits,
        "contexts": contexts,
    }


@_with_rag_limit
def rag_reindex_tool(
    *,
    raw_dir: Optional[str] = None,
    index_dir: Optional[str] = None,
    embedding_model: str = "intfloat/multilingual-e5-small",
    batch_size: int = 32,
    limit_books: Optional[int] = None,
    device: Optional[str] = _RAG_DEVICE,
) -> Dict[str, Any]:
    repo_root = _repo_root()
    cfg0 = default_config(repo_root)
    resolved_emb = resolve_embedding_model_id(embedding_model, repo_root)
    cfg = RagConfig(
        raw_dir=Path(raw_dir) if raw_dir else cfg0.raw_dir,
        index_dir=Path(index_dir) if index_dir else cfg0.index_dir,
        embedding_model=resolved_emb,
        chunk_words=cfg0.chunk_words,
        chunk_overlap_words=cfg0.chunk_overlap_words,
        min_chunk_words=cfg0.min_chunk_words,
    )
    out = build_or_update_index(cfg, limit_books=limit_books, batch_size=int(batch_size), device=device)
    _invalidate_index_cache(cfg.index_dir)
    return out
