from __future__ import annotations

import hashlib
import logging
import os
from collections import OrderedDict
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Dict, Hashable, List, Optional, Sequence, TypeVar, Union
import threading

from ..concurrency import get_resource_limits

_RAG_DEVICE: Optional[str] = os.getenv("AGENT_WARMUP_DEVICE") or None

from RAG.simple_rag.category_index import resolve_category_index_dirs, resolve_default_category_index_dirs, merged_search_scope
from RAG.simple_rag.config import RagConfig, default_config
from RAG.simple_rag.context_utils import build_neighbor_contexts, build_source_index
from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.pipeline import build_or_update_index
from RAG.simple_rag.query_rewrite import LLMRewriter, NoRewrite, TemplateRewriter
from RAG.simple_rag.retrieval import BM25Retriever, MultiRouteRetriever, RetrievedChunk
from RAG.simple_rag.reranker import CrossEncoderReranker
from RAG.simple_rag.vector_store import NumpyVectorStore

from RAG.simple_rag.scoring import overlap_score

from ..hf_local_model import resolve_embedding_model_id, resolve_rerank_model_id
from .rag_query import require_english_rag_query

logger = logging.getLogger(__name__)

_T = TypeVar("_T")

# 全局锁只保护缓存字典本身；真正的索引/模型加载在 per-key 锁下进行，
# 否则一次冷加载（整库 npy + 模型权重）会让所有 RAG 查询连缓存读取都被卡住。
_LOCK = threading.RLock()
_LOAD_LOCKS: Dict[Hashable, threading.Lock] = {}
_STORE_CACHE: Dict[str, NumpyVectorStore] = {}
_EMBEDDER_CACHE: Dict[tuple[str, Optional[str]], Embedder] = {}
_BM25_CACHE: Dict[str, BM25Retriever] = {}
_RERANKER_CACHE: Dict[tuple[str, Optional[str]], CrossEncoderReranker] = {}
_SOURCE_INDEX_CACHE: Dict[str, Dict[str, Dict[int, str]]] = {}

_QUERY_EMB_CACHE_SIZE = 128
_QUERY_EMB_CACHE: OrderedDict[str, Any] = OrderedDict()


def _query_emb_key(query: str, model: str) -> str:
    """查询向量缓存键。"""
    return hashlib.md5(f"{model}||{query}".encode()).hexdigest()


def _cached_load(cache: Dict[Any, _T], key: Hashable, loader: Callable[[], _T]) -> _T:
    """双检缓存：快速路径只读字典；未命中时按 key 串行加载，不阻塞其它 key。"""
    with _LOCK:
        cached = cache.get(key)
        if cached is not None:
            return cached
        load_lock = _LOAD_LOCKS.setdefault(("load", id(cache), key), threading.Lock())
    with load_lock:
        with _LOCK:
            cached = cache.get(key)
            if cached is not None:
                return cached
        loaded = loader()
        with _LOCK:
            cache[key] = loaded
        return loaded


def _get_store(index_dir: Path) -> NumpyVectorStore:
    """加载或缓存向量库。"""
    key = str(index_dir.resolve())

    def _load() -> NumpyVectorStore:
        st = NumpyVectorStore(index_dir)
        st.load()
        return st

    return _cached_load(_STORE_CACHE, key, _load)


def _get_embedder(embedding_model: str, device: Optional[str]) -> Embedder:
    """加载或缓存 Embedder。"""
    resolved = resolve_embedding_model_id(embedding_model, _repo_root())
    key = (resolved, device)
    return _cached_load(_EMBEDDER_CACHE, key, lambda: Embedder(resolved, device=device))


def _get_bm25(index_dir: Path) -> BM25Retriever:
    """加载或缓存 BM25 检索器。"""
    key = str(index_dir.resolve())
    return _cached_load(
        _BM25_CACHE, key,
        lambda: BM25Retriever(metas=_get_store(index_dir)._meta),  # noqa: SLF001
    )


def _get_reranker(rerank_model: str, device: Optional[str]) -> CrossEncoderReranker:
    """加载或缓存 reranker。"""
    resolved = resolve_rerank_model_id(rerank_model, _repo_root())
    key = (resolved, device)
    return _cached_load(_RERANKER_CACHE, key, lambda: CrossEncoderReranker(resolved, device=device))


def _get_source_index(index_dir: Path) -> Dict[str, Dict[int, str]]:
    """加载或缓存来源索引。"""
    key = str(index_dir.resolve())
    return _cached_load(
        _SOURCE_INDEX_CACHE, key,
        lambda: build_source_index(_get_store(index_dir)._meta),  # noqa: SLF001
    )


def _embed_query_cached(embedder: Embedder, query: str, model_name: str):
    """带 LRU 的查询向量缓存，避免重复计算相同查询。"""
    ck = _query_emb_key(query, model_name)
    with _LOCK:
        if ck in _QUERY_EMB_CACHE:
            _QUERY_EMB_CACHE.move_to_end(ck)
            return _QUERY_EMB_CACHE[ck]
    tokenizer = getattr(getattr(embedder, "model", None), "tokenizer", None)
    if tokenizer is not None and len(tokenizer("query: " + query, add_special_tokens=True)["input_ids"]) > 256:
        raise ValueError("rag.search query exceeds 256 embedding tokens")
    vec = embedder.embed_queries([query], batch_size=1, normalize=True).vectors[0]
    with _LOCK:
        _QUERY_EMB_CACHE[ck] = vec
        if len(_QUERY_EMB_CACHE) > _QUERY_EMB_CACHE_SIZE:
            _QUERY_EMB_CACHE.popitem(last=False)
    return vec


def _invalidate_index_cache(index_dir: Path) -> None:
    """使指定索引目录的缓存失效。"""
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
    """预热指定索引的向量库、embedding 与可选 reranker。"""
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
    """定位 agentAndRag 仓库根目录。"""
    return Path(__file__).resolve().parents[3]


def _with_rag_limit(handler):
    """包装 handler，使其占用 RAG 并发槽位。"""

    @wraps(handler)
    def _wrapped(*args: Any, **kwargs: Any) -> Dict[str, Any]:
        """在 RAG 槽位内调用原 handler。"""
        limits = get_resource_limits()
        with limits.rag.slot(timeout_s=limits.acquire_timeout_s):
            return handler(*args, **kwargs)

    return _wrapped


class _LimitedSyncRewriter:
    """在 LLM 槽位内执行查询改写的同步包装器。"""
    def __init__(self, delegate: Any) -> None:
        """绑定被包装的 rewriter。"""
        self.delegate = delegate

    def rewrite(self, query: str) -> List[str]:
        """占用 LLM 槽位后改写查询。"""
        limits = get_resource_limits()
        with limits.llm.sync_slot(timeout_s=limits.acquire_timeout_s):
            return self.delegate.rewrite(query)


class _CachedDenseRetriever:
    """使用缓存查询向量的稠密检索器。"""
    def __init__(self, *, store: NumpyVectorStore, embedder: Embedder, categories: tuple[str, ...] | None = None) -> None:
        """绑定向量库与 Embedder。"""
        self.store = store
        self.embedder = embedder
        self.categories = categories

    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]:
        """对查询做稠密检索。"""
        q = (query or "").strip()
        if not q:
            return []
        q_emb = _embed_query_cached(self.embedder, q, self.embedder.model_name_or_path)
        hits = self.store.search(q_emb, top_k=int(top_k), **({"categories": self.categories} if self.categories is not None else {}))
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
    categories: tuple[str, ...] | None = None,
    retrieve_k: int = 20,
) -> MultiRouteRetriever:
    """构建带缓存的多路检索器。"""
    st = _get_store(index_dir)
    em = _get_embedder(embedding_model, device)
    dense = _CachedDenseRetriever(store=st, embedder=em, categories=categories)
    bm25 = _get_bm25(index_dir)
    if categories is not None:
        base_bm25 = bm25
        rows = st.rows_for_categories(categories)
        class ScopedBM25:
            def retrieve(self, query, *, top_k):
                return base_bm25.retrieve(query, top_k=top_k, rows=rows)
        bm25 = ScopedBM25()
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
    return MultiRouteRetriever(retrievers=[("dense", dense), ("bm25", bm25)], rewriter=rewriter, top_k_per_route=retrieve_k)


def _hit_from_meta(meta: dict, score: float, *, category: Optional[str], index_dir: Path) -> Dict[str, Any]:
    """从 meta 构造命中字典。"""
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
        **{key: meta[key] for key in ("category_ids", "source_version", "source_spans", "duplicate_source_spans", "page_sequence_start", "page_sequence_end", "page_sequence_kind", "section_path", "content_type", "content_hash", "token_count") if key in meta},
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
    categories: tuple[str, ...] | None = None,
) -> List[Dict[str, Any]]:
    """在单个分类索引上执行稠密或多路检索，并规范化命中元数据。

    缺失、空或不可加载的索引按无命中处理；稠密路径复用缓存向量，多路路径可使用
    查询改写与混合召回。返回项附带 category 与实际 index_dir，供跨索引合并审计。
    """
    if not index_dir.exists():
        return []
    # Empty placeholder stores: still loadable, size==0 → no hits
    try:
        st = _get_store(index_dir)
    except FileNotFoundError:
        logger.warning("rag index missing files, treated as empty index_dir=%s", index_dir)
        return []
    except Exception:  # noqa: BLE001
        # 维度不匹配、meta 损坏等不是"无命中"，静默吞掉会让上层误判为证据不足而走 Web 兜底。
        logger.exception("rag index failed to load, treated as empty index_dir=%s", index_dir)
        return []
    if st.size == 0:
        return []

    if not multi_route:
        em = _get_embedder(embedding_model, device)
        q_emb = _embed_query_cached(em, query, embedding_model)
        raw_hits = st.search(q_emb, top_k=retrieve_k, **({"categories": categories} if categories is not None else {}))
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
        categories=categories,
        retrieve_k=retrieve_k,
    )
    return [
        _hit_from_meta(h.meta, h.score, category=category, index_dir=index_dir)
        for h in mr.retrieve(query, top_k=retrieve_k)
    ]


def _merge_hits_by_score(hits: List[Dict[str, Any]], *, top_k: int) -> List[Dict[str, Any]]:
    """按分数合并多索引命中。"""
    best: Dict[str, Dict[str, Any]] = {}
    orphans: List[Dict[str, Any]] = []
    for h in hits:
        cid = h.get("content_hash") or h.get("chunk_id")
        if not isinstance(cid, str) or not cid:
            orphans.append(h)
            continue
        prev = best.get(cid)
        if prev is not None:
            provenance = prev.get("provenance") or [{k: prev.get(k) for k in ("chunk_id", "book_id", "source_path", "source_version", "source_spans")}]
            provenance = [*provenance, {k: h.get(k) for k in ("chunk_id", "book_id", "source_path", "source_version", "source_spans")}]
            if float(h.get("score") or 0) > float(prev.get("score") or 0):
                h = {**h, "provenance": provenance}
            else:
                prev["provenance"] = provenance
        if prev is None or float(h.get("score") or 0) > float(prev.get("score") or 0):
            best[cid] = h
    merged = list(best.values()) + orphans
    merged.sort(key=lambda x: float(x.get("score") or 0), reverse=True)
    return merged[: max(int(top_k), 0)]


def _expand_neighbors_multi(hits: List[Dict[str, Any]], *, neighbor_n: int) -> List[dict]:
    """多索引邻接块扩展。"""
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
    rerank: bool = True,
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
    """执行 RAG 检索并返回 hits/contexts。"""
    query = require_english_rag_query(query)
    repo_root = _repo_root()
    cfg0 = default_config(repo_root)
    resolved_emb = resolve_embedding_model_id(embedding_model, repo_root)
    resolved_rr = resolve_rerank_model_id(rerank_model, repo_root)

    merged_scope = merged_search_scope(repo_root, category) if not index_dir else None
    cat_dirs = resolve_category_index_dirs(repo_root=repo_root, category=category) if merged_scope is None else []
    # Explicit index_dir wins over category when both provided without category dirs
    if merged_scope is not None:
        primary_index, allowed_categories = merged_scope
        search_targets = [(None, primary_index)]
    elif cat_dirs:
        search_targets: List[tuple[Optional[str], Path]] = []
        # recover category id from dir name
        for d in cat_dirs:
            search_targets.append((d.name, d))
        primary_index = cat_dirs[0]
    elif index_dir:
        primary_index = Path(index_dir)
        search_targets = [(None, primary_index)]
    else:
        cat_dirs = resolve_default_category_index_dirs(repo_root=repo_root)
        search_targets = [(d.name, d) for d in cat_dirs]
        primary_index = cat_dirs[0] if cat_dirs else cfg0.index_dir

    cfg = RagConfig(
        raw_dir=cfg0.raw_dir,
        index_dir=primary_index,
        embedding_model=resolved_emb,
        chunk_words=cfg0.chunk_words,
        chunk_overlap_words=cfg0.chunk_overlap_words,
        min_chunk_words=cfg0.min_chunk_words,
    )

    # 每个分类索引都按合并后的候选数取 top，再跨索引按分数合并。
    retrieve_k = int(top_k)
    if rerank:
        retrieve_k = max(retrieve_k, int(rerank_candidates))

    hits: List[Dict[str, Any]] = []
    for cat_id, idir in search_targets:
        hits.extend(
            _retrieve_from_index(
                index_dir=idir,
                query=query,
                retrieve_k=retrieve_k,
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
                **({"categories": allowed_categories} if merged_scope is not None else {}),
            )
        )
    hits = _merge_hits_by_score(hits, top_k=retrieve_k)

    # Dense E5 scores are not calibrated relevance probabilities and often
    # remain high for semantically wrong chunks.  When rerank=True, rerank by
    # default; deployments may opt back into a skip threshold explicitly.
    _rerank_skip_thr = float(os.getenv("RAG_RERANK_SKIP_THRESHOLD", "1.10"))
    dense_top_score = hits[0].get("score", 0.0) if hits else 0.0
    should_rerank = rerank and hits and dense_top_score < _rerank_skip_thr

    if should_rerank:
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
        """按字符上限截断文本。"""
        s = (s or "").strip()
        if per_text_max_chars > 0 and len(s) > per_text_max_chars:
            return s[:per_text_max_chars] + "\n...(truncated)..."
        return s

    # Strip internal field before return
    score_kind = "rerank" if should_rerank else "rrf" if multi_route else "dense"
    for h in hits:
        h.pop("_index_dir", None)
        h["score_kind"] = score_kind

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
        "score_kind": score_kind,
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
    """从原文重建向量索引。"""
    repo_root = _repo_root()
    cfg0 = default_config(repo_root)
    target = (Path(index_dir) if index_dir else cfg0.index_dir).resolve()
    managed = {path.resolve() for path in resolve_default_category_index_dirs(repo_root=repo_root)}
    if target in managed or target.is_relative_to((repo_root / "RAG/data/releases").resolve()):
        raise ValueError("managed production indexes require an audited immutable release; rag.reindex cannot modify them")
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
