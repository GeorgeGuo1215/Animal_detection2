from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Protocol, Tuple

import numpy as np
from pathlib import Path

from .embeddings import Embedder
from .vector_store import NumpyVectorStore
from .query_rewrite import NoRewrite, QueryRewriter


@dataclass(frozen=True)
class RetrievedChunk:
    chunk_id: str
    score: float
    meta: dict


class Retriever(Protocol):
    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]: ...


class DenseRetriever:
    def __init__(self, *, index_dir: str, embedding_model: str, device: Optional[str] = None) -> None:
        self.store = NumpyVectorStore(Path(index_dir))
        self.store.load()
        self.embedder = Embedder(embedding_model, device=device)

    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]:
        q = (query or "").strip()
        if not q:
            return []
        q_emb = self.embedder.embed_queries([q], batch_size=1, normalize=True).vectors[0]
        hits = self.store.search(q_emb, top_k=int(top_k))
        return [RetrievedChunk(chunk_id=str(m.get("chunk_id")), score=float(s), meta=m) for m, s in hits]


EN_STOP = {
    "the", "a", "an", "and", "or", "to", "of", "in", "on", "for", "with", "as",
    "is", "are", "was", "were", "be", "by", "that", "this", "it", "from", "at",
    "into", "during", "within", "without", "over", "under", "between",
}

_CJK_RANGES = (
    "\u4e00-\u9fff"   # CJK Unified Ideographs
    "\u3400-\u4dbf"   # CJK Extension A
    "\uf900-\ufaff"   # CJK Compatibility Ideographs
)
_RE_CJK_CHAR = re.compile(f"[{_CJK_RANGES}]")
_RE_EN_WORD = re.compile(r"[a-z][a-z0-9\-]{2,}")


try:
    import jieba as _jieba
    _HAS_JIEBA = True
except ImportError:
    _jieba = None  # type: ignore[assignment]
    _HAS_JIEBA = False


def _tokenize(text: str) -> List[str]:
    """Tokenize mixed Chinese-English text for BM25.

    English: standard lowercase words (3+ chars), stop-words removed.
    Chinese: jieba cut_for_search (full words + sub-words) when available,
    falls back to overlapping bigrams otherwise.
    """
    s = (text or "").lower()
    tokens: List[str] = []

    en_toks = _RE_EN_WORD.findall(s)
    tokens.extend(t for t in en_toks if t not in EN_STOP)

    if _HAS_JIEBA:
        for w in _jieba.cut_for_search(s):
            w = w.strip()
            if len(w) >= 2 and _RE_CJK_CHAR.search(w):
                tokens.append(w)
    else:
        cjk_chars = _RE_CJK_CHAR.findall(s)
        if len(cjk_chars) >= 2:
            for i in range(len(cjk_chars) - 1):
                tokens.append(cjk_chars[i] + cjk_chars[i + 1])
        elif len(cjk_chars) == 1:
            tokens.append(cjk_chars[0])

    return tokens


class BM25Retriever:
    """
    轻量 BM25（不引入额外依赖），使用倒排索引加速检索。
    """

    def __init__(self, *, metas: List[dict], k1: float = 1.5, b: float = 0.75) -> None:
        self.metas = metas
        self.k1 = float(k1)
        self.b = float(b)
        self.N = len(metas)

        self.doc_len = np.empty(self.N, dtype=np.float32)
        self.df: Dict[str, int] = {}
        # term -> (doc_indices_array, term_freq_array)
        self._inverted: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}

        term_postings: Dict[str, List[Tuple[int, int]]] = {}
        for i, m in enumerate(metas):
            toks = _tokenize(m.get("text", ""))
            tf: Dict[str, int] = {}
            for t in toks:
                tf[t] = tf.get(t, 0) + 1
            self.doc_len[i] = float(sum(tf.values()))
            for t, f in tf.items():
                self.df[t] = self.df.get(t, 0) + 1
                term_postings.setdefault(t, []).append((i, f))

        self.avgdl = float(self.doc_len.mean()) if self.N else 0.0

        for t, postings in term_postings.items():
            ids = np.array([p[0] for p in postings], dtype=np.int32)
            freqs = np.array([p[1] for p in postings], dtype=np.float32)
            self._inverted[t] = (ids, freqs)

    def _idf(self, term: str) -> float:
        df = self.df.get(term, 0)
        return math.log(1.0 + (self.N - df + 0.5) / (df + 0.5))

    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]:
        q_terms = _tokenize(query)
        if not q_terms or self.N == 0:
            return []

        scores = np.zeros(self.N, dtype=np.float32)
        for t in q_terms:
            posting = self._inverted.get(t)
            if posting is None:
                continue
            ids, freqs = posting
            idf = self._idf(t)
            dl = self.doc_len[ids]
            denom = freqs + self.k1 * (1.0 - self.b + self.b * (dl / (self.avgdl + 1e-9)))
            scores[ids] += idf * (freqs * (self.k1 + 1.0) / (denom + 1e-9))

        k = min(int(top_k), self.N)
        idx = np.argpartition(-scores, kth=k - 1)[:k]
        idx = idx[np.argsort(-scores[idx])]
        out: List[RetrievedChunk] = []
        for i in idx:
            m = self.metas[int(i)]
            out.append(RetrievedChunk(chunk_id=str(m.get("chunk_id")), score=float(scores[int(i)]), meta=m))
        return out


@dataclass
class MultiRouteRetriever:
    """
    多路召回：
    - 支持 query rewrite（输出多个 query）
    - 支持多 retriever（dense/bm25/...）并做融合

    融合策略（简单稳妥）：
    - 每个 retriever 内：取 top_k_per_route
    - 默认使用 RRF（Reciprocal Rank Fusion）按 rank 融合，更稳且不受分数尺度影响
    """

    retrievers: List[Tuple[str, Retriever]]
    rewriter: QueryRewriter = NoRewrite()
    top_k_per_route: int = 20
    fusion: str = "rrf"  # "rrf" | "minmax"
    rrf_k: int = 60      # RRF 常用超参，越大越“平滑”

    def retrieve(self, query: str, *, top_k: int) -> List[RetrievedChunk]:
        queries = self.rewriter.rewrite(query)
        if not queries:
            return []

        # chunk_id -> (meta, score_sum)
        merged: Dict[str, Tuple[dict, float]] = {}

        for q in queries:
            for name, r in self.retrievers:
                hits = r.retrieve(q, top_k=int(self.top_k_per_route))
                if not hits:
                    continue

                if self.fusion == "rrf":
                    # RRF：score += 1 / (rrf_k + rank)
                    for rank, h in enumerate(hits, start=1):
                        add = 1.0 / float(int(self.rrf_k) + rank)
                        old = merged.get(h.chunk_id)
                        if old is None:
                            merged[h.chunk_id] = (h.meta, add)
                        else:
                            merged[h.chunk_id] = (old[0], old[1] + add)
                elif self.fusion == "minmax":
                    # min-max 归一化到 [0,1] 再融合，避免不同 retriever 分数尺度差太多
                    scs = [h.score for h in hits]
                    mn, mx = min(scs), max(scs)
                    for h in hits:
                        norm = 0.0 if mx <= mn else (h.score - mn) / (mx - mn)
                        old = merged.get(h.chunk_id)
                        if old is None:
                            merged[h.chunk_id] = (h.meta, float(norm))
                        else:
                            merged[h.chunk_id] = (old[0], old[1] + float(norm))
                else:
                    raise ValueError(f"未知 fusion: {self.fusion}（可选 rrf/minmax）")

        out = [RetrievedChunk(chunk_id=cid, meta=meta, score=score) for cid, (meta, score) in merged.items()]
        out.sort(key=lambda x: x.score, reverse=True)
        return out[: int(top_k)]


def build_default_multiroute(
    *,
    index_dir: str,
    embedding_model: str,
    device: Optional[str] = None,
    enable_bm25: bool = True,
    rewriter: Optional[QueryRewriter] = None,
) -> MultiRouteRetriever:
    """
    便捷工厂：dense + (可选) bm25。
    """
    store = NumpyVectorStore(Path(index_dir))
    store.load()
    metas = store._meta  # noqa: SLF001

    routes: List[Tuple[str, Retriever]] = []
    routes.append(("dense", DenseRetriever(index_dir=index_dir, embedding_model=embedding_model, device=device)))
    if enable_bm25:
        routes.append(("bm25", BM25Retriever(metas=metas)))

    return MultiRouteRetriever(
        retrievers=routes,
        rewriter=rewriter or NoRewrite(),
        top_k_per_route=20,
    )


