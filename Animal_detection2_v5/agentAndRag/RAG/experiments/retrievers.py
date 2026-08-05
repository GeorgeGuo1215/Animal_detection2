from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from simple_rag.embeddings import Embedder
from simple_rag.query_rewrite import NoRewrite
from simple_rag.retrieval import build_default_multiroute
from simple_rag.vector_store import NumpyVectorStore

from common import EN_STOP, book_id_from_source_path


def _tokenize(text: str) -> List[str]:
    toks = re.findall(r"[a-z][a-z0-9\-]{2,}", (text or "").lower())
    return [t for t in toks if t not in EN_STOP]


@dataclass
class Hit:
    chunk_id: str
    score: float
    book_id: str


class DenseRetriever:
    def __init__(self, index_dir: Path, embedding_model: str, device: Optional[str] = None) -> None:
        self.store = NumpyVectorStore(index_dir)
        self.store.load()
        self.embedder = Embedder(embedding_model, device=device)

    def retrieve(self, query: str, top_k: int = 5) -> List[Hit]:
        q = self.embedder.embed_queries([query], batch_size=1, normalize=True).vectors[0]
        hits = self.store.search(q, top_k=top_k)
        out: List[Hit] = []
        for meta, score in hits:
            out.append(
                Hit(
                    chunk_id=str(meta.get("chunk_id")),
                    score=float(score),
                    book_id=book_id_from_source_path(str(meta.get("source_path"))),
                )
            )
        return out


class BM25Retriever:
    """
    轻量 BM25（不引入额外依赖）。
    用于当作 sparse baseline 或做 hybrid。
    """

    def __init__(self, metas: List[dict], k1: float = 1.5, b: float = 0.75) -> None:
        self.metas = metas
        self.k1 = float(k1)
        self.b = float(b)

        # 建倒排
        self.doc_len: List[int] = []
        self.avgdl = 0.0
        self.df: Dict[str, int] = {}
        self.tf: List[Dict[str, int]] = []

        for m in metas:
            toks = _tokenize(m.get("text", ""))
            tf: Dict[str, int] = {}
            for t in toks:
                tf[t] = tf.get(t, 0) + 1
            self.tf.append(tf)
            dl = sum(tf.values())
            self.doc_len.append(dl)
            for t in tf.keys():
                self.df[t] = self.df.get(t, 0) + 1

        self.N = len(metas)
        self.avgdl = (sum(self.doc_len) / self.N) if self.N else 0.0

    def _idf(self, term: str) -> float:
        df = self.df.get(term, 0)
        # BM25+ 常见 idf
        return math.log(1.0 + (self.N - df + 0.5) / (df + 0.5))

    def retrieve(self, query: str, top_k: int = 5) -> List[Hit]:
        q_terms = _tokenize(query)
        if not q_terms or self.N == 0:
            return []

        scores = np.zeros((self.N,), dtype=np.float32)
        for t in q_terms:
            idf = self._idf(t)
            for i, tf in enumerate(self.tf):
                f = tf.get(t, 0)
                if f <= 0:
                    continue
                dl = self.doc_len[i]
                denom = f + self.k1 * (1.0 - self.b + self.b * (dl / (self.avgdl + 1e-9)))
                scores[i] += idf * (f * (self.k1 + 1.0) / (denom + 1e-9))

        k = min(int(top_k), self.N)
        idx = np.argpartition(-scores, kth=k - 1)[:k]
        idx = idx[np.argsort(-scores[idx])]

        out: List[Hit] = []
        for i in idx:
            m = self.metas[int(i)]
            out.append(
                Hit(
                    chunk_id=str(m.get("chunk_id")),
                    score=float(scores[int(i)]),
                    book_id=book_id_from_source_path(str(m.get("source_path"))),
                )
            )
        return out


class HybridRetriever:
    """
    Experiments 侧的 hybrid：直接复用 simple_rag 的 MultiRouteRetriever（dense + bm25 + RRF 融合）。

    - 不再做 “w * dense + (1-w) * bm25_norm” 的 score-level 加权
    - 改为 rank-level 的 RRF 融合（更稳，不依赖分数尺度）
    """

    def __init__(
        self,
        index_dir: Path,
        embedding_model: str,
        device: Optional[str] = None,
        *,
        top_k_per_route: int = 20,
        fusion: str = "rrf",
        rrf_k: int = 60,
    ) -> None:
        self.mr = build_default_multiroute(
            index_dir=str(index_dir),
            embedding_model=str(embedding_model),
            device=device,
            enable_bm25=True,
            rewriter=NoRewrite(),
        )
        self.mr.fusion = str(fusion)
        self.mr.rrf_k = int(rrf_k)
        self.mr.top_k_per_route = int(top_k_per_route)

    def retrieve(self, query: str, top_k: int = 5) -> List[Hit]:
        out: List[Hit] = []
        for h in self.mr.retrieve(query, top_k=int(top_k)):
            sp = str((h.meta or {}).get("source_path") or "")
            out.append(
                Hit(
                    chunk_id=str(h.chunk_id),
                    score=float(h.score),
                    book_id=book_id_from_source_path(sp),
                )
            )
        return out


class TwoStageBookThenChunk:
    """
    多层级检索（最小可用版）：
    1) Stage1：Dense 全库检索 top_k_books * m 个 chunk，然后投票得到 top_k_books 个书
    2) Stage2：在这些书的 chunk 子集里再做一次 Dense 检索（过滤 meta）

    注意：这是论文里的“层级检索”基线，实现简单但能体现 isolation 的价值。
    """

    def __init__(self, index_dir: Path, embedding_model: str, device: Optional[str] = None) -> None:
        self.store = NumpyVectorStore(index_dir)
        self.store.load()
        self.embedder = Embedder(embedding_model, device=device)

        # 预建 book_id -> 行号列表，便于过滤子库
        self.book_to_rows: Dict[str, List[int]] = {}
        for i, m in enumerate(self.store._meta):  # noqa: SLF001（论文实验工具，允许）
            bid = book_id_from_source_path(str(m.get("source_path")))
            self.book_to_rows.setdefault(bid, []).append(i)

    def retrieve(self, query: str, top_k: int = 5, top_k_books: int = 3) -> List[Hit]:
        q = self.embedder.embed_queries([query], batch_size=1, normalize=True).vectors[0]
        # Stage1：先取更多候选
        stage1 = self.store.search(q, top_k=top_k_books * 30)
        # 投票 book
        vote: Dict[str, int] = {}
        for meta, _score in stage1:
            bid = book_id_from_source_path(str(meta.get("source_path")))
            vote[bid] = vote.get(bid, 0) + 1
        books = sorted(vote.items(), key=lambda kv: kv[1], reverse=True)[: int(top_k_books)]
        book_set = {b for b, _ in books}

        # Stage2：过滤到子库再算 dot（numpy）
        emb = self.store._emb  # noqa: SLF001
        meta = self.store._meta  # noqa: SLF001
        rows: List[int] = []
        for bid in book_set:
            rows.extend(self.book_to_rows.get(bid, []))
        if not rows:
            return []

        sub = emb[rows, :]
        scores = sub @ q
        k = min(int(top_k), len(rows))
        idx = np.argpartition(-scores, kth=k - 1)[:k]
        idx = idx[np.argsort(-scores[idx])]

        out: List[Hit] = []
        for j in idx:
            row = rows[int(j)]
            m = meta[row]
            out.append(
                Hit(
                    chunk_id=str(m.get("chunk_id")),
                    score=float(scores[int(j)]),
                    book_id=book_id_from_source_path(str(m.get("source_path"))),
                )
            )
        return out


