"""
RAG 检索链路各环节耗时基准测试。

用法：
    cd agentAndRag
    python -m RAG.experiments.bench_rag_latency [--queries N] [--top-k K] [--device cpu]

输出每个环节的 平均耗时 / P50 / P95 / 最大值，方便定位瓶颈。
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO))

from RAG.simple_rag.config import default_config
from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.vector_store import NumpyVectorStore
from RAG.simple_rag.retrieval import BM25Retriever, MultiRouteRetriever
from RAG.simple_rag.reranker import CrossEncoderReranker
from RAG.simple_rag.query_rewrite import TemplateRewriter, NoRewrite
from RAG.simple_rag.context_utils import build_neighbor_contexts, build_source_index


SAMPLE_QUERIES = [
    "What are the symptoms of canine parvovirus?",
    "Treatment for feline chronic kidney disease",
    "Dose of metronidazole for dogs",
    "Differential diagnosis of vomiting in cats",
    "How to manage diabetes mellitus in dogs?",
    "What is the function of the liver in animals?",
    "Contraindications of NSAIDs in cats",
    "Nutritional management of pancreatitis in dogs",
    "Clinical signs of heartworm disease",
    "Emergency treatment for GDV in dogs",
]


def _percentile(data: List[float], p: float) -> float:
    s = sorted(data)
    k = (len(s) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    w = k - lo
    return s[lo] * (1 - w) + s[hi] * w


def _bench(label: str, fn: Callable[[], Any], n: int) -> Dict[str, Any]:
    times: List[float] = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    avg = statistics.mean(times) * 1000
    p50 = _percentile(times, 0.50) * 1000
    p95 = _percentile(times, 0.95) * 1000
    mx = max(times) * 1000
    return {"label": label, "n": n, "avg_ms": avg, "p50_ms": p50, "p95_ms": p95, "max_ms": mx}


def main() -> None:
    parser = argparse.ArgumentParser(description="RAG latency benchmark")
    parser.add_argument("--queries", type=int, default=10, help="Benchmark iterations per stage")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--rerank-candidates", type=int, default=10)
    parser.add_argument("--expand-neighbors", type=int, default=1)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--embedding-model", type=str, default="intfloat/multilingual-e5-small")
    parser.add_argument("--rerank-model", type=str, default="BAAI/bge-reranker-large")
    args = parser.parse_args()

    cfg = default_config(_REPO)
    n = args.queries
    results: List[Dict[str, Any]] = []

    # ── 1. Store Load ──
    print("[1/7] Loading vector store …")
    t0 = time.perf_counter()
    store = NumpyVectorStore(cfg.index_dir)
    store.load()
    load_ms = (time.perf_counter() - t0) * 1000
    results.append({"label": "store.load (cold)", "n": 1, "avg_ms": load_ms,
                     "p50_ms": load_ms, "p95_ms": load_ms, "max_ms": load_ms})
    metas = store._meta  # noqa: SLF001
    print(f"   index size = {store.size} chunks, dim = {store.config.dim}")

    # ── 2. Embedder Load ──
    print("[2/7] Loading embedder …")
    t0 = time.perf_counter()
    embedder = Embedder(args.embedding_model, device=args.device)
    load_ms = (time.perf_counter() - t0) * 1000
    results.append({"label": "embedder.load (cold)", "n": 1, "avg_ms": load_ms,
                     "p50_ms": load_ms, "p95_ms": load_ms, "max_ms": load_ms})

    # warmup: one dummy encode
    _ = embedder.embed_queries(["warmup"], batch_size=1, normalize=True)

    # ── 3. Query Embedding ──
    print(f"[3/7] Benchmarking query embedding × {n} …")
    queries = (SAMPLE_QUERIES * ((n // len(SAMPLE_QUERIES)) + 1))[:n]

    def _embed_one(q: str = queries[0]):
        return embedder.embed_queries([q], batch_size=1, normalize=True).vectors[0]

    results.append(_bench("embed_query (single)", lambda: _embed_one(queries[0]), n))

    # ── 4. Dense Vector Search ──
    print(f"[4/7] Benchmarking dense vector search × {n} …")
    q_vecs = [_embed_one(q) for q in queries]

    def _dense_search(i: int = 0):
        return store.search(q_vecs[i % len(q_vecs)], top_k=args.rerank_candidates)

    results.append(_bench("dense_search (numpy dot)", lambda: _dense_search(0), n))

    # ── 5. BM25 Retrieval ──
    print(f"[5/7] Benchmarking BM25 retrieval × {n} …")
    t0 = time.perf_counter()
    bm25 = BM25Retriever(metas=metas)
    bm25_build_ms = (time.perf_counter() - t0) * 1000
    results.append({"label": "bm25.build_index", "n": 1, "avg_ms": bm25_build_ms,
                     "p50_ms": bm25_build_ms, "p95_ms": bm25_build_ms, "max_ms": bm25_build_ms})

    results.append(_bench("bm25.retrieve", lambda: bm25.retrieve(queries[0], top_k=args.rerank_candidates), n))

    # ── 6. Reranker ──
    print(f"[6/7] Benchmarking reranker × {min(n, 5)} …")
    try:
        t0 = time.perf_counter()
        reranker = CrossEncoderReranker(args.rerank_model, device=args.device)
        rr_load_ms = (time.perf_counter() - t0) * 1000
        results.append({"label": "reranker.load (cold)", "n": 1, "avg_ms": rr_load_ms,
                         "p50_ms": rr_load_ms, "p95_ms": rr_load_ms, "max_ms": rr_load_ms})

        sample_hits = store.search(q_vecs[0], top_k=args.rerank_candidates)
        passages = [(m.get("text") or "") for m, _ in sample_hits]

        results.append(_bench(
            f"reranker.rerank ({len(passages)} passages)",
            lambda: reranker.rerank(query=queries[0], passages=passages, top_k=args.top_k),
            min(n, 5),
        ))
    except Exception as exc:
        print(f"   [SKIP] reranker: {exc}")

    # ── 7. Neighbor Context ──
    print(f"[7/8] Benchmarking neighbor context expansion (no cache) × {n} …")
    sample_hits_dict = [
        {"source_path": m.get("source_path"), "chunk_index": m.get("chunk_index"),
         "score": float(s), "text": m.get("text")}
        for m, s in store.search(q_vecs[0], top_k=args.top_k)
    ]

    results.append(_bench(
        f"neighbor_contexts (no index cache)",
        lambda: build_neighbor_contexts(metas=metas, hits=sample_hits_dict, neighbor_n=args.expand_neighbors),
        n,
    ))

    # ── 8. Neighbor Context with pre-built source index ──
    print(f"[8/8] Building source_index + benchmarking with cache × {n} …")
    t0 = time.perf_counter()
    src_idx = build_source_index(metas)
    si_build_ms = (time.perf_counter() - t0) * 1000
    results.append({"label": "source_index.build (one-time)", "n": 1,
                     "avg_ms": si_build_ms, "p50_ms": si_build_ms, "p95_ms": si_build_ms, "max_ms": si_build_ms})

    results.append(_bench(
        f"neighbor_contexts (with index cache)",
        lambda: build_neighbor_contexts(metas=metas, hits=sample_hits_dict,
                                        neighbor_n=args.expand_neighbors, _source_index=src_idx),
        n,
    ))

    # ── Summary ──
    print("\n" + "=" * 80)
    print(f"{'Stage':<42} {'Avg(ms)':>9} {'P50(ms)':>9} {'P95(ms)':>9} {'Max(ms)':>9}")
    print("-" * 80)
    for r in results:
        print(f"{r['label']:<42} {r['avg_ms']:>9.2f} {r['p50_ms']:>9.2f} {r['p95_ms']:>9.2f} {r['max_ms']:>9.2f}")
    print("=" * 80)

    total_hot = sum(r["avg_ms"] for r in results
                    if "cold" not in r["label"] and "build" not in r["label"] and "load" not in r["label"])
    print(f"\nEstimated hot-path total (embed+search+rerank+context): ~{total_hot:.1f} ms")


if __name__ == "__main__":
    main()
