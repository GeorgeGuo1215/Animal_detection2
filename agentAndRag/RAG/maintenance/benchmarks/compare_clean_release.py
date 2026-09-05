"""Compare immutable releases through the production retrieval entry point.

Measures end-to-end latency including the production resource semaphore. Runs
locally cached embedding/reranking models only; does not call LLM/Web APIs.
Legacy source snapshots may be supplied to measure the actual pre-change code.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import psutil

ROOT = Path(__file__).resolve().parents[3]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def noise(text: str) -> bool:
    """Independent, conservative visible-noise detector; not the retained flag."""
    return bool(
        re.search(r'all rights reserved|<\|[^>]+\|>|ISBN(?:-1[03])?\s*:', text, re.I)
        or len(re.findall(r'\.{5,}\s*\d+', text)) >= 2
        or (len(re.findall(r'\b(?:19|20)\d{2}[a-z]?\b', text)) >= 4
            and len(re.findall(r'\b(?:Journal|et al|Press|doi)\b', text, re.I)) >= 3)
        or re.search(r'([A-Za-z])\1{20,}', text)
        or (len(re.findall(r'[a-z]{3,}[A-Z][a-z]{2,}', text)) >= 10
            and len(re.findall(r'[a-z]{3,}[.!?](?:\s|$)', text)) < 2)
    )


def stats(values):
    return {'count': len(values), 'p50_ms': float(np.percentile(values, 50)),
            'p95_ms': float(np.percentile(values, 95)), 'max_ms': max(values)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--taxonomy', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--legacy-sources')
    parser.add_argument('--mode', choices=('dense', 'rerank', 'hybrid'), default='rerank')
    parser.add_argument('--candidates', type=int, default=10)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--concurrency', default='1,4,8')
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--cases', default=str(ROOT / 'RAG/tests/fixtures/retrieval_regression_cases.json'))
    args = parser.parse_args()
    os.environ.setdefault('AGENT_RAG_MAX_CONCURRENCY', '1')
    os.environ.setdefault('AGENT_RESOURCE_ACQUIRE_TIMEOUT_SEC', '120')
    from RAG.simple_rag import category_index
    category_index.default_taxonomy_path = lambda root: Path(args.taxonomy).resolve()
    from agent_api.app.tools import rag_tools as current
    module = current
    if args.legacy_sources:
        folder = Path(args.legacy_sources)
        module = load_module('agent_api.app.tools._baseline_rag', folder / 'rag_tools.py')
        module._repo_root = lambda: ROOT
        module.NumpyVectorStore = load_module('baseline_vectors', folder / 'vector_store.py').NumpyVectorStore
        module.CrossEncoderReranker = load_module('baseline_reranker', folder / 'reranker.py').CrossEncoderReranker
    cases = json.loads(Path(args.cases).read_text(encoding='utf-8'))
    artifacts = {name: hashlib.sha256(path.read_bytes()).hexdigest()
                 for name in ('taxonomy.json', 'manifest.json', 'validated.json')
                 if (path := Path(args.taxonomy).resolve().parent / name).exists()}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    process = psutil.Process()
    rows = []
    comparisons = []

    def query(case):
        started = time.perf_counter()
        result = module.rag_search_tool(query=case['query'], category=case.get('category'), device=args.device,
            rerank=args.mode != 'dense', multi_route=args.mode == 'hybrid', rewrite='none',
            rerank_candidates=args.candidates, expand_neighbors=1, top_k=5)
        elapsed = (time.perf_counter() - started) * 1000
        hits = result['hits']
        expected = set(case.get('expected_books', []))
        rank = next((i+1 for i,h in enumerate(hits) if h.get('book_id') in expected), None)
        anchors = case.get('evidence_patterns', [])
        evidence_rank = next((i+1 for i,h in enumerate(hits)
                              if anchors and not noise(h.get('text', ''))
                              and all(re.search(pattern, h.get('text', ''), re.I) for pattern in anchors)), None)
        return {'id': case['id'], 'query': case['query'], 'category': case.get('category'), 'elapsed_ms': elapsed,
                'book_hit': rank is not None, 'reciprocal_rank': 1/rank if rank else 0,
                'evidence_hit': evidence_rank is not None, 'evidence_reciprocal_rank': 1/evidence_rank if evidence_rank else 0,
                'noise_hits': sum(noise(h.get('text', '')) for h in hits), 'hit_count': len(hits), 'hits': hits,
                'contexts': result['contexts']}

    # Warm every scope before timing; publish cold cost separately.
    started = time.perf_counter()
    for case in cases:
        query(case)
    cold_seconds = time.perf_counter() - started
    memory_before = process.memory_info().rss / 1024**2
    for round_id in range(args.rounds):
        for concurrency in map(int, args.concurrency.split(',')):
            started = time.perf_counter()
            with ThreadPoolExecutor(max_workers=concurrency) as pool:
                batch = list(pool.map(query, cases))
            elapsed = time.perf_counter() - started
            comparisons.append({'round': round_id+1, 'concurrency': concurrency,
                                'qps': len(batch)/elapsed, **stats([row['elapsed_ms'] for row in batch]),
                                'rss_mb': process.memory_info().rss / 1024**2})
            if round_id == 0 and concurrency == 1:
                rows = batch
            payload = {'taxonomy': str(Path(args.taxonomy).resolve()), 'mode': args.mode, 'candidates': args.candidates,
                       'artifacts': artifacts, 'cases_sha256': hashlib.sha256(Path(args.cases).read_bytes()).hexdigest(),
                       'legacy_sources': bool(args.legacy_sources), 'cold_seconds': cold_seconds,
                       'rss_warm_mb': memory_before, 'performance': comparisons,
                       'quality': {'cases': len(rows), 'book_recall_at_5': statistics.fmean(r['book_hit'] for r in rows) if rows else None,
                                   'mrr': statistics.fmean(r['reciprocal_rank'] for r in rows) if rows else None,
                                   'evidence_recall_at_5': statistics.fmean(r['evidence_hit'] for r in rows) if rows else None,
                                   'noise_hits': sum(r['noise_hits'] for r in rows), 'hits': sum(r['hit_count'] for r in rows)},
                       'rows': rows}
            temp = output.with_suffix('.tmp')
            temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
            temp.replace(output)
            print(args.mode, args.candidates, 'round', round_id+1, 'concurrency', concurrency,
                  'p95_ms', round(comparisons[-1]['p95_ms'], 1), flush=True)


if __name__ == '__main__':
    main()
