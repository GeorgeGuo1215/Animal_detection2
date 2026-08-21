"""对受控100题执行单查询/双查询的本地RAG来源恢复测试。

该脚本不调用LLM或Web。fixture查询由已知索引片段构造，因此结果仅表示
“目标资料可找回性上限”，不能当作自然临床问题准确率。
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence


_HERE = Path(__file__).resolve().parent
_AGENT_API = _HERE.parents[1]
_ROOT = _HERE.parents[2]
for _path in (str(_AGENT_API), str(_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from agent_api.tests.moe.run_evidence_architecture_compare_live import (  # noqa: E402
    EvalCase,
    load_cases,
    retrieval_metrics,
)
from app.tools.rag_query import is_english_rag_query  # noqa: E402
from app.tools.rag_tools import rag_search_tool  # noqa: E402


def _percentile(values: Sequence[float], quantile: float) -> float:
    """返回线性插值分位数。"""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _evidence_from_result(query: str, result: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """将RAG hits裁成来源评测所需字段。"""
    output: List[Dict[str, Any]] = []
    for hit in result.get("hits") or []:
        if not isinstance(hit, Mapping):
            continue
        output.append({
            "query": query,
            "source_path": hit.get("source_path"),
            "source_file": hit.get("source_file"),
            "book_id": hit.get("book_id"),
            "chunk_id": hit.get("chunk_id"),
            "chunk_index": hit.get("chunk_index"),
            "category": hit.get("category"),
            "score": hit.get("score"),
            "text": str(hit.get("text") or "")[:500],
        })
    return output


def _dedupe_evidence(items: Iterable[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    """按chunk/source组合去重并保留首次出现顺序。"""
    output: List[Dict[str, Any]] = []
    seen = set()
    for item in items:
        key = (
            str(item.get("chunk_id") or ""),
            str(item.get("source_path") or item.get("source_file") or ""),
            str(item.get("chunk_index") or ""),
        )
        if key in seen:
            continue
        seen.add(key)
        output.append(dict(item))
    return output


def run_profile(case: EvalCase, *, query_count: int, top_k: int) -> Dict[str, Any]:
    """执行一条fixture在指定查询数下的来源恢复测试。"""
    queries = [
        query for query in case.seed_rag_queries if is_english_rag_query(query)
    ][:query_count]
    if not queries:
        raise ValueError(f"{case.case_id}: no seed RAG query")

    started = time.perf_counter()
    calls: List[Dict[str, Any]] = []
    evidence: List[Dict[str, Any]] = []
    for query in queries:
        call_started = time.perf_counter()
        result = rag_search_tool(
            query=query,
            category=list(case.categories) or None,
            top_k=top_k,
            rerank=True,
            multi_route=False,
            rewrite="none",
            expand_neighbors=0,
        )
        calls.append({
            "query": query,
            "latency_ms": round((time.perf_counter() - call_started) * 1000.0, 1),
            "hits": len(result.get("hits") or []),
        })
        evidence.extend(_evidence_from_result(query, result))
    evidence = _dedupe_evidence(evidence)
    metrics = retrieval_metrics(
        evidence,
        expected_sources=case.expected_sources,
        expected_domains=case.expected_domains,
        k=max(top_k, top_k * query_count),
    )
    return {
        "case_id": case.case_id,
        "profile": f"q{query_count}",
        "category": list(case.categories),
        "expected_sources": list(case.expected_sources),
        "queries": queries,
        "calls": calls,
        "evidence": evidence,
        "recall_at_k": metrics["recall_at_k"],
        "mrr": metrics["mrr"],
        "source_coverage": metrics["expected_source_coverage"],
        "latency_ms": round((time.perf_counter() - started) * 1000.0, 1),
    }


def build_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """按单/双查询汇总来源恢复率和延迟。"""
    output: Dict[str, Any] = {
        "note": "controlled exact-source recovery upper bound; not clinical QA accuracy",
        "profiles": {},
    }
    for profile in ("q1", "q2"):
        subset = [row for row in rows if row.get("profile") == profile]
        latencies = [float(row.get("latency_ms") or 0.0) for row in subset]
        output["profiles"][profile] = {
            "cases": len(subset),
            "recall_at_k": round(statistics.mean(float(row["recall_at_k"]) for row in subset), 4)
            if subset else 0.0,
            "mrr": round(statistics.mean(float(row["mrr"]) for row in subset), 4)
            if subset else 0.0,
            "latency_ms_mean": round(statistics.mean(latencies), 1) if latencies else 0.0,
            "latency_ms_p50": round(_percentile(latencies, 0.50), 1),
            "latency_ms_p95": round(_percentile(latencies, 0.95), 1),
        }
    return output


def main() -> int:
    """命令行入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixtures",
        default=str(_HERE / "fixtures" / "evidence_architecture_100.json"),
    )
    parser.add_argument("--limit", type=int, default=0, help="0表示全部")
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument(
        "--out-dir",
        default=str(_HERE / "reports" / "evidence_architecture_20260821" / "controlled_retrieval_100"),
    )
    args = parser.parse_args()
    cases = load_cases(Path(args.fixtures))
    if args.limit > 0:
        cases = cases[: args.limit]
    skipped = [
        {
            "case_id": case.case_id,
            "reason": "no valid English controlled query (source/query encoding requires cleanup)",
        }
        for case in cases
        if not any(is_english_rag_query(query) for query in case.seed_rag_queries)
    ]
    cases = [
        case for case in cases
        if any(is_english_rag_query(query) for query in case.seed_rag_queries)
    ]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    jsonl = out_dir / "results.jsonl"
    jsonl.write_text("", encoding="utf-8")
    rows: List[Dict[str, Any]] = []
    total = len(cases) * 2
    progress = 0
    for case in cases:
        for count in (1, 2):
            row = run_profile(case, query_count=count, top_k=args.top_k)
            rows.append(row)
            with jsonl.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")
            progress += 1
            print(json.dumps({
                "progress": f"{progress}/{total}",
                "case": case.case_id,
                "profile": row["profile"],
                "recall_at_k": row["recall_at_k"],
                "mrr": row["mrr"],
                "latency_ms": row["latency_ms"],
            }, ensure_ascii=False), flush=True)
    summary = build_summary(rows)
    summary["skipped"] = skipped
    summary["valid_cases"] = len(cases)
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
