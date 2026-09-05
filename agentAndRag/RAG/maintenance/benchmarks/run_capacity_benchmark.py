"""可复现的 RAG CPU/GPU、并发和多实例容量基准。

该工具不调用外部 LLM 或 Web Search。默认从测试 fixture 读取 24 条固定查询，
每种设备组合在独立子进程中运行，并将原始 JSON/CSV 和 Markdown 报告写到指定目录。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

import numpy as np

from RAG.simple_rag.category_index import resolve_category_index_dirs
from RAG.simple_rag.context_utils import build_neighbor_contexts, build_source_index
from RAG.simple_rag.embeddings import Embedder
from RAG.simple_rag.reranker import CrossEncoderReranker
from RAG.simple_rag.retrieval import BM25Retriever
from RAG.simple_rag.scoring import overlap_score
from RAG.simple_rag.vector_store import NumpyVectorStore
from agent_api.app.hf_local_model import resolve_embedding_model_id, resolve_rerank_model_id


RAG_ROOT = Path(__file__).resolve().parents[2]
AGENT_ROOT = RAG_ROOT.parent
DEFAULT_CASES = RAG_ROOT / "tests" / "fixtures" / "retrieval_regression_cases.json"


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def _summary(values: Iterable[float]) -> dict[str, float]:
    data = [float(value) for value in values]
    return {
        "count": len(data),
        "mean_ms": statistics.fmean(data) if data else 0.0,
        "p50_ms": _percentile(data, 0.50),
        "p95_ms": _percentile(data, 0.95),
        "p99_ms": _percentile(data, 0.99),
        "max_ms": max(data, default=0.0),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """原子写入基准结果，避免长场景中断留下半截 JSON。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def _memory_snapshot() -> dict[str, float]:
    try:
        import psutil

        process = psutil.Process()
        info = process.memory_full_info()
        virtual = psutil.virtual_memory()
        return {
            "rss_mb": info.rss / 1024**2,
            "uss_mb": getattr(info, "uss", info.rss) / 1024**2,
            "private_mb": getattr(info, "private", info.rss) / 1024**2,
            "system_available_mb": virtual.available / 1024**2,
            "system_total_mb": virtual.total / 1024**2,
        }
    except Exception:
        return {}


def _gpu_snapshot(*, synchronize: bool = True, include_nvidia: bool = True) -> dict[str, float]:
    snapshot: dict[str, float] = {}
    try:
        import torch

        if torch.cuda.is_available():
            if synchronize:
                torch.cuda.synchronize()
            snapshot.update(
                {
                    "torch_allocated_mb": torch.cuda.memory_allocated() / 1024**2,
                    "torch_reserved_mb": torch.cuda.memory_reserved() / 1024**2,
                    "torch_peak_allocated_mb": torch.cuda.max_memory_allocated() / 1024**2,
                }
            )
    except Exception:
        pass
    if not include_nvidia:
        return snapshot
    try:
        output = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used,memory.total", "--format=csv,noheader,nounits"],
            text=True,
            timeout=10,
        ).strip().splitlines()[0]
        used, total = [float(value.strip()) for value in output.split(",")[:2]]
        snapshot.update({"nvidia_used_mb": used, "nvidia_total_mb": total})
    except Exception:
        pass
    return snapshot


class _Sampler:
    def __init__(self, interval_s: float = 0.5) -> None:
        self.interval_s = interval_s
        self.samples: list[dict[str, float]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            self.samples.append(
                {
                    "elapsed_s": time.perf_counter(),
                    **_memory_snapshot(),
                    **_gpu_snapshot(synchronize=False, include_nvidia=True),
                }
            )
            self._stop.wait(self.interval_s)

    def __enter__(self) -> "_Sampler":
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._stop.set()
        self._thread.join(timeout=5)


@dataclass(frozen=True)
class Scenario:
    name: str
    embedding_device: str
    reranker_device: Optional[str]
    rerank: bool


class BenchmarkPipeline:
    def __init__(self, scenario: Scenario, cases: list[dict]) -> None:
        self.scenario = scenario
        self.cases = cases
        self.stores: dict[str, NumpyVectorStore] = {}
        self.source_indexes: dict[str, dict] = {}
        self.bm25: dict[str, BM25Retriever] = {}
        self.embedder: Optional[Embedder] = None
        self.reranker: Optional[CrossEncoderReranker] = None
        self._serial_lock = threading.Lock()

    def load(self) -> dict[str, float]:
        timings: dict[str, float] = {}
        started = time.perf_counter()
        unique_categories = sorted({case["category"] for case in self.cases})
        for category in unique_categories:
            dirs = resolve_category_index_dirs(repo_root=AGENT_ROOT, category=category)
            if len(dirs) != 1:
                raise RuntimeError(f"category {category} resolved to {len(dirs)} indexes")
            store = NumpyVectorStore(dirs[0])
            store.load()
            self.stores[category] = store
            self.source_indexes[category] = build_source_index(store._meta)  # noqa: SLF001
        timings["index_and_metadata_load_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        for category, store in self.stores.items():
            self.bm25[category] = BM25Retriever(metas=store._meta)  # noqa: SLF001
        timings["bm25_build_ms"] = (time.perf_counter() - started) * 1000

        embedding_model = resolve_embedding_model_id("intfloat/multilingual-e5-small", AGENT_ROOT)
        started = time.perf_counter()
        self.embedder = Embedder(embedding_model, device=self.scenario.embedding_device)
        timings["embedding_model_load_ms"] = (time.perf_counter() - started) * 1000

        if self.scenario.rerank and self.scenario.reranker_device:
            reranker_model = resolve_rerank_model_id("BAAI/bge-reranker-large", AGENT_ROOT)
            started = time.perf_counter()
            self.reranker = CrossEncoderReranker(reranker_model, device=self.scenario.reranker_device)
            timings["reranker_model_load_ms"] = (time.perf_counter() - started) * 1000
        else:
            timings["reranker_model_load_ms"] = 0.0
        return timings

    def run_case(self, case: dict, *, serialize: bool = True) -> dict[str, Any]:
        lock = self._serial_lock if serialize else _NullLock()
        with lock:
            return self._run_case(case)

    def _run_case(self, case: dict) -> dict[str, Any]:
        assert self.embedder is not None
        stage: dict[str, float] = {}
        total_started = time.perf_counter()
        store = self.stores[case["category"]]

        started = time.perf_counter()
        query_vector = self.embedder.embed_queries([case["query"]], batch_size=1, normalize=True).vectors[0]
        stage["query_embedding_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        hits = [
            {**meta, "score": float(score), "category": case["category"]}
            for meta, score in store.search(query_vector, top_k=10 if self.scenario.rerank else 5)
        ]
        stage["dense_search_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        best: dict[str, dict] = {}
        for hit in hits:
            chunk_id = str(hit.get("chunk_id") or "")
            if chunk_id and (chunk_id not in best or hit["score"] > best[chunk_id]["score"]):
                best[chunk_id] = hit
        hits = sorted(best.values(), key=lambda item: item["score"], reverse=True)
        stage["merge_deduplicate_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        if self.reranker is not None and hits:
            if len(hits) > 4:
                hits = hits[: max(int(len(hits) * 0.75), 5)]
            order = self.reranker.rerank(
                query=case["query"],
                passages=[str(hit.get("text") or "") for hit in hits],
                top_k=5,
                batch_size=32,
            )
            hits = [
                {**hits[item.index], "score_retrieval": hits[item.index]["score"], "score": item.score}
                for item in order
            ]
            filtered = [hit for hit in hits if overlap_score(case["query"], str(hit.get("text") or "")) >= 0.15]
            if filtered:
                hits = filtered
        else:
            hits = hits[:5]
        stage["rerank_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        contexts = build_neighbor_contexts(
            metas=store._meta,  # noqa: SLF001
            hits=hits,
            neighbor_n=1,
            _source_index=self.source_indexes[case["category"]],
        )
        stage["context_expand_ms"] = (time.perf_counter() - started) * 1000

        started = time.perf_counter()
        result_shape = json.dumps({"hits": hits, "contexts": contexts}, ensure_ascii=False)
        stage["serialization_ms"] = (time.perf_counter() - started) * 1000
        stage["total_ms"] = (time.perf_counter() - total_started) * 1000

        expected = set(case["expected_books"])
        actual = [str(hit.get("book_id") or "") for hit in hits]
        first_rank = next((index + 1 for index, book in enumerate(actual) if book in expected), None)
        return {
            "case_id": case["id"],
            "domain": case["domain"],
            "category": case["category"],
            "stage": stage,
            "actual_books": actual,
            "hit": first_rank is not None,
            "reciprocal_rank": 0.0 if first_rank is None else 1.0 / first_rank,
            "response_bytes": len(result_shape.encode("utf-8")),
        }


class _NullLock:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return None


def _run_worker(args: argparse.Namespace) -> None:
    cases = json.loads(Path(args.cases).read_text(encoding="utf-8"))
    if int(args.limit_cases or 0) > 0:
        cases = cases[: int(args.limit_cases)]
    scenario = Scenario(
        name=args.scenario,
        embedding_device=args.embedding_device,
        reranker_device=args.reranker_device or None,
        rerank=bool(args.rerank),
    )
    import_started = time.perf_counter()
    before = {**_memory_snapshot(), **_gpu_snapshot()}
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats()
    except Exception:
        pass

    pipeline = BenchmarkPipeline(scenario, cases)
    with _Sampler() as sampler:
        load_timings = pipeline.load()
        after_load = {**_memory_snapshot(), **_gpu_snapshot()}
        concurrency_results: dict[str, dict] = {}
        rows: list[dict] = []
        for concurrency in [int(value) for value in args.concurrency.split(",") if value.strip()]:
            started = time.perf_counter()
            with ThreadPoolExecutor(max_workers=concurrency) as executor:
                results = list(executor.map(lambda case: pipeline.run_case(case, serialize=True), cases))
            elapsed = time.perf_counter() - started
            concurrency_results[str(concurrency)] = {
                "elapsed_s": elapsed,
                "qps": len(results) / elapsed if elapsed else 0.0,
                "latency": _summary(item["stage"]["total_ms"] for item in results),
            }
            for result in results:
                rows.append({"concurrency": concurrency, **result})
            _write_json_atomic(
                Path(args.output).with_suffix(".partial.json"),
                {
                    "status": "partial",
                    "scenario": asdict(scenario),
                    "load_timings": load_timings,
                    "completed_concurrency": sorted(concurrency_results),
                    "concurrency": concurrency_results,
                    "rows": rows,
                },
            )
        after = {**_memory_snapshot(), **_gpu_snapshot()}

    stage_names = sorted({name for row in rows for name in row["stage"]})
    stage_summary = {name: _summary(row["stage"][name] for row in rows) for name in stage_names}
    baseline_rows = [row for row in rows if row["concurrency"] == 1]
    output = {
        "scenario": asdict(scenario),
        "pid": os.getpid(),
        "process_elapsed_ms": (time.perf_counter() - import_started) * 1000,
        "load_timings": load_timings,
        "memory_before": before,
        "memory_after_load": after_load,
        "memory_after": after,
        "resource_samples": sampler.samples,
        "stage_summary": stage_summary,
        "concurrency": concurrency_results,
        "quality": {
            "cases": len(baseline_rows),
            "recall_at_5": statistics.fmean(float(row["hit"]) for row in baseline_rows),
            "mrr": statistics.fmean(row["reciprocal_rank"] for row in baseline_rows),
        },
        "rows": rows,
    }
    _write_json_atomic(Path(args.output), output)
    partial = Path(args.output).with_suffix(".partial.json")
    if partial.exists():
        partial.unlink()


def _environment() -> dict[str, Any]:
    env: dict[str, Any] = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "platform": platform.platform(),
        "python": sys.version,
        "cpu_count": os.cpu_count(),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=AGENT_ROOT, text=True).strip(),
        "memory": _memory_snapshot(),
        "gpu": _gpu_snapshot(),
        "flags": {
            name: os.getenv(name, "<unset>")
            for name in (
                "AGENT_WARMUP_DEVICE",
                "AGENT_WARMUP_RAG",
                "AGENT_WARMUP_BM25",
                "AGENT_WARMUP_RERANKER",
                "AGENT_WARMUP_CATEGORIES",
                "AGENT_RAG_MAX_CONCURRENCY",
                "RAG_RERANK_SKIP_THRESHOLD",
            )
        },
    }
    try:
        import torch

        env.update({"torch": torch.__version__, "cuda": torch.version.cuda, "cuda_available": torch.cuda.is_available()})
    except Exception as exc:
        env["torch_error"] = str(exc)
    taxonomy = RAG_ROOT / "data" / "category_taxonomy.json"
    cases = DEFAULT_CASES
    env["artifacts"] = {
        str(path.relative_to(AGENT_ROOT)): {"bytes": path.stat().st_size, "sha256": _sha256(path)}
        for path in (taxonomy, cases)
    }
    env.update(_runtime_provenance())
    return env


def _runtime_provenance() -> dict[str, Any]:
    cases = json.loads(DEFAULT_CASES.read_text(encoding="utf-8"))
    categories = sorted({case["category"] for case in cases})
    indexes: dict[str, dict[str, Any]] = {}
    for category in categories:
        dirs = resolve_category_index_dirs(repo_root=AGENT_ROOT, category=category)
        if len(dirs) != 1:
            continue
        index_dir = dirs[0]
        files: dict[str, dict[str, Any]] = {}
        for name in ("store_config.json", "embeddings.npy", "meta.jsonl"):
            path = index_dir / name
            if path.is_file():
                files[name] = {"bytes": path.stat().st_size, "sha256": _sha256(path)}
        indexes[category] = {"path": str(index_dir), "files": files}

    code_digest = hashlib.sha256()
    code_files = sorted((RAG_ROOT / "simple_rag").glob("*.py")) + [Path(__file__).resolve()]
    for path in code_files:
        code_digest.update(path.relative_to(AGENT_ROOT).as_posix().encode("utf-8"))
        code_digest.update(path.read_bytes())
    tracked_status = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=AGENT_ROOT,
        text=True,
    )
    embedding_model = resolve_embedding_model_id("intfloat/multilingual-e5-small", AGENT_ROOT)
    reranker_model = resolve_rerank_model_id("BAAI/bge-reranker-large", AGENT_ROOT)
    return {
        "models": {
            "embedding": embedding_model,
            "reranker": reranker_model,
        },
        "indexes": indexes,
        "code_fingerprint_sha256": code_digest.hexdigest(),
        "tracked_worktree_dirty": bool(tracked_status.strip()),
    }


def _render_report(environment: dict, results: list[dict], multi_instance: list[dict], output_dir: Path) -> None:
    cases_artifact = next(
        value
        for key, value in environment["artifacts"].items()
        if key.replace("\\", "/").endswith("RAG/tests/fixtures/retrieval_regression_cases.json")
    )
    lines = [
        "# RAG 运行资源与多实例容量报告",
        "",
        f"- 测试时间：{environment['timestamp']}",
        f"- Git：`{environment['git_commit']}`",
        f"- 硬件：{environment['platform']}，CPU 逻辑核 {environment['cpu_count']}，GPU {environment['gpu']}",
        f"- 查询集：24 条固定 D1–D8 用例，SHA256 `{cases_artifact['sha256']}`",
        f"- RAG 代码指纹：`{environment.get('code_fingerprint_sha256', '<legacy>')}`，tracked worktree dirty={environment.get('tracked_worktree_dirty', '<unknown>')}",
        f"- 模型：Embedding `{environment.get('models', {}).get('embedding', '<unknown>')}`；Reranker `{environment.get('models', {}).get('reranker', '<unknown>')}`",
        f"- 索引指纹：{len(environment.get('indexes', {}))} 个分类，详见 `benchmark_summary.json`",
        "- 外部 LLM/Web：未调用",
        "",
        "## 结果摘要",
        "",
        "| 方案 | 索引加载 ms | BM25 构建 ms | Embedding 加载 ms | Reranker 加载 ms | P50 ms | P95 ms | Recall@5 | MRR | RSS 增量 MB | Torch 峰值 MB |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        load = result["load_timings"]
        latency = result["concurrency"]["1"]["latency"]
        before = result["memory_before"]
        after = result["memory_after_load"]
        rss_delta = float(after.get("rss_mb", 0)) - float(before.get("rss_mb", 0))
        peak = result["memory_after"].get("torch_peak_allocated_mb", 0)
        lines.append(
            f"| {result['scenario']['name']} | {load['index_and_metadata_load_ms']:.1f} | {load['bm25_build_ms']:.1f} | "
            f"{load['embedding_model_load_ms']:.1f} | {load['reranker_model_load_ms']:.1f} | "
            f"{latency['p50_ms']:.1f} | {latency['p95_ms']:.1f} | "
            f"{result['quality']['recall_at_5']:.3f} | {result['quality']['mrr']:.3f} | "
            f"{rss_delta:.1f} | {float(peak):.1f} |"
        )

    reranked_results = [result for result in results if result["scenario"]["rerank"]]
    best_quality = max(result["quality"]["recall_at_5"] for result in reranked_results)
    acceptable = [
        result
        for result in reranked_results
        if result["quality"]["recall_at_5"] >= best_quality - 0.02
    ]
    fastest = min(acceptable, key=lambda result: result["concurrency"]["1"]["latency"]["p50_ms"])
    current = next((result for result in results if result["scenario"]["name"] == "current_gpu"), None)
    total_memory = float(environment.get("memory", {}).get("system_total_mb", 0))
    total_vram = float(environment.get("gpu", {}).get("nvidia_total_mb", 0))
    rss_delta = max(
        float(fastest["memory_after_load"].get("rss_mb", 0)) - float(fastest["memory_before"].get("rss_mb", 0)),
        1.0,
    )
    gpu_delta = max(
        float(fastest["memory_after_load"].get("nvidia_used_mb", 0)) - float(fastest["memory_before"].get("nvidia_used_mb", 0)),
        float(fastest["memory_after"].get("torch_peak_allocated_mb", 0)),
        1.0,
    )
    ram_capacity = max(math.floor((total_memory * 0.80) / rss_delta), 0) if total_memory else 0
    gpu_capacity = max(math.floor((total_vram * 0.80) / gpu_delta), 0) if total_vram else 0

    lines.extend(
        [
            "",
            "## 多实例实测",
            "",
            "| 实例数 | 状态 | 含冷启动总耗时 s | 含冷启动 QPS | 热查询 QPS | RSS 增量 MB | Torch 峰值合计 MB |",
            "|---:|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in multi_instance:
        lines.append(
            f"| {row['replicas']} | {row['status']} | {row.get('elapsed_s', 0):.2f} | "
            f"{row.get('qps', 0):.3f} | {row.get('warm_qps', 0):.3f} | "
            f"{row.get('rss_delta_mb', 0):.1f} | {row.get('torch_peak_sum_mb', 0):.1f} |"
        )
    lines.extend(
        [
            "",
            "## 分阶段与并发",
            "",
        ]
    )
    for result in results:
        lines.append(f"### {result['scenario']['name']}")
        lines.append("")
        lines.append("| 阶段 | P50 ms | P95 ms | Max ms |")
        lines.append("|---|---:|---:|---:|")
        for stage, summary in result["stage_summary"].items():
            lines.append(f"| {stage} | {summary['p50_ms']:.2f} | {summary['p95_ms']:.2f} | {summary['max_ms']:.2f} |")
        lines.append("")
        lines.append("| 并发请求 | QPS | 批次耗时 s | P95 ms |")
        lines.append("|---:|---:|---:|---:|")
        for concurrency, data in result["concurrency"].items():
            lines.append(f"| {concurrency} | {data['qps']:.3f} | {data['elapsed_s']:.2f} | {data['latency']['p95_ms']:.1f} |")
        lines.append("")

    lines.extend(
        [
            "## 部署判断与后续动刀点",
            "",
            f"1. 在保留 Reranker 且目标书籍 Recall@5 不下降的方案中，本机最快方案为 **{fastest['scenario']['name']}**。",
            f"2. 按 20% 系统预留粗算，该方案 RAM 上限约 {ram_capacity} 个、显存上限约 {gpu_capacity} 个；最终实例数还必须受实测吞吐拐点约束。",
            "3. Dense matrix、BM25、metadata 和邻接上下文均在 CPU/主存执行，不应复制到每个 GPU Agent Worker；优先抽成共享 RAG 服务。",
            "4. Query embedding 单次输入很小，若 CPU 方案延迟可接受，应将 GPU 留给计算更重的 CrossEncoder reranker；若 CPU embedding 明显拖慢，则在共享服务中统一 GPU batch。",
            "5. 当前 `AGENT_RAG_MAX_CONCURRENCY=1` 会把同一进程请求串行化；增加 API Worker 只会重复加载模型和索引。先实现共享队列和跨请求 batch，再决定 GPU 副本数。",
            "6. 分类索引目前逐库串行扫描。短期可并行 CPU dot/BM25，长期可考虑 mmap/共享内存或独立向量检索服务，避免每进程复制 metadata 与 BM25。",
            "7. 建议新增兼容配置 `RAG_EMBEDDING_DEVICE`、`RAG_RERANKER_DEVICE`，未设置时回退 `AGENT_WARMUP_DEVICE`；本轮未改变生产默认值。",
            "8. 多实例上线前以 `min(VRAM上限, RAM上限, 吞吐拐点)` 为硬限制，并设置 80% 显存/内存保护与 OOM 熔断。",
            "",
            "## 质量结论边界",
            "",
            "- `gpu_no_rerank` 的 13 ms 结果只作为性能下界，不能作为生产推荐。当前 24 条用例验证的是分类隔离和目标书籍命中，无法证明段落级相关性与事实充分性不下降。",
            "- Reranker 的保留或裁剪必须再使用段落级人工标注、nDCG/专家评分和最终回答正确率验证；本报告不据此关闭生产 Reranker。",
            "- 多实例的“含冷启动 QPS”包含并发模型加载，适用于发布/扩容阶段；“热查询 QPS”才用于稳定运行容量估算。",
            "",
            "## 可溯源材料",
            "",
            "- `benchmark_summary.json`：完整环境、方案和逐查询数据。",
            "- `stage_latency.csv`：每个请求的阶段耗时和质量结果。",
            "- `resource_samples.csv`：采样期间的 RAM/VRAM 数据。",
            "- 结果仅适用于报告所列 commit、模型和索引指纹。",
        ]
    )
    (output_dir / "RAG_RUNTIME_CAPACITY_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _run_orchestrator(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    environment = _environment()
    scenarios = [
        Scenario("all_cpu", "cpu", "cpu", True),
        Scenario("current_gpu", "cuda", "cuda", True),
        Scenario("cpu_embed_gpu_rerank", "cpu", "cuda", True),
        Scenario("gpu_embed_cpu_rerank", "cuda", "cpu", True),
        Scenario("gpu_no_rerank", "cuda", None, False),
    ]
    if not environment.get("cuda_available"):
        scenarios = [scenarios[0]]

    results: list[dict] = []
    for scenario in scenarios:
        target = output_dir / f"scenario_{scenario.name}.json"
        command = [
            sys.executable,
            "-m",
            "RAG.maintenance.benchmarks.run_capacity_benchmark",
            "--worker",
            "--scenario",
            scenario.name,
            "--embedding-device",
            scenario.embedding_device,
            "--reranker-device",
            scenario.reranker_device or "",
            "--concurrency",
            args.concurrency,
            "--cases",
            str(Path(args.cases).resolve()),
            "--limit-cases",
            str(int(args.limit_cases or 0)),
            "--output",
            str(target),
        ]
        if scenario.rerank:
            command.append("--rerank")
        subprocess.run(command, cwd=AGENT_ROOT, check=True)
        results.append(json.loads(target.read_text(encoding="utf-8")))

    multi_instance: list[dict] = []
    if environment.get("cuda_available"):
        current = next(result for result in results if result["scenario"]["name"] == "current_gpu")
        base_elapsed = current["concurrency"]["1"]["elapsed_s"]
        case_count = current["quality"]["cases"]
        multi_instance.append(
            {
                "replicas": 1,
                "status": "ok",
                "elapsed_s": base_elapsed,
                "qps": case_count / base_elapsed,
                "warm_qps": current["concurrency"]["1"]["qps"],
                "rss_delta_mb": current["memory_after_load"].get("rss_mb", 0) - current["memory_before"].get("rss_mb", 0),
                "torch_peak_sum_mb": current["memory_after"].get("torch_peak_allocated_mb", 0),
            }
        )
        for replicas in (2, 3):
            per_rss = max(
                current["memory_after_load"].get("rss_mb", 0) - current["memory_before"].get("rss_mb", 0),
                1.0,
            )
            per_gpu = max(current["memory_after"].get("torch_peak_allocated_mb", 0), 1.0)
            total_ram = float(environment.get("memory", {}).get("system_total_mb", 0))
            total_gpu = float(environment.get("gpu", {}).get("nvidia_total_mb", 0))
            if (total_ram and replicas * per_rss > total_ram * 0.85) or (
                total_gpu and replicas * per_gpu > total_gpu * 0.85
            ):
                multi_instance.append({"replicas": replicas, "status": "skipped_resource_guard"})
                break
            commands: list[tuple[subprocess.Popen, Path]] = []
            started = time.perf_counter()
            for replica in range(replicas):
                target = output_dir / f"replicas_{replicas}_{replica}.json"
                command = [
                    sys.executable,
                    "-m",
                    "RAG.maintenance.benchmarks.run_capacity_benchmark",
                    "--worker",
                    "--scenario",
                    f"current_gpu_replica_{replica}",
                    "--embedding-device",
                    "cuda",
                    "--reranker-device",
                    "cuda",
                    "--rerank",
                    "--concurrency",
                    "1",
                    "--cases",
                    str(Path(args.cases).resolve()),
                    "--limit-cases",
                    str(int(args.limit_cases or 0)),
                    "--output",
                    str(target),
                ]
                commands.append((subprocess.Popen(command, cwd=AGENT_ROOT), target))
            return_codes = [process.wait() for process, _ in commands]
            elapsed = time.perf_counter() - started
            if any(code != 0 for code in return_codes):
                multi_instance.append({"replicas": replicas, "status": f"failed:{return_codes}"})
                break
            replica_results = [json.loads(target.read_text(encoding="utf-8")) for _, target in commands]
            multi_instance.append(
                {
                    "replicas": replicas,
                    "status": "ok",
                    "elapsed_s": elapsed,
                    "qps": (
                        replicas
                        * (
                            min(len(json.loads(Path(args.cases).read_text(encoding="utf-8"))), int(args.limit_cases))
                            if int(args.limit_cases or 0) > 0
                            else len(json.loads(Path(args.cases).read_text(encoding="utf-8")))
                        )
                    )
                    / elapsed,
                    "warm_qps": (
                        sum(result["quality"]["cases"] for result in replica_results)
                        / max(result["concurrency"]["1"]["elapsed_s"] for result in replica_results)
                    ),
                    "rss_delta_mb": sum(
                        result["memory_after_load"].get("rss_mb", 0) - result["memory_before"].get("rss_mb", 0)
                        for result in replica_results
                    ),
                    "torch_peak_sum_mb": sum(result["memory_after"].get("torch_peak_allocated_mb", 0) for result in replica_results),
                }
            )

    summary = {"environment": environment, "results": results, "multi_instance": multi_instance}
    (output_dir / "benchmark_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (output_dir / "stage_latency.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scenario", "concurrency", "case_id", "domain", "category", "stage", "milliseconds", "hit", "reciprocal_rank"])
        for result in results:
            for row in result["rows"]:
                for stage, milliseconds in row["stage"].items():
                    writer.writerow([result["scenario"]["name"], row["concurrency"], row["case_id"], row["domain"], row["category"], stage, milliseconds, row["hit"], row["reciprocal_rank"]])
    with (output_dir / "resource_samples.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        keys = ["scenario", "elapsed_s", "rss_mb", "uss_mb", "private_mb", "system_available_mb", "nvidia_used_mb", "nvidia_total_mb", "torch_allocated_mb", "torch_reserved_mb", "torch_peak_allocated_mb"]
        writer = csv.DictWriter(handle, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        for result in results:
            for sample in result["resource_samples"]:
                writer.writerow({"scenario": result["scenario"]["name"], **sample})
    _render_report(environment, results, multi_instance, output_dir)
    print(output_dir / "RAG_RUNTIME_CAPACITY_REPORT.md")


def _render_existing_summary(output_dir: Path) -> None:
    summary_path = output_dir / "benchmark_summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError(f"benchmark summary not found: {summary_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["environment"].update(_runtime_provenance())
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    multi_instance = summary.get("multi_instance") or []
    current = next(
        (result for result in summary["results"] if result["scenario"]["name"] == "current_gpu"),
        None,
    )
    for row in multi_instance:
        replicas = int(row.get("replicas") or 0)
        if row.get("warm_qps") is not None:
            continue
        if replicas == 1 and current is not None:
            row["warm_qps"] = current["concurrency"]["1"]["qps"]
            continue
        replica_results = []
        for replica in range(replicas):
            path = output_dir / f"replicas_{replicas}_{replica}.json"
            if path.is_file():
                replica_results.append(json.loads(path.read_text(encoding="utf-8")))
        if replica_results:
            row["warm_qps"] = sum(result["quality"]["cases"] for result in replica_results) / max(
                result["concurrency"]["1"]["elapsed_s"] for result in replica_results
            )
    _render_report(
        summary["environment"],
        summary["results"],
        multi_instance,
        output_dir,
    )
    print(output_dir / "RAG_RUNTIME_CAPACITY_REPORT.md")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=str(AGENT_ROOT.parent / ".test-tmp" / "reports" / "rag-runtime-capacity-20260824"))
    parser.add_argument("--cases", default=str(DEFAULT_CASES))
    parser.add_argument("--concurrency", default="1,2,4,8")
    parser.add_argument("--limit-cases", type=int, default=0)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--report-only", action="store_true", help="从已有 benchmark_summary.json 重建 Markdown 报告")
    parser.add_argument("--scenario", default="manual")
    parser.add_argument("--embedding-device", default="cpu")
    parser.add_argument("--reranker-device", default="")
    parser.add_argument("--rerank", action="store_true")
    parser.add_argument("--output")
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.report_only:
        _render_existing_summary(Path(args.output_dir).resolve())
    elif args.worker:
        if not args.output:
            raise SystemExit("--output is required in worker mode")
        _run_worker(args)
    else:
        _run_orchestrator(args)


if __name__ == "__main__":
    main()
