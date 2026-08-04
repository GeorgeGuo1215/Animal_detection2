from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def configure_cpu_runtime() -> None:
    # 只用 AGENT_WARMUP_DEVICE 把 embedding / 检索留在 CPU，不要再设
    # CUDA_VISIBLE_DEVICES=-1：对 CUDA build 的 torch 而言 -1 不是合法设备序号，
    # 进程会在 warmup 结束后触发 ACCESS_VIOLATION（Windows exit code 3221225477）。
    os.environ["AGENT_WARMUP_DEVICE"] = "cpu"
    os.environ.setdefault("AGENT_WARMUP_BM25", "1")
    os.environ.setdefault("AGENT_WARMUP_RERANKER", "0")
    os.environ.setdefault("AGENT_WARMUP_CATEGORIES", "0")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")


def main() -> None:
    parser = argparse.ArgumentParser(description="Start Agent API after main-thread CPU RAG warmup.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--warmup-only", action="store_true")
    args = parser.parse_args()

    configure_cpu_runtime()

    import uvicorn

    from agent_api.app import main as app_main

    app_main._run_rag_warmup_unlimited()
    if args.warmup_only:
        return
    os.environ["AGENT_WARMUP_RAG"] = "0"
    uvicorn.run(app_main.app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
