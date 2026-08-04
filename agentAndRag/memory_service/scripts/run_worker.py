"""只跑 worker 不起 HTTP 服务。

API 与 worker 可以分开扩容：API 实例扛写入并发，worker 实例扛 LLM 处理量。
两者互不感知，协调完全交给数据库。

    python memory_service/scripts/run_worker.py --workers 4

在 agentAndRag/ 目录下执行。
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
from dataclasses import replace
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app import db  # noqa: E402
from memory_service.app.config import load_config  # noqa: E402
from memory_service.app.embedding import init_embedder  # noqa: E402
from memory_service.app.llm import init_llm  # noqa: E402
from memory_service.app.worker import WorkerPool  # noqa: E402

logger = logging.getLogger("memory_worker")


def main() -> int:
    parser = argparse.ArgumentParser(description="记忆服务后台 worker")
    parser.add_argument("--workers", type=int, default=None, help="worker 线程数")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = load_config()
    if args.workers:
        cfg = replace(cfg, worker_concurrency=args.workers)

    db.init_pool(cfg)
    pool = WorkerPool(cfg, init_embedder(cfg), init_llm(cfg))
    pool.start()
    logger.info("worker 已启动，%s 线程，Ctrl+C 退出", cfg.worker_concurrency)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        logger.info("正在停止 worker ...")
        pool.stop()
        db.close_pool()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
