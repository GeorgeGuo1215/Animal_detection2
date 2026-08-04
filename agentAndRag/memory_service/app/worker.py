"""后台 worker：消费记忆任务队列。

worker 是普通线程而不是协程——它的工作是"取一条任务、调几次 LLM、写几行库"，
全是阻塞 IO，线程模型足够，也省得为了 async 把数据库层整个换掉。

多实例部署时不需要任何协调：出队靠 SKIP LOCKED，同用户串行靠事务级咨询锁。
"""

from __future__ import annotations

import logging
import threading
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

from . import db
from .config import MemoryConfig
from .memory import consolidator, queue

logger = logging.getLogger(__name__)


class MemoryWorker:
    """处理单个任务的最小单元，可以脱离线程单独调用 run_once 做测试。"""

    def __init__(
        self,
        cfg: MemoryConfig,
        embedder,
        llm,
        *,
        name: str = "worker",
        clock: Optional[Callable[[], datetime]] = None,
    ) -> None:
        self.cfg = cfg
        self.embedder = embedder
        self.llm = llm
        self.name = name
        # 时钟可注入，长跑模拟靠它推进虚拟时间。
        self.clock = clock or datetime.now
        self.processed = 0
        self.failed = 0

    def run_once(self) -> bool:
        """处理至多一个任务，返回是否真的干了活。

        返回 False 有三种情况：队列空、用户正被别的 worker 处理、任务失败。
        调用方据此决定是立刻再来一轮还是先歇一会儿。
        """
        task: Optional[Dict[str, Any]] = None
        try:
            with db.connection() as conn:
                task = queue.claim(conn)
                if task is None:
                    return False

                if not db.try_user_lock(conn, task["userId"]):
                    # 同一用户已有 worker 在处理。整个事务回滚，任务自动回到 pending，
                    # 交给下一轮或别的 worker。
                    conn.rollback()
                    logger.debug(
                        "%s: user %s is busy, releasing task %s",
                        self.name, task["userId"], task["id"],
                    )
                    return False

                result = consolidator.consolidate(
                    conn,
                    user_id=task["userId"],
                    cfg=self.cfg,
                    embedder=self.embedder,
                    llm=self.llm,
                    now=self.clock(),
                )
                queue.complete(conn, task["id"])

            self.processed += 1
            logger.info(
                "%s: consolidated user %s (%s pages promoted, %s llm calls)",
                self.name, task["userId"], result.get("promoted", 0),
                result.get("llm_calls", 0),
            )
            return True

        except Exception as exc:  # noqa: BLE001 - worker 不能因单个任务崩掉
            if task is None:
                logger.exception("%s: failed to claim a task", self.name)
                return False

            self.failed += 1
            logger.exception(
                "%s: task %s for user %s failed", self.name, task["id"], task["userId"]
            )
            self._record_failure(task, exc)
            return False

    def _record_failure(self, task: Dict[str, Any], exc: BaseException) -> None:
        """在新事务里记录失败——处理任务的那个事务已经回滚了。"""
        try:
            with db.connection() as conn:
                queue.fail(
                    conn,
                    task_id=task["id"],
                    user_id=task["userId"],
                    kind=task["kind"],
                    error=f"{type(exc).__name__}: {exc}",
                    attempts=int(task["attempts"]),
                    max_attempts=self.cfg.task_max_attempts,
                )
        except Exception:  # noqa: BLE001
            # 连记录失败都失败，说明数据库整个不可用；任务会被 requeue_stale_running 捞回。
            logger.exception("%s: could not record failure for task %s", self.name, task["id"])

    def run_forever(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            try:
                did_work = self.run_once()
            except Exception:  # noqa: BLE001
                logger.exception("%s: unexpected error in worker loop", self.name)
                did_work = False
            if not did_work:
                stop_event.wait(self.cfg.worker_poll_interval)


class WorkerPool:
    """一组 worker 线程，随服务生命周期启停。"""

    def __init__(self, cfg: MemoryConfig, embedder, llm) -> None:
        self.cfg = cfg
        self.embedder = embedder
        self.llm = llm
        self.workers: List[MemoryWorker] = []
        self._threads: List[threading.Thread] = []
        self._stop = threading.Event()

    def start(self) -> None:
        if self._threads:
            return

        # 上一次进程被强杀时可能留下卡在 running 的任务，启动时捞回来。
        try:
            with db.connection() as conn:
                recovered = queue.requeue_stale_running(conn)
            if recovered:
                logger.warning("memory_service: requeued %s stale tasks", recovered)
        except Exception:  # noqa: BLE001
            logger.exception("memory_service: stale task recovery failed")

        for index in range(self.cfg.worker_concurrency):
            worker = MemoryWorker(
                self.cfg, self.embedder, self.llm, name=f"memory-worker-{index}"
            )
            thread = threading.Thread(
                target=worker.run_forever, args=(self._stop,), daemon=True,
                name=f"memory-worker-{index}",
            )
            self.workers.append(worker)
            self._threads.append(thread)
            thread.start()
        logger.info("memory_service: started %s workers", len(self._threads))

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=timeout)
        self._threads.clear()
        self.workers.clear()

    @property
    def stats(self) -> Dict[str, int]:
        return {
            "workers": len(self._threads),
            "processed": sum(w.processed for w in self.workers),
            "failed": sum(w.failed for w in self.workers),
        }
