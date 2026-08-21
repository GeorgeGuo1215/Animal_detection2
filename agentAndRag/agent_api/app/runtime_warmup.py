from __future__ import annotations

import os
import traceback
from pathlib import Path
from typing import Any

from .concurrency import ResourceBusyError, get_resource_limits
from .hf_local_model import is_local_path, resolve_embedding_model_id, resolve_rerank_model_id
from .tools.rag_tools import warmup_rag_cache


def _warmup_rag_unlimited() -> dict[str, Any]:
    """不受槽位限制地预热各分类 RAG 索引、embedding 与 reranker。"""
    try:
        repo_root = Path(__file__).resolve().parents[2]
        device = os.getenv("AGENT_WARMUP_DEVICE") or None
        hf_offline = os.getenv("AGENT_HF_OFFLINE", "").strip().lower() in {"1", "true", "yes"}
        if hf_offline:
            os.environ.setdefault("HF_HUB_OFFLINE", "1")
            os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
            print("[warmup] Hugging Face offline mode enabled.")

        embedding_model = resolve_embedding_model_id(None, repo_root)
        rerank_model = resolve_rerank_model_id(None, repo_root)
        enable_bm25 = os.getenv("AGENT_WARMUP_BM25", "1") == "1"
        enable_reranker = os.getenv("AGENT_WARMUP_RERANKER", "1") == "1"
        if hf_offline and enable_reranker and not is_local_path(rerank_model):
            print("[warmup] Reranker disabled: offline mode has no local model path.")
            enable_reranker = False
        if hf_offline and not is_local_path(embedding_model):
            return {"status": "skipped", "reason": "hf_offline_no_local_embedding"}

        from RAG.simple_rag.category_index import (
            default_category_root,
            resolve_default_category_index_dirs,
        )

        category_dirs = resolve_default_category_index_dirs(repo_root=repo_root)
        category_root = default_category_root(repo_root).resolve()
        warmed: list[dict[str, Any]] = []
        index_size = 0
        reranker_ready = False
        for index, directory in enumerate(category_dirs):
            if not directory.exists():
                continue
            try:
                state = warmup_rag_cache(
                    index_dir=directory,
                    embedding_model=embedding_model,
                    device=device,
                    enable_bm25=enable_bm25 and index == 0,
                    enable_reranker=enable_reranker and not reranker_ready,
                    rerank_model=rerank_model,
                )
                if enable_reranker:
                    reranker_ready = True
                size = int(state.get("index_size") or 0)
                if size > 0:
                    warmed.append({"category": directory.name, "index_size": size})
                    index_size += size
            except Exception as exc:  # noqa: BLE001
                print(f"[warmup] category {directory.name} skipped: {exc}")

        actual_device = device or "cpu"
        try:
            import torch

            if torch.cuda.is_available() and actual_device != "cpu":
                print(f"[warmup] Device: {actual_device} ({torch.cuda.get_device_name(0)})")
            else:
                print(f"[warmup] Device: {actual_device}")
        except ImportError:
            print(f"[warmup] Device: {actual_device} (torch unavailable)")
        print(f"[warmup] Embedding: {embedding_model}")
        print(f"[warmup] Reranker: {rerank_model if enable_reranker else 'disabled'}")
        print(f"[warmup] Index: {index_size} chunks across {len(warmed)} categories")
        return {
            "status": "ok",
            "device": actual_device,
            "index_size": index_size,
            "index_dir": str(category_root),
            "category_indexes": len(warmed),
            "reranker": bool(enable_reranker),
        }
    except Exception as exc:  # noqa: BLE001
        print(f"[warmup] RAG warmup failed: {exc}")
        print(traceback.format_exc())
        return {"status": "failed", "error": str(exc)}


def warmup_rag_runtime() -> dict[str, Any]:
    """在 RAG 并发槽位内执行运行时预热。"""
    limits = get_resource_limits()
    try:
        with limits.rag.slot(timeout_s=max(limits.acquire_timeout_s, 300.0)):
            return _warmup_rag_unlimited()
    except ResourceBusyError as exc:
        print(f"[warmup] RAG warmup skipped: {exc}")
        return {"status": "failed", "error": str(exc)}
