"""文本向量化。

模型与 RAG 子系统保持一致（intfloat/multilingual-e5-small，384 维），默认跑 CPU。
模型是懒加载的：导入本模块不会触发下载或占用内存，只有真正编码时才初始化，
这样单测和 CLI 工具可以直接注入假实现而完全不碰 torch。
"""

from __future__ import annotations

import logging
import threading
from typing import List, Optional, Protocol, Sequence

from .config import MemoryConfig

logger = logging.getLogger(__name__)


class Embedder(Protocol):
    """向量化接口。测试注入假实现时满足这两个方法。"""

    def embed_documents(self, texts: Sequence[str]) -> List[List[float]]: ...

    def embed_query(self, text: str) -> List[float]: ...


def _with_e5_prefix(texts: Sequence[str], prefix: str) -> List[str]:
    """E5 系列要求区分 query/passage 前缀，不加会明显掉召回。

    调用方已经自己加过前缀时不重复添加。
    """
    out: List[str] = []
    for text in texts:
        s = (text or "").strip()
        low = s.lower()
        if low.startswith("query:") or low.startswith("passage:"):
            out.append(s)
        else:
            out.append(f"{prefix} {s}")
    return out


class SentenceTransformerEmbedder:
    """sentence-transformers 封装，输出已归一化的 float 列表。

    归一化后余弦相似度等价于内积，pgvector 侧的距离计算也更稳定。
    """

    def __init__(self, model_name: str, device: str = "cpu", dim: int = 384) -> None:
        self.model_name = model_name
        self.device = device
        self.dim = dim
        self._model = None
        self._lock = threading.Lock()

    @property
    def _is_e5(self) -> bool:
        return "e5" in (self.model_name or "").lower()

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        with self._lock:
            if self._model is None:
                from sentence_transformers import SentenceTransformer

                logger.info(
                    "memory_service: loading embedding model %s on %s",
                    self.model_name,
                    self.device,
                )
                self._model = SentenceTransformer(self.model_name, device=self.device)
        return self._model

    def _encode(self, texts: Sequence[str], prefix: str) -> List[List[float]]:
        if not texts:
            return []
        payload = _with_e5_prefix(texts, prefix) if self._is_e5 else list(texts)
        model = self._ensure_model()
        vectors = model.encode(
            payload,
            batch_size=16,
            show_progress_bar=False,
            normalize_embeddings=True,
        )
        return [[float(x) for x in vec] for vec in vectors]

    def embed_documents(self, texts: Sequence[str]) -> List[List[float]]:
        return self._encode(texts, "passage:")

    def embed_query(self, text: str) -> List[float]:
        result = self._encode([text], "query:")
        return result[0] if result else []


_embedder: Optional[Embedder] = None


def init_embedder(cfg: MemoryConfig) -> Embedder:
    global _embedder
    if _embedder is None:
        _embedder = SentenceTransformerEmbedder(
            model_name=cfg.embedding_model,
            device=cfg.embedding_device,
            dim=cfg.embedding_dim,
        )
    return _embedder


def set_embedder(embedder: Optional[Embedder]) -> None:
    """替换全局向量化实现，供测试与长跑模拟注入确定性假实现。"""
    global _embedder
    _embedder = embedder


def get_embedder() -> Embedder:
    if _embedder is None:
        raise RuntimeError("embedder is not initialised; call init_embedder() first")
    return _embedder
