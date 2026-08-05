from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class StoreConfig:
    dim: int
    metric: str = "cosine_dot"  # 约定：向量已归一化，score=dot(emb, q)


@dataclass(frozen=True)
class StoreFiles:
    embeddings_npy: Path
    meta_jsonl: Path
    config_json: Path


def store_files(index_dir: Path) -> StoreFiles:
    return StoreFiles(
        embeddings_npy=index_dir / "embeddings.npy",
        meta_jsonl=index_dir / "meta.jsonl",
        config_json=index_dir / "store_config.json",
    )


class NumpyVectorStore:
    """
    极简向量库：embeddings.npy + meta.jsonl
    - embeddings: float32, shape (N, dim)，要求已归一化
    - meta.jsonl: 每行一个 dict，与 embeddings 行号一一对应
    """

    def __init__(self, index_dir: Path) -> None:
        self.index_dir = index_dir
        self.files = store_files(index_dir)
        self._cfg: Optional[StoreConfig] = None
        self._emb: Optional[np.ndarray] = None
        self._meta: List[dict] = []
        self._chunkid_set: set[str] = set()

    def exists(self) -> bool:
        return self.files.embeddings_npy.exists() and self.files.meta_jsonl.exists() and self.files.config_json.exists()

    @property
    def config(self) -> StoreConfig:
        if self._cfg is None:
            raise RuntimeError("store config 尚未加载/初始化")
        return self._cfg

    @property
    def size(self) -> int:
        return len(self._meta)

    def load(self) -> None:
        cfg = json.loads(self.files.config_json.read_text(encoding="utf-8"))
        self._cfg = StoreConfig(**cfg)
        self._emb = np.load(self.files.embeddings_npy).astype(np.float32, copy=False)
        self._meta = []
        self._chunkid_set = set()
        with self.files.meta_jsonl.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                m = json.loads(line)
                self._meta.append(m)
                cid = m.get("chunk_id")
                if isinstance(cid, str):
                    self._chunkid_set.add(cid)
        if self._emb.ndim != 2 or self._emb.shape[0] != len(self._meta):
            raise ValueError(f"embeddings/meta 不一致：emb={self._emb.shape} meta={len(self._meta)}")
        if self._emb.shape[1] != self.config.dim:
            raise ValueError(f"dim 不一致：cfg={self.config.dim} emb={self._emb.shape[1]}")

    def init_new(self, cfg: StoreConfig) -> None:
        self.index_dir.mkdir(parents=True, exist_ok=True)
        self._cfg = cfg
        self._emb = np.zeros((0, cfg.dim), dtype=np.float32)
        self._meta = []
        self._chunkid_set = set()
        self.files.config_json.write_text(json.dumps(asdict(cfg), ensure_ascii=False, indent=2), encoding="utf-8")
        self.files.meta_jsonl.write_text("", encoding="utf-8")
        np.save(self.files.embeddings_npy, self._emb)

    def add(self, vectors: np.ndarray, metas: List[dict]) -> int:
        if self._cfg is None or self._emb is None:
            raise RuntimeError("store 未初始化/未加载")
        if vectors.dtype != np.float32:
            vectors = vectors.astype(np.float32)
        if vectors.ndim != 2 or vectors.shape[0] != len(metas):
            raise ValueError("vectors/metas 数量不一致")
        if vectors.shape[1] != self.config.dim:
            raise ValueError(f"dim 不匹配：store={self.config.dim} vectors={vectors.shape[1]}")

        keep_vecs: list[np.ndarray] = []
        keep_metas: list[dict] = []
        for v, m in zip(vectors, metas):
            cid = m.get("chunk_id")
            if isinstance(cid, str) and cid in self._chunkid_set:
                continue
            keep_vecs.append(v)
            keep_metas.append(m)

        if not keep_metas:
            return 0

        add_mat = np.vstack(keep_vecs).astype(np.float32, copy=False)
        self._emb = np.vstack([self._emb, add_mat])
        self._meta.extend(keep_metas)
        for m in keep_metas:
            cid = m.get("chunk_id")
            if isinstance(cid, str):
                self._chunkid_set.add(cid)

        # 持久化：简单起见每次重写（50 本书规模 OK）
        np.save(self.files.embeddings_npy, self._emb)
        with self.files.meta_jsonl.open("a", encoding="utf-8") as f:
            for m in keep_metas:
                f.write(json.dumps(m, ensure_ascii=False) + "\n")
        return len(keep_metas)

    def search(self, query_vec: np.ndarray, top_k: int = 5) -> List[Tuple[dict, float]]:
        if self._cfg is None or self._emb is None:
            raise RuntimeError("store 未初始化/未加载")
        q = query_vec.astype(np.float32, copy=False)
        if q.ndim != 1 or q.shape[0] != self.config.dim:
            raise ValueError(f"query dim 不匹配：expected {self.config.dim}, got {q.shape}")

        # 已归一化：score=dot
        scores = self._emb @ q  # (N,)
        if self.size == 0:
            return []
        k = min(int(top_k), self.size)
        idx = np.argpartition(-scores, kth=k - 1)[:k]
        idx = idx[np.argsort(-scores[idx])]
        return [(self._meta[int(i)], float(scores[int(i)])) for i in idx]


