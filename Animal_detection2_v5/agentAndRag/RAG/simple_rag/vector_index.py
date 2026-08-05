from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from .text_utils import TextChunk


@dataclass(frozen=True)
class IndexFiles:
    index_bin: Path
    meta_jsonl: Path
    config_json: Path


def index_files(index_dir: Path) -> IndexFiles:
    return IndexFiles(
        index_bin=index_dir / "index_hnsw.bin",
        meta_jsonl=index_dir / "meta.jsonl",
        config_json=index_dir / "index_config.json",
    )


@dataclass
class IndexConfig:
    dim: int
    space: str = "cosine"  # cosine or l2
    ef_construction: int = 200
    M: int = 16


class HnswIndex:
    """
    hnswlib 索引 + meta.jsonl（行号对应 label id）。
    - label: int（从 0 开始递增）
    - meta: TextChunk 的字典形式
    """

    def __init__(self, index_dir: Path) -> None:
        self.index_dir = index_dir
        self.files = index_files(index_dir)
        self._index = None
        self._config: Optional[IndexConfig] = None
        self._id_to_meta: Dict[int, dict] = {}
        self._chunkid_set: set[str] = set()

    @property
    def config(self) -> IndexConfig:
        if self._config is None:
            raise RuntimeError("index config 尚未加载/初始化")
        return self._config

    @property
    def size(self) -> int:
        return len(self._id_to_meta)

    def exists(self) -> bool:
        return self.files.index_bin.exists() and self.files.meta_jsonl.exists() and self.files.config_json.exists()

    def load(self) -> None:
        import hnswlib

        cfg = json.loads(self.files.config_json.read_text(encoding="utf-8"))
        self._config = IndexConfig(**cfg)

        p = hnswlib.Index(space=self._config.space, dim=self._config.dim)
        p.load_index(str(self.files.index_bin))
        self._index = p

        self._id_to_meta = {}
        self._chunkid_set = set()
        with self.files.meta_jsonl.open("r", encoding="utf-8") as f:
            for line_no, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                meta = json.loads(line)
                self._id_to_meta[line_no] = meta
                cid = meta.get("chunk_id")
                if isinstance(cid, str):
                    self._chunkid_set.add(cid)

    def init_new(self, config: IndexConfig, max_elements: int) -> None:
        import hnswlib

        self.index_dir.mkdir(parents=True, exist_ok=True)
        self._config = config
        p = hnswlib.Index(space=config.space, dim=config.dim)
        p.init_index(max_elements=max_elements, ef_construction=config.ef_construction, M=config.M)
        # 查询质量/速度权衡（越大越准但越慢）
        p.set_ef(min(128, max(16, max_elements)))
        self._index = p

        self._id_to_meta = {}
        self._chunkid_set = set()

        self.files.config_json.write_text(json.dumps(asdict(config), ensure_ascii=False, indent=2), encoding="utf-8")
        self.files.meta_jsonl.write_text("", encoding="utf-8")

    def _ensure_capacity(self, target_size: int) -> None:
        if self._index is None:
            raise RuntimeError("index 未初始化/未加载")
        # hnswlib 支持扩容
        cur_max = self._index.get_max_elements()
        if target_size <= cur_max:
            return
        new_max = max(target_size, int(cur_max * 1.5) + 1)
        self._index.resize_index(new_max)

    def add(self, vectors: np.ndarray, metas: List[dict]) -> int:
        if self._index is None:
            raise RuntimeError("index 未初始化/未加载")
        if vectors.dtype != np.float32:
            vectors = vectors.astype(np.float32)
        if vectors.ndim != 2 or vectors.shape[0] != len(metas):
            raise ValueError("vectors/metas 数量不一致")
        if vectors.shape[1] != self.config.dim:
            raise ValueError(f"dim 不匹配：index={self.config.dim} vectors={vectors.shape[1]}")

        # 去重：跳过 chunk_id 已存在的
        keep_vecs: list[np.ndarray] = []
        keep_meta: list[dict] = []
        for v, m in zip(vectors, metas):
            cid = m.get("chunk_id")
            if isinstance(cid, str) and cid in self._chunkid_set:
                continue
            keep_vecs.append(v)
            keep_meta.append(m)

        if not keep_meta:
            return 0

        start = self.size
        n_add = len(keep_meta)
        self._ensure_capacity(start + n_add)
        labels = np.arange(start, start + n_add, dtype=np.int64)
        self._index.add_items(np.vstack(keep_vecs), labels)

        with self.files.meta_jsonl.open("a", encoding="utf-8") as f:
            for i, m in enumerate(keep_meta):
                label = int(labels[i])
                self._id_to_meta[label] = m
                cid = m.get("chunk_id")
                if isinstance(cid, str):
                    self._chunkid_set.add(cid)
                f.write(json.dumps(m, ensure_ascii=False) + "\n")
        return n_add

    def save(self) -> None:
        if self._index is None:
            raise RuntimeError("index 未初始化/未加载")
        self.index_dir.mkdir(parents=True, exist_ok=True)
        self._index.save_index(str(self.files.index_bin))

    def search(self, query_vec: np.ndarray, top_k: int = 5) -> List[Tuple[dict, float]]:
        if self._index is None:
            raise RuntimeError("index 未初始化/未加载")
        if query_vec.dtype != np.float32:
            query_vec = query_vec.astype(np.float32)
        if query_vec.ndim == 1:
            query_vec = query_vec.reshape(1, -1)
        labels, distances = self._index.knn_query(query_vec, k=top_k)
        labs = labels[0].tolist()
        dists = distances[0].tolist()

        out: List[Tuple[dict, float]] = []
        for lab, dist in zip(labs, dists):
            meta = self._id_to_meta.get(int(lab))
            if meta is None:
                continue
            # cosine: hnswlib 返回距离 = 1 - cosine_similarity（越小越相似）
            score = 1.0 - float(dist) if self.config.space == "cosine" else -float(dist)
            out.append((meta, score))
        return out


def chunk_to_meta(chunk: TextChunk) -> dict:
    return {
        "chunk_id": chunk.chunk_id,
        "source_path": chunk.source_path,
        "source_sha1": chunk.source_sha1,
        "chunk_index": chunk.chunk_index,
        "n_words": chunk.n_words,
        "text": chunk.text,
    }


