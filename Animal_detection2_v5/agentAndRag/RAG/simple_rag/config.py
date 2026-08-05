from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RagConfig:
    # 输入数据目录：你的 .mmd 书籍文件都在这里（支持递归）
    raw_dir: Path

    # 索引输出目录：会生成 index.bin / meta.jsonl 等文件
    index_dir: Path

    # embedding 模型（sentence-transformers）
    embedding_model: str = "sentence-transformers/all-MiniLM-L6-v2"

    # 分块参数（按“词”近似）
    chunk_words: int = 800
    chunk_overlap_words: int = 150

    # 过滤太短的块，避免垃圾块占索引
    min_chunk_words: int = 40


def default_config(repo_root: Path) -> RagConfig:
    """
    repo_root: DeepSeek-OCR 仓库根目录（也就是包含 RAG/ 的目录）
    """
    rag_dir = repo_root / "RAG"
    return RagConfig(
        raw_dir=rag_dir / "data" / "raw",
        index_dir=rag_dir / "data" / "rag_index_e5",
    )


