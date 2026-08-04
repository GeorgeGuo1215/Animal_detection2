"""
Hugging Face Hub 本地缓存路径解析（与 sentence-transformers 加载约定一致）。

请求里若仍传 Hub id（如 intfloat/multilingual-e5-small），在 HF_HUB_OFFLINE=1 时会触发 API 检查失败；
此处将 Hub id / 缓存顶层目录统一解析到 snapshots/<revision>/ 或含 config.json 的目录。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def hf_home() -> Path:
    h = os.getenv("HF_HOME", "").strip()
    if h:
        return Path(os.path.expanduser(h))
    return Path(os.path.expanduser("~")) / ".cache" / "huggingface"


def hf_hub_cache_repo_dir(repo_id: str) -> Path:
    rid = (repo_id or "").strip()
    return hf_home() / "hub" / f"models--{rid.replace('/', '--')}"


def resolve_hf_hub_weights_dir(root: Path) -> Optional[Path]:
    """缓存顶层目录 -> snapshots/<revision>/；已是权重目录则原样返回。"""
    if not root.is_dir():
        return None
    if (root / "config.json").is_file():
        return root
    snaps = root / "snapshots"
    if not snaps.is_dir():
        return None
    for ref_rel in ("refs/main", "refs/heads/main"):
        ref = root / ref_rel
        if ref.is_file():
            try:
                rev = ref.read_text(encoding="utf-8").strip()
                if rev:
                    p = snaps / rev
                    if p.is_dir() and (p / "config.json").is_file():
                        return p
            except OSError:
                pass
    candidates = [p for p in snaps.iterdir() if p.is_dir() and (p / "config.json").is_file()]
    if not candidates:
        candidates = [p for p in snaps.iterdir() if p.is_dir()]
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        return sorted(candidates, key=lambda x: x.name)[-1]
    return None


def expand_user_path(p: str) -> str:
    return os.path.expanduser(p.strip().strip('"').strip("'"))


def maybe_resolve_hf_cache_path(p: str) -> str:
    root = Path(expand_user_path(p))
    resolved = resolve_hf_hub_weights_dir(root)
    return str(resolved) if resolved else str(root)


def is_local_path(p: str) -> bool:
    try:
        return os.path.isdir(p) or os.path.isfile(p)
    except OSError:
        return False


def resolve_embedding_model_id(requested: Optional[str], repo_root: Path) -> str:
    """
    与 main 启动预热相同的解析顺序；requested 为空则用 AGENT_WARMUP_EMBEDDING_MODEL / 默认 Hub id。
    """
    req = (requested or "").strip() or os.getenv("AGENT_WARMUP_EMBEDDING_MODEL", "intfloat/multilingual-e5-small").strip()
    embed_override = os.getenv("AGENT_EMBEDDING_MODEL_PATH", "").strip()
    if embed_override:
        return maybe_resolve_hf_cache_path(embed_override)

    local_embed = repo_root / "models" / "multilingual-e5-small"
    if local_embed.is_dir():
        resolved = resolve_hf_hub_weights_dir(local_embed)
        return str(resolved) if resolved else str(local_embed)

    if is_local_path(expand_user_path(req)):
        return maybe_resolve_hf_cache_path(req)

    if "/" in req:
        cache_embed = resolve_hf_hub_weights_dir(hf_hub_cache_repo_dir(req))
        if cache_embed:
            return str(cache_embed)

    return req


def resolve_rerank_model_id(requested: Optional[str], repo_root: Path) -> str:
    req = (requested or "").strip() or os.getenv("AGENT_WARMUP_RERANK_MODEL", "BAAI/bge-reranker-large").strip()
    rerank_override = os.getenv("AGENT_RERANKER_MODEL_PATH", "").strip()
    if rerank_override:
        return maybe_resolve_hf_cache_path(rerank_override)

    local_rr = repo_root / "models" / "bge-reranker-large"
    if local_rr.is_dir():
        resolved = resolve_hf_hub_weights_dir(local_rr)
        return str(resolved) if resolved else str(local_rr)

    if is_local_path(expand_user_path(req)):
        return maybe_resolve_hf_cache_path(req)

    if "/" in req:
        cache_rr = resolve_hf_hub_weights_dir(hf_hub_cache_repo_dir(req))
        if cache_rr:
            return str(cache_rr)

    return req
