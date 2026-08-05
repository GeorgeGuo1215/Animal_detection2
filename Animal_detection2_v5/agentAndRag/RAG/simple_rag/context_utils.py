from __future__ import annotations

from typing import Dict, List, Optional, Tuple


def _merge_intervals(intervals: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    if not intervals:
        return []
    ivs = sorted((int(l), int(r)) for l, r in intervals)
    out: List[Tuple[int, int]] = []
    cur_l, cur_r = ivs[0]
    for l, r in ivs[1:]:
        if l <= cur_r + 1:
            cur_r = max(cur_r, r)
        else:
            out.append((cur_l, cur_r))
            cur_l, cur_r = l, r
    out.append((cur_l, cur_r))
    return out


def build_source_index(metas: List[dict]) -> Dict[str, Dict[int, str]]:
    """
    Pre-build source_path -> {chunk_index: text} index.
    Call once at warmup; reuse across requests.
    """
    by_src: Dict[str, Dict[int, str]] = {}
    for m in metas:
        sp = m.get("source_path")
        ci = m.get("chunk_index")
        txt = m.get("text")
        if not isinstance(sp, str) or not sp:
            continue
        try:
            cii = int(ci)
        except Exception:
            continue
        if not isinstance(txt, str) or not txt.strip():
            continue
        by_src.setdefault(sp, {})[cii] = txt
    return by_src


def build_neighbor_contexts(
    *,
    metas: List[dict],
    hits: List[dict],
    neighbor_n: int,
    _source_index: Optional[Dict[str, Dict[int, str]]] = None,
) -> List[dict]:
    """
    小块检索命中后，把同一 source_path 的邻居 chunk 拼接成更完整的 context。

    入参约定：
    - metas: 来自 meta.jsonl 的列表（每项至少含 source_path/chunk_index/text）
    - hits: 检索命中列表（每项至少含 source_path/chunk_index/score）
    - _source_index: 可选预建索引，若为 None 则从 metas 重建

    返回：
    - contexts: List[dict]
    """
    n = int(neighbor_n)
    if n <= 0 or not hits:
        return []

    by_src = _source_index if _source_index is not None else build_source_index(metas)

    win_by_src: Dict[str, List[Tuple[int, int]]] = {}
    hit_scores: Dict[str, Dict[int, float]] = {}
    for h in hits:
        sp = h.get("source_path")
        ci = h.get("chunk_index")
        if not isinstance(sp, str) or not sp:
            continue
        try:
            cii = int(ci)
        except Exception:
            continue
        l, r = cii - n, cii + n
        win_by_src.setdefault(sp, []).append((l, r))
        sp_scores = hit_scores.setdefault(sp, {})
        try:
            sc = float(h.get("score") or 0.0)
        except Exception:
            sc = 0.0
        if cii not in sp_scores or sc > sp_scores[cii]:
            sp_scores[cii] = sc

    contexts: List[dict] = []
    for sp, ivs in win_by_src.items():
        merged = _merge_intervals(ivs)
        chunks = by_src.get(sp, {})
        sp_sc = hit_scores.get(sp, {})
        for l, r in merged:
            parts: List[str] = []
            for idx in range(int(l), int(r) + 1):
                t = chunks.get(idx)
                if t:
                    parts.append(t.strip())
            if not parts:
                continue
            text = "\n\n".join(parts).strip()

            best = max(
                (sp_sc[ci] for ci in range(int(l), int(r) + 1) if ci in sp_sc),
                default=0.0,
            )

            contexts.append(
                {
                    "score": float(best),
                    "source_path": sp,
                    "chunk_index_start": int(l),
                    "chunk_index_end": int(r),
                    "n_chunks": int(len(parts)),
                    "text": text,
                }
            )

    contexts.sort(key=lambda x: float(x.get("score") or 0.0), reverse=True)
    return contexts
