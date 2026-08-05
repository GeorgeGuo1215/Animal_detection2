from __future__ import annotations

"""
Context 检查器（不调用 LLM）

用途：
- 对同一道选择题（query / question_full）在不同 K 下进行检索
- 输出：
  1) rerank 前的 topK 小块文本（raw）
  2) rerank 后的 topK 小块文本（reranked）
  3) 邻居拼接前/后的 contexts（可选 expand_neighbors）
  4) 截断（clip）前/后的 contexts
- 同时给出便于检查的指标（chars、overlap、unique books、是否发生截断等）

设计目标：
- 逻辑风格尽量贴近 run_one_click_sweep.py（固定 CONFIG + 可复现 + 可落盘）
- 只做检索/重排/拼接，不调用大模型
"""

import json
import os
import re
import sys
import hashlib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve()
# 允许直接运行：python RAG/experiments/inspect_qa_contexts.py
sys.path.insert(0, str(HERE.parent))      # experiments/
sys.path.insert(0, str(HERE.parents[1]))  # RAG/

from common import read_jsonl  # noqa: E402
from retrievers import BM25Retriever, DenseRetriever, HybridRetriever, TwoStageBookThenChunk  # noqa: E402
from simple_rag.context_utils import build_neighbor_contexts  # noqa: E402
from simple_rag.reranker import CrossEncoderReranker  # noqa: E402
from simple_rag.vector_store import NumpyVectorStore  # noqa: E402


# ------------------------------------------------------------
# CONFIG：按需修改（仿照 sweep.py 的用法）
# ------------------------------------------------------------

CONFIG = {
    # 输入 evalset（由 build_qa_evalset_from_alpaca.py 生成）
    "qa_eval": "RAG/experiments/out/qa_eval.jsonl",
    # 输出目录：每题一个 markdown + 总 trace.jsonl
    "out_dir": "RAG/experiments/out/context_inspect",
    "trace_jsonl": "RAG/experiments/out/context_inspect/trace.jsonl",
    # 只检查前 N 题；None 表示全量
    "limit": 1,
    # 也可指定 qid 白名单（非空时优先）
    "only_qids": ["300QuestionsInAds_alpaca:106"],

    # 不同 K 值（你要对比的）
    "ks": [5],
    # 检索方案（与 run_one_click_sweep.py 对齐）
    "retrievers": ["hybrid"],

    # 不同 embedding/index（可只留一套）
    "index_dirs": [
        {"index_dir": "RAG/data/rag_index_e5", "embedding_model": "intfloat/multilingual-e5-small"},
    ],

    # 邻居拼接：0=关闭；1=拼 i-1..i+1；2=拼 i-2..i+2
    "expand_neighbors": 0,

    # reranker：对召回的小块候选进行重排，然后再做邻居拼接
    "enable_rerank": True,
    "rerank_model": "BAAI/bge-reranker-large",
    # rerank 候选数（建议 10~30）
    "rerank_candidates": 20,
    "rerank_batch_size": 32,
    "rerank_device": None,  # 可选 cpu/cuda
    # rerank 后去噪过滤（与你的 sweep 配置保持一致）
    "rerank_filter_overlap": 0.15,
    "rerank_keep_topn": 5,
    # option-aware 数字过滤 + RAG gate
    "enable_option_numeric_filter": True,
    "option_numeric_min_keep": 1,
    "option_numeric_keep_topn": 0,
    "enable_rag_gate": True,
    "gate_min_ctx": 1,
    "gate_min_overlap_max": 0.12,
    "enable_option_coverage_gate": True,
    "gate_min_option_hits": 1,

    # 额外抗噪：与 run_one_click_sweep.py 对齐
    "enable_garbled_filter": True,
    "garbled_min_alpha_ratio": 0.45,
    "garbled_min_word_per_100": 4.0,
    "enable_option_ambiguity_gate": True,
    "option_ambiguity_max_hit_options": 2,

    # contexts 截断预算（用于“截断前后对比”）
    "per_ctx_max_chars": 5000,
    "total_max_chars": 24000,

    # 输出时单段最多打印多少字符（避免 markdown 爆炸；不影响 trace_jsonl 里的原始文本）
    "md_print_max_chars": 3000,

    # ------------------------------------------------------------
    # 可选：端到端选择题 Accuracy（调用 LLM）
    # 默认关闭，避免你只想看 contexts 时误触发大模型调用
    # ------------------------------------------------------------
    "enable_llm_accuracy": False,
    # 选用哪个 stage 的 contexts 来喂给 LLM（推荐最“干净”的 rerank_neighbor_filtered）
    "accuracy_stage": "rerank_neighbor_filtered",
    # 实时打印频率：每 N 题打印一次滚动准确率
    "accuracy_print_every": 10,
    # LLM 配置（OpenAI 兼容；默认 DeepSeek）
    "llm": {
        "base_url": os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com"),
        "model": os.getenv("OPENAI_MODEL", "deepseek-reasoner"),
        "api_key": os.getenv("OPENAI_API_KEY", "") or os.getenv("API_KEY", "") or os.getenv("DEEPSEEK_API_KEY", ""),
        "timeout": 1200,
        "enable_thinking": False,
    },
}


_RE_WORD = re.compile(r"[A-Za-z][A-Za-z0-9\\-]{2,}")
_RE_NUM = re.compile(
    r"\b(\d+(?:\.\d+)?)\s*(hours?|hrs?|hr|h|days?|d|minutes?|mins?|min|msec|ms|sec|s|"
    r"mg/kg|mg|g/kg|g|kg|ml|l|nm|um|μm|mm|cm|m)\b",
    re.IGNORECASE,
)
EN_STOP_MINI = {
    "the",
    "a",
    "an",
    "and",
    "or",
    "to",
    "of",
    "in",
    "on",
    "for",
    "with",
    "as",
    "is",
    "are",
    "was",
    "were",
    "be",
    "by",
    "that",
    "this",
    "it",
    "from",
    "at",
    "into",
    "during",
    "within",
    "without",
    "over",
    "under",
    "between",
}

OPTION_STOP_EXTRA = {
    "all",
    "above",
    "none",
    "except",
    "true",
    "false",
    "both",
    "either",
    "neither",
    "following",
    "statement",
    "least",
    "most",
    "type",
    "produced",
    "light",
    "lamp",
}

_RE_CHOICE = re.compile(r"\b([ABCD])\b", re.IGNORECASE)


def normalize_choice(text: str) -> Optional[str]:
    """
    把模型输出/标注里形如 'A' / 'A)' / '答案是 A' 提取成 'A'/'B'/'C'/'D'。
    """
    if not text:
        return None
    m = _RE_CHOICE.search(str(text))
    if not m:
        return None
    return m.group(1).upper()


def call_llm_choice(*, llm: dict, question_full: str, contexts: list[str]) -> tuple[str, str, str, str]:
    """
    端到端选择题：只让模型输出 A/B/C/D。
    返回 (pred_raw, reasoning, system_prompt, user_prompt)
    """
    from openai import OpenAI

    client = OpenAI(api_key=llm["api_key"], base_url=llm["base_url"], timeout=int(llm["timeout"]))
    ctx = "\n\n".join([f"[Context {i+1}]\n{c}" for i, c in enumerate(contexts or [])])
    system = (
        "你是一个专业的兽医领域选择题作答助手。"
        "你会优先依据给定的 Context 作答；若 Context 不足以支持，请依据常识作答。"
        "最终只输出一个选项字母：A/B/C/D。不要输出其他内容。"
    )
    user = f"{ctx}\n\n[Question]\n{question_full}\n\n请只输出最终选项字母："
    kwargs = {
        "model": llm["model"],
        "messages": [{"role": "system", "content": system}, {"role": "user", "content": user}],
        "temperature": 0.0,
        "stream": False,
    }
    if llm.get("enable_thinking") and "reasoner" in str(llm.get("model", "")).lower():
        kwargs["extra_body"] = {"thinking": {"type": "enabled"}}
    resp = client.chat.completions.create(**kwargs)
    msg = resp.choices[0].message
    content = (getattr(msg, "content", None) or "").strip()
    reasoning = (getattr(msg, "reasoning_content", None) or "").strip()
    return content, reasoning, system, user


def _tokens(text: str) -> List[str]:
    return [t.lower() for t in _RE_WORD.findall(text or "")]


def overlap_score(query: str, ctx: str) -> float:
    tq = set(_tokens(query))
    if not tq:
        return 0.0
    tc = set(_tokens(ctx))
    if not tc:
        return 0.0
    return len(tq & tc) / float(len(tq))


def _norm_text(s: str) -> str:
    return re.sub(r"\s+", " ", (s or "").strip())


def _text_hash(s: str) -> str:
    return hashlib.sha1(_norm_text(s).encode("utf-8")).hexdigest()


def dedup_by_text(
    *,
    hits: List[dict],
    cid2txt: Dict[str, str],
) -> Tuple[List[dict], List[str], dict]:
    """
    对 hits 去重：若不同 chunk_id 但 text 完全相同（常见于重复导入的同一本书副本），只保留一次。
    返回 (hits_dedup, ctxs_dedup, stats)
    """
    out_hits: List[dict] = []
    out_ctx: List[str] = []
    seen = set()
    dup = 0
    for h in hits:
        cid = str(h.get("chunk_id") or "")
        txt = cid2txt.get(cid, "") if cid else ""
        th = _text_hash(txt)
        if th in seen:
            dup += 1
            continue
        seen.add(th)
        hh = dict(h)
        hh["text_hash"] = th
        hh["rank"] = len(out_hits) + 1
        out_hits.append(hh)
        out_ctx.append(txt)
    return out_hits, out_ctx, {"dedup_removed": dup, "dedup_kept": len(out_hits)}


def _alpha_ratio(s: str) -> float:
    s0 = (s or "")
    if not s0:
        return 0.0
    nonspace = [ch for ch in s0 if not ch.isspace()]
    if not nonspace:
        return 0.0
    alpha = sum(1 for ch in nonspace if ("A" <= ch <= "Z") or ("a" <= ch <= "z"))
    return alpha / float(len(nonspace))


def _word_per_100(s: str) -> float:
    s0 = (s or "")
    n = len(s0)
    if n <= 0:
        return 0.0
    w = len(_RE_WORD.findall(s0))
    return w / (n / 100.0)


def filter_garbled_contexts(
    *,
    ctxs: list[str],
    min_alpha_ratio: float,
    min_word_per_100: float,
) -> tuple[list[str], dict]:
    kept: list[str] = []
    dropped: list[dict] = []
    for i, c in enumerate(ctxs):
        c0 = (c or "").strip()
        if not c0:
            continue
        if len(c0) < 80:
            kept.append(c0)
            continue
        ar = _alpha_ratio(c0)
        wp = _word_per_100(c0)
        if ar < float(min_alpha_ratio) and wp < float(min_word_per_100):
            dropped.append({"i": i, "alpha_ratio": ar, "word_per_100": wp, "len": len(c0)})
            continue
        kept.append(c0)
    return kept, {
        "enabled": True,
        "min_alpha_ratio": float(min_alpha_ratio),
        "min_word_per_100": float(min_word_per_100),
        "dropped_n": len(dropped),
        "kept_n": len(kept),
        "dropped_preview": dropped[:10],
    }


def parse_mcq_options(question_full: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for ln in (question_full or "").splitlines():
        m = re.match(r"^\s*([a-dA-D])\)\s*(.+?)\s*$", ln)
        if not m:
            continue
        out[m.group(1).upper()] = m.group(2).strip()
    return out


def option_ambiguity_gate(
    *,
    query: str,
    question_full: str,
    ctxs: list[str],
    max_hit_options: int,
) -> tuple[bool, dict]:
    opts = parse_mcq_options(question_full)
    if not opts or not ctxs:
        return False, {"enabled": False, "reason": "no_options_or_no_context"}
    tq = set(_tokens(query))
    ctx_tokens = [set(_tokens(c)) for c in ctxs]

    opt_kw: dict[str, set[str]] = {}
    for k, txt in opts.items():
        toks = set(_tokens(txt))
        toks = {t for t in toks if t not in EN_STOP_MINI and t not in OPTION_STOP_EXTRA and t not in tq}
        if toks:
            opt_kw[k] = toks
    if not opt_kw:
        return False, {"enabled": False, "reason": "no_distinct_keywords"}

    opt_hits: dict[str, int] = {}
    hit_opts: list[str] = []
    for k, kws in opt_kw.items():
        need = 1 if len(kws) <= 3 else 2
        best = 0
        for toks in ctx_tokens:
            best = max(best, len(kws & toks))
        opt_hits[k] = int(best)
        if best >= need:
            hit_opts.append(k)

    trig = len(hit_opts) > int(max_hit_options)
    return trig, {
        "enabled": True,
        "max_hit_options": int(max_hit_options),
        "options": opts,
        "distinct_keywords": {k: sorted(list(v))[:30] for k, v in opt_kw.items()},
        "opt_hits": opt_hits,
        "hit_opts": sorted(hit_opts),
        "triggered": trig,
    }

def extract_num_units(text: str) -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}
    for num, unit in _RE_NUM.findall(text or ""):
        u = (unit or "").lower()
        if u in {"h", "hr", "hrs", "hour", "hours"}:
            u = "hour"
        elif u in {"d", "day", "days"}:
            u = "day"
        elif u in {"min", "mins", "minute", "minutes"}:
            u = "min"
        elif u in {"s", "sec"}:
            u = "sec"
        out.setdefault(u, set()).add(str(num))
    return out


def extract_units_only(text: str) -> set[str]:
    units = set()
    for _num, unit in _RE_NUM.findall(text or ""):
        u = (unit or "").lower()
        if u in {"h", "hr", "hrs", "hour", "hours"}:
            u = "hour"
        elif u in {"d", "day", "days"}:
            u = "day"
        elif u in {"min", "mins", "minute", "minutes"}:
            u = "min"
        elif u in {"s", "sec"}:
            u = "sec"
        units.add(u)
    return units


def extract_option_keywords(question_full: str, *, min_len: int = 4) -> set[str]:
    toks = _RE_WORD.findall(question_full or "")
    out = set()
    for t in toks:
        w = t.lower()
        if len(w) < int(min_len):
            continue
        if w in EN_STOP_MINI:
            continue
        out.add(w)
    return out


def option_aware_numeric_filter(
    *,
    question_full: str,
    ctxs: list[str],
    ctx_meta: list[dict],
    min_keep: int,
    keep_topn: int,
) -> tuple[list[str], list[dict], dict]:
    allowed_nums = extract_num_units(question_full)
    allowed_units = extract_units_only(question_full)
    allowed_nums = {u: ns for u, ns in allowed_nums.items() if ns}
    protected = extract_option_keywords(question_full)
    if not allowed_nums and not allowed_units:
        return ctxs, ctx_meta, {"enabled": False, "reason": "no_numeric_or_units_in_options"}

    keep_idx: list[int] = []
    drop: list[dict] = []
    for i, c in enumerate(ctxs):
        c0 = c or ""
        toks = set(t.lower() for t in _RE_WORD.findall(c0))
        has_protected = bool(protected and (toks & protected))

        nums = extract_num_units(c0)
        units = extract_units_only(c0)
        bad_units = []
        for u, allow_set in allowed_nums.items():
            if u not in nums:
                continue
            have = nums.get(u) or set()
            if not (have & allow_set):
                bad_units.append({"unit": u, "have": sorted(have)[:10], "allowed": sorted(allow_set)[:10]})

        unit_mismatch = False
        mismatch_detail = None
        if allowed_units and units:
            if not (units & allowed_units):
                unit_mismatch = True
                mismatch_detail = {"have_units": sorted(units)[:10], "allowed_units": sorted(allowed_units)[:10]}

        if (bad_units or unit_mismatch) and (not has_protected):
            drop.append({"i": i, "bad_units": bad_units, "unit_mismatch": unit_mismatch, "unit_detail": mismatch_detail})
        else:
            keep_idx.append(i)

    if len(keep_idx) < int(min_keep):
        return ctxs, ctx_meta, {"enabled": True, "fallback": True, "kept": len(keep_idx), "drop_n": len(drop), "drop": drop[:5], "protected_keywords_n": len(protected)}

    if int(keep_topn) > 0:
        keep_idx = keep_idx[: int(keep_topn)]

    new_ctxs = [ctxs[i] for i in keep_idx]
    new_meta = [ctx_meta[i] for i in keep_idx] if len(ctx_meta) == len(ctxs) else ctx_meta
    return new_ctxs, new_meta, {
        "enabled": True,
        "fallback": False,
        "allowed_nums": {u: sorted(ns)[:10] for u, ns in allowed_nums.items()},
        "allowed_units": sorted(allowed_units)[:20],
        "protected_keywords_n": len(protected),
        "kept": len(new_ctxs),
        "drop_n": len(drop),
        "drop": drop[:5],
    }


def rag_gate(
    *,
    query: str,
    question_full: str,
    ctxs: list[str],
    min_ctx: int,
    min_overlap_max: float,
) -> tuple[bool, dict]:
    if len(ctxs) < int(min_ctx):
        return True, {"trigger": "too_few_ctx", "n_ctx": len(ctxs), "min_ctx": int(min_ctx)}
    q = (question_full or query or "").strip()
    ovs = [overlap_score(q, c) for c in ctxs]
    mx = max(ovs) if ovs else 0.0
    if mx < float(min_overlap_max):
        return True, {"trigger": "low_overlap", "overlap_max": mx, "min_overlap_max": float(min_overlap_max)}
    return False, {"trigger": None, "overlap_max": mx, "n_ctx": len(ctxs)}


def option_coverage_gate(*, question_full: str, ctxs: list[str], min_hit: int) -> tuple[bool, dict]:
    kws = extract_option_keywords(question_full)
    if not kws:
        return False, {"trigger": None, "reason": "no_keywords"}
    hit = 0
    for c in ctxs:
        toks = set(t.lower() for t in _RE_WORD.findall(c or ""))
        if toks & kws:
            hit += 1
    if hit < int(min_hit):
        return True, {"trigger": "low_option_coverage", "hit": hit, "min_hit": int(min_hit), "kw_n": len(kws)}
    return False, {"trigger": None, "hit": hit, "kw_n": len(kws)}


def clip_contexts(ctxs: List[str], *, per_ctx_max_chars: int, total_max_chars: int) -> Tuple[List[str], dict]:
    out: List[str] = []
    used = 0
    truncated = 0
    skipped_empty = 0
    for c in ctxs:
        if used >= total_max_chars:
            break
        c = (c or "").strip()
        if not c:
            skipped_empty += 1
            continue
        if len(c) > per_ctx_max_chars:
            c = c[:per_ctx_max_chars] + "\n...(截断)..."
            truncated += 1
        if used + len(c) > total_max_chars:
            c = c[: max(0, total_max_chars - used)]
            truncated += 1
        out.append(c)
        used += len(c)
    stats = {
        "n_in": len(ctxs),
        "n_out": len(out),
        "total_chars_out": used,
        "truncated": truncated,
        "skipped_empty": skipped_empty,
        "per_ctx_max_chars": int(per_ctx_max_chars),
        "total_max_chars": int(total_max_chars),
    }
    return out, stats


def build_chunkid_to_text(metas: List[dict]) -> Dict[str, str]:
    m: Dict[str, str] = {}
    for row in metas:
        cid = row.get("chunk_id")
        txt = row.get("text")
        if isinstance(cid, str) and isinstance(txt, str) and txt:
            m[cid] = txt
    return m


def build_chunkid_to_meta(metas: List[dict]) -> Dict[str, dict]:
    m: Dict[str, dict] = {}
    for row in metas:
        cid = row.get("chunk_id")
        if isinstance(cid, str) and cid:
            m[cid] = row
    return m


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


@dataclass
class StageView:
    name: str
    hits: List[dict]
    ctxs_raw: List[str]
    ctxs_clipped: List[str]
    clip_stats: dict
    stats: dict


def summarize_contexts(query: str, ctxs: List[str]) -> dict:
    if not ctxs:
        return {"n": 0, "total_chars": 0, "overlap_avg": 0.0, "overlap_max": 0.0}
    ovs = [overlap_score(query, c) for c in ctxs]
    return {
        "n": len(ctxs),
        "total_chars": sum(len(c) for c in ctxs),
        "overlap_avg": sum(ovs) / float(len(ovs)),
        "overlap_max": max(ovs) if ovs else 0.0,
    }


def retrieve_hits(
    *,
    retriever_name: str,
    query: str,
    k: int,
    index_dir: Path,
    embedding_model: str,
    metas: List[dict],
) -> List:
    if retriever_name == "dense":
        r = DenseRetriever(index_dir, embedding_model)
        return r.retrieve(query, top_k=int(k))
    if retriever_name == "bm25":
        r = BM25Retriever(metas)
        return r.retrieve(query, top_k=int(k))
    if retriever_name == "hybrid":
        r = HybridRetriever(index_dir, embedding_model)
        return r.retrieve(query, top_k=int(k))
    if retriever_name == "two_stage":
        r = TwoStageBookThenChunk(index_dir, embedding_model)
        return r.retrieve(query, top_k=int(k), top_k_books=3)
    raise ValueError(f"未知 retriever: {retriever_name}")


def make_stage_views(
    *,
    q: str,
    question_full: str,
    hits_obj: List,
    cid2txt: Dict[str, str],
    cid2meta: Dict[str, dict],
    metas: List[dict],
    k: int,
    enable_rerank: bool,
    reranker: Optional[CrossEncoderReranker],
    expand_neighbors: int,
    per_ctx_max_chars: int,
    total_max_chars: int,
) -> List[StageView]:
    """
    返回 4 个 stage：
    - retrieval_raw
    - retrieval_neighbor
    - rerank_raw（若 enable_rerank）
    - rerank_neighbor（若 enable_rerank）
    """
    stages: List[StageView] = []

    # ---- 基础：retrieval hits -> dict
    hits: List[dict] = []
    for rank, h in enumerate(hits_obj[: int(k)], start=1):
        m = cid2meta.get(getattr(h, "chunk_id", ""))
        hits.append(
            {
                "rank": rank,
                "chunk_id": getattr(h, "chunk_id", None),
                "book_id": getattr(h, "book_id", None),
                "score": float(getattr(h, "score", 0.0) or 0.0),
                "source_path": (m or {}).get("source_path"),
                "chunk_index": (m or {}).get("chunk_index"),
            }
        )
    hits_d, ctxs_raw, dstat = dedup_by_text(hits=hits, cid2txt=cid2txt)
    ctxs_clipped, clip_stats = clip_contexts(
        ctxs_raw, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars
    )
    clip_stats["dedup"] = dstat
    stages.append(
        StageView(
            name="retrieval_raw",
            hits=hits_d,
            ctxs_raw=ctxs_raw,
            ctxs_clipped=ctxs_clipped,
            clip_stats=clip_stats,
            stats=summarize_contexts(q, ctxs_clipped),
        )
    )

    # ---- neighbor（基于 hits 的 source_path/chunk_index）
    if int(expand_neighbors) > 0:
        hit_rows = [
            {"score": float(h["score"]), "source_path": h.get("source_path"), "chunk_index": h.get("chunk_index")}
            for h in hits_d
        ]
        ctx_objs = build_neighbor_contexts(metas=metas, hits=hit_rows, neighbor_n=int(expand_neighbors))
        ctxs2 = [c.get("text", "") for c in ctx_objs if isinstance(c.get("text"), str)]
        # neighbor 拼接后也去重（极少见，但做了更稳）
        seen2 = set()
        ctxs2_d: List[str] = []
        for t in ctxs2:
            th = _text_hash(t)
            if th in seen2:
                continue
            seen2.add(th)
            ctxs2_d.append(t)
        ctxs2_clip, clip2 = clip_contexts(
            ctxs2_d, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars
        )
        clip2["dedup"] = {"dedup_removed": len(ctxs2) - len(ctxs2_d), "dedup_kept": len(ctxs2_d)}
        stages.append(
            StageView(
                name="retrieval_neighbor",
                hits=hits_d,
                ctxs_raw=ctxs2_d,
                ctxs_clipped=ctxs2_clip,
                clip_stats=clip2,
                stats=summarize_contexts(q, ctxs2_clip),
            )
        )

    # ---- rerank
    if enable_rerank and reranker is not None:
        # rerank 只对有文本的候选做
        cand_hits = []
        cand_texts: List[str] = []
        seen_txt = set()
        for h in hits_obj:
            txt = cid2txt.get(getattr(h, "chunk_id", ""), "")
            if not txt:
                continue
            th = _text_hash(txt)
            if th in seen_txt:
                continue
            seen_txt.add(th)
            cand_hits.append(h)
            cand_texts.append(txt)
            if len(cand_hits) >= int(CONFIG.get("rerank_candidates") or 10):
                break

        order = reranker.rerank(
            query=q,
            passages=cand_texts,
            top_k=int(k),
            batch_size=int(CONFIG.get("rerank_batch_size") or 32),
        )
        reranked_obj = []
        for r0 in order:
            h0 = cand_hits[int(r0.index)]
            # 不改对象，单独产出 rerank 视图
            reranked_obj.append((h0, float(r0.score)))

        hits_r: List[dict] = []
        for rank, (h, s) in enumerate(reranked_obj, start=1):
            m = cid2meta.get(getattr(h, "chunk_id", ""))
            hits_r.append(
                {
                    "rank": rank,
                    "chunk_id": getattr(h, "chunk_id", None),
                    "book_id": getattr(h, "book_id", None),
                    "score_retrieval": float(getattr(h, "score", 0.0) or 0.0),
                    "score": float(s),
                    "score_rerank": float(s),
                    "source_path": (m or {}).get("source_path"),
                    "chunk_index": (m or {}).get("chunk_index"),
                }
            )
        hits_r_d, ctxs_r, dstat_r = dedup_by_text(hits=hits_r, cid2txt=cid2txt)
        ctxs_r_clip, clipr = clip_contexts(ctxs_r, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars)
        clipr["dedup"] = dstat_r
        stages.append(
            StageView(
                name="rerank_raw",
                hits=hits_r_d,
                ctxs_raw=ctxs_r,
                ctxs_clipped=ctxs_r_clip,
                clip_stats=clipr,
                stats=summarize_contexts(q, ctxs_r_clip),
            )
        )

        # ---- rerank_filtered：对最终（未邻居拼接时）的 contexts 做同样过滤/门控，便于与 sweep 对齐
        ctxs_f = list(ctxs_r)
        filt_info = {"option_numeric": None, "garbled": None, "rag_gate": None, "option_coverage_gate": None, "option_ambiguity_gate": None}

        if bool(CONFIG.get("enable_garbled_filter")) and ctxs_f:
            ctxs_f, ginfo = filter_garbled_contexts(
                ctxs=ctxs_f,
                min_alpha_ratio=float(CONFIG.get("garbled_min_alpha_ratio") or 0.0),
                min_word_per_100=float(CONFIG.get("garbled_min_word_per_100") or 0.0),
            )
            filt_info["garbled"] = ginfo

        if bool(CONFIG.get("enable_option_numeric_filter")) and ctxs_f:
            ctxs_f, _meta_tmp, finfo = option_aware_numeric_filter(
                question_full=question_full,
                ctxs=ctxs_f,
                ctx_meta=[{} for _ in ctxs_f],
                min_keep=int(CONFIG.get("option_numeric_min_keep") or 1),
                keep_topn=int(CONFIG.get("option_numeric_keep_topn") or 0),
            )
            filt_info["option_numeric"] = finfo

        gate_info = None
        if bool(CONFIG.get("enable_rag_gate")) and ctxs_f:
            trig, gate_info = rag_gate(
                query=q,
                question_full=question_full,
                ctxs=ctxs_f,
                min_ctx=int(CONFIG.get("gate_min_ctx") or 1),
                min_overlap_max=float(CONFIG.get("gate_min_overlap_max") or 0.0),
            )
            if trig:
                ctxs_f = []
        filt_info["rag_gate"] = gate_info

        cov_info = None
        if bool(CONFIG.get("enable_option_coverage_gate")) and ctxs_f:
            trig2, cov_info = option_coverage_gate(
                question_full=question_full,
                ctxs=ctxs_f,
                min_hit=int(CONFIG.get("gate_min_option_hits") or 1),
            )
            if trig2:
                ctxs_f = []
        filt_info["option_coverage_gate"] = cov_info

        amb_info = None
        if bool(CONFIG.get("enable_option_ambiguity_gate")) and ctxs_f:
            trig3, amb_info = option_ambiguity_gate(
                query=q,
                question_full=question_full,
                ctxs=ctxs_f,
                max_hit_options=int(CONFIG.get("option_ambiguity_max_hit_options") or 1),
            )
            if trig3:
                ctxs_f = []
        filt_info["option_ambiguity_gate"] = amb_info

        ctxs_f_clip, clipf0 = clip_contexts(ctxs_f, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars)
        clipf0["filters"] = filt_info
        stages.append(
            StageView(
                name="rerank_filtered",
                hits=hits_r_d,
                ctxs_raw=ctxs_f,
                ctxs_clipped=ctxs_f_clip,
                clip_stats=clipf0,
                stats=summarize_contexts(q, ctxs_f_clip),
            )
        )

        # rerank 后去噪过滤：先按 overlap 阈值过滤，再可选只保留 topN
        hits_r_filtered = list(hits_r_d)
        thr = float(CONFIG.get("rerank_filter_overlap") or 0.0)
        if thr > 0.0:
            kept = []
            for h in hits_r_filtered:
                txt = cid2txt.get(str(h.get("chunk_id") or ""), "")
                ov = overlap_score(q, txt)
                hh = dict(h)
                hh["overlap"] = float(ov)
                if ov >= thr:
                    kept.append(hh)
            if kept:
                hits_r_filtered = kept
        topn = int(CONFIG.get("rerank_keep_topn") or 0)
        if topn > 0 and len(hits_r_filtered) > topn:
            hits_r_filtered = hits_r_filtered[:topn]

        if int(expand_neighbors) > 0:
            hit_rows = [
                {"score": float(h["score"]), "source_path": h.get("source_path"), "chunk_index": h.get("chunk_index")}
                for h in hits_r_filtered
            ]
            ctx_objs = build_neighbor_contexts(metas=metas, hits=hit_rows, neighbor_n=int(expand_neighbors))
            ctxs4 = [c.get("text", "") for c in ctx_objs if isinstance(c.get("text"), str)]
            # neighbor 拼接后去重
            seen4 = set()
            ctxs4_d: List[str] = []
            for t in ctxs4:
                th = _text_hash(t)
                if th in seen4:
                    continue
                seen4.add(th)
                ctxs4_d.append(t)
            ctxs4_clip, clip4 = clip_contexts(ctxs4_d, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars)
            clip4["dedup"] = {"dedup_removed": len(ctxs4) - len(ctxs4_d), "dedup_kept": len(ctxs4_d)}
            stages.append(
                StageView(
                    name="rerank_neighbor",
                    hits=hits_r_filtered,
                    ctxs_raw=ctxs4_d,
                    ctxs_clipped=ctxs4_clip,
                    clip_stats=clip4,
                    stats=summarize_contexts(q, ctxs4_clip),
                )
            )

            # rerank_neighbor_filtered：在拼接后做 option-aware 数字过滤 + RAG gate
            ctxs_f = list(ctxs4_d)
            meta_f = list(hits_r_filtered)
            filter_info = {"option_numeric": None, "garbled": None, "rag_gate": None, "option_coverage_gate": None, "option_ambiguity_gate": None}

            if bool(CONFIG.get("enable_garbled_filter")) and ctxs_f:
                ctxs_f, ginfo = filter_garbled_contexts(
                    ctxs=ctxs_f,
                    min_alpha_ratio=float(CONFIG.get("garbled_min_alpha_ratio") or 0.0),
                    min_word_per_100=float(CONFIG.get("garbled_min_word_per_100") or 0.0),
                )
                filter_info["garbled"] = ginfo
            if bool(CONFIG.get("enable_option_numeric_filter")) and ctxs_f:
                # 这里 ctx_meta 只是为了对齐长度；inspect 主要看 ctx 文本
                ctxs_f, _meta_tmp, finfo = option_aware_numeric_filter(
                    question_full=question_full,
                    ctxs=ctxs_f,
                    ctx_meta=[{} for _ in ctxs_f],
                    min_keep=int(CONFIG.get("option_numeric_min_keep") or 1),
                    keep_topn=int(CONFIG.get("option_numeric_keep_topn") or 0),
                )
                filter_info["option_numeric"] = finfo
                # meta_f 不做严格对齐（因为过滤后的 ctxs_f 可能减少）

            gate_info = None
            if bool(CONFIG.get("enable_rag_gate")) and ctxs_f:
                trig, gate_info = rag_gate(
                    query=q,
                    question_full=question_full,
                    ctxs=ctxs_f,
                    min_ctx=int(CONFIG.get("gate_min_ctx") or 1),
                    min_overlap_max=float(CONFIG.get("gate_min_overlap_max") or 0.0),
                )
                if trig:
                    ctxs_f = []
                    meta_f = []
            filter_info["rag_gate"] = gate_info

            cov_info = None
            if bool(CONFIG.get("enable_option_coverage_gate")) and ctxs_f:
                trig2, cov_info = option_coverage_gate(
                    question_full=question_full,
                    ctxs=ctxs_f,
                    min_hit=int(CONFIG.get("gate_min_option_hits") or 1),
                )
                if trig2:
                    ctxs_f = []
                    meta_f = []
            filter_info["option_coverage_gate"] = cov_info

            amb_info = None
            if bool(CONFIG.get("enable_option_ambiguity_gate")) and ctxs_f:
                trig3, amb_info = option_ambiguity_gate(
                    query=q,
                    question_full=question_full,
                    ctxs=ctxs_f,
                    max_hit_options=int(CONFIG.get("option_ambiguity_max_hit_options") or 1),
                )
                if trig3:
                    ctxs_f = []
                    meta_f = []
            filter_info["option_ambiguity_gate"] = amb_info

            ctxs_f_clip, clipf = clip_contexts(ctxs_f, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars)
            clipf["filters"] = filter_info
            stages.append(
                StageView(
                    name="rerank_neighbor_filtered",
                    hits=meta_f,
                    ctxs_raw=ctxs_f,
                    ctxs_clipped=ctxs_f_clip,
                    clip_stats=clipf,
                    stats=summarize_contexts(q, ctxs_f_clip),
                )
            )

    return stages


def dump_markdown(
    *,
    out_path: Path,
    item: dict,
    case: dict,
    k: int,
    stages: List[StageView],
    md_print_max_chars: int,
) -> None:
    qid = str(item.get("qid") or "")
    q = str(item.get("query") or "")
    q_full = str(item.get("question_full") or "")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    def clip_for_md(s: str) -> str:
        s = (s or "").strip()
        if len(s) <= int(md_print_max_chars):
            return s
        return s[: int(md_print_max_chars)] + "\n...(省略)..."

    lines: List[str] = []
    lines.append(f"# Context Inspect\n")
    lines.append(f"- ts: `{datetime.now().isoformat(timespec='seconds')}`")
    lines.append(f"- qid: `{qid}`")
    lines.append(f"- retriever: `{case.get('retriever')}`")
    lines.append(f"- k: `{k}`")
    lines.append(f"- index_dir: `{case.get('index_dir')}`")
    lines.append(f"- embedding_model: `{case.get('embedding_model')}`")
    lines.append("")
    lines.append("## Query")
    lines.append("")
    lines.append("```")
    lines.append(q)
    lines.append("```")
    lines.append("")
    lines.append("## Question Full")
    lines.append("")
    lines.append("```")
    lines.append(q_full)
    lines.append("```")
    lines.append("")

    lines.append("## Stage Summary")
    lines.append("")
    lines.append("| stage | n_ctx | total_chars | overlap_avg | overlap_max | truncated | total_chars_out |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    for st in stages:
        lines.append(
            f"| {st.name} | {st.stats.get('n',0)} | {st.stats.get('total_chars',0)} | "
            f"{st.stats.get('overlap_avg',0.0):.3f} | {st.stats.get('overlap_max',0.0):.3f} | "
            f"{st.clip_stats.get('truncated',0)} | {st.clip_stats.get('total_chars_out',0)} |"
        )
    lines.append("")

    for st in stages:
        lines.append(f"## {st.name}")
        lines.append("")
        lines.append("### hits")
        lines.append("")
        lines.append("```json")
        lines.append(json.dumps(st.hits, ensure_ascii=False, indent=2))
        lines.append("```")
        lines.append("")
        lines.append("### contexts_raw (pre-clip)")
        lines.append("")
        for i, c in enumerate(st.ctxs_raw, start=1):
            lines.append(f"#### [raw {i}] len={len(c)} overlap={overlap_score(q, c):.3f}")
            lines.append("")
            lines.append("```")
            lines.append(clip_for_md(c))
            lines.append("```")
            lines.append("")
        lines.append("### contexts_clipped (post-clip)")
        lines.append("")
        for i, c in enumerate(st.ctxs_clipped, start=1):
            lines.append(f"#### [clip {i}] len={len(c)} overlap={overlap_score(q, c):.3f}")
            lines.append("")
            lines.append("```")
            lines.append(clip_for_md(c))
            lines.append("```")
            lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    qa_eval = Path(CONFIG["qa_eval"])
    out_dir = Path(CONFIG["out_dir"])
    trace_path = Path(CONFIG["trace_jsonl"])
    ensure_dir(out_dir)
    ensure_dir(trace_path.parent)

    rows = list(read_jsonl(qa_eval))
    if CONFIG.get("only_qids"):
        only = set(str(x) for x in (CONFIG.get("only_qids") or []))
        rows = [r for r in rows if str(r.get("qid") or "") in only]
    if CONFIG.get("limit") is not None:
        rows = rows[: int(CONFIG["limit"])]

    trace_fp = trace_path.open("w", encoding="utf-8")

    # 端到端 Accuracy：按 (index_dir, retriever, k) 维护滚动统计
    enable_llm_acc = bool(CONFIG.get("enable_llm_accuracy")) or (os.getenv("INSPECT_USE_LLM", "").strip() == "1")
    acc_stage_name = str(CONFIG.get("accuracy_stage") or "").strip() or "rerank_neighbor_filtered"
    acc_print_every = int(CONFIG.get("accuracy_print_every") or 10)
    llm_cfg = dict(CONFIG.get("llm") or {})
    if enable_llm_acc:
        if not (llm_cfg.get("base_url") and llm_cfg.get("api_key") and llm_cfg.get("model")):
            raise SystemExit(
                "enable_llm_accuracy=True 但未配置 LLM：请设置环境变量 OPENAI_API_KEY（或 API_KEY / DEEPSEEK_API_KEY）。"
            )
    acc_stats: Dict[str, Dict[str, int]] = {}  # key -> {"correct": x, "total": y}

    for idx_cfg in CONFIG["index_dirs"]:
        index_dir = Path(idx_cfg["index_dir"])
        store = NumpyVectorStore(index_dir)
        store.load()
        metas = store._meta  # noqa: SLF001
        cid2txt = build_chunkid_to_text(metas)
        cid2meta = build_chunkid_to_meta(metas)

        embedding_model = str(idx_cfg["embedding_model"])
        for retriever in CONFIG["retrievers"]:
            for item in rows:
                qid = str(item.get("qid") or "")
                q = str(item.get("query") or "")
                q_full = str(item.get("question_full") or "")
                gold = normalize_choice(str(item.get("gold_choice") or item.get("gold_output") or ""))
                for k in CONFIG["ks"]:
                    pre_k = int(k)
                    if bool(CONFIG.get("enable_rerank")):
                        pre_k = max(pre_k, int(CONFIG.get("rerank_candidates") or 10))

                    hits_obj = retrieve_hits(
                        retriever_name=str(retriever),
                        query=q,
                        k=pre_k,
                        index_dir=index_dir,
                        embedding_model=embedding_model,
                        metas=metas,
                    )

                    reranker = None
                    if bool(CONFIG.get("enable_rerank")):
                        reranker = CrossEncoderReranker(
                            str(CONFIG.get("rerank_model") or "BAAI/bge-reranker-large"),
                            device=CONFIG.get("rerank_device"),
                        )

                    stages = make_stage_views(
                        q=q,
                        question_full=q_full,
                        hits_obj=hits_obj,
                        cid2txt=cid2txt,
                        cid2meta=cid2meta,
                        metas=metas,
                        k=int(k),
                        enable_rerank=bool(CONFIG.get("enable_rerank")),
                        reranker=reranker,
                        expand_neighbors=int(CONFIG.get("expand_neighbors") or 0),
                        per_ctx_max_chars=int(CONFIG.get("per_ctx_max_chars") or 5000),
                        total_max_chars=int(CONFIG.get("total_max_chars") or 24000),
                    )

                    case = {
                        "retriever": str(retriever),
                        "k": int(k),
                        "index_dir": str(index_dir),
                        "embedding_model": embedding_model,
                        "expand_neighbors": int(CONFIG.get("expand_neighbors") or 0),
                        "enable_rerank": bool(CONFIG.get("enable_rerank")),
                        "rerank_model": str(CONFIG.get("rerank_model") or ""),
                        "rerank_candidates": int(CONFIG.get("rerank_candidates") or 0),
                    }

                    # 可选：LLM 端到端作答并实时打印 Accuracy
                    llm_pred_raw = None
                    llm_pred = None
                    llm_is_correct = None
                    llm_reasoning = None
                    llm_prompts = None
                    if enable_llm_acc:
                        # 找指定 stage；找不到就退化为最后一个 stage（通常最接近最终 contexts）
                        st_for_acc = None
                        for st in stages:
                            if st.name == acc_stage_name:
                                st_for_acc = st
                                break
                        if st_for_acc is None and stages:
                            st_for_acc = stages[-1]
                        ctxs_for_llm = list(getattr(st_for_acc, "ctxs_clipped", []) or []) if st_for_acc else []

                        llm_pred_raw, llm_reasoning, sys_prompt, user_prompt = call_llm_choice(
                            llm=llm_cfg,
                            question_full=q_full,
                            contexts=ctxs_for_llm,
                        )
                        llm_pred = normalize_choice(str(llm_pred_raw or ""))
                        llm_is_correct = (gold is not None and llm_pred == gold)
                        llm_prompts = {"system": sys_prompt, "user": user_prompt}

                        acc_key = f"{Path(index_dir).name}:{retriever}:k{int(k)}:{acc_stage_name}"
                        st0 = acc_stats.setdefault(acc_key, {"correct": 0, "total": 0})
                        st0["total"] += 1
                        if llm_is_correct:
                            st0["correct"] += 1

                        if acc_print_every > 0 and (st0["total"] % acc_print_every == 0):
                            acc = st0["correct"] / float(st0["total"]) if st0["total"] > 0 else 0.0
                            print(
                                f"[ACC] {acc_key} {st0['correct']}/{st0['total']} = {acc:.3f}  (qid={qid})"
                            )

                    # trace.jsonl：结构化记录，方便你用脚本二次分析
                    trace_fp.write(
                        json.dumps(
                            {
                                "ts": datetime.now().isoformat(timespec="seconds"),
                                "qid": qid,
                                "case": case,
                                "query": q,
                                "question_full": q_full,
                                "gold_choice": gold,
                                "stages": [
                                    {
                                        "name": st.name,
                                        "hits": st.hits,
                                        "stats": st.stats,
                                        "clip_stats": st.clip_stats,
                                        "contexts_raw": st.ctxs_raw,
                                        "contexts_clipped": st.ctxs_clipped,
                                    }
                                    for st in stages
                                ],
                                "llm": {
                                    "enabled": enable_llm_acc,
                                    "stage": acc_stage_name if enable_llm_acc else None,
                                    "base_url": llm_cfg.get("base_url") if enable_llm_acc else None,
                                    "model": llm_cfg.get("model") if enable_llm_acc else None,
                                    "pred_choice": llm_pred,
                                    "pred_raw": llm_pred_raw,
                                    "is_correct": llm_is_correct,
                                    "reasoning_content": llm_reasoning,
                                    "prompts": llm_prompts,
                                },
                            },
                            ensure_ascii=False,
                        )
                        + "\n"
                    )
                    trace_fp.flush()

                    # markdown：每题每 case-k 一份文件，便于肉眼检查
                    md_name = f"{qid}__{Path(index_dir).name}__{retriever}__k{int(k)}.md"
                    dump_markdown(
                        out_path=out_dir / md_name,
                        item=item,
                        case=case,
                        k=int(k),
                        stages=stages,
                        md_print_max_chars=int(CONFIG.get("md_print_max_chars") or 3000),
                    )

    trace_fp.close()
    print(f"[DONE] trace: {trace_path}")
    print(f"[DONE] markdown dir: {out_dir}")
    if enable_llm_acc and acc_stats:
        print("\n[ACC SUMMARY]")
        for k0 in sorted(acc_stats.keys()):
            c = acc_stats[k0]["correct"]
            t = acc_stats[k0]["total"]
            acc = c / float(t) if t > 0 else 0.0
            print(f"- {k0}: {c}/{t} = {acc:.3f}")


if __name__ == "__main__":
    main()


