from __future__ import annotations




import os
import sys
import csv
import re
import traceback
import json
import hashlib
from pathlib import Path
from typing import Optional
from datetime import datetime

# 允许直接运行：python RAG/experiments/run_one_click_sweep.py
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))      # experiments/
sys.path.insert(0, str(HERE.parents[1]))  # RAG/


from common import read_jsonl  # noqa: E402
from retrievers import BM25Retriever, DenseRetriever, HybridRetriever, TwoStageBookThenChunk  # noqa: E402
from simple_rag.context_utils import build_neighbor_contexts  # noqa: E402
from simple_rag.reranker import CrossEncoderReranker  # noqa: E402
from simple_rag.vector_store import NumpyVectorStore  # noqa: E402
from tqdm import tqdm  # noqa: E402

# ------------------------------------------------------------
# CONFIG：按需修改
# ------------------------------------------------------------

CONFIG = {
    # QA 数据来源
    "alpaca_inputs": [
        "record-thinkking/300QuestionsInAds_alpaca.json",
        "record-thinkking/300QuestionsInAnatomy_alpaca.json",
    ],
    # 逐题追踪日志：每行一个 JSON（case/k/上下文/思考过程/输出/是否正确）
    "trace_out_jsonl": "RAG/experiments/out/qa_trace(12.26).jsonl",
    # 断点续传：读取已有 qa_trace.jsonl，跳过已完成 (case, qid)
    "resume_from_trace": True,
    # 生成的 QA evalset（jsonl）
    "qa_eval_out": "RAG/experiments/out/qa_eval(12.26).jsonl",
    # 最终结果 CSV
    "sweep_out_csv": "RAG/experiments/out/qa_sweep_results(12.26).csv",
    # 评测用 K 值
    "ks": [5],
    # 对比的检索方案（含 no_rag 基线）
    "retrievers": ["hybrid"],
    # 不同 embedding 对应不同 index_dir（论文对比 embedding 的关键）
    # 注意：index_dir 必须事先通过 ingest.py 建好（本脚本默认不自动建库，避免误跑很久）
    "index_dirs": [
        {"index_dir": "RAG/data/rag_index_e5", "embedding_model": "intfloat/multilingual-e5-small"},
    ],
    # LLM（OpenAI 兼容接口；建议用环境变量，不把 key 写进代码）
    "llm": {
        # record-thinkking/process_qa_with_reasoning.py 的配置（不做引用/导入）
        "base_url": os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com"),
        "model": os.getenv("OPENAI_MODEL", "deepseek-reasoner"),
        # 只从环境变量读取，避免把 key 写进仓库
        "api_key": os.getenv("OPENAI_API_KEY", "") or os.getenv("API_KEY", "") or os.getenv("DEEPSEEK_API_KEY", ""),
        "timeout": 1200,
        # 是否开启思考过程（仅对 deepseek-reasoner 等支持 reasoning 的模型有效）
        "enable_thinking": True,
    },
    # 评测规模控制（论文最终可设为 None 全量 596）
    "limit": None,
    # context 控制（避免 token 爆炸）
    "per_ctx_max_chars": 5000,
    "total_max_chars": 24000,
    # 小块命中后拼接同一 source_path 的相邻 chunk（推荐用于更完整证据链）
    # 0=关闭；1=拼 i-1,i,i+1；2=拼 i-2..i+2
    "expand_neighbors": 0,
    # reranker：先对召回的小块候选 rerank，再做邻居拼接/截断，最后喂给 LLM
    "enable_rerank": True,
    "rerank_model": "BAAI/bge-reranker-large",
    # rerank 前召回的小块数量（建议 10~30，越大越准但越慢）
    "rerank_candidates": 20,
    "rerank_batch_size": 32,
    # 可选：cpu / cuda；None 则由 sentence-transformers 自动选择
    "rerank_device": None,
    # rerank 后去噪过滤（推荐开启，避免把跑题块拼接放大）
    # - overlap 是轻量词面重合度（越大越像“题干关键词命中”）
    # - 过滤后若没有块通过阈值，会自动回退到未过滤结果
    "rerank_filter_overlap": 0.2,
    # rerank 后仅保留前 N 个块作为“种子块”再去拼接邻居；0=不限制
    "rerank_keep_topn": 5,
    # option-aware 数字过滤（针对 hours/days/mg 等冲突数字导致的“被误导”）
    "enable_option_numeric_filter": True,
    "option_numeric_min_keep": 1,
    "option_numeric_keep_topn": 0,
    # RAG gate：最终上下文质量太差则直接不给 contexts（query和context相关度太低，相当于自动退回 no-rag）
    "enable_rag_gate": True,
    "gate_min_ctx": 1,
    "gate_min_overlap_max": 0.12,
    # 选项覆盖 gate：contexts 若几乎不包含选项关键词，则回退 no-rag
    "enable_option_coverage_gate": True,
    "gate_min_option_hits": 1,

    # ------------------------------------------------------------
    # 额外抗噪：针对 compare_examples.jsonl 里常见的“OCR/目录碎片/多选项歧义”带偏
    # ------------------------------------------------------------
    # 1) OCR/目录碎片过滤：把明显“乱码/碎片化”的段落剔除，减少误导
    "enable_garbled_filter": True,
    "garbled_min_alpha_ratio": 0.45,      # 非空白字符中 A-Za-z 的比例过低则可疑
    "garbled_min_word_per_100": 4.0,      # 每 100 字符至少多少个英文词（_RE_WORD）

    # 2) 选项歧义 gate：如果 contexts 同时“支持”多个选项（尤其是包含关系选项），直接回退 no-rag
    "enable_option_ambiguity_gate": False,
    "option_ambiguity_max_hit_options": 3,  # >1 视为歧义
}

_RE_CHOICE = re.compile(r"\b([ABCD])\b", re.IGNORECASE)
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

def _tokens(text: str) -> list[str]:
    return [t.lower() for t in _RE_WORD.findall(text or "")]

OPTION_STOP_EXTRA = {
    # 选择题选项里非常“泛化”的词：容易导致 option 命中统计失真
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


def overlap_score(query: str, ctx: str) -> float:
    tq = set(t.lower() for t in _RE_WORD.findall(query or ""))
    if not tq:
        return 0.0
    tc = set(t.lower() for t in _RE_WORD.findall(ctx or ""))
    if not tc:
        return 0.0
    return len(tq & tc) / float(len(tq))


def _norm_text(s: str) -> str:
    # 轻量归一化：压缩空白，避免“同文本不同空格”导致去重失败
    return re.sub(r"\s+", " ", (s or "").strip())


def _text_hash(s: str) -> str:
    return hashlib.sha1(_norm_text(s).encode("utf-8")).hexdigest()


def extract_num_units(text: str) -> dict[str, set[str]]:
    """
    提取 (number, unit) 对，按 unit 归一化聚合。
    返回：unit -> set(number_str)
    """
    out: dict[str, set[str]] = {}
    for num, unit in _RE_NUM.findall(text or ""):
        u = (unit or "").lower()
        # 归一化一些常见单位写法
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
    """
    仅提取出现过的单位（不要求带数字），用于“单位一致性”。
    """
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
    """
    从选择题 question_full 中抽取“选项关键词”（英文 token）。
    用于关键句保护/覆盖率 gate。
    """
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
    ctx_meta: list[dict],
    min_alpha_ratio: float,
    min_word_per_100: float,
) -> tuple[list[str], list[dict], dict]:
    """
    过滤明显“乱码/目录碎片/OCR 破碎段”。
    设计原则：宁可保守不过滤（避免误伤），只剔除非常可疑的段落。
    """
    kept_ctxs: list[str] = []
    kept_meta: list[dict] = []
    dropped: list[dict] = []

    for i, c in enumerate(ctxs):
        c0 = (c or "").strip()
        if not c0:
            continue
        # 很短的段落不做 garbled 判定（避免误伤定义句/公式）
        if len(c0) < 80:
            kept_ctxs.append(c0)
            if i < len(ctx_meta):
                kept_meta.append(ctx_meta[i])
            continue

        ar = _alpha_ratio(c0)
        wp = _word_per_100(c0)
        is_garbled = (ar < float(min_alpha_ratio) and wp < float(min_word_per_100))
        if is_garbled:
            dropped.append({"i": i, "alpha_ratio": ar, "word_per_100": wp, "len": len(c0)})
            continue
        kept_ctxs.append(c0)
        if i < len(ctx_meta):
            kept_meta.append(ctx_meta[i])

    info = {
        "enabled": True,
        "min_alpha_ratio": float(min_alpha_ratio),
        "min_word_per_100": float(min_word_per_100),
        "dropped_n": len(dropped),
        "kept_n": len(kept_ctxs),
        "dropped_preview": dropped[:10],
    }
    return kept_ctxs, kept_meta, info


def parse_mcq_options(question_full: str) -> dict[str, str]:
    """
    从 question_full 解析 a)/b)/c)/d) 选项。
    """
    out: dict[str, str] = {}
    lines = (question_full or "").splitlines()
    for ln in lines:
        m = re.match(r"^\s*([a-dA-D])\)\s*(.+?)\s*$", ln)
        if not m:
            continue
        k = m.group(1).upper()
        out[k] = m.group(2).strip()
    return out


def option_ambiguity_gate(
    *,
    query: str,
    question_full: str,
    ctxs: list[str],
    max_hit_options: int,
) -> tuple[bool, dict]:
    """
    若 contexts 同时命中多个选项的“区分性关键词”，认为上下文可能带偏（尤其是包含关系选项），回退 no-rag。
    """
    opts = parse_mcq_options(question_full)
    if not opts or not ctxs:
        return False, {"enabled": False, "reason": "no_options_or_no_context"}

    tq = set(_tokens(query))
    ctx_tokens = [set(_tokens(c)) for c in ctxs]

    # 选项区分词：从选项里抽词，再剔除 query 里的词和常见泛词
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
        # 在任一 context 里命中就算命中；短选项允许 1 个词命中，长选项要求 >=2
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

def option_aware_numeric_filter(
    *,
    question_full: str,
    ctxs: list[str],
    ctx_meta: list[dict],
    min_keep: int = 1,
    keep_topn: int = 0,
) -> tuple[list[str], list[dict], dict]:
    """
    选项感知过滤：
    - 从 question_full（含选项）里提取数字/单位集合（unit -> allowed numbers）
    - 若某 unit 在选项中出现，则 context 中该 unit 的数字若与 allowed 无交集，则视为“潜在误导”，优先剔除
    - 若过滤后数量过少（<min_keep），回退到原始 ctxs（避免空上下文）
    - 可选：keep_topn 限制保留前 N 段（0=不限制）
    """
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
        # 关键句保护：若包含选项关键词，则尽量保留（不因数字/单位冲突直接丢）
        has_protected = bool(protected and (toks & protected))

        nums = extract_num_units(c0)
        units = extract_units_only(c0)
        bad_units = []
        # 数字一致性：仅当选项里出现过该单位时才做约束
        for u, allow_set in allowed_nums.items():
            if u not in nums:
                continue
            have = nums.get(u) or set()
            if not (have & allow_set):
                bad_units.append({"unit": u, "have": sorted(have)[:10], "allowed": sorted(allow_set)[:10]})

        # 单位一致性：若选项里出现了“长度/尺寸”等单位，而 context 出现完全不同单位，则认为可能误导
        unit_mismatch = False
        mismatch_detail = None
        if allowed_units and units:
            # 只在“选项包含单位”时启用；允许 context 同时出现选项单位之外的其它单位，但若完全不包含选项单位则标记
            if not (units & allowed_units):
                unit_mismatch = True
                mismatch_detail = {"have_units": sorted(units)[:10], "allowed_units": sorted(allowed_units)[:10]}

        if (bad_units or unit_mismatch) and (not has_protected):
            drop.append(
                {
                    "i": i,
                    "bad_units": bad_units,
                    "unit_mismatch": unit_mismatch,
                    "unit_detail": mismatch_detail,
                }
            )
        else:
            keep_idx.append(i)
        if bad_units:
            drop.append({"i": i, "bad_units": bad_units})
        else:
            keep_idx.append(i)

    # 若全部被判为 bad，则不强行丢空（回退）
    if len(keep_idx) < int(min_keep):
        return ctxs, ctx_meta, {
            "enabled": True,
            "fallback": True,
            "kept": len(keep_idx),
            "drop_n": len(drop),
            "drop": drop[:5],
            "protected_keywords_n": len(protected),
        }

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
    """
    简单 RAG gate：
    - ctx 数量太少或最大 overlap 太低 => 认为“RAG 可能误导”，直接回退 no-rag（空 contexts）
    """
    if len(ctxs) < int(min_ctx):
        return True, {"trigger": "too_few_ctx", "n_ctx": len(ctxs), "min_ctx": int(min_ctx)}
    # 用“题干+选项”算 overlap（更贴近选择题）
    q = (question_full or query or "").strip()
    ovs = [overlap_score(q, c) for c in ctxs]
    mx = max(ovs) if ovs else 0.0
    if mx < float(min_overlap_max):
        return True, {"trigger": "low_overlap", "overlap_max": mx, "min_overlap_max": float(min_overlap_max)}
    return False, {"trigger": None, "overlap_max": mx, "n_ctx": len(ctxs)}


def option_coverage_gate(
    *,
    question_full: str,
    ctxs: list[str],
    min_hit: int = 1,
) -> tuple[bool, dict]:
    """
    选项覆盖 gate：若 contexts 中几乎不包含选项关键词，则认为“证据不支持作答”，触发 gate。
    """
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

# 提取选项ABCD
def normalize_choice(text: str) -> Optional[str]:
    if not text:
        return None
    m = _RE_CHOICE.search(text)
    if not m:
        return None
    return m.group(1).upper()

# 构建 从 chunk_id 到 text 的哈希表
def build_chunkid_to_text(metas: list[dict]) -> dict[str, str]:
    m = {}
    for row in metas:
        cid = row.get("chunk_id")
        txt = row.get("text")
        if isinstance(cid, str) and isinstance(txt, str) and txt:
            m[cid] = txt
    return m


def build_chunkid_to_meta(metas: list[dict]) -> dict[str, dict]:
    m: dict[str, dict] = {}
    for row in metas:
        cid = row.get("chunk_id")
        if isinstance(cid, str) and cid:
            m[cid] = row
    return m

# 截断上下文，作为rag的输入要限制片长度
def clip_contexts(ctxs: list[str], *, per_ctx_max_chars: int, total_max_chars: int) -> list[str]:
    out: list[str] = []
    used = 0
    for c in ctxs:
        if used >= total_max_chars:
            break
        c = (c or "").strip()
        if not c:
            continue
        if len(c) > per_ctx_max_chars:
            c = c[:per_ctx_max_chars] + "\n...(截断)..."
        if used + len(c) > total_max_chars:
            c = c[: max(0, total_max_chars - used)]
        out.append(c)
        used += len(c)
    return out

# 内部函数，防转义，一般_是内部函数
def _json_dumps(obj: dict) -> str:
    return json.dumps(obj, ensure_ascii=False)

# 用 | 把 5 段 key=value 拼成一根长字符串，作为简单的身份证
def _case_key(*, retriever: str, k: int, index_dir: str, embedding_model: str, llm: dict) -> str:
    """
    用于断点续传的 case 唯一标识：
    同一 case（同一索引/检索策略/k/LLM 配置）下的每个 qid 只跑一次。
    """
    return "|".join(
        [
            f"retriever={retriever}",
            f"k={int(k)}",
            # index_dir 统一用绝对规范化路径，避免相对/绝对、大小写导致无法匹配
            f"index_dir={_norm_abs_path(index_dir)}",
            f"embedding_model={embedding_model}",
            # base_url 不作为断点匹配条件（网络波动/配置微调不应导致无法续传）
            f"llm_model={llm.get('model')}",
        ]
    )

# 把相对路径转绝对路径并做 normcase/normpath
def _norm_abs_path(p: str) -> str:
    # 以仓库根目录为基准，把相对路径转绝对路径并做 normcase/normpath
    pp = Path(p)
    if not pp.is_absolute():
        pp = (HERE.parents[1] / pp).resolve()
    return os.path.normcase(os.path.normpath(str(pp)))

# 父函数，从 obj 中提取 case_key
def _trace_case_key_from_obj(obj: dict) -> Optional[str]:
    c = obj.get("case")
    if not isinstance(c, dict):
        return None
    retr = str(c.get("retriever", "") or "")
    k = int(c.get("k", 0) or 0)
    index_dir = str(c.get("index_dir", "") or "")
    emb = str(c.get("embedding_model", "") or "")
    llm_model = str(c.get("llm_model", "") or "")
    if not retr or not index_dir or not emb:
        return None
    return _case_key(
        retriever=retr,
        k=k,
        index_dir=index_dir,
        embedding_model=emb,
        llm={"model": llm_model},
    )


def build_planned_cases(eval_qids: list[str]) -> list[dict]:
    """
    生成固定顺序的计划 case：
    - retriever 按 CONFIG["retrievers"] 顺序
    - no_rag 只跑一次（k=0）
    - 其他 retriever 按 CONFIG["ks"] 顺序
    - 每个 case 再按 index_dirs 顺序展开（两套 embedding 各自一套 case）
    """
    planned = []
    for retr in CONFIG["retrievers"]:
        ks = [0] if retr == "no_rag" else list(CONFIG["ks"])
        for k in ks:
            for idx_cfg in CONFIG["index_dirs"]:
                index_dir = idx_cfg["index_dir"]
                emb = idx_cfg["embedding_model"]
                ck = _case_key(
                    retriever=retr,
                    k=int(k),
                    index_dir=index_dir,
                    embedding_model=emb,
                    llm={"model": CONFIG["llm"].get("model")},
                )
                planned.append(
                    {
                        "retriever": retr,
                        "k": int(k),
                        "index_dir": index_dir,
                        "embedding_model": emb,
                        "case_key": ck,
                    }
                )
    return planned

# 断点续传检查
def scan_trace_for_done(
    trace_path: Path, planned_case_keys: set[str], eval_qids: set[str]
) -> tuple[dict[str, set[str]], dict[str, tuple[int, int]], dict[str, int]]:
    """
    逐行扫描 qa_trace.jsonl：
    - done_by_case[case_key] = set(qid)
    - per_case_counts[case_key] = (correct, total)
    - stats: 一些调试计数，方便终端检查
    """
    done_by_case: dict[str, set[str]] = {ck: set[str]() for ck in planned_case_keys}
    per_case: dict[str, tuple[int, int]] = {}
    stats = {"lines": 0, "json_ok": 0, "matched": 0, "unmatched": 0, "qid_missing": 0, "case_missing": 0}

    if not trace_path.exists():
        return done_by_case, per_case, stats

    with trace_path.open("r", encoding="utf-8") as f:
        for line in f:
            stats["lines"] += 1
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                stats["json_ok"] += 1
            except Exception:
                continue

            qid = str(obj.get("qid", "") or "")
            if not qid:
                stats["qid_missing"] += 1
                continue
            if qid not in eval_qids:
                # 不是当前 evalset 的题，忽略
                continue

            ck = _trace_case_key_from_obj(obj)
            if not ck:
                stats["case_missing"] += 1
                continue

            if ck not in planned_case_keys:
                stats["unmatched"] += 1
                continue

            stats["matched"] += 1
            done_by_case.setdefault(ck, set[str]()).add(qid)

            is_correct = bool(obj.get("is_correct"))
            cur = per_case.get(ck, (0, 0))
            per_case[ck] = (cur[0] + (1 if is_correct else 0), cur[1] + 1)

    return done_by_case, per_case, stats


def load_trace_done(trace_path: Path) -> tuple[set[tuple[str, str]], dict[str, tuple[int, int]]]:
    """
    从 trace.jsonl 读取已完成记录，返回：
    依赖case qid来读取
    """
    done: set[tuple[str, str]] = set()
    per_case: dict[str, tuple[int, int]] = {}

    if not trace_path.exists():
        return done, per_case

    with trace_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue

            # 不新增 trace 字段：断点续传只依赖既有结构（case + qid）
            c = obj.get("case") or {}
            ck = _case_key(
                retriever=str(c.get("retriever", "")),
                k=int(c.get("k", 0) or 0),
                index_dir=str(c.get("index_dir", "")),
                embedding_model=str(c.get("embedding_model", "")),
                llm={"base_url": c.get("llm_base_url"), "model": c.get("llm_model")},
            )

            qid = str(obj.get("qid", "") or "")
            if not qid:
                continue
            done.add((ck, qid))

            is_correct = bool(obj.get("is_correct"))
            cur = per_case.get(ck, (0, 0))
            per_case[ck] = (cur[0] + (1 if is_correct else 0), cur[1] + 1)

    return done, per_case


def call_llm_choice(*, llm: dict, question_full: str, contexts: list[str]) -> tuple[str, str, str, str]:
    """
    官方 OpenAI SDK 调用方式（DeepSeek OpenAI 兼容）。
    """
    from openai import OpenAI

    client = OpenAI(api_key=llm["api_key"], base_url=llm["base_url"], timeout=int(llm["timeout"]))
    ctx = "\n\n".join([f"[Context {i+1}]\n{c}" for i, c in enumerate(contexts)])
    system = (
        "你是一个专业的兽医医学领域选择题作答助手，负责为学生提供可靠的选择题解答服务。我需要你发挥极致的性能来评测不同rag系统在兽医学相关书籍中的性能"
        "你会优先依据给定的 Context 作答，这些context可能不够完整，需要你根据上下文进行推理，并且也有可能有遗漏的地方，甚至是不相干的内容；最重要的是，若 Context 不足以支持，请依据常识作答，context仅仅作参考作用。"
        "最终只输出一个选项字母：A/B/C/D。不要输出其他内容。"
    )
    user = f"{ctx}\n\n[Question]\n{question_full}\n\n请只输出最终选项字母："

    kwargs = {
        "model": llm["model"],
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": user},
        ],
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


def eval_one_case(
    *,
    eval_rows: list[dict],
    retriever_name: str,
    k: int,
    index_dir: str,
    embedding_model: str,
    llm: dict,
    per_ctx_max_chars: int,
    total_max_chars: int,
    trace_fp,
    done_set: set[tuple[str, str]],
    case_key: str,
) -> tuple[Optional[float], int, int]:
    """
    返回 (accuracy, correct, total)
    """
    metas: list[dict] = []
    cid2txt: dict[str, str] = {}
    cid2meta: dict[str, dict] = {}
    r = None
    reranker = None

    if retriever_name != "no_rag":
        store = NumpyVectorStore(Path(index_dir))
        store.load()
        metas = store._meta  # noqa: SLF001
        cid2txt = build_chunkid_to_text(metas)
        cid2meta = build_chunkid_to_meta(metas)

        if retriever_name == "dense":
            r = DenseRetriever(Path(index_dir), embedding_model)
        elif retriever_name == "bm25":
            r = BM25Retriever(metas)
        elif retriever_name == "hybrid":
            r = HybridRetriever(Path(index_dir), embedding_model)
        elif retriever_name == "two_stage":
            r = TwoStageBookThenChunk(Path(index_dir), embedding_model)
        else:
            raise ValueError(f"未知 retriever: {retriever_name}")

        if bool(CONFIG.get("enable_rerank")):
            reranker = CrossEncoderReranker(
                str(CONFIG.get("rerank_model") or "BAAI/bge-reranker-large"),
                device=CONFIG.get("rerank_device"),
            )

    correct = 0
    total = 0
    for i, row in enumerate(
        tqdm(eval_rows, desc=f"{Path(index_dir).name}:{retriever_name}:k{k}", unit="题", leave=False),
        start=1,
    ):
        qid = row.get("qid", "")
        q = row.get("query", "")
        q_full = row.get("question_full", "")
        gold = normalize_choice(row.get("gold_choice", "") or row.get("gold_output", ""))

        # 断点续传：已完成的直接跳过
        if qid and (case_key, str(qid)) in done_set:
            continue

        ctxs: list[str] = []
        ctx_meta: list[dict] = []
        # 这些信息会被写入 trace；no_rag 或关闭过滤时也应有默认值，避免 UnboundLocalError
        ctx_filter_info = None
        garbled_info = None
        gate_info = None
        cov_info = None
        amb_info = None
        if retriever_name != "no_rag":
            pre_k = int(k)
            if reranker is not None:
                pre_k = max(pre_k, int(CONFIG.get("rerank_candidates") or 10))

            if retriever_name == "two_stage":
                hits = r.retrieve(q, top_k=pre_k, top_k_books=3)
            else:
                hits = r.retrieve(q, top_k=pre_k)

            # 先 rerank 小块：对 (query, chunk_text) 打分并重排，只保留 top_k 作为“种子块”
            if reranker is not None and hits:
                cand_hits = []
                cand_texts: list[str] = []
                seen_txt = set()
                for h in hits:
                    txt = cid2txt.get(h.chunk_id)
                    if not txt:
                        continue
                    # 去重：相同文本（常见于重复导入的同一本书）只保留一次，避免浪费 rerank 预算
                    th = _text_hash(txt)
                    if th in seen_txt:
                        continue
                    seen_txt.add(th)
                    cand_hits.append(h)
                    cand_texts.append(txt)
                if cand_hits:
                    order = reranker.rerank(
                        query=q,
                        passages=cand_texts,
                        top_k=int(k),
                        batch_size=int(CONFIG.get("rerank_batch_size") or 32),
                    )
                    new_hits = []
                    for r0 in order:
                        h0 = cand_hits[int(r0.index)]
                        # 将 rerank 分数写回 score，后续邻居拼接/区间打分会使用它
                        h0.score = float(r0.score)
                        new_hits.append(h0)
                    hits = new_hits

                    # rerank 后去噪过滤：词面 overlap 过滤（避免拼接把跑题块放大）
                    thr = float(CONFIG.get("rerank_filter_overlap") or 0.0)
                    if thr > 0.0:
                        kept = []
                        for h1 in hits:
                            txt1 = cid2txt.get(h1.chunk_id, "")
                            ov = overlap_score(q, txt1)
                            # 写入对象，方便 trace 分析
                            try:
                                h1.overlap = float(ov)  # type: ignore[attr-defined]
                            except Exception:
                                pass
                            if ov >= thr:
                                kept.append(h1)
                        if kept:
                            hits = kept

                    topn = int(CONFIG.get("rerank_keep_topn") or 0)
                    if topn > 0 and len(hits) > topn:
                        hits = hits[:topn]
            for rank, h in enumerate(hits, start=1):
                txt = cid2txt.get(h.chunk_id)
                if txt:
                    # 去重：相同文本只保留一次（避免上下文重复、浪费 token）
                    th = _text_hash(txt)
                    if any(m.get("text_hash") == th for m in ctx_meta):
                        continue
                    ctxs.append(txt)
                ctx_meta.append(
                    {
                        "rank": len(ctx_meta) + 1,
                        "chunk_id": getattr(h, "chunk_id", None),
                        "book_id": getattr(h, "book_id", None),
                        "score": getattr(h, "score", None),
                        "overlap": getattr(h, "overlap", None),
                        "text_hash": _text_hash(txt or "") if txt else None,
                    }
                )

            # 邻居拼接：把命中 chunk 的 source_path/chunk_index 邻域拼成更完整 contexts
            if int(CONFIG.get("expand_neighbors") or 0) > 0:
                hit_rows: list[dict] = []
                for h in hits:
                    m = cid2meta.get(getattr(h, "chunk_id", ""))
                    if not isinstance(m, dict):
                        continue
                    hit_rows.append(
                        {
                            "score": float(getattr(h, "score", 0.0) or 0.0),
                            "source_path": m.get("source_path"),
                            "chunk_index": m.get("chunk_index"),
                        }
                    )
                contexts = build_neighbor_contexts(
                    metas=metas,
                    hits=hit_rows,
                    neighbor_n=int(CONFIG.get("expand_neighbors") or 0),
                )
                if contexts:
                    ctxs = [c.get("text", "") for c in contexts if isinstance(c.get("text"), str)]
                    # context_meta 也记录区间，便于 trace 分析
                    ctx_meta = [
                        {
                            "rank": i + 1,
                            "source_path": c.get("source_path"),
                            "chunk_index_start": c.get("chunk_index_start"),
                            "chunk_index_end": c.get("chunk_index_end"),
                            "n_chunks": c.get("n_chunks"),
                            "score": c.get("score"),
                        }
                        for i, c in enumerate(contexts)
                    ]

            # option-aware 数字过滤（在拼接后执行，直接作用于最终 contexts）
            if bool(CONFIG.get("enable_option_numeric_filter")) and ctxs:
                ctxs, ctx_meta, ctx_filter_info = option_aware_numeric_filter(
                    question_full=q_full,
                    ctxs=ctxs,
                    ctx_meta=ctx_meta,
                    min_keep=int(CONFIG.get("option_numeric_min_keep") or 1),
                    keep_topn=int(CONFIG.get("option_numeric_keep_topn") or 0),
                )

            # garbled/OCR 噪声过滤（在最终 contexts 上做）
            garbled_info = None
            if bool(CONFIG.get("enable_garbled_filter")) and ctxs:
                ctxs, ctx_meta, garbled_info = filter_garbled_contexts(
                    ctxs=ctxs,
                    ctx_meta=ctx_meta,
                    min_alpha_ratio=float(CONFIG.get("garbled_min_alpha_ratio") or 0.0),
                    min_word_per_100=float(CONFIG.get("garbled_min_word_per_100") or 0.0),
                )

            # RAG gate：上下文质量不达标则回退 no-rag（不给 contexts）
            if bool(CONFIG.get("enable_rag_gate")) and ctxs:
                trig, gate_info = rag_gate(
                    query=q,
                    question_full=q_full,
                    ctxs=ctxs,
                    min_ctx=int(CONFIG.get("gate_min_ctx") or 1),
                    min_overlap_max=float(CONFIG.get("gate_min_overlap_max") or 0.0),
                )
                if trig:
                    ctxs = []
                    ctx_meta = []

            # 选项覆盖 gate：上下文若不覆盖选项关键词，也回退（避免“领域相关但无法作答”的误导）
            if bool(CONFIG.get("enable_option_coverage_gate")) and ctxs:
                trig2, cov_info = option_coverage_gate(
                    question_full=q_full,
                    ctxs=ctxs,
                    min_hit=int(CONFIG.get("gate_min_option_hits") or 1),
                )
                if trig2:
                    ctxs = []
                    ctx_meta = []

            # 选项歧义 gate：上下文同时命中多个选项时，容易把模型带到“包含关系”选项，直接回退 no-rag
            amb_info = None
            if bool(CONFIG.get("enable_option_ambiguity_gate")) and ctxs:
                trig3, amb_info = option_ambiguity_gate(
                    query=q,
                    question_full=q_full,
                    ctxs=ctxs,
                    max_hit_options=int(CONFIG.get("option_ambiguity_max_hit_options") or 1),
                )
                if trig3:
                    ctxs = []
                    ctx_meta = []

            ctxs = clip_contexts(ctxs, per_ctx_max_chars=per_ctx_max_chars, total_max_chars=total_max_chars)

        pred_raw, reasoning, sys_prompt, user_prompt = call_llm_choice(
            llm=llm, question_full=q_full, contexts=ctxs
        )
        pred = normalize_choice(pred_raw)
        total += 1
        is_correct = (gold is not None and pred == gold)
        if is_correct:
            correct += 1

        if trace_fp is not None:
            trace_fp.write(
                _json_dumps(
                    {
                        "ts": datetime.now().isoformat(timespec="seconds"),
                        "case": {
                            "retriever": retriever_name,
                            "k": int(k),
                            "index_dir": index_dir,
                            "embedding_model": embedding_model,
                            "llm_base_url": llm.get("base_url"),
                            "llm_model": llm.get("model"),
                        },
                        "i": i,
                        "qid": qid,
                        "query": q,
                        "question_full": q_full,
                        "gold_choice": gold,
                        "pred_choice": pred,
                        "pred_raw": pred_raw,
                        "is_correct": is_correct,
                        "reasoning_content": reasoning,
                        "contexts": ctxs,
                        "context_meta": ctx_meta,
                        "context_filters": {
                            "option_numeric": ctx_filter_info,
                            "garbled": garbled_info,
                            "rag_gate": gate_info,
                            "option_coverage_gate": cov_info,
                            "option_ambiguity_gate": amb_info,
                        },
                        "prompts": {"system": sys_prompt, "user": user_prompt},
                    }
                )
                + "\n"
            )
            trace_fp.flush()
        if qid:
            done_set.add((case_key, str(qid)))

    acc = None if total <= 0 else (correct / float(total))
    return acc, correct, total


def ensure_qa_evalset() -> None:
    """
    从 alpaca.json 生成 qa_eval.jsonl（query=题干，按 \\n\\n 分割取前半段）。
    """
    from build_qa_evalset_from_alpaca import iter_rows  # noqa: E402
    from common import write_jsonl  # noqa: E402

    out = Path(CONFIG["qa_eval_out"])
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = list(iter_rows([Path(p) for p in CONFIG["alpaca_inputs"]]))
    write_jsonl(out, rows)
    print(f"[OK] 生成 QA evalset：{out}（{len(rows)} 条）")


def main() -> None:
    llm = CONFIG["llm"]
    if not (llm.get("base_url") and llm.get("api_key") and llm.get("model")):
        raise SystemExit(
            "未找到可用 LLM 配置：请设置环境变量 OPENAI_API_KEY（或 API_KEY / DEEPSEEK_API_KEY）。"
        )

    ensure_qa_evalset()

    out_csv = Path(CONFIG["sweep_out_csv"])
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    eval_rows = list(read_jsonl(Path(CONFIG["qa_eval_out"])))
    if CONFIG.get("limit") is not None:
        eval_rows = eval_rows[: int(CONFIG["limit"])]

    trace_path = Path(CONFIG["trace_out_jsonl"])
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    # 先读取 evalset 的 qid 列表（固定 596）
    eval_qids = [str(r.get("qid", "")) for r in eval_rows if r.get("qid")]
    eval_qid_set = set(eval_qids)

    planned = build_planned_cases(eval_qids)
    planned_case_keys = {c["case_key"] for c in planned}

    if CONFIG.get("resume_from_trace") and trace_path.exists():
        done_by_case, per_case_prev, stats = scan_trace_for_done(trace_path, planned_case_keys, eval_qid_set)
        trace_fp = trace_path.open("a", encoding="utf-8")
    else:
        done_by_case = {ck: set() for ck in planned_case_keys}
        per_case_prev = {}
        stats = {"lines": 0, "json_ok": 0, "matched": 0, "unmatched": 0, "qid_missing": 0, "case_missing": 0}
        trace_fp = trace_path.open("w", encoding="utf-8")

    # 打印断点续传扫描结果（方便你监视）
    print("\n[RESUME] eval questions:", len(eval_qids))
    print("[RESUME] trace:", str(trace_path))
    print("[RESUME] trace_stats:", stats)

    # 打印每个 case 的完成度，并列出缺失数量（不展开全部缺失 qid，避免刷屏）
    print("\n[RESUME] Planned cases:", len(planned))
    for c in planned:
        ck = c["case_key"]
        done_n = len(done_by_case.get(ck, set()))
        miss_n = len(eval_qids) - done_n
        if miss_n == 0:
            continue
        print(
            f"[MISS] retriever={c['retriever']} k={c['k']} "
            f"index_dir={c['index_dir']} emb={c['embedding_model']} "
            f"done={done_n}/{len(eval_qids)} missing={miss_n}"
        )

    # 结果表每次重新生成（避免重复 header/重复行）；断点续传会把历史 trace 的已完成计数加回去
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "retriever",
                "k",
                "index_dir",
                "embedding_model",
                "accuracy",
                "correct",
                "total",
                "error",
            ],
        )
        writer.writeheader()

        # 固定顺序执行：按 planned 的顺序（retriever -> k -> index_dir）
        cases = planned

        for c in tqdm(cases, desc="评测组合", unit="case"):
            retr = c["retriever"]
            k = int(c["k"])
            index_dir = c["index_dir"]
            emb = c["embedding_model"]
            name = f"{Path(index_dir).name}:{retr}:k{k}"
            ck = _case_key(
                retriever=retr,
                k=int(k),
                index_dir=index_dir,
                embedding_model=emb,
                llm=llm,
            )
            try:
                prev_correct, prev_total = per_case_prev.get(ck, (0, 0))
                acc0, correct0, total0 = eval_one_case(
                    eval_rows=eval_rows,
                    retriever_name=retr,
                    k=int(k),
                    index_dir=index_dir,
                    embedding_model=emb,
                    llm=llm,
                    per_ctx_max_chars=int(CONFIG["per_ctx_max_chars"]),
                    total_max_chars=int(CONFIG["total_max_chars"]),
                    trace_fp=trace_fp,
                    done_set={(ck, qid) for qid in done_by_case.get(ck, set())},
                    case_key=ck,
                )
                correct = prev_correct + correct0
                total = prev_total + total0
                acc = None if total <= 0 else (correct / float(total))
                row = {
                    "name": name,
                    "retriever": retr,
                    "k": int(k),
                    "index_dir": index_dir,
                    "embedding_model": emb,
                    "accuracy": acc,
                    "correct": correct,
                    "total": total,
                    "error": "",
                }
            except Exception as e:
                row = {
                    "name": name,
                    "retriever": retr,
                    "k": int(k),
                    "index_dir": index_dir,
                    "embedding_model": emb,
                    "accuracy": None,
                    "correct": 0,
                    "total": 0,
                    "error": f"{type(e).__name__}: {e}",
                }
                print("\n[ERROR]", row)
                print(traceback.format_exc())
            writer.writerow(row)
            print(row)

    trace_fp.close()
    print(f"\n[DONE] 已写入：{out_csv}")
    print(f"[DONE] 逐题追踪日志：{trace_path}")


if __name__ == "__main__":
    main()


