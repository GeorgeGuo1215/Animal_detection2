from __future__ import annotations

"""
错题归因分析（基于 qa_trace.jsonl）。

目标：
- 针对每个 case（retriever/k/index_dir/embedding_model）筛出错题
- 结合模型 reasoning_content + 检索 contexts 做归因
- 输出：
  - out/wrong_analysis.jsonl：逐题记录（含 label/解释）
  - out/wrong_summary.csv：每个 case 的统计汇总

说明：
- 默认使用“规则 + 轻量统计”做归因，不额外调用 LLM（省钱、可复现）
- 你也可以开启 USE_LLM=True，让模型对“错因”做自然语言归因（更像论文描述）
"""

import csv
import json
import os
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from tqdm import tqdm


# ----------------------------
# CONFIG（按需改）
# ----------------------------

TRACE_PATH = Path("RAG/experiments/out/qa_trace(12.26).jsonl")
OUT_JSONL = Path("RAG/experiments/out/wrong_analysis(12.26).jsonl")
OUT_SUMMARY_CSV = Path("RAG/experiments/out/wrong_summary(12.26).csv")

# 是否启用 LLM 进行更“论文友好”的归因描述（默认关闭）
# 建议：论文最终版再开启；日常分析先用规则归因，省钱且可复现。
USE_LLM = False

LLM = {
    "base_url": os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com"),
    "model": os.getenv("OPENAI_MODEL", "deepseek-reasoner"),
    # 只从环境变量读取，避免把 key 写进仓库
    "api_key": os.getenv("OPENAI_API_KEY", "") or os.getenv("API_KEY", "") or os.getenv("DEEPSEEK_API_KEY", ""),
    "timeout": 12000,
}

# 每题最多取多少段 context 做归因（避免太长）
MAX_CTX_FOR_ANALYSIS = 3
MAX_REASONING_CHARS = 1500
MAX_CTX_CHARS = 1200

# 抽样策略：从 baseline_retriever（默认 no_rag）答错里挑 N 个 qid
# 可复现：按 qid 排序取前 N
SAMPLE_N = 1000

# baseline retriever：用于抽样错题 qid 的“参照组”
BASELINE_RETRIEVER = "hybrid"

# 只分析 k=5 的各种 RAG 情况（默认不包含 baseline_retriever）
ANALYZE_K = 5


_RE_WORD = re.compile(r"[A-Za-z][A-Za-z0-9\\-]{2,}")


def read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except Exception:
                continue


def write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


def _case_id(case: dict) -> str:
    # 用 trace 里已有字段拼一个稳定 case_id（不引入新字段）
    return "|".join(
        [
            f"retriever={case.get('retriever')}",
            f"k={case.get('k')}",
            f"index_dir={case.get('index_dir')}",
            f"embedding_model={case.get('embedding_model')}",
            f"llm_model={case.get('llm_model')}",
        ]
    )


def summarize_accuracy(
    *,
    trace_path: Path,
    only_retriever: Optional[str] = None,
    only_k: Optional[int] = None,
    only_qids: Optional[set[str]] = None,
) -> Dict[str, Dict[str, Any]]:
    """
    从 trace.jsonl 中按 case 统计准确率（不需要重新跑实验）。
    返回：case_id -> {total, correct, acc, case}
    """
    per_total: Counter[str] = Counter()
    per_correct: Counter[str] = Counter()
    per_case_obj: Dict[str, dict] = {}

    for obj in read_jsonl(trace_path):
        case = obj.get("case")
        if not isinstance(case, dict):
            continue
        retr = str(case.get("retriever") or "")
        if only_retriever is not None and retr != str(only_retriever):
            continue
        if only_k is not None:
            try:
                kk = int(case.get("k") or 0)
            except Exception:
                kk = 0
            if kk != int(only_k):
                continue
        qid = str(obj.get("qid") or "")
        if only_qids is not None and qid not in only_qids:
            continue

        cid = _case_id(case)
        per_case_obj[cid] = case
        per_total[cid] += 1
        if obj.get("is_correct") is True:
            per_correct[cid] += 1

    out: Dict[str, Dict[str, Any]] = {}
    for cid in per_total.keys():
        t = int(per_total[cid])
        c = int(per_correct[cid])
        acc = None if t <= 0 else (c / float(t))
        out[cid] = {"total": t, "correct": c, "acc": acc, "case": per_case_obj.get(cid, {})}
    return out


def _group_key(case: dict) -> str:
    """
    把“同一套实验环境”聚到一起：同 index_dir + embedding_model + llm_model。
    retriever/k 作为组内变量用于对比。
    """
    return "|".join(
        [
            f"index_dir={case.get('index_dir')}",
            f"embedding_model={case.get('embedding_model')}",
            f"llm_model={case.get('llm_model')}",
        ]
    )


def compare_to_baseline(
    *,
    baseline_trace_path: Path,
    target_trace_path: Path,
    baseline_retriever: str,
    baseline_k: int,
    target_retrievers: Optional[set[str]] = None,
    target_k: Optional[int] = None,
    max_examples: int = 30,
) -> tuple[list[dict], list[dict]]:
    """
    对齐同一批 qid，在每个“实验组”（index_dir/emb/llm）内对比 baseline 与各 case：
    - n_common：两者都有记录的题数（用于公平比较）
    - acc_base_on_common / acc_case_on_common / delta
    - flips：baseline 对但 case 错 / baseline 错但 case 对

    返回：(rows_csv, examples_jsonl)
    """
    # baseline_groups: group_key -> baseline_case_id -> qid -> record
    baseline_groups: Dict[str, Dict[str, Dict[str, dict]]] = defaultdict(lambda: defaultdict(dict))
    baseline_case_by_id: Dict[str, dict] = {}
    # target_groups: group_key -> case_id -> qid -> record
    target_groups: Dict[str, Dict[str, Dict[str, dict]]] = defaultdict(lambda: defaultdict(dict))
    target_case_by_id: Dict[str, dict] = {}

    # 先加载 baseline trace（只保留 baseline_retriever + baseline_k）
    for obj in read_jsonl(baseline_trace_path):
        case = obj.get("case")
        if not isinstance(case, dict):
            continue
        retr = str(case.get("retriever") or "")
        if retr != baseline_retriever:
            continue
        try:
            kk = int(case.get("k") or 0)
        except Exception:
            kk = 0
        if kk != int(baseline_k):
            continue
        qid = str(obj.get("qid") or "")
        if not qid:
            continue
        cid = _case_id(case)
        baseline_case_by_id[cid] = case
        gk = _group_key(case)
        baseline_groups[gk][cid][qid] = obj

    # 再加载 target trace（只保留 target_retrievers + target_k）
    for obj in read_jsonl(target_trace_path):
        case = obj.get("case")
        if not isinstance(case, dict):
            continue
        retr = str(case.get("retriever") or "")
        if target_retrievers is not None and retr not in target_retrievers:
            continue
        if target_k is not None:
            try:
                kk = int(case.get("k") or 0)
            except Exception:
                kk = 0
            if kk != int(target_k):
                continue
        qid = str(obj.get("qid") or "")
        if not qid:
            continue
        cid = _case_id(case)
        target_case_by_id[cid] = case
        gk = _group_key(case)
        target_groups[gk][cid][qid] = obj

    rows: list[dict] = []
    examples: list[dict] = []

    # 只在同一个 group_key 内对齐（index_dir/emb/llm 一致才公平）
    for gk, t_cases in target_groups.items():
        b_cases = baseline_groups.get(gk)
        if not b_cases:
            continue
        base_id = sorted(b_cases.keys())[0]
        base_map = b_cases.get(base_id, {})
        base_qids = set(base_map.keys())
        if not base_qids:
            continue

        for cid, qmap in t_cases.items():
            ccase = target_case_by_id.get(cid, {})
            retr = str(ccase.get("retriever") or "")

            common = sorted(base_qids & set(qmap.keys()))
            if not common:
                continue

            base_correct = 0
            case_correct = 0
            flip_base_correct_case_wrong: list[str] = []
            flip_base_wrong_case_correct: list[str] = []
            for qid in common:
                b = base_map[qid]
                t = qmap[qid]
                b_ok = (b.get("is_correct") is True)
                t_ok = (t.get("is_correct") is True)
                base_correct += 1 if b_ok else 0
                case_correct += 1 if t_ok else 0
                if b_ok and (not t_ok):
                    flip_base_correct_case_wrong.append(qid)
                elif (not b_ok) and t_ok:
                    flip_base_wrong_case_correct.append(qid)

            n = len(common)
            acc_b = base_correct / float(n) if n else None
            acc_t = case_correct / float(n) if n else None
            delta = None if (acc_b is None or acc_t is None) else (acc_t - acc_b)
            rows.append(
                {
                    "group": gk,
                    "baseline_trace": str(baseline_trace_path),
                    "target_trace": str(target_trace_path),
                    "baseline_case_id": base_id,
                    "case_id": cid,
                    "retriever": retr,
                    "k": ccase.get("k"),
                    "n_common": n,
                    "acc_base_on_common": f"{acc_b:.4f}" if acc_b is not None else "",
                    "acc_case_on_common": f"{acc_t:.4f}" if acc_t is not None else "",
                    "delta": f"{delta:.4f}" if delta is not None else "",
                    "flip_base_correct_case_wrong": len(flip_base_correct_case_wrong),
                    "flip_base_wrong_case_correct": len(flip_base_wrong_case_correct),
                }
            )

            # 输出一些“带偏/纠正”的例子，方便你快速看原因
            for qid in flip_base_correct_case_wrong[: max_examples]:
                b = base_map[qid]
                t = qmap[qid]
                examples.append(
                    {
                        "group": gk,
                        "type": "harmful_rag_flip",
                        "qid": qid,
                        "baseline": {
                            "case_id": base_id,
                            "pred": b.get("pred_choice"),
                            "gold": b.get("gold_choice"),
                            "contexts": b.get("contexts", []),
                        },
                        "case": {
                            "case_id": cid,
                            "pred": t.get("pred_choice"),
                            "gold": t.get("gold_choice"),
                            "contexts": t.get("contexts", []),
                        },
                        "question_full": t.get("question_full") or b.get("question_full"),
                        "query": t.get("query") or b.get("query"),
                    }
                )
            for qid in flip_base_wrong_case_correct[: max_examples]:
                b = base_map[qid]
                t = qmap[qid]
                examples.append(
                    {
                        "group": gk,
                        "type": "helpful_rag_flip",
                        "qid": qid,
                        "baseline": {
                            "case_id": base_id,
                            "pred": b.get("pred_choice"),
                            "gold": b.get("gold_choice"),
                            "contexts": b.get("contexts", []),
                        },
                        "case": {
                            "case_id": cid,
                            "pred": t.get("pred_choice"),
                            "gold": t.get("gold_choice"),
                            "contexts": t.get("contexts", []),
                        },
                        "question_full": t.get("question_full") or b.get("question_full"),
                        "query": t.get("query") or b.get("query"),
                    }
                )

    # 稳定排序：先按 delta 升序（越负越糟），再按 n_common 降序
    def _krow(r: dict) -> tuple:
        try:
            d = float(r.get("delta") or 0.0)
        except Exception:
            d = 0.0
        return (d, -int(r.get("n_common") or 0), str(r.get("case_id") or ""))

    rows.sort(key=_krow)
    return rows, examples


def _tokens(text: str) -> List[str]:
    return [t.lower() for t in _RE_WORD.findall(text or "")]


def _overlap_score(a: str, b: str) -> float:
    ta = set(_tokens(a))
    tb = set(_tokens(b))
    if not ta or not tb:
        return 0.0
    return len(ta & tb) / float(len(ta))


def _truncate(s: str, n: int) -> str:
    s = (s or "").strip()
    if len(s) <= n:
        return s
    return s[:n] + " ...(截断)..."


def heuristic_label(item: dict) -> Tuple[str, str]:
    """
    规则归因（尽量简单、可复现）：
    - retrieval_miss: query 与 top contexts 词面重合很低，或 contexts 为空
    - retrieval_noise: contexts 很多但重合低（噪声注入）
    - generation_error: contexts 与 query 重合较高，但仍答错（模型推理/选择错误）
    - unknown: 兜底
    """
    query = item.get("query", "") or ""
    contexts: List[str] = item.get("contexts", []) or []
    reasoning = item.get("reasoning_content", "") or ""

    if not contexts:
        return "retrieval_miss", "未检索到上下文（contexts 为空），更像检索缺失或该题不在语料覆盖范围。"

    top_ctx = "\n\n".join(contexts[:MAX_CTX_FOR_ANALYSIS])
    ov = _overlap_score(query, top_ctx)
    if ov < 0.05:
        if len(contexts) >= 3:
            return "retrieval_noise", f"检索到了多段上下文但与题干词面重合很低（overlap={ov:.3f}），更像噪声干扰。"
        return "retrieval_miss", f"上下文与题干重合很低（overlap={ov:.3f}），更像没有命中相关证据。"

    # 有一定重合，但仍错
    if "not sure" in reasoning.lower() or "uncertain" in reasoning.lower() or "guess" in reasoning.lower():
        return "generation_error", "有相关上下文但 reasoning 表现出不确定/猜测，更像生成侧选择错误。"

    return "generation_error", f"上下文与题干有一定相关性（overlap={ov:.3f}），但仍答错，更像生成/推理侧错误或上下文不够直接支持。"


def llm_attribution(item: dict) -> Tuple[str, str]:
    """
    可选：用 LLM 对错因做更细的文字归因（论文更好写）。
    返回 (label, explanation)
    """
    from openai import OpenAI

    client = OpenAI(api_key=LLM["api_key"], base_url=LLM["base_url"], timeout=int(LLM["timeout"]))
    case = item.get("case", {}) or {}
    query = item.get("query", "") or ""
    q_full = item.get("question_full", "") or ""
    gold = item.get("gold_choice")
    pred = item.get("pred_choice")
    reasoning = _truncate(item.get("reasoning_content", "") or "", MAX_REASONING_CHARS)
    contexts = item.get("contexts", []) or []
    ctx = "\n\n".join([_truncate(c, MAX_CTX_CHARS) for c in contexts[:MAX_CTX_FOR_ANALYSIS]])

    system = (
        "你是论文评测分析员。给定一道选择题的：题干、检索到的上下文、模型思考过程、预测与标准答案。"
        "请做“错因归因”。"
        "只输出 JSON：{label, explanation}。\n"
        "label 只能从以下选择一个：retrieval_miss, retrieval_noise, generation_error, out_of_scope。\n"
        "retrieval_miss=没检索到正确证据；retrieval_noise=检索到的多是噪声带偏；generation_error=证据可能足够但模型选错；out_of_scope=题目与书库关系弱/书里可能没有。\n"
        "explanation 用中文，1-3 句，指出关键证据与判断依据。"
    )
    user = (
        f"[Case]\n{case}\n\n"
        f"[Question]\n{q_full}\n\n"
        f"[Query]\n{query}\n\n"
        f"[Gold]\n{gold}\n\n"
        f"[Pred]\n{pred}\n\n"
        f"[Contexts]\n{ctx}\n\n"
        f"[Reasoning]\n{reasoning}\n"
    )
    resp = client.chat.completions.create(
        model=LLM["model"],
        messages=[{"role": "system", "content": system}, {"role": "user", "content": user}],
        temperature=0.0,
        stream=False,
    )
    txt = (resp.choices[0].message.content or "").strip()
    try:
        obj = json.loads(txt)
        label = str(obj.get("label") or "unknown")
        explanation = str(obj.get("explanation") or "")
        return label, explanation
    except Exception:
        # 兜底
        return "unknown", _truncate(txt, 500)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="错题归因 / accuracy 汇总（基于 qa_trace.jsonl）")
    ap.add_argument("--trace", type=str, default=str(TRACE_PATH), help="qa_trace.jsonl 路径")
    ap.add_argument("--out-jsonl", type=str, default=str(OUT_JSONL))
    ap.add_argument("--out-summary-csv", type=str, default=str(OUT_SUMMARY_CSV))

    ap.add_argument("--baseline-retriever", type=str, default=str(BASELINE_RETRIEVER), help="用于抽样错题 qid 的 baseline 组")
    ap.add_argument("--sample-n", type=int, default=int(SAMPLE_N))
    ap.add_argument("--analyze-k", type=int, default=int(ANALYZE_K))

    # 只做 accuracy 汇总（不做错因归因）
    ap.add_argument("--acc-only", action="store_true", help="只输出 accuracy 汇总并退出")
    ap.add_argument("--only-retriever", type=str, default=None, help="仅统计某个 retriever（如 no_rag/hybrid/dense）")
    ap.add_argument("--only-k", type=int, default=None, help="仅统计某个 k（如 0/5）")
    ap.add_argument("--only-qids", type=str, default="", help="仅统计这些 qid（逗号分隔）；留空表示全量")

    # 对齐 baseline 的“公平对比”
    ap.add_argument("--compare-baseline", action="store_true", help="对齐同一批 qid，对比 baseline 与各 case 的 delta accuracy")
    ap.add_argument("--baseline-trace", type=str, default="", help="baseline 组 trace（留空则使用 --trace）")
    ap.add_argument("--target-trace", type=str, default="", help="target 组 trace（留空则使用 --trace）")
    ap.add_argument("--baseline-k", type=int, default=0, help="baseline 的 k（默认 0）")
    ap.add_argument("--target-retrievers", type=str, default="", help="仅对比这些 retriever（逗号分隔；留空表示全部）")
    ap.add_argument("--target-k", type=int, default=None, help="仅对比这个 k（例如 5）")
    ap.add_argument("--compare-out-csv", type=str, default="", help="对比报告 CSV 输出路径（默认：out_summary 同目录 compare_baseline.csv）")
    ap.add_argument("--compare-out-jsonl", type=str, default="", help="翻转样例 JSONL 输出路径（默认：out_summary 同目录 compare_examples.jsonl）")
    ap.add_argument("--max-examples", type=int, default=30, help="每个 case 最多输出多少个翻转样例")

    args = ap.parse_args()

    trace_path = Path(args.trace)
    out_jsonl = Path(args.out_jsonl)
    out_summary_csv = Path(args.out_summary_csv)
    baseline_retriever = str(args.baseline_retriever or "no_rag")
    sample_n = int(args.sample_n)
    analyze_k = int(args.analyze_k)

    only_qids = None
    if str(args.only_qids or "").strip():
        only_qids = {x.strip() for x in str(args.only_qids).split(",") if x.strip()}

    if not trace_path.exists():
        raise SystemExit(f"找不到 trace：{trace_path}")

    if args.acc_only:
        acc = summarize_accuracy(
            trace_path=trace_path,
            only_retriever=str(args.only_retriever) if args.only_retriever else None,
            only_k=int(args.only_k) if args.only_k is not None else None,
            only_qids=only_qids,
        )
        if not acc:
            raise SystemExit("没有找到匹配记录（请检查 --only-retriever/--only-k/--only-qids 过滤条件）")
        print("[ACC] summary (by case_id):")
        for cid in sorted(acc.keys()):
            a = acc[cid]["acc"]
            if a is None:
                print(f"- {cid}  correct={acc[cid]['correct']} total={acc[cid]['total']} acc=")
            else:
                print(f"- {cid}  correct={acc[cid]['correct']} total={acc[cid]['total']} acc={a:.4f}")
        return

    if args.compare_baseline:
        baseline_trace_path = Path(args.baseline_trace) if str(args.baseline_trace or "").strip() else trace_path
        target_trace_path = Path(args.target_trace) if str(args.target_trace or "").strip() else trace_path
        if not baseline_trace_path.exists():
            raise SystemExit(f"找不到 baseline trace：{baseline_trace_path}")
        if not target_trace_path.exists():
            raise SystemExit(f"找不到 target trace：{target_trace_path}")
        targets = None
        if str(args.target_retrievers or "").strip():
            targets = {x.strip() for x in str(args.target_retrievers).split(",") if x.strip()}
        rows, examples = compare_to_baseline(
            baseline_trace_path=baseline_trace_path,
            target_trace_path=target_trace_path,
            baseline_retriever=baseline_retriever,
            baseline_k=int(args.baseline_k),
            target_retrievers=targets,
            target_k=int(args.target_k) if args.target_k is not None else None,
            max_examples=int(args.max_examples),
        )
        if not rows:
            raise SystemExit("compare_baseline：没有找到可对齐的 baseline/case 组合（可能两份 trace 的 index_dir/emb/llm 不一致，或 baseline/target 过滤条件不匹配）")

        out_dir = out_summary_csv.parent
        out_csv = Path(args.compare_out_csv) if str(args.compare_out_csv or "").strip() else (out_dir / "compare_baseline.csv")
        out_js = Path(args.compare_out_jsonl) if str(args.compare_out_jsonl or "").strip() else (out_dir / "compare_examples.jsonl")

        out_csv.parent.mkdir(parents=True, exist_ok=True)
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "group",
                "baseline_trace",
                "target_trace",
                "baseline_case_id",
                "case_id",
                "retriever",
                "k",
                "n_common",
                "acc_base_on_common",
                "acc_case_on_common",
                "delta",
                "flip_base_correct_case_wrong",
                "flip_base_wrong_case_correct",
            ]
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in fieldnames})

        write_jsonl(out_js, examples)
        print(f"[DONE] compare_csv: {out_csv}")
        print(f"[DONE] compare_examples: {out_js}")
        return

    # 1) 先从 baseline_retriever 的错题里抽 qid
    norag_wrong_qids: set[str] = set()
    for obj in tqdm(read_jsonl(trace_path), desc=f"扫描 trace（抽 baseline={baseline_retriever} 错题）", unit="条"):
        case = obj.get("case")
        if not isinstance(case, dict):
            continue
        if str(case.get("retriever")) != baseline_retriever:
            continue
        if obj.get("is_correct") is False:
            qid = str(obj.get("qid") or "")
            if qid:
                norag_wrong_qids.add(qid)

    picked_qids = sorted(norag_wrong_qids)[: int(sample_n)]
    if not picked_qids:
        raise SystemExit(f"在 trace 里没有找到 baseline 错题记录（retriever={baseline_retriever} 且 is_correct=false）")

    print(f"\n[SAMPLE] 从 baseline={baseline_retriever} 错题中抽取的 qid（用于分析）:")
    for q in picked_qids:
        print(" -", q)

    # 2) 只保留：这些 qid + analyze_k + 各种 RAG retriever（默认不含 baseline_retriever）
    wrong_items: List[dict] = []
    per_case_total = Counter()
    per_case_wrong = Counter()
    seen_cases: set[str] = set()
    qid_case_seen: dict[str, set[str]] = defaultdict(set)

    for obj in tqdm(read_jsonl(trace_path), desc=f"扫描 trace（筛选 k={analyze_k} & 抽样 qid）", unit="条"):
        case = obj.get("case")
        if not isinstance(case, dict):
            continue

        qid = str(obj.get("qid") or "")
        if qid not in picked_qids:
            continue

        retr = str(case.get("retriever") or "")
        if retr == baseline_retriever:
            continue
        try:
            k = int(case.get("k") or 0)
        except Exception:
            k = 0
        if k != int(analyze_k):
            continue

        cid = _case_id(case)
        seen_cases.add(cid)
        qid_case_seen[qid].add(cid)
        per_case_total[cid] += 1
        if obj.get("is_correct") is False:
            per_case_wrong[cid] += 1
            wrong_items.append(obj)

    if not seen_cases:
        raise SystemExit(f"没有找到匹配记录：抽样 qid + k={analyze_k} 的 RAG 组合")

    # 3) 打印缺失情况（方便核对：每个 qid 在哪些 case 下缺记录）
    print(f"\n[CHECK] k={analyze_k} 下参与分析的 case 数: {len(seen_cases)}")
    for qid in picked_qids:
        miss = sorted(seen_cases - qid_case_seen.get(qid, set()))
        if miss:
            print(f"[MISS] qid={qid} 缺少 {len(miss)} 个 case 记录（可能没跑到/中断）：")
            for m in miss[:10]:
                print("   -", m)
            if len(miss) > 10:
                print("   ...(省略)...")

    out_rows = []
    label_counter_by_case: Dict[str, Counter] = defaultdict(Counter)

    if USE_LLM and not (LLM["api_key"] and LLM["base_url"] and LLM["model"]):
        raise SystemExit("USE_LLM=True 但未配置 LLM（请设环境变量 API_KEY/DEEPSEEK_API_KEY 等）")

    for obj in tqdm(wrong_items, desc="归因错题（仅抽样 qid 且 k=5）", unit="题"):
        case = obj.get("case", {}) or {}
        cid = _case_id(case)

        if USE_LLM:
            label, explanation = llm_attribution(obj)
        else:
            label, explanation = heuristic_label(obj)

        label_counter_by_case[cid][label] += 1
        out_rows.append(
            {
                "case_id": cid,
                "qid": obj.get("qid"),
                "query": obj.get("query"),
                "gold_choice": obj.get("gold_choice"),
                "pred_choice": obj.get("pred_choice"),
                "label": label,
                "explanation": explanation,
                "retriever": case.get("retriever"),
                "k": case.get("k"),
                "index_dir": case.get("index_dir"),
                "embedding_model": case.get("embedding_model"),
            }
        )

    write_jsonl(out_jsonl, out_rows)

    # 汇总表
    out_summary_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_summary_csv.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "case_id",
            "total",
            "wrong",
            "acc",
            "top_label",
            "top_label_count",
            "label_breakdown",
        ]
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for cid in sorted(per_case_total.keys()):
            total = per_case_total[cid]
            wrong = per_case_wrong[cid]
            acc = None if total <= 0 else (1.0 - wrong / float(total))
            labels = label_counter_by_case.get(cid, Counter())
            top_label, top_cnt = ("", 0)
            if labels:
                top_label, top_cnt = labels.most_common(1)[0]
            w.writerow(
                {
                    "case_id": cid,
                    "total": total,
                    "wrong": wrong,
                    "acc": f"{acc:.4f}" if acc is not None else "",
                    "top_label": top_label,
                    "top_label_count": top_cnt,
                    "label_breakdown": dict(labels),
                }
            )

    print(f"\n[DONE] 逐题错因：{out_jsonl}")
    print(f"[DONE] 汇总表：  {out_summary_csv}")


if __name__ == "__main__":
    main()


