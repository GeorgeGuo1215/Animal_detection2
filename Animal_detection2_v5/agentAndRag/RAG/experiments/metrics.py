from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Set, Tuple


@dataclass
class MetricResult:
    recall_at_k: float
    precision_at_k: float
    mrr_at_k: float
    ndcg_at_k: float
    book_acc_at_k: float


def recall_precision_mrr_ndcg_bookacc(
    *,
    ranked_chunk_ids: List[str],
    ranked_book_ids: List[str],
    gold_chunk_ids: Set[str],
    gold_book_id: Optional[str],
    k: int,
) -> Tuple[float, float, float, float, float]:
    top = ranked_chunk_ids[:k]
    top_books = ranked_book_ids[:k]

    hit_positions = [i for i, cid in enumerate(top, start=1) if cid in gold_chunk_ids]
    recall = 1.0 if hit_positions else 0.0
    precision = (len(hit_positions) / float(k)) if k > 0 else 0.0
    mrr = (1.0 / float(hit_positions[0])) if hit_positions else 0.0

    # nDCG：这里用二值相关度（命中 gold_chunk_ids 记 1）
    rels = [1 if cid in gold_chunk_ids else 0 for cid in top]
    dcg = 0.0
    for i, rel in enumerate(rels, start=1):
        if rel <= 0:
            continue
        dcg += (2.0**rel - 1.0) / (math.log2(i + 1.0))
    ideal_rels = sorted(rels, reverse=True)
    idcg = 0.0
    for i, rel in enumerate(ideal_rels, start=1):
        if rel <= 0:
            continue
        idcg += (2.0**rel - 1.0) / (math.log2(i + 1.0))
    ndcg = 0.0 if idcg <= 0 else dcg / idcg

    if gold_book_id is None:
        book_acc = 0.0
    else:
        book_acc = 1.0 if gold_book_id in top_books else 0.0
    return recall, precision, mrr, ndcg, book_acc


def aggregate(results: Iterable[Tuple[float, float, float, float, float]]) -> MetricResult:
    rs = list(results)
    if not rs:
        return MetricResult(0.0, 0.0, 0.0, 0.0, 0.0)
    n = float(len(rs))
    return MetricResult(
        recall_at_k=sum(x[0] for x in rs) / n,
        precision_at_k=sum(x[1] for x in rs) / n,
        mrr_at_k=sum(x[2] for x in rs) / n,
        ndcg_at_k=sum(x[3] for x in rs) / n,
        book_acc_at_k=sum(x[4] for x in rs) / n,
    )


