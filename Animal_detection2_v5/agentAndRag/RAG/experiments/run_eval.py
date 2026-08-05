from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
# 允许直接运行：python RAG/experiments/run_eval.py ...
sys.path.insert(0, str(HERE.parent))      # experiments/（common/metrics/retrievers）
sys.path.insert(0, str(HERE.parents[1]))  # RAG/（simple_rag）

from common import read_jsonl  # noqa: E402
from metrics import aggregate, recall_precision_mrr_ndcg_bookacc  # noqa: E402
from retrievers import BM25Retriever, DenseRetriever, HybridRetriever, TwoStageBookThenChunk  # noqa: E402
from simple_rag.vector_store import NumpyVectorStore  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="RAG 检索评测：Recall@K / Precision@K / MRR / nDCG / Book-Acc")
    parser.add_argument("--index-dir", type=str, default="RAG/data/rag_index_e5", help="向量库目录（embeddings.npy/meta.jsonl）")
    parser.add_argument("--evalset", type=str, default="RAG/experiments/out/eval_silver.jsonl")
    parser.add_argument("--k", type=int, default=5)
    parser.add_argument(
        "--retriever",
        type=str,
        default="dense",
        choices=["dense", "bm25", "hybrid", "two_stage"],
    )
    parser.add_argument(
        "--embedding-model",
        type=str,
        default="intfloat/multilingual-e5-small",
        help="dense/hybrid/two_stage 需要",
    )
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--hybrid-w", type=float, default=0.6)
    parser.add_argument("--top-k-books", type=int, default=3, help="two_stage 用：先选书的数量")
    args = parser.parse_args()

    index_dir = Path(args.index_dir)
    eval_path = Path(args.evalset)
    k = int(args.k)

    # 从 meta.jsonl 读取 chunk 文本，供 BM25 / hybrid 使用
    store = NumpyVectorStore(index_dir)
    store.load()
    metas = store._meta  # noqa: SLF001

    if args.retriever == "dense":
        r = DenseRetriever(index_dir, args.embedding_model, device=args.device)
    elif args.retriever == "bm25":
        r = BM25Retriever(metas)
    elif args.retriever == "hybrid":
        # 复用 simple_rag 的 multi-route（dense+bm25, RRF 融合）
        # 注意：hybrid_w 参数不再使用（保留 CLI 兼容）
        r = HybridRetriever(index_dir, args.embedding_model, device=args.device)
    elif args.retriever == "two_stage":
        r = TwoStageBookThenChunk(index_dir, args.embedding_model, device=args.device)
    else:
        raise SystemExit(f"未知 retriever: {args.retriever}")

    per_query = []
    n = 0
    for row in read_jsonl(eval_path):
        q = row.get("query", "")
        gold_chunk_id = row.get("gold_chunk_id")
        gold_book_id = row.get("gold_book_id")
        if not q or not gold_chunk_id:
            continue

        if args.retriever == "two_stage":
            hits = r.retrieve(q, top_k=k, top_k_books=int(args.top_k_books))
        else:
            hits = r.retrieve(q, top_k=k)

        ranked_chunk_ids = [h.chunk_id for h in hits]
        ranked_book_ids = [h.book_id for h in hits]

        per_query.append(
            recall_precision_mrr_ndcg_bookacc(
                ranked_chunk_ids=ranked_chunk_ids,
                ranked_book_ids=ranked_book_ids,
                gold_chunk_ids={str(gold_chunk_id)},
                gold_book_id=str(gold_book_id) if gold_book_id else None,
                k=k,
            )
        )
        n += 1

    res = aggregate(per_query)
    print(
        "\n".join(
            [
                f"evalset: {eval_path}",
                f"index:   {index_dir}",
                f"method:  {args.retriever}",
                f"K:       {k}",
                f"N:       {n}",
                "",
                f"Recall@{k}:      {res.recall_at_k:.4f}",
                f"Precision@{k}:   {res.precision_at_k:.4f}",
                f"MRR@{k}:         {res.mrr_at_k:.4f}",
                f"nDCG@{k}:        {res.ndcg_at_k:.4f}",
                f"Book-Acc@{k}:    {res.book_acc_at_k:.4f}",
            ]
        )
    )


if __name__ == "__main__":
    main()


