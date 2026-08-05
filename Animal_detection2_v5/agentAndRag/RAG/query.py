from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import List


from RAG.simple_rag.config import RagConfig, default_config
from RAG.simple_rag.pipeline import search
from RAG.simple_rag.query_rewrite import LLMRewriter, NoRewrite, TemplateRewriter
from RAG.simple_rag.retrieval import build_default_multiroute
from RAG.simple_rag.context_utils import build_neighbor_contexts
from RAG.simple_rag.reranker import CrossEncoderReranker
from RAG.simple_rag.vector_store import NumpyVectorStore


_RE_WORD = re.compile(r"[A-Za-z][A-Za-z0-9\\-]{2,}")


def overlap_score(query: str, ctx: str) -> float:
    tq = set(t.lower() for t in _RE_WORD.findall(query or ""))
    if not tq:
        return 0.0
    tc = set(t.lower() for t in _RE_WORD.findall(ctx or ""))
    if not tc:
        return 0.0
    return len(tq & tc) / float(len(tq))


def _norm_text(s: str) -> str:
    return re.sub(r"\s+", " ", (s or "").strip())


def _text_hash(s: str) -> str:
    return hashlib.sha1(_norm_text(s).encode("utf-8")).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description="本地 RAG 向量索引查询")
    parser.add_argument("query", type=str, help="你的问题/关键词")
    parser.add_argument("--raw-dir", type=str, default=None, help="原始 .mmd 目录（默认 RAG/data/raw）")
    parser.add_argument("--index-dir", type=str, default=None, help="索引目录（默认 RAG/data/rag_index）")
    parser.add_argument(
        "--embedding-model",
        type=str,
        default="intfloat/multilingual-e5-small",
        help="必须与入库时一致（默认 multilingual-e5-small）",
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--as-json", action="store_true", help="以 JSON 输出")
    parser.add_argument(
        "--multi-route",
        action="store_true",
        help="启用多路召回（dense + bm25）并做简单融合",
    )
    parser.add_argument(
        "--rewrite",
        type=str,
        default="template",
        choices=["none", "template", "llm"],
        help="query 重写策略（用于 multi-route）",
    )
    parser.add_argument("--rewrite-base-url", type=str, default=None, help="LLM 重写使用的 OpenAI-compatible base URL")
    parser.add_argument("--rewrite-api-key", type=str, default=None, help="LLM 重写使用的 API Key（默认环境变量）")
    parser.add_argument("--rewrite-model", type=str, default=None, help="LLM 重写使用的模型名（默认环境变量）")
    parser.add_argument("--rewrite-max-out", type=int, default=5, help="LLM/template 重写最多返回多少个 query")
    parser.add_argument("--rewrite-timeout-s", type=float, default=60.0, help="LLM 重写请求超时秒数")
    parser.add_argument(
        "--expand-neighbors",
        type=int,
        default=1,
        help="命中后拼接同一 source_path 的相邻 chunk（例如 1 表示拼 i-1,i,i+1；默认 0 不拼接）",
    )
    parser.add_argument("--rerank", action="store_true", help="启用 reranker：先召回小块，再用 CrossEncoder 重新排序")
    parser.add_argument(
        "--rerank-model",
        type=str,
        default="BAAI/bge-reranker-large",
        help="reranker 模型名或本地路径（默认 BAAI/bge-reranker-large）",
    )
    parser.add_argument(
        "--rerank-candidates",
        type=int,
        default=10,
        help="rerank 前从 retriever 召回的小块数量（默认 10）",
    )
    parser.add_argument("--rerank-batch-size", type=int, default=32, help="reranker batch size（默认 32）")
    parser.add_argument(
        "--rerank-keep-topn",
        type=int,
        default=0,
        help="rerank 后仅保留前 N 个块再做邻居拼接（0=不限制）",
    )
    parser.add_argument(
        "--rerank-filter-overlap",
        type=float,
        default=0.15,
        help="rerank 后按词面 overlap_score 过滤噪声块（0=不启用；建议 0.10~0.25）",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[0].parent
    cfg0 = default_config(repo_root)
    cfg = RagConfig(
        raw_dir=Path(args.raw_dir) if args.raw_dir else cfg0.raw_dir,
        index_dir=Path(args.index_dir) if args.index_dir else cfg0.index_dir,
        embedding_model=args.embedding_model,
        chunk_words=cfg0.chunk_words,
        chunk_overlap_words=cfg0.chunk_overlap_words,
        min_chunk_words=cfg0.min_chunk_words,
    )

    retrieve_k = int(args.top_k)
    if args.rerank:
        retrieve_k = max(retrieve_k, int(args.rerank_candidates))

    if not args.multi_route:
        hits = search(cfg, args.query, top_k=retrieve_k, device=args.device)
    else:
        if args.rewrite == "none":
            rewriter = NoRewrite()
        elif args.rewrite == "llm":
            rewriter = LLMRewriter(
                base_url=args.rewrite_base_url,
                api_key=args.rewrite_api_key,
                model=args.rewrite_model,
                max_out=int(args.rewrite_max_out),
                timeout_s=float(args.rewrite_timeout_s),
            )
        else:
            rewriter = TemplateRewriter(max_out=int(args.rewrite_max_out))
        mr = build_default_multiroute(
            index_dir=str(cfg.index_dir),
            embedding_model=cfg.embedding_model,
            device=args.device,
            enable_bm25=True,
            rewriter=rewriter,
        )
        hits = [
            {
                "score": float(h.score),
                "source_path": h.meta.get("source_path"),
                "chunk_index": h.meta.get("chunk_index"),
                "n_words": h.meta.get("n_words"),
                "text": h.meta.get("text"),
                "chunk_id": h.meta.get("chunk_id"),
            }
            for h in mr.retrieve(args.query, top_k=retrieve_k)
        ]

    # 先 rerank 小块：根据 (query, chunk_text) 重新排序，然后只保留 top_k 作为“种子块”
    if args.rerank and hits:
        passages = [(h.get("text") or "").strip() for h in hits]
        rr = CrossEncoderReranker(args.rerank_model, device=args.device)
        order = rr.rerank(
            query=args.query,
            passages=passages,
            top_k=int(args.top_k),
            batch_size=int(args.rerank_batch_size),
        )
        new_hits: List[dict] = []
        for r in order:
            h = dict(hits[int(r.index)])
            # 保留检索分数，方便分析
            h["score_retrieval"] = float(h.get("score") or 0.0)
            # 统一把 score 设为 rerank 分数（用于后续排序/邻居拼接区间打分）
            h["score"] = float(r.score)
            h["score_rerank"] = float(r.score)
            new_hits.append(h)
        hits = new_hits

        # rerank 后去噪过滤：先按 overlap 阈值过滤，再可选只保留 topN
        thr = float(args.rerank_filter_overlap or 0.0)
        if thr > 0.0:
            kept = []
            for h in hits:
                ov = overlap_score(args.query, (h.get("text") or ""))
                hh = dict(h)
                hh["overlap"] = float(ov)
                if ov >= thr:
                    kept.append(hh)
            if kept:
                hits = kept
        topn = int(args.rerank_keep_topn or 0)
        if topn > 0 and len(hits) > topn:
            hits = hits[:topn]

    # 去重：相同文本只保留一次（常见于重复导入的同一本书/重复文件）
    if hits:
        seen = set()
        deduped: List[dict] = []
        for h in hits:
            txt = (h.get("text") or "").strip()
            th = _text_hash(txt)
            if th in seen:
                continue
            seen.add(th)
            hh = dict(h)
            hh["text_hash"] = th
            deduped.append(hh)
        hits = deduped

    # 小块命中后：邻居拼接为更完整的 contexts（可直接喂给 LLM）
    contexts: List[dict] = []
    if int(args.expand_neighbors) > 0 and hits:
        store = NumpyVectorStore(cfg.index_dir)
        store.load()
        contexts = build_neighbor_contexts(
            metas=store._meta,  # noqa: SLF001（CLI 工具允许直接读）
            hits=hits,
            neighbor_n=int(args.expand_neighbors),
        )
    if args.as_json:
        if contexts:
            print(
                json.dumps(
                    {"hits": hits, "contexts": contexts, "expand_neighbors": int(args.expand_neighbors)},
                    ensure_ascii=False,
                    indent=2,
                )
            )
        else:
            print(json.dumps(hits, ensure_ascii=False, indent=2))
        return

    if contexts:
        print(f"\n[INFO] expand_neighbors={int(args.expand_neighbors)}：输出拼接后的 contexts（已去重合并）")
        for i, c in enumerate(contexts[: int(args.top_k)], start=1):
            print(
                f"\n[{i}] score={float(c['score']):.4f}  chunk={c['chunk_index_start']}..{c['chunk_index_end']}  n_chunks={c['n_chunks']}"
            )
            print(f"    source: {c.get('source_path')}")
            text = (c.get("text") or "").strip()
            if len(text) > 1200:
                text = text[:1200] + "\n...(截断)..."
            print(text)
        return

    for i, h in enumerate(hits, start=1):
        print(f"\n[{i}] score={h['score']:.4f}  chunk={h.get('chunk_index')}  words={h.get('n_words')}")
        print(f"    source: {h.get('source_path')}")
        text = (h.get("text") or "").strip()
        if len(text) > 1200:
            text = text[:1200] + "\n...(截断)..."
        print(text)


if __name__ == "__main__":
    main()


