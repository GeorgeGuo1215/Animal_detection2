from __future__ import annotations

"""
切片/截断检查器

用途：
- 给定一个 query（或完整题干 question_full）
- 指定检索策略/retriever、k、索引目录、截断参数
- 打印：
  1) 原始检索到的每段 chunk（不截断）
  2) 截断后最终喂给模型的 contexts（对比截断影响）
  3) 可选：把 contexts + question_full 发给 LLM 看回答（用于检查“截断导致答错”）
"""

import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))      # experiments/
sys.path.insert(0, str(HERE.parents[1]))  # RAG/

from retrievers import BM25Retriever, DenseRetriever, HybridRetriever, TwoStageBookThenChunk
from simple_rag.vector_store import NumpyVectorStore


# ----------------------------
# CONFIG（你只需要改这里）
# ----------------------------

CFG = {
    # 选择一个索引（两套都可以测）
    "index_dir": "RAG/data/rag_index_e5",
    "embedding_model": "intfloat/multilingual-e5-small",
    # 检索策略
    "retriever": "dense",  # dense / bm25 / hybrid / two_stage
    "k": 5,
    "top_k_books": 3,  # two_stage 用

    # 把 query 写死在这里（你要检查哪道题就改这行）
    "query": "salmonellae",

    # 也可以把完整题干写死（用于发给 LLM）
    "question_full": "Which medium is used for growing salmonellae?\n\na) Blood agar\nb) Chocolate agar\nc) McConkey agar\nd) Deoxycholate citrate agar",

    # 截断参数（你可以自由调）
    "per_ctx_max_chars": 1200,
    "total_max_chars": 6000,

    # 是否调用 LLM 试答（可关）
    "call_llm": False,
    "llm": {
        "base_url": os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com"),
        "model": os.getenv("OPENAI_MODEL", "deepseek-reasoner"),
        "api_key": os.getenv("OPENAI_API_KEY", "") or os.getenv("API_KEY", "") or os.getenv("DEEPSEEK_API_KEY", ""),
        "timeout": 1200,
        "enable_thinking": True,
    },
}


def build_chunkid_to_text(metas: list[dict]) -> dict[str, str]:
    m = {}
    for row in metas:
        cid = row.get("chunk_id")
        txt = row.get("text")
        if isinstance(cid, str) and isinstance(txt, str) and txt:
            m[cid] = txt
    return m


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


def call_llm_choice(*, llm: dict, question_full: str, contexts: list[str]) -> Tuple[str, str]:
    from openai import OpenAI

    client = OpenAI(api_key=llm["api_key"], base_url=llm["base_url"], timeout=int(llm["timeout"]))
    ctx = "\n\n".join([f"[Context {i+1}]\n{c}" for i, c in enumerate(contexts)])
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
    return content, reasoning


def main() -> None:
    index_dir = Path(CFG["index_dir"])
    store = NumpyVectorStore(index_dir)
    store.load()
    metas = store._meta  # noqa: SLF001
    cid2txt = build_chunkid_to_text(metas)

    retr = CFG["retriever"]
    if retr == "dense":
        r = DenseRetriever(index_dir, CFG["embedding_model"])
    elif retr == "bm25":
        r = BM25Retriever(metas)
    elif retr == "hybrid":
        r = HybridRetriever(index_dir, CFG["embedding_model"])
    elif retr == "two_stage":
        r = TwoStageBookThenChunk(index_dir, CFG["embedding_model"])
    else:
        raise SystemExit(f"未知 retriever: {retr}")

    q = CFG["query"]
    if retr == "two_stage":
        hits = r.retrieve(q, top_k=int(CFG["k"]), top_k_books=int(CFG["top_k_books"]))
    else:
        hits = r.retrieve(q, top_k=int(CFG["k"]))

    raw_ctxs: List[str] = []
    print("\n================ 原始检索结果（不截断） ================\n")
    for i, h in enumerate(hits, start=1):
        txt = cid2txt.get(h.chunk_id, "")
        raw_ctxs.append(txt)
        print(f"[{i}] score={getattr(h, 'score', None)}  book={getattr(h, 'book_id', None)}  chunk_id={h.chunk_id}")
        print(txt[:3000] + ("\n...(超长省略)...\n" if len(txt) > 3000 else "\n"))

    clipped = clip_contexts(
        raw_ctxs,
        per_ctx_max_chars=int(CFG["per_ctx_max_chars"]),
        total_max_chars=int(CFG["total_max_chars"]),
    )
    print("\n================ 截断后实际输入 LLM 的 contexts ================\n")
    for i, c in enumerate(clipped, start=1):
        print(f"[Context {i}] len={len(c)}")
        print(c)
        print("\n---\n")

    if CFG["call_llm"]:
        llm = CFG["llm"]
        if not (llm.get("api_key") and llm.get("base_url") and llm.get("model")):
            raise SystemExit("call_llm=True 但未配置 llm.api_key/base_url/model")
        ans, reasoning = call_llm_choice(llm=llm, question_full=CFG["question_full"], contexts=clipped)
        print("\n================ LLM 试答 ================\n")
        print("Answer:", ans)
        if reasoning:
            print("\n[reasoning_content]\n", reasoning[:3000])


if __name__ == "__main__":
    main()



