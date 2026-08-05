## RAG 评测实验（论文用）

目标：对比不同 RAG 方案在同一批书籍 `.mmd` 语料上的检索质量。

### 你提到的三个主变量（建议这样“可操作化”）

- **embedding 模型选择**：同一套 chunk/同一套 query，分别用不同 embedding 建库，测 Recall@K/MRR 等。
- **isolation（隔离）**：把“检索范围”从全库变成“先确定 book/domain，再在子库检索”，并对比：
  - **Book-Accuracy@K**：topK 里是否出现正确 book
  - **Chunk-Recall@K**：是否命中正确 chunk（或同 book 的相关 chunk）
- **多层级查询**（multi-stage / hierarchical retrieval）：
  - Stage1：粗检（book 或 chapter）→ Stage2：细检（chunk）
  - 可选 Stage3：rerank（不做也可以先写成 future work）

### 统一检索指标（建议论文主表）

- Recall@K
- Precision@K
- MRR@K
- nDCG@K（如果你的标注是多相关度/多 gold）
- Book-Accuracy@K（做 isolation/多层级时尤其关键）

### 数据集（建议两套）

1) **Silver 集（自动生成）**：从 chunk 自动抽取关键词/标题作为 query，gold=该 chunk（用于快速大规模对比）。
2) **Human 集（人工标注）**：抽 100~300 个真实问题，人标注相关 chunk/book（论文结论更有说服力）。

---

## 端到端 Accuracy@K（使用你提供的两份选择题）

你已决定主打 **端到端选择题准确率**（不需要检索 gold 标注）。

### 1) 生成 QA 评测集（query 来自 input 的题干部分）

```bash
python RAG/experiments/build_qa_evalset_from_alpaca.py
```

输出：`RAG/experiments/out/qa_eval.jsonl`

### 2) 为不同 embedding 建多套索引（可选，但论文必须做）

要比较 embedding 模型，必须分别建库到不同目录，例如：

```bash
python RAG/ingest.py --embedding-model intfloat/multilingual-e5-small --index-dir RAG/data/rag_index_e5
python RAG/ingest.py --embedding-model sentence-transformers/all-MiniLM-L6-v2 --index-dir RAG/data/rag_index_minilm
```

### 3) 单次评测（Accuracy@K）

```bash
python RAG/experiments/run_qa_rag_eval.py --retriever dense --k 5 --index-dir RAG/data/rag_index_e5 --embedding-model intfloat/multilingual-e5-small --base-url ... --api-key ... --model ...
```

也可以跑 **No-RAG** 基线：

```bash
python RAG/experiments/run_qa_rag_eval.py --retriever no_rag --k 0 --base-url ... --api-key ... --model ...
```

### 4) 一键跑表（sweep 输出 CSV）

```bash
python RAG/experiments/run_qa_sweep.py ^
  --index-dirs RAG/data/rag_index_e5 RAG/data/rag_index_minilm ^
  --embedding-models intfloat/multilingual-e5-small sentence-transformers/all-MiniLM-L6-v2 ^
  --retrievers no_rag dense bm25 hybrid two_stage ^
  --ks 1 3 5 10 ^
  --base-url ... --api-key ... --model ...
```

输出：`RAG/experiments/out/qa_sweep_results.csv`

### 5) 真·一键跑表（参数写在代码里）

如果你不想敲命令行参数，直接改脚本顶部的 `CONFIG`，然后运行：

```bash
python RAG/experiments/run_one_click_sweep.py
```


