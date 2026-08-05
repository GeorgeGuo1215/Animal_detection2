## 简易 RAG（基于 `.mmd` 书籍）

本目录会把 `RAG/data/raw/**/*.mmd` 中的内容清洗、分块、生成向量，并建立本地索引（纯 `numpy` 持久化，避免 Windows 下编译依赖）。

---

## 目录结构（你现在项目里实际存在的）

- `RAG/data/raw/`: 原始书籍 `.mmd`（支持递归子目录）
- `RAG/data/rag_index/`: 默认索引产物目录（也可以建多套索引目录对比 embedding）
  - `embeddings.npy`: 文本块向量矩阵（float32，已归一化）
  - `meta.jsonl`: 每行一个 chunk 元数据（含 source_path/chunk_index/text 等），与 embeddings 行号一一对应
  - `store_config.json`: 向量维度等配置
- `RAG/simple_rag/`: RAG 核心库（清洗/分块/embedding/向量库/检索扩展点）
- `RAG/experiments/`: 评测与实验脚本（Accuracy@K、trace、断点续传、错题归因、截断检查等）
- `RAG/ingest.py`: 建库/增量入库入口
- `RAG/query.py`: 查询入口（可选启用多路召回 + query rewrite）

### 1) 安装依赖

在仓库根目录执行：

```bash
python -m pip install -r RAG/requirements.txt
```

说明：
- `sentence-transformers` 会依赖 `torch`；如果你的环境没自动装好，可按你机器情况安装 CPU 或 CUDA 版本的 PyTorch。

### 2) 入库（建立索引）

```bash
python RAG/ingest.py
```

默认：
- 原始数据目录：`RAG/data/raw`
- 索引输出：`RAG/data/rag_index`
- embedding 模型：`intfloat/multilingual-e5-small`（中英都能用）

注意（重要）：E5 系列模型建议使用 `query:` / `passage:` 前缀。
本项目已在 `simple_rag/embeddings.py` 中对包含 `e5` 的模型名自动加前缀。
如果你之前建库时没有前缀，**建议重新运行 `ingest.py` 重建索引**，检索质量通常会更稳。

常用参数示例：

```bash
python RAG/ingest.py --batch-size 32
python RAG/ingest.py --limit-books 3
python RAG/ingest.py --embedding-model sentence-transformers/all-MiniLM-L6-v2
```

#### 分块策略说明（已升级为“句子边界递归分块”）

`simple_rag/text_utils.py` 的 `chunk_text()` 现在会：
- 先按空行做“段落/页”分段
- 段落内优先按句号/问号/感叹号切句（中英文标点都支持）
- 句子过长则降级为逗号/分号切分，再不行按词硬切
- 最后按 `chunk_words/chunk_overlap_words` 合并为 chunk（并做 overlap）

这样相比纯按词/按行切：
- chunk 更接近自然语义边界，减少“切断关键句”
- 对问答/检索通常更稳

### 3) 查询（检索 topK 片段）

```bash
python RAG/query.py "麻醉镇痛的基本原则是什么？" --top-k 5
```

也可用 JSON 输出（便于你后续接 DeepSeek/LLM 生成回答）：

```bash
python RAG/query.py "radiography positioning" --top-k 5 --as-json
```

#### 小块命中后拼接邻居（推荐用于喂给 LLM）

对于英文书籍类语料，建议用“小块检索 + 邻居拼接”的方式获得更完整的证据链：

```bash
python RAG/query.py "Which medium is used for growing salmonellae?" --top-k 5 --multi-route --rewrite template --expand-neighbors 1
```

说明：
- `--expand-neighbors 1` 会把命中 chunk 的 `i-1,i,i+1` 拼成一个更完整的 context（同一本书中重叠窗口会自动合并去重）
- `--as-json` 时会输出 `{hits, contexts, expand_neighbors}`，其中 `contexts` 更适合直接作为 LLM 输入

#### 加入 reranker（bge-reranker-large）

推荐的顺序是：**先召回一批小块 candidates → reranker 重排 → 只取 topK 种子块 → 再做邻居拼接**。

命令行用法示例（先召回 10 个候选，用 reranker 取最终 top5）：

```bash
python RAG/query.py "salmonellae culture medium" --multi-route --rewrite template --rerank --rerank-model BAAI/bge-reranker-large --rerank-candidates 10 --top-k 5 --expand-neighbors 1
```

#### 多路召回 + Query Rewrite（复杂 RAG 特质的扩展点）

你现在可以在 `query.py` 开启多路召回（dense + bm25）并做 query 重写：

```bash
python RAG/query.py "Which medium is used for growing salmonellae?" --top-k 5 --multi-route --rewrite template
```

实现位置：
- **Query 重写**：`simple_rag/query_rewrite.py`
  - `NoRewrite`: 不重写
  - `TemplateRewriter`: 规则/模板扩展（definition/what is/indications 等），便于你后续加“模板转换、术语扩展、题干清洗”等
- **多路召回融合**：`simple_rag/retrieval.py`
  - `MultiRouteRetriever`: 支持多个 retriever + 多 query 融合
  - `build_default_multiroute`: 默认 dense + bm25 的便捷构造

你可以通过“新增这些类的实例”来扩展复杂系统特质：
- 替换/叠加 query rewriter（比如你后面加 LLM rewrite、同义词扩展、领域词典扩展）
- 替换/叠加检索路由（dense/bm25/章节检索/书级检索/重排序器等）

### 4) 产物说明

索引目录 `RAG/data/rag_index/` 会生成：
- `embeddings.npy`: 所有文本块的向量（已归一化）
- `meta.jsonl`: 每个向量对应的文本块与来源信息
- `store_config.json`: 向量库配置（dim 等）

---

## 评测（Accuracy@K + Trace + 断点续传）

你当前的评测主入口是：
- `RAG/experiments/run_one_click_sweep.py`：一键跑组合（不同 embedding/index、不同策略、不同 k），并输出：
  - `qa_sweep_results.csv`：组合级 Accuracy@K 汇总
  - `qa_trace.jsonl`：逐题 trace（包含 contexts、reasoning_content、pred、是否正确），支持断点续传

错题归因与截断影响检查：
- `RAG/experiments/analyze_wrong_traces.py`：抽样错题（no_rag 错）并在 k=5 的各组合下做归因分析
- `RAG/experiments/inspect_context_effect.py`：把 query 写死在脚本顶部，自由调截断参数，直观看“截断/切片”对 context 与作答的影响


