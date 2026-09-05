# RAG 维护工具

本目录仅用于离线索引维护与容量评估。生产请求路径不得导入这里的模块。

## indexing

- `clean_rebuild`：生产推荐的可逆清洗、token 分块、向量复用及带门禁的原子发布。
- `rebuild_category_indexes`：旧格式维护；从 OCR 输出和分类表重建全量、分类索引及 taxonomy。
- `validate_category_indexes`：逐书验证分类、索引文件和检索命中。
- `append_books_to_category_indexes`：兼容已进入全量索引的单书增量追加。
- `split_index_by_category`：把既有全量索引按分类表切分。
- `copy_ocr_mmd_to_raw`：从 OCR 输出复制规范书籍 MMD。
- `import_mmd_to_raw`：通用 MMD 导入及审计日志。
- `chunk_stats`：统计索引分块分布。

所有写索引操作应先输出到临时目录，验证通过后再原子切换运行目录。不要直接覆盖正在被 Worker 使用的索引。

## benchmarks

`run_capacity_benchmark` 在独立子进程中比较 CPU/GPU 组合、并发和多实例资源成本。查询集固定在 `RAG/tests/fixtures/retrieval_regression_cases.json`。

```powershell
python -m RAG.maintenance.benchmarks.run_capacity_benchmark `
  --output-dir ..\.test-tmp\reports\rag-runtime-capacity-20260824
```

原始报告含环境和索引指纹，但不含 API Key 等敏感变量。

## 清洗版本验收

以下命令从 `agentAndRag` 运行，报告放在 Git 忽略的临时目录。旧版比较需要修改前保存的实际源码快照（`rag_tools.py`、`vector_store.py`、`reranker.py`），不能让两组调用同一套候选代码冒充基线。

```powershell
python -m RAG.maintenance.benchmarks.compare_clean_release --taxonomy <OLD_TAXONOMY> --legacy-sources <BASELINE_SOURCE_DIR> --output <BASELINE_REPORT>
python -m RAG.maintenance.benchmarks.compare_clean_release --taxonomy <NEW_RELEASE>/taxonomy.json --output <CANDIDATE_REPORT>
python -m RAG.maintenance.benchmarks.audit_clean_release --release <NEW_RELEASE> --legacy-metadata RAG/data/rag_index_e5/meta.jsonl --output <AUDIT_REPORT>
python -m RAG.maintenance.benchmarks.accept_clean_release --release <NEW_RELEASE> --baseline <BASELINE_REPORT> --candidate <CANDIDATE_REPORT> --audit <AUDIT_REPORT> --cases RAG/tests/fixtures/retrieval_regression_cases.json
```

默认 rerank、10 候选、三轮并发1/4/8，计时包含生产工具入口排队。可用 `--mode dense|hybrid` 和 `--candidates 20|40` 做消融，不能把不同配置的分数共用阈值。门禁要求证据命中不降、噪声下降，三轮中位 p95/QPS/RSS 回归不超过5%，源文/窗口复现通过且当前文件指纹匹配。报告保留首轮命中文本用于内部复核，禁止将该原始报告或完整书籍提交到 Git；对外仅输出汇总。

固定24条词法证据测试不足以代表临床正确性。本次结果与限制见 `docs/technical_debt_acceptance_2026-09-05.md`。扩大人工标注集后应重新校准门禁，冷启动另行记录。
