# PetMind RAG

本目录只保留生产检索核心、当前索引配置、受维护的离线工具和分层测试。历史实验、训练数据生成脚本与旧索引不属于生产依赖。

## 目录

```text
RAG/
├── simple_rag/             # 清洗、分块、embedding、检索、重排和上下文扩展
├── maintenance/
│   ├── indexing/           # 建库、增量导入、分类切分和验证
│   └── benchmarks/         # 性能、资源和容量基准
├── tests/
│   ├── unit/               # 不加载真实模型和生产索引
│   ├── integration/        # 临时小索引或真实组件链路
│   ├── regression/         # 固定 D1–D8 检索质量基线
│   └── fixtures/
└── data/
    ├── releases/          # 不可变清洗版本、来源与隔离记录，不纳入 Git
    ├── rag_index_e5/       # 旧全量索引，保留以便复现/回退
    ├── rag_index_e5_by_cat/# 旧分类索引，保留以便回退
    ├── category_taxonomy.json
    └── veterinary_materials_classification_2.0.xlsx
```

生产代码只能依赖 `RAG.simple_rag`。`RAG.maintenance` 是离线运维入口，禁止从 Agent 请求路径导入。

## 依赖

```powershell
python -m pip install -r RAG/requirements.txt
```

默认 embedding 为 `intfloat/multilingual-e5-small`，Reranker 为 `BAAI/bge-reranker-large`。E5 的 query/passage 前缀由封装自动添加。

## 索引维护

生产采用 `clean_rebuild` 的不可变 release 流程。命令从 `agentAndRag` 目录执行，`<NEW_RELEASE>` 必须是尚未发布的新路径，例如 `RAG/data/releases/clean-YYYYMMDD`。

```powershell
python -m RAG.maintenance.indexing.clean_rebuild prepare --source-root <OCR_OUTPUT> --output-root <NEW_RELEASE>
python -m RAG.maintenance.indexing.clean_rebuild embed --output-root <NEW_RELEASE> --device cuda --batch-size 64
```

`prepare` 只读原始书籍，按现有分类表选书；缺少原文的书籍可由旧索引顺序去重还原为普通 MMD，然后走相同清洗/分块流程。保留来源哈希、章节、页序、字符区间和可逆隔离记录。`page_sequence` 是源分页序号；`reconstructed` 明确表示旧索引重建，不能当作印刷页码。`books/086.mmd` 等是逻辑来源标识，对应 release 的 `sources/086.mmd`，不是公开下载地址。

分块按实际 tokenizer 计数，目标320、最大384、相邻重叠48 tokens；表格按完整行并携带表头。任何解析规则或来源变化都会使分块缓存失效。向量复用须匹配文本及 embedding 模型指纹。

随后使用 `audit_clean_release` 验证源文和分块可复现性，使用 `compare_clean_release` 比较旧版与候选的固定问题质量及三轮并发1/4/8性能，再由 `accept_clean_release` 生成绑定当前制品哈希的 `acceptance.json`。具体命令见维护文档。只有三个门禁都通过才能发布：

```powershell
python -m RAG.maintenance.indexing.clean_rebuild publish --output-root <NEW_RELEASE>
```

发布只原子切换 `category_taxonomy.json`，备份旧配置至 release 的 `previous_taxonomy.json`。taxonomy 按 mtime 失效；部署时仍应滚动重启并检查 `/ready`，保证每个进程都装载新索引且释放旧缓存。已发布 release 禁止原地 prepare/embed；下一次清洗使用新的目录。

回退时先确认 `previous_taxonomy.json` 引用的旧索引仍完整，再通过同目录临时文件加 `os.replace` 原子恢复活动 taxonomy，滚动重启服务并复核 readiness/固定查询。不要删除仍被当前进程或回退版本引用的索引。

本次活动版本为 `clean-20260905`。Git 仅提交代码、配置和不含原文的指标；部署到其他机器必须先传输经过校验的 release（含向量/元数据），或用相同源文件重建并重新验收，不能仅拉取代码就期望索引存在。原 `rebuild_category_indexes` 等入口仅用于旧格式维护，不应覆盖受管理的 release。

其他维护入口和适用范围见 [maintenance/README.md](maintenance/README.md)。

## 手工查询

```powershell
python -m RAG.maintenance.query "feline urinary obstruction emergency" `
  --category clinical.emergency_critical --top-k 5 --rerank --as-json
```

Agent 工具要求传入英文检索词；中文问题应先由统一意图/查询规划阶段转换成英文检索表达。

## 测试

```powershell
python -m pytest -c RAG/pytest.ini RAG/tests/unit -q
python -m pytest -c RAG/pytest.ini RAG/tests/integration -q
$env:RUN_RAG_REGRESSION='1'
python -m pytest -c RAG/pytest.ini RAG/tests/regression -q
```

`unit` 默认适合 CI；`regression`、`slow`、`gpu` 需要本地模型或生产索引。

## 容量基准

```powershell
python -m RAG.maintenance.benchmarks.run_capacity_benchmark
```

默认报告写入项目 `.test-tmp/reports/rag-runtime-capacity-20260824`，不会纳入 Git。基准不调用外部 LLM 或 Web Search。
