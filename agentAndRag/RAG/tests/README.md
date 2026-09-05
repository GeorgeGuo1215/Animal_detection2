# RAG 测试分层

- `unit`：纯函数、小矩阵和 mock，禁止加载真实模型或生产索引。
- `integration`：临时小索引、维护入口及 Agent `rag.search` 组件链路。
- `regression`：24 条 D1–D8 固定查询，验证分类隔离和目标书籍 Top-5 命中。
- `fixtures`：可审计、稳定且体积小的测试输入。

标记为 `slow`、`gpu` 或 `regression` 的用例不进入普通快速测试。真实回归需要显式设置 `RUN_RAG_REGRESSION=1`。
