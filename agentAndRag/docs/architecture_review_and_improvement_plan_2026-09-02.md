# Agent / RAG / 生产前端 架构审查与改进计划

> 审查日期：2026-09-02（Asia/Shanghai）
>
> 审查范围：`agentAndRag/agent_api/app`（Gateway + Worker + MoE 编排）、`agentAndRag/RAG`（`simple_rag` 生产核心与 `maintenance`）、`petmindAgentFrontend`（React + Vite 生产前端）。
>
> 结论优先级：**实际代码与测试 > specs > agent.md > 本文**。本文只记录审查结论与本轮落地范围，不替代 `agent.md`。

## 0. 总览

系统整体设计是健康的：Gateway/Worker 分离、Run 先持久化再入队、积分 reserve/settle 使用行锁、LLM 共享连接池加资源槽位、SQLite 访问全部 `to_thread`、前端 JWT 仅存内存且 refresh 单飞、Markdown 经 `rehype-sanitize`。本次审查没有发现"两套并行业务逻辑"，`app/llm/*`、`platform/run_service.py`、`services/moe/orchestrator.py`、`experts.py` 等旧路径均为 `sys.modules` 别名，只被测试引用。

真正的风险集中在三处：

1. **Run 生命周期的原子性与时序**：认领非 CAS、取消与认领竞态、事件序号 max+1、终态状态先于终态事件提交导致 SSE 提前断流、Worker 优雅停机会把用户任务误标 `cancelled`。
2. **前端 SSE 状态机缺少守卫**：`streamRun/resumeRun` 不做 401 刷新、abort 后缓冲事件仍派发、新建会诊 busy 门闩过晚、无 Error Boundary。
3. **事件循环上的同步 IO 与 RAG 锁粒度**：`_resolve_species` 同步 MySQL 直接在事件循环执行且每请求两次；RAG 模型/索引加载持全局 `RLock`；索引异常被吞成"无命中"。

## 1. Agent 执行面（`agent_api/app`）

### 1.1 并发 / 竞态

| 严重度 | 位置 | 问题 | 方案 | 本轮 |
| --- | --- | --- | --- | --- |
| 高 | `platform/runs/service.py` `execute_run` | `get → 判断 status → 写 running → commit`，无 CAS/行锁；队列重复投递或双 Worker 时两边都能通过检查 | 改为 `UPDATE ... WHERE id=? AND status IN ('queued','retry') AND cancel_requested = false`，`rowcount == 0` 直接返回 | ✅ |
| 高 | `routes_platform_core.py` `cancel_run` × `execute_run` | API 把 `queued` 改成 `cancelled` 并退款的同时，Worker 可能已读到旧 `queued` 并覆盖为 `running`，继续跑完整 MoE | 取消走条件更新；认领 SQL 排除 `cancel_requested`；认领成功后仍在流内周期检查取消 | ✅ |
| 高 | `append_run_event` | `SELECT max(sequence)+1` 非原子，并发写撞 `uq_platform_run_event_sequence` 直接抛 `IntegrityError` 打断执行 | 捕获唯一约束冲突并重试（有限次） | ✅ |
| 高 | `execute_run` × `run_event_stream` | 先 commit `status=completed/failed/cancelled`，再 append 终态事件；SSE 轮询看到"终态 + 无新行"即断开，客户端丢失 `completed` 事件，前端 busy 卡死 | SSE 只在看到终态事件类型后退出；若 Run 已终态但迟迟没有终态事件，给一个宽限窗口再退出 | ✅ |
| 高 | `execute_run` `except asyncio.CancelledError` | 用户取消与 Worker 停机（`_QUEUE_TASK.cancel()`）走同一分支，停机会把进行中的 Run 标为 `cancelled` 并退款，用户看到"任务已取消" | 用户取消抛自定义 `RunCancelled`；真实 `CancelledError` 把 Run 置回 `retry` 并重抛，重启后由回收逻辑重新入队 | ✅ |
| 中 | `worker_forever` 回收 | 启动时无条件 `running → retry`，多 Worker 时会重跑别人的任务 | 单 GPU 部署当前只允许一个 Worker；配合 CAS 认领后双执行已被阻断。长期需要 `claimed_by/lease_until` 列（需迁移） | 文档 |
| 中 | `worker_forever` 主循环 | 任何 Redis 抖动异常直接让消费者协程退出，`/ready` 变 503 但进程不退出，队列静默停摆 | 主循环捕获异常、退避重连；异常只影响单次迭代 | ✅ |
| 中 | `_run_cancel_requested` | 每个 delta 事件都查一次 DB，与 `append_run_event` 一起每个 token 两次事务 | 时间节流（默认 0.5s 查一次） | ✅ |
| 中 | `enqueue_run` | 每次入队新建 Redis 连接再关闭 | 进程级共享 `redis.asyncio.Redis`，shutdown 统一关闭 | ✅ |
| 中 | `enqueue_run` 无 Redis 回退 | `asyncio.create_task` 不持有引用，异常变成 "Task exception was never retrieved" | 持有引用集合 + done callback 记录异常 | ✅ |
| 中 | `cancel_requested` 状态 | 非终态且不在回收集合中；Worker 崩溃时处于该状态的 Run 永远不会结束 | 回收逻辑把 `cancel_requested` 直接终结为 `cancelled` 并退款 | ✅ |
| 低 | `raise asyncio.CancelledError` 手动模拟取消 | 与真实取消混淆 | 见上，改用 `RunCancelled` | ✅ |

### 1.2 事件循环阻塞 / 性能

| 严重度 | 位置 | 问题 | 方案 | 本轮 |
| --- | --- | --- | --- | --- |
| 高 | `services/moe/orchestration/service.py` `_resolve_species` | 同步 MySQL `fetch_animal_profile` 直接在事件循环执行，且 `_decide_task_policy` 与 `_run_experts` 各调一次 | 改为异步 + `asyncio.to_thread`，并按请求缓存一次结果 | ✅ |
| 中 | `append_run_event` | 每个事件单独事务，长答复写放大 | 可选批量刷盘或 Redis Stream；当前先保证正确性 | 后续 |
| 中 | `run_event_stream` | 每 0.5s 开一次 session 轮询 | `LISTEN/NOTIFY` 或 Redis pub/sub 推送 | 后续 |
| 中 | `routes_openai.py` | 三处 `except Exception: pass` 吞掉 QA 审计落库失败 | 至少 `logger.exception` | ✅ |

### 1.3 架构与冗余

- `app/llm/*`、`platform/run_service.py`、`services/moe/orchestrator.py`、`services/moe/experts.py`、`tools/tools_builtin.py`、`context/*`、`persistence/*`、`sql_search/*` 均为别名层，只被测试引用。建议：新代码只允许导入权威路径（可在 `tests/architecture` 里加静态检查），别名层在测试迁移完成后删除。
- `tools/builtin/rag.py` 与 `tools/rag_tools.py` 是"契约/白名单"与"实现"分层，不是重复。
- Worker 同时挂载 `openai_router` / `chat_moe_router` 与 Redis 消费者，边界略糊，但由内部 Token 保护，暂不调整。

## 2. RAG（`RAG/simple_rag` + `agent_api/app/tools/rag_tools.py`）

| 严重度 | 位置 | 问题 | 方案 | 本轮 |
| --- | --- | --- | --- | --- |
| 中 | `rag_tools.py` `_get_store/_get_embedder/_get_bm25/_get_reranker` | 缓存未命中时在全局 `RLock` 内加载整库 `npy`/模型权重，期间所有 RAG 请求（包括查询向量 LRU 读取）全部阻塞 | 双检锁：全局锁只保护缓存字典，加载在 per-key 锁下进行 | ✅ |
| 中 | `rag_tools.py` `_retrieve_from_index` | 任何加载异常（维度不匹配、meta 损坏）被吞成 `[]`，上层误判为"本地证据不足"触发 Web 兜底 | 记录 warning 日志（含索引目录与异常类型） | ✅ |
| 中 | `retrieval_policy.py` `rag_requires_web_fallback` | `RAG_RELEVANCE_THRESHOLD=0.90` 作用在最终 `score` 上；默认 rerank=True 时该值为 CrossEncoder sigmoid 分（0～1，语义正确），但 `rerank=False` 时为 E5 余弦（弱相关也常 >0.8），`multi_route` RRF 分则远小于 0.9 | 优先读取 `score_rerank`；无 rerank 时保持现行为，并在文档标注阈值语义 | ✅ |
| 中 | 建库双轨 | `rag.reindex` → `pipeline.build_or_update_index` 产出的 `chunk_id`/字段与生产 `rebuild_category_indexes`（semantic-v2）不兼容 | `rag.reindex` 保持管理工具但文档明确不用于生产分类索引；生产只走 `RAG.maintenance.indexing.rebuild_category_indexes` | 文档 |
| 中 | `vector_store.py` `add()` | 直接覆盖写 `embeddings.npy`，在线服务读取时 CLI 重建可能读到半写文件 | 重建到新目录后切换 taxonomy 活动路径；或临时文件 + rename | 后续 |
| 中 | Clinical 专家类目扇出 | 一次查询串扫十余个分类索引，延迟随类目数线性增长 | 粗路由 Top-M 类目或合并索引 + category 过滤 | 后续 |
| 低 | `text_utils.recursive_sentence_chunks`、`category_index.expert_category_warmup_ids`、`pipeline.py` 未用 `asdict` | 死代码 | 删除 | ✅ |
| 低 | `rag_tools.py` `per_store_k` | 恒等于 `retrieve_k` 的冗余分支 | 删除 | ✅ |
| 低 | 默认 `multi_route=False` 但 `AGENT_WARMUP_BM25=1` | BM25 预热与默认检索路径脱节 | 生产 `.env` 可设 `AGENT_WARMUP_BM25=0` 省内存；保留开关 | 文档 |

## 3. 生产前端（`petmindAgentFrontend/src`）

| 严重度 | 位置 | 问题 | 方案 | 本轮 |
| --- | --- | --- | --- | --- |
| 高 | `api.ts` `streamRun/resumeRun` | 直接 `fetch`，401 不走 refresh；Access Token 15 分钟过期后首条消息必失败 | 抽 `authorizedFetch`：401 → 共享 `refresh()` → 重试一次 | ✅ |
| 高 | `api.ts` `consumeSse` | abort 后已解码的缓冲块仍同步派发 `onEvent`，切会话后 `completed/delta` 写入新会话、误清 sessionStorage | 每块派发前检查 `signal.aborted`；`ChatPage` 事件处理按"流代际"守卫 | ✅ |
| 高 | `App.tsx` `submitMessage` | `busy` 在 `createConversation` 之后才置 true，连点会并发创建多个会话/Run | 入口用 `submittingRef` 立即上锁 | ✅ |
| 高 | `api.ts` `parseSseBlock` + 无 Error Boundary | 坏 JSON 事件抛穿整棵树白屏 | `JSON.parse` 容错 + 顶层 `ErrorBoundary` | ✅ |
| 中 | `App.tsx` `submitMessage` | 替换 `controller.current` 前不 abort 旧控制器，旧恢复流泄漏 | 赋值前 abort | ✅ |
| 中 | `App.tsx` `stop()` | 取消后残留 `streaming` 占位，不刷新最终消息 | 移除占位并 `refreshMessages` | ✅ |
| 中 | `App.tsx` `Sidebar` 搜索 | 只清 debounce timer，不丢弃过期响应 | 请求序号守卫 | ✅ |
| 中 | `api.ts` `client.rename/archive` | 无调用方 | 删除 | ✅ |
| 中 | `App.tsx` 613 行上帝文件 | 页面、SSE 状态机、后台全部同文件 | 后续拆 `hooks/useRunStream`、`pages/*` | 后续 |
| 中 | `types.ts` `RunEvent.data: Record<string, unknown>` | SSE 契约松散 | 按 event 联合类型 + 运行时校验 | 后续 |
| 中 | `security-headers.conf` | HTTP 容器无条件发 HSTS；`style-src 'unsafe-inline'` | HSTS 由 TLS 边缘层发；样式改 nonce | 后续 |
| 中 | `Dockerfile` pnpm 9 vs `package.json` pnpm 10 | 构建器与 lockfile 不一致 | 对齐主版本 | ✅ |
| 中 | 流式渲染 | 每次 flush 全量 `setMessages` + Markdown 重解析 | streaming 内容独立 state、完成消息 memo | 后续 |

## 4. 本轮实施清单

### Agent

1. `platform/runs/service.py`
   - `RunCancelled` 异常；`claim_run()` CAS 认领；`_run_cancel_requested` 节流；
   - `append_run_event` 唯一冲突重试；
   - 终态处理：用户取消 → `cancelled`；Worker 停机 → `retry` 并重抛；
   - `run_event_stream` 以终态事件为退出条件，附带宽限窗口；
   - 共享 Redis 客户端 `get_run_queue_redis()/close_run_queue_redis()`；本地回退任务持引用；
   - `worker_forever` 主循环容错、回收 `cancel_requested`。
2. `routes_platform_core.py` `cancel_run` 条件更新。
3. `worker_main.py` / `main.py` shutdown 关闭共享 Redis 客户端。
4. `orchestration/service.py` `_resolve_species` 异步化 + 请求级缓存。
5. `routes_openai.py` 吞异常处补日志。

### RAG

6. `rag_tools.py` 双检加载锁、异常日志、删除冗余分支。
7. `retrieval_policy.py` 阈值优先读 `score_rerank`。
8. 删除 `recursive_sentence_chunks`、`expert_category_warmup_ids`、未用 import。

### 前端

9. `api.ts`：`authorizedFetch`、SSE abort 守卫、JSON 容错、错误体解析、删除死 API。
10. `App.tsx`：`submittingRef`、流代际守卫、abort 旧控制器、`stop()` 清理、搜索请求序号。
11. `ErrorBoundary.tsx` + `main.tsx` 接入。
12. `Dockerfile` pnpm 版本对齐。

### 验证

- `pytest agent_api/tests/platform agent_api/tests/moe agent_api/tests/rag agent_api/tests/concurrency`
- `pytest RAG/tests/unit RAG/tests/integration`
- 前端 `pnpm lint`、`pnpm test`、`pnpm build`

实际结果（2026-09-02，conda `RAG` 环境 / Python 3.12 / SQLAlchemy 2.0.45）：

| 命令 | 结果 |
| --- | --- |
| `pytest agent_api/tests/platform`（含新增 `test_run_lifecycle.py` 7 例） | 32 passed |
| `pytest agent_api/tests --ignore=agent_api/tests/platform` | 286 passed |
| `pytest RAG/tests/unit RAG/tests/integration` | 34 passed |
| 前端 `tsc -b`、`eslint src --max-warnings=0`、`vite build` | 全部通过 |

注意：系统默认 `python`（anaconda base）的 SQLAlchemy 过旧，无法导入 `async_sessionmaker`；后端测试需在 `RAG` 环境运行，并沿用仓库内 `--basetemp`。

## 5. 原始技术债清单（2026-09-02 快照；最新结论见第 6 节）

1. Run 队列租约：为 `platform_agent_runs` 增加 `claimed_by/lease_until`，Worker 心跳续租，回收只处理租约过期任务；多 Worker 前必须完成。
2. Run 事件写放大：delta 批量刷盘或走 Redis Stream，终态再落 PostgreSQL；SSE 改为推送而非轮询。
3. RAG 索引原子发布：重建到新目录后切换 taxonomy 活动路径；服务侧按 mtime 失效缓存。
4. RAG 类目扇出：粗路由或合并索引。
5. 前端拆分 `useRunStream` 状态机并补竞态单测（双击发送、切会话中途 completed、SSE 401）。
6. 前端 SSE/Admin 契约类型化与运行时校验；路由级 code splitting；长列表虚拟化。
7. 别名导入层在测试迁移后删除，并在 `tests/architecture` 增加禁止旧路径的检查。
8. 平台 Run 未绑定 `animal_id`：`RunCreateRequest` 无该字段，`execute_run` 全程不调用 `bind_tool_request_scope`，`expert_runtime/service.py:180` 会把 `sql.search`/`vitals.summary` 等个体工具全部过滤掉。OpenAI 兼容路径（`routes_openai.py`）有绑定，平台路径没有。属于产品/契约决策：若平台会诊需要个体数据，需在会话或 Run 上持久化 `animal_id` 并在 Worker 侧包裹作用域。
9. 前端 `WorkspaceFrame` 同时挂载桌面与移动两份 `Sidebar`，首屏双倍列表请求、双份监听状态；应改为单实例 + CSS 响应式。
10. 前端邀请/重置令牌走 URL query（`?token=`），会进入 Referrer 与浏览器历史；建议后端短码兑换或 fragment 传递，接受后立即 `replace` 清除。接受邀请后 `window.location.assign('/chat')` 全页刷新丢掉内存 Access Token，完全依赖 refresh Cookie 已种下。
11. `platform/services.py` 的 `adjust_credits` 幂等检查与写入之间存在窗口，并发相同 key 可能撞唯一约束；应捕获 `IntegrityError` 后回读已有 ledger。
12. `observability/jsonl_trace.py` 同步写盘且记录原始问句；需脱敏/采样并改 `asyncio.to_thread`。
13. RAG 查询长度无上限，E5/CrossEncoder 静默截断；Agent 侧命中无文本 hash 去重，近重复 chunk 可能重复进入上下文。


## 6. 2026-09-05 落实复核

前文保留原审查时的状态，不代表当前实现。最终验证、索引质量、性能及真实界面结果见 [本轮验收报告](technical_debt_acceptance_2026-09-05.md)。

| 原技术债 | 当前实现与复核依据 |
|---|---|
| 1 租约 | migration `20260905_0005`；数据库时间租约、心跳、execution_epoch 栅栏，所有事件/终态写入验证所有权；真实 PostgreSQL 竞态与接管测试 |
| 2 事件与 SSE | delta 首包立即输出，其余 100ms/32 个合并；序号在 Run 行锁内分配；事务提交后 NOTIFY，单进程共享监听与 5s 兜底查询；答案/结算/终态事件原子提交 |
| 3 原子发布 | `clean_rebuild prepare/embed/publish`；不可变 release，原文/分块/向量/报告指纹绑定，质量/性能/复现门禁后原子切换 taxonomy；活动 release 拒绝覆盖 |
| 4 类目扇出 | 共享合并向量矩阵 + 类目行集合过滤；mmap 元数据、解析 LRU、连续向量子集缓存；避免每类重复载入全库 |
| 5–6 前端状态及契约 | `useRunStream`、代际与 abort 守卫、取消等待终态；SSE 联合类型及运行时校验；Admin/Settings 懒加载、48 条游标分页、超过100条虚拟化、完成答复 memo |
| 7 别名层 | 测试迁移至权威 feature 路径，删除24个旧别名，架构测试禁止回流 |
| 8 animal_id | 按产品定位保持通用会诊平台；本轮不新增强制动物 ID 绑定，不把它当成待修功能缺陷 |
| 9 双 Sidebar | 单实例响应式侧栏；移动端路由变化自动收起，非会诊页有可达的移动导航 |
| 10 邀请令牌 | query/fragment 捕获后立即清理地址，StrictMode 用例；注册直接接纳会话，不依赖全页刷新 |
| 11 积分幂等 | 行锁 + savepoint/唯一约束冲突回读；同一幂等键多请求不重复入账 |
| 12 trace | 有界异步队列、批处理与采样；只写运行元数据，不写问句、答复或密钥，停机有界排空 |
| 13 查询边界 | E5 查询字符/token 上限；CrossEncoder 按实际成对预算分窗；文本哈希去重保留来源；dense/RRF 使用独立阈值，未校准时请求补证 |
| 补充：记忆旁路 | 持久化 outbox、独立维护任务、超时/重试及执行前偏好复核；回答终态不等待记忆生成 |
| 实测发现 | 页内可取消确认框、完整密钥关闭后清除、审计资源ID先flush、profile即时同步、搜索过滤、首次标题持久化、重答标题更新、移动导航与取消文案 |

剩余方向不是本轮已完成保证：扩大兽医人工标注跨类别评测集；章节内目录与 OCR 表格逐页抽检；引入资料时效与来源可信等级；对重排/纯向量/RRF 单独校准阈值；GPU 公平队列与动态批处理；将冷启动和并发尾延迟纳入持续基线；完善订单自助查看和错误态反馈；逐步迁移 FastAPI lifespan。
