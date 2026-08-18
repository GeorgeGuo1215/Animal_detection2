# PetMind 兽医 Agent 生产平台维护文档

> 最后全量核对：2026-08-16（Asia/Shanghai）
>
> 仓库：`C:\Users\ROG\Animal_detection2`
>
> Agent 工作目录：`C:\Users\ROG\Animal_detection2\agentAndRag`
>
> Web 前端：`C:\Users\ROG\Animal_detection2\petmindAgentFrontend`
>
> 当前交付分支：`test-agent`

本文是当前代码的零上下文维护入口，覆盖生产 Web 平台、OpenAI 兼容 Agent、MoE、RAG、MCP、长期记忆、认证、额度、任务队列、前端和部署。规格契约位于 `../specs/petmind-agent-platform/`。发生冲突时优先级为：**实际代码与测试 > specs > 本文 > 旧 README**。

## 1. 当前交付状态

本轮已经落地：

1. React + Vite + TypeScript 的 PetMind 生产前端，使用暖米色、陶土色、纸张和铅绘视觉。
2. 邀请注册、登录、刷新、退出、忘记密码和重置密码。
3. Access JWT、Refresh Token 轮换、Argon2id 密码、RBAC 和数据库 API Key。
4. PostgreSQL 持久化用户、会话、消息、Agent Run、事件、套餐、订单、订阅、积分和审计。
5. Redis 原子限流、持久任务队列、独立 Agent Worker、失败队列和重启恢复。
6. SSE、最长 120 秒同步、异步三种调用方式，共用同一个持久化 Run。
7. SSE `Last-Event-ID` 续传；网页刷新后可恢复正在执行的会诊并清理瞬态错误。
8. 对话历史、标题搜索、删除、Markdown 导出、模型/回答身份选择和侧边栏收起。
9. 套餐、订单、订阅、积分账本、API Key 撤销和管理后台。
10. `/v1/chat/completions` 与 `/v1/models` 保持兼容，并接受数据库 `pm_live_*` API Key。
11. Memory Service 与生产 Run 对齐：平台用户 ID 是记忆主体，Run ID 是幂等 `turn_id`。
12. MoE 的 Router、专家、Critic、Aggregator、RAG、MCP 和医生 D1～D8 意图链路继续复用。
13. PetHealth 外部心率异常信号会触发真实 vitals 核实，外部 flag 本身不作为诊断事实。
14. 生产提示词集中在 `agent_api/app/prompts/`，业务层不再散落长段 system prompt。
15. 统一启动器可监管 Memory、Agent API 和 Redis-backed Platform Worker。

首期明确不做：

- 不开放匿名注册，只允许管理员邀请。
- 不实现组织/诊所多租户。
- 不建立宠物档案，也不绑定 PetHealth 账号或宠物数据。
- 不接真实支付渠道；只保留订单状态机、后台确认和 HMAC 测试 Webhook。
- `/chat-moe` 继续是开发测试入口，不作为生产会话存储。
- 不把系统提示词、模型内部推理或原始工具载荷暴露给网页用户。

## 2. 总体架构

```mermaid
flowchart LR
    WEB["PetMind Web :5173<br/>登录 / 聊天 / 套餐 / 后台"] --> API["FastAPI :8000<br/>/api/v1"]
    SDK["第三方 SDK"] --> OAI["OpenAI 兼容接口<br/>/v1/chat/completions"]
    API --> PG["Agent PostgreSQL :15433<br/>platform_* 表"]
    API --> REDIS["Redis :16379<br/>限流 / Run 队列"]
    REDIS --> WORKER["Platform Worker<br/>默认单 GPU 串行消费"]
    WORKER --> MOE["MoE / RAG / MCP"]
    OAI --> MOE
    WORKER --> MEMORY["Memory Service :8300"]
    OAI --> MEMORY
    MEMORY --> MPG["Memory PostgreSQL + pgvector"]
    MOE --> VITALS["PetHealth Vitals PostgreSQL"]
    MOE --> MYSQL["PetMind MySQL 只读数据"]
    MOE --> WEBSEARCH["Tavily WebSearch"]
```

生产网页调用 `/api/v1`；第三方调用方使用 `/v1`。两条入口共享 MoE、RAG、MCP 和 Memory，但认证、会话持久化与计费语义不同。

## 3. 数据边界

| 存储 | 主要内容 | 边界 |
| --- | --- | --- |
| Agent PostgreSQL | `platform_*` 用户、认证、会话、消息、Run、事件、套餐、订单、积分、审计 | 生产 Web 的权威业务库 |
| Redis | 平台限流状态、Run 队列、失败队列 | 不是回答和订单的权威存储 |
| Memory PostgreSQL | 用户短期记忆、片段、页面、画像、知识、任务与幂等收据 | 跨会话长期记忆，不替代完整聊天记录 |
| Chat-MoE SQLite | `/chat-moe` 测试 Session 和专家轨迹 | 只用于开发测试，不用于生产网页 |
| QA SQLite | `/v1` 的 QA、工具和反馈审计 | 运维辅助，不是生产会话库 |
| PetHealth Vitals PostgreSQL | `Pet`、`PetHealthMetric` 等真实生命体征 | 外部 flag 不能代替真实查询结果 |
| PetMind MySQL | `animals`、`daily_reports`、`sensor_events` 等只读业务数据 | 与平台账号、Memory 库相互独立 |
| RAG 索引 | 兽医教材和分类知识证据 | 一般医学证据不能证明当前患者确诊 |

平台迁移只创建 `platform_*` 表，不覆盖 PetHealth、Memory 或 SQLite 数据。对话最后活跃满 365 天后软删除，30 天宽限期后物理清理；订单、积分账本和审计不随聊天清理。

## 4. 目录结构

```text
Animal_detection2/
|-- logo.jpg                              # 品牌源图，不覆盖
|-- specs/petmind-agent-platform/         # 产品/API/数据/安全/UI/部署规格
|-- petmindAgentFrontend/                 # React + Vite + TypeScript
|   |-- src/App.tsx                       # 页面、聊天、后台和交互
|   |-- src/api.ts                        # JWT 刷新、SSE 和 API client
|   |-- src/auth.tsx                      # 内存 Access Token 与登录态
|   |-- src/styles.css                    # 暖色铅绘主题
|   |-- public/brand/                     # Logo 派生资产
|   `-- tests/e2e/                        # Playwright 桌面/移动端测试
`-- agentAndRag/
    |-- agent.md                          # 本文
    |-- .env.example                      # Agent/Memory 示例
    |-- .env.platform.example             # 平台生产配置示例
    |-- docker-compose.platform.yml       # PostgreSQL 16 + Redis 7
    |-- start_agent.bat / start_agent.sh  # 统一启动入口
    |-- agent-rag.service                 # Linux systemd 模板
    |-- agent_api/
    |   |-- alembic/                      # platform_* 迁移
    |   |-- app/main.py                   # FastAPI 生命周期和中间件
    |   |-- app/platform/                 # 模型、配置、安全、服务、队列、清理
    |   |-- app/routers/routes_platform_* # /api/v1 路由
    |   |-- app/routers/routes_openai.py  # /v1 兼容路由
    |   |-- app/prompts/                  # 全量生产提示词
    |   |-- app/services/moe/             # Router/Experts/Broker/Critic/Aggregator
    |   |   `-- retrieval_policy.py       # 专家检索要求、RAG→Web 回退判定
    |   |-- app/memory/                   # Agent 到 Memory 的共享集成层
    |   |-- app/tools/                    # RAG/MCP/SQL ToolRegistry
    |   |-- scripts/run_agent_stack.py    # 三进程 Supervisor
    |   |-- scripts/run_platform_worker.py
    |   |-- scripts/bootstrap_platform_admin.py
    |   `-- tests/platform/               # 平台专项测试
    |-- memory_service/                   # 独立记忆 API + Worker + SQL + 测试
    |-- RAG/                              # ingest、检索和本地索引
    |-- mcp_servers/                      # WebSearch、营养、体征 MCP
    `-- agent_api_logs/                   # 本地日志/SQLite，禁止提交
```

## 5. 接口总览

### 5.1 平台账号

- `POST /api/v1/auth/invitations/accept`
- `POST /api/v1/auth/login`
- `POST /api/v1/auth/refresh`
- `POST /api/v1/auth/logout`
- `POST /api/v1/auth/password/forgot`
- `POST /api/v1/auth/password/reset`
- `GET /api/v1/me`
- `GET|POST /api/v1/me/api-keys`
- `DELETE /api/v1/me/api-keys/{key_id}`

### 5.2 会话与任务

- `POST|GET /api/v1/conversations`
- `GET|PATCH|DELETE /api/v1/conversations/{id}`
- `GET /api/v1/conversations/{id}/messages`
- `GET /api/v1/conversations/search?q=`
- `POST /api/v1/conversations/{id}/runs`
- `GET|DELETE /api/v1/runs/{id}`
- `GET /api/v1/runs/{id}/events`

Run 的 `delivery` 支持 `sse|sync|async`。所有方式先持久化用户消息和 Run、预占积分，再入 Redis 队列。断开浏览器不会自动取消任务；显式 DELETE 才发起取消。

### 5.3 套餐、订单和后台

- `GET /api/v1/plans`
- `POST|GET /api/v1/orders`
- `GET /api/v1/subscription`
- `GET /api/v1/credits`
- `POST /api/v1/payments/test-webhook`
- `/api/v1/admin/*`：总览、邀请、用户、套餐、订单、订阅、API Key、限流、积分、Run 和审计。

### 5.4 OpenAI 兼容接口

- `GET /v1/models`
- `POST /v1/chat/completions`

模型：

| model | 执行方式 |
| --- | --- |
| `agent-plan-solve` | Plan-and-Solve |
| `agent-multi-turn` | 单 Agent 多轮工具循环 |
| `agent-moe` | Router + 并行专家 + Critic + Aggregator |

`/v1` 仍由调用方重发当前 Session 的完整 `messages`。生产平台则由 PostgreSQL 读取所属会话历史。用户长期记忆只是补充，不能替代完整消息历史。

## 6. 认证、权限与输入安全

- 角色：`VET`、`SUPPORT_ADMIN`、`BILLING_ADMIN`、`SUPER_ADMIN`。
- 账号由管理员邀请；邀请绑定邮箱、角色、初始套餐和有效期。
- 密码使用 Argon2id；当前长度约束为 10～200 字符。
- Access JWT 默认 15 分钟；Refresh Token 默认 30 天并轮换。
- Refresh Token 只保存 SHA-256 摘要，并放入 HttpOnly Cookie。
- 前端 Access Token 仅存在内存，不写入 localStorage。
- `pm_live_*` API Key 完整值只在创建时返回一次；数据库保存前缀和摘要。
- API Key 支持作用域、过期、撤销和最后使用时间；撤销接口已实装。
- 旧 `keys.txt/AGENT_API_KEYS` 由 `AGENT_LEGACY_API_KEYS_ENABLED` 控制兼容期，并记录使用审计。
- 用户身份只从 JWT/API Key Principal 获得；平台路由不信任 body 中的 `user_id`。
- `/v1` 使用数据库 API Key 时，会把认证用户写入请求上下文并作为 Memory `user_id`。
- 平台 Schema 使用严格 Pydantic 模型和 `extra="forbid"`。
- 统一平台错误格式为 `code/message/request_id/details`，响应带 `X-Request-Id`。
- 生产必须限制 CORS Origin、Allowed Host、代理头和请求体大小。

限流由 `PlatformRateLimitMiddleware` 通过 Redis Lua 原子执行，按 IP、用户/API Key、路由共同构造桶；返回 `429`、`Retry-After` 和限流响应头。旧单进程限流中间件跳过 `/api/v1` 与 `/v1`，避免双重计数。

## 7. 平台会诊 Run

```mermaid
sequenceDiagram
    participant U as Web
    participant A as API
    participant P as PostgreSQL
    participant R as Redis
    participant W as Agent Worker
    participant M as Memory
    participant O as MoE

    U->>A: 创建 Run + Idempotency-Key
    A->>P: 保存消息/Run并预占积分
    A->>R: RPUSH run_id
    A-->>U: SSE / 202 / 同步等待
    W->>R: BLPOP
    W->>P: 领取 queued/retry Run
    W->>M: 加载当前用户长期记忆
    W->>O: 会话历史 + 记忆 + 当前问题
    O-->>P: 持久化脱敏阶段与 delta
    W->>P: 保存最终回答、用量并结算积分
    W->>M: user_id + run_id 幂等写回
    P-->>U: Last-Event-ID 续传
```

关键不变量：

1. 会话、消息、Run 和搜索查询必须带认证用户所有权条件。
2. `client_message_id` 与 `Idempotency-Key` 防止重复消息/任务。
3. Run ID 同时作为 Memory `turn_id`，避免重试生成两份记忆。
4. 积分先预占、后结算；失败且没有实际用量时释放全部预占。
5. Redis 入队失败时 Run 标记为 `failed/queue_unavailable` 并退还积分。
6. Worker 启动时重新入队 `queued/running/retry`，状态领取规则防止重复执行。
7. 失败 Run 进入 `petmind:platform:runs:dead` 便于运维检查。
8. SSE 仅暴露理解、路由、会诊、安全复核和整理答复等脱敏阶段。

网页刷新恢复依赖 sessionStorage 中的 `run_id/lastEvent`。恢复过程中会轮询 Run 状态；一旦完成、失败或取消，会清理恢复状态和临时错误，再从 PostgreSQL 拉取最终消息。

## 8. MoE、提示词和事实边界

生产提示词统一位于 `agent_api/app/prompts/`：

| 文件 | 职责 |
| --- | --- |
| `solve.py` | 共享终答、引用、证据层级与结构 |
| `plan_and_solve.py` | Planner |
| `multi_turn.py` | 多轮决策和工具循环 |
| `moe_task_policy.py` | 单次统一决策：D1～D8、分类边界、证据需求和专家选择 |
| `moe_experts.py` | 专家 persona、单轮任务归纳和结论格式 |
| `moe_critic.py` | 事实、安全、禁忌和边界审核 |
| `moe_aggregator.py` | 综合终答与引用规则 |
| `moe_history.py` | 多轮事实状态标签 |
| `moe.py` | 动态提示拼接和 PetHealth 注入 |
| `intent_contracts.py` | D1～D8 意图及输出结构 |
| `memory.py` | 将记忆包装为不可信历史数据 |

医生侧 D1～D8：病历结构化、鉴别诊断、检查规划、报告解读、治疗与用药安全、专业知识快答、多轮病例管理、急症/能力边界。

MoE 顺序：统一任务策略 → 确定性门控 → 可选 PetHealth vitals 核实 → Tool Broker 执行证据任务 → 并行专家单轮归纳 → Critic → Aggregator。统一任务策略一次输出 D1～D8、路由、证据需求、检索 query 和工具所有者，不再经过关键词规则或第二次 Router LLM。每位专家保留独立上下文，但不再自己进行多轮“要工具/再判断”循环；PetHealth 心率提示会注入策略层、相关专家和 Aggregator，外部标志本身不能作为诊断事实。

专家只允许提交一次统一信封 `{"action":"final","opinion":{...}}`；不再接受 `action=tool`、顶层 `conclusion`、`final_answer` 或 `call_tool` 别名。Task Policy 先分配证据任务，Tool Broker 确定性执行并去重，专家随后结合结果一次性形成意见。每个专家会话维护：

- `required_tools`：本轮必须完成的检索；
- `recommended_tools`：建议调用但不阻止专家提交的工具；
- `attempted_tools`：专家实际发起的工具动作；
- `completed_tools/successful_tools`：已经完成/成功的工具；
- `pending_tools`：返回 final 前仍必须执行的工具；
- `unavailable_required_tools`：请求显式禁用或部署未注册的必需工具。

程序只对真正的必需工具执行证据门禁：`pending_tools` 非空时先由执行层补齐或标记不可用，再进入专家单轮归纳；`recommended_tools` 不会锁死终答。常规诊断、鉴别、急症和报告解读不再按 D2～D8 意图一刀切强制工具调用。

必须调用工具的情况：

| 场景 | 必需工具 | 原因 |
| --- | --- | --- |
| 用户明确要求检索本地知识库/本地资料 | `rag.search` | 任务交付物本身包含本地检索结果 |
| 用户明确要求联网检索，或核对最新/现行指南、共识、版本 | `mcp.web_search.web_search` | 模型记忆不能替代请求时点的外部证据 |
| 药学专家回答具体剂量、相互作用、禁忌、物种毒性、不良反应、停换药或洗脱 | `rag.search`；本地证据不足时追加 Web Search | 高风险用药结论必须有可追溯药理依据 |
| `pethealth_server.heart_rate_abnormal=true` 且有 animal ID | Orchestrator 调用 `mcp.vitals_alert.check_vitals` | 外部 flag 只是核验触发器，真实体征必须来自数据库工具 |
| 用户要求某只动物的实时体征/数据库事实，且对应工具在请求中可用 | 对应 vitals/SQL/MCP 工具 | 不能用通用医学知识编造个体实时事实 |

显式 `tools=[]`/`tool_choice=none` 始终保持禁用语义；系统会记录必需工具不可用并降低结论可信度，不会偷偷绕过调用方限制。专家未调用 RAG/Web 时，程序还会移除其 evidence 中虚构的“本地知识库、网络、指南、文献或来源”声明。

`OUTPUT_CONTRACT` 只定义所有专家共用的 `action=final/opinion` JSON 信封、证据真实性、风险和置信度。禁忌联用、洗脱、替代药物等用药规则属于 `PHARMACY_SAFETY_CONTRACT`，只注入药学专家，不再污染临床、营养和行为专家提示词。

必须维持的事实边界：

- `user_report`：用户陈述；
- `assistant_inference`：主 Agent 旧推断；
- `expert_inference`：专家旧推断；
- 真实工具结果：当前数据库/MCP 返回；
- RAG/Web：一般医学证据。

旧回答中的“可能患病”不能在下一轮升级为既往确诊。医学证据不能直接证明当前患者患病。记忆中的文本按不可信数据处理，当前输入、真实工具与安全规则优先。

## 9. PetHealth 被动心率告警

请求示例：

```json
{
  "model": "agent-moe",
  "animal_id": "pet-id",
  "pethealth_server": {
    "animal_id": "pet-id",
    "heart_rate_abnormal": true,
    "vitals_window_hours": 24
  },
  "messages": [{"role": "user", "content": "它今天情况怎么样？"}]
}
```

触发条件是 `heart_rate_abnormal=true` 且能解析 animal ID。`pethealth_server.animal_id` 缺失时回退 body `animal_id` 或 `X-Animal-Id`。窗口限制为 1～720 小时。

Router 把请求视为宠物健康上下文，但不能只凭 flag 设置急症。路由后 Orchestrator 调用 `mcp.vitals_alert.check_vitals`，真实结果进入 Trace、`last_run_context.pethealth_vitals` 和 Aggregator payload。若工具未允许、未注册、数据库不可用、无数据或找不到宠物，终答必须明确无法核实，禁止编造数值。

默认工具白名单包含 `mcp.vitals_alert.check_vitals`；显式 `tools=[]` 或 `tool_choice="none"` 始终优先，不允许绕过调用方的禁用语义。

## 10. Memory Service

Memory Service 默认端口 8300，使用独立 PostgreSQL + pgvector，向量维度 384，默认 embedding 为 `intfloat/multilingual-e5-small`。

请求内流程：加载用户画像、近期对话和相关知识 → 包装为不可信历史 → 注入 Agent → 成功生成后写回消息 → 幂等收据 → 异步整理任务。

核心规则：

- 平台：`platform_users.id` 是稳定 `user_id`，`agent_runs.id` 是 `turn_id`。
- `/v1`：优先使用认证 Principal，其次兼容 `user_id/user/X-User-Id`。
- `/chat-moe`：规范化测试用户名后映射到独立 `chatmoe:` 命名空间。
- 不同用户必须完全隔离；宠物 ID 不能代替用户 ID。
- 写回失败不应伪造成功；required 模式下加载失败阻止推理。

常用变量：

| 变量 | 说明 |
| --- | --- |
| `AGENT_MEMORY_ENABLED` | 是否启用 Agent Memory Client |
| `AGENT_MEMORY_REQUIRED` | 不可用时是否拒绝请求/启动 |
| `AGENT_MEMORY_URL` | 默认 `http://127.0.0.1:8300` |
| `AGENT_MEMORY_TIMEOUT` | Memory HTTP 超时 |
| `MEMORY_DB_DSN` | Memory PostgreSQL |
| `MEMORY_WORKERS_ENABLED` | 是否在 Memory API 进程启动后台整理 Worker |
| `MEMORY_WORKER_CONCURRENCY` | 默认 2 |

`MEMORY_WORKERS_ENABLED=0` 可只启动 API，不消费待整理记忆；适合迁移、诊断或禁止外部 LLM 读取历史数据的环境。启用 Worker 前必须确认组织已授权把相应记忆交给配置的 LLM。

## 11. RAG、MCP 与工具

主要工具：

- `rag.search`
- `sql.search`
- `vitals.summary`
- `mcp.web_search.web_search`
- `mcp.web_search.ingredient_check`
- `mcp.nutritional_planner.calculate_meal_plan`
- `mcp.nutritional_planner.generate_exercise_plan`
- `mcp.vitals_alert.check_vitals`

`rag.reindex` 和 `debug.echo` 是管理/调试工具，不向公开 MoE 专家默认开放。MCP 由 `agent_api/mcp_servers.json` 注册；`command: null` 表示使用当前 Python。`AGENT_ENABLE_MCP=0` 禁用 MCP。

MoE 在专家执行前只进行一次统一任务策略调用，同时输出 D1-D8 主/次意图、专家相关性、急症判断和结构化证据任务。D1-D8 分类边界与每类路由指导均由同一提示词注册表提供；检索决策不依赖问题关键词。LLM 输出 `local_knowledge/current_web/medication_reference/patient_vitals` 能力及 `required/recommended`，程序再映射成实际工具、分配唯一负责专家并执行安全门禁。`required` 未完成时禁止专家直接 final，`recommended` 不锁定工具调用。旧的独立意图分类器、独立 LLM Router 与双轨迁移模式已移除，生产链路固定使用统一任务策略架构。

MoE 专家采用 Task-driven single pass：分配工具最多执行两批（分配任务；必需本地证据较弱或语义覆盖不足时再做 Web 兜底），随后各专家并行生成一次结构化意见；仅在 JSON 为空或格式损坏时允许一次协议修复，不重新做诊疗决策。同一专家的多个证据任务即使映射到相同工具，也按“工具名＋规范化参数”保留不同查询，仅对完全相同调用去重。可通过 `MOE_EXPERT_TIMEOUT_SEC`、`MOE_EXPERT_FINAL_MAX_TOKENS`、`MOE_EXPERT_FORMAT_REPAIR_ATTEMPTS` 和 `MOE_EXPERT_FINALIZE_RESERVE_SEC` 调整。RAG 默认启用 reranker；必需本地证据先由 `RAG_WEB_FALLBACK_MIN_HITS=2`、`RAG_RELEVANCE_THRESHOLD=0.90` 做数值初筛，通过初筛的多个证据任务再合并为一次非思考模式的语义充分性审计，逐项判断 `supported/partial/unsupported`，后两者补充 Web 证据。该审计默认启用，可通过 `MOE_EVIDENCE_SUFFICIENCY_ENABLED` 和 `MOE_EVIDENCE_SUFFICIENCY_TIMEOUT_SEC` 控制。急症只保底启用临床专家，药学专家仅在中毒、剂量、相互作用或用药安全等语义相关时启用。

MoE 的 Task Policy、专家结构化意见、Critic 与 Aggregator 对 DeepSeek 显式关闭 thinking mode，避免默认隐藏推理占满输出预算并造成长时间零 delta。Aggregator 流式正文为空时自动进行一次非流式兜底；部分正文后连接中断则明确失败，不把截断内容误记为成功终答。网页端只持久化并展示脱敏后的专家任务、工具摘要和结构化意见，不下发系统提示词、隐藏推理或原始工具载荷。

RAG 当前使用 multilingual-e5-small、384 维向量、Dense/BM25/邻居扩展/可选 CrossEncoder 重排和分类索引。索引位于 `RAG/data/`，体积大且禁止提交。分类配置复制到 data 中，但活动路径迁移服务器后必须重新核对。MoE 专家提交 `rag.search.query` 时使用英语，用户输入和最终回答仍可为中文。

2026-08-14 真实联调：同一测试用户跨会话召回宠物“小栗”和标记 `MEM-c9032c50`；随后真实调用 `mcp.web_search.web_search` 检索 WSAVA 疫苗指南。QA 审计记录 `tools_used=["mcp.web_search.web_search"]`、`used_web_search=1`，终答包含 VIN/PMC 来源链接，并成功写回记忆。

## 12. 前端实现

页面：登录、接受邀请、忘记/重置密码、套餐、聊天、设置/API Key、帮助和后台。

聊天页支持：

- PostgreSQL 会话历史和全文搜索；
- 新建、选择、删除和 Markdown 导出；
- Markdown、引用、代码块与复制成功动画；
- 兽医/宠物主回答身份选择；
- SSE 流式输出、停止、刷新续传和错误恢复；
- 左下角设置、帮助、会员、API Key、退出菜单；
- 桌面侧边栏收起为 68px 图标栏并再次展开；
- 移动端抽屉、键盘焦点、加载和错误状态。

业务数据均来自 API/数据库；品牌文案、帮助说明、路由标签、视觉 Token 和固定模型展示属于前端静态配置。收费页的价格、积分、有效期和启用状态不在前端写死。

开发代理将 `/api` 转发到 `http://127.0.0.1:8000`，因此浏览器网络面板会显示请求发往 5173，这是 Vite 同源代理，不代表 `/api/v1/me` 由前端静态服务器处理。

品牌源文件 `../logo.jpg` 保持不变；`public/brand/` 保存裁边、透明和多尺寸派生资产。

## 13. 套餐、订单与积分

默认套餐代码：`trial`、`pro_monthly`、`pro_yearly`。套餐价格、积分和有效期由后台数据库配置，前端只显示 `active=true` 的套餐。

订单状态：`pending_payment → paid → fulfilled`，异常状态包括 `cancelled/expired/refunded`。测试 Webhook 使用时间戳、事件 ID 和 HMAC 签名，允许误差 5 分钟，并按外部事件 ID 幂等。重复回调不能重复开通订阅或发积分。

Run 创建时按最大 Token 估算预占额度；完成后按输入/输出长度、专家次数和工具次数结算，多余预占退回。失败或取消且无实际用量时退回全部预占。

## 14. 本地启动

### 14.1 平台基础设施

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
docker compose -f docker-compose.platform.yml up -d
```

默认开发端口：Agent PostgreSQL `15433`，Redis `16379`。Memory PostgreSQL 使用独立实例/数据库，当前本机为 `127.0.0.1:5432/petmemory_dev`。

### 14.2 迁移与管理员

```powershell
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m alembic -c agent_api/alembic.ini upgrade head
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m agent_api.scripts.bootstrap_platform_admin --email admin@example.com
```

密码建议通过交互输入，避免进入 Shell 历史。开发环境可设置 `AGENT_PLATFORM_AUTO_CREATE_SCHEMA=1`；生产必须为 0，并先执行 Alembic。

### 14.3 Agent、Memory 与 Worker

```powershell
.\start_agent.bat cpu
```

```bash
bash start_agent.sh cuda
```

统一 Supervisor 顺序：启动 Memory → 等待 `/health` → 启动 Agent → 若配置 Redis URL 则启动 Platform Worker。任一子进程退出时回收其余进程。`AGENT_PLATFORM_WORKER=0` 可在拆分部署时关闭内置 Worker。

统一启动脚本、`.env.example` 和 systemd 模板默认采用热启动：`AGENT_WARMUP_RAG=1`、`AGENT_WARMUP_BM25=1`、`AGENT_WARMUP_RERANKER=1`、`AGENT_WARMUP_CATEGORIES=1`、`MEMORY_WARMUP_EMBEDDING=1`。服务先提供 `/health`，RAG/分类索引完成加载后 `/ready` 才返回可接流量；资源不足时可显式设为 0，但会把首次请求延迟转移给用户。

### 14.4 前端

生产用 Docker Nginx 托管 `pnpm build` 产物，对外端口仍为 5173，`/api` 反代到 Agent：

```bash
docker compose up -d --build
```

本地热更新仍可用 `pnpm dev`（不要与 Nginx 同时占用 5173）。开发代理将 `/api` 转发到 `http://127.0.0.1:8002`。

本地入口：前端 `http://127.0.0.1:5173`、Agent `http://127.0.0.1:8002`、Memory `http://127.0.0.1:8300`。

## 15. 关键环境变量

生产平台至少配置：

```dotenv
AGENT_PLATFORM_ENABLED=1
AGENT_PLATFORM_ENV=production
AGENT_PLATFORM_DB_URL=postgresql+asyncpg://...
AGENT_PLATFORM_REDIS_URL=redis://...
AGENT_PLATFORM_AUTO_CREATE_SCHEMA=0
AGENT_PLATFORM_JWT_SECRET=<至少32字节随机值>
AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET=<独立随机值>
AGENT_PLATFORM_COOKIE_SECURE=1
AGENT_PLATFORM_EXPOSE_DEV_TOKENS=0
AGENT_PLATFORM_FRONTEND_ORIGIN=https://...
AGENT_PLATFORM_ALLOWED_HOSTS=...
AGENT_LEGACY_API_KEYS_ENABLED=0
AGENT_TRUST_PROXY_HEADERS=1
AGENT_FORWARDED_ALLOW_IPS=127.0.0.1
```

LLM/WebSearch 密钥从 `.env` 或服务环境读取，禁止写入源码、systemd unit 或提交记录。Memory LLM 可使用 DeepSeek，但处理历史记忆属于外部数据传输，必须由部署方明确授权。

## 16. 生产部署

1. 平台 PostgreSQL、Memory PostgreSQL、Redis 均使用私网和独立凭据。
2. Redis 开启 AOF；API 和 Worker 必须使用同一 Redis URL。
3. 先备份，再执行 Alembic；严禁生产 `create_all`。
4. API 可横向扩容，但单 GPU Worker 默认并发 1；不要让多个 Worker 争用同一张卡。
5. 反向代理启用 TLS、精确 CORS/Host、请求体限制和至少 130 秒读取超时。
6. SSE 路由关闭代理缓冲与缓存，并透传 `Authorization`、Cookie、`Last-Event-ID` 和请求 ID。
7. Access JWT 15 分钟、Refresh 30 天；对话保留一年不等于登录态保留一年。
8. 备份平台 PostgreSQL、Memory PostgreSQL 和 Redis AOF；恢复演练需覆盖未完成 Run 重入队。
9. Memory `/health` 不应暴露公网。
10. 日志、Trace 和审计可能包含健康信息，必须限制访问并设置脱敏/留存周期。

systemd 模板当前指向 `/home/sam/Animal_detection2/agentAndRag`、Python 环境和端口 8002/8300。复制到 `/etc/systemd/system` 前必须核对用户、工作目录、EnvironmentFile 和 Python。系统级 unit 可在无人登录时运行；用户级 unit 依赖用户会话或 linger。不要同时启动两份监听相同端口的服务。

## 17. 验证基线

2026-08-16 最新实测：

| 范围 | 结果 |
| --- | --- |
| 平台专项 pytest | 4 passed |
| Memory Service 独立 PostgreSQL | 156 passed |
| 前端 Vitest | 2 passed |
| 前端生产构建 | passed |
| Playwright 桌面/移动端 | 13 passed，1 个仅桌面用例在移动端 skipped |
| Agent `/ready` | ready=true，memory=ok，MCP enabled |
| Memory `/health` | database=ok，2 workers |
| 真实 DeepSeek + Memory + WebSearch | passed |
| MoE + MCP 全量单元回归 | 202 passed |
| 真实 DeepSeek 强制检索 | 临床/药学专家各完成 RAG 8 命中 + Web Search 5 条，pending=0 |
| 真实 DeepSeek 非锁定路径 | 普通“猫尿血怎么办”1 轮 final，无工具调用，PASS |
| 单轮链路真实耗时 | “猫频繁进出猫砂盆，如何排急症？”由 72.6 秒降至 26.9 秒；Task Policy 14.5→2.8 秒，Critic 5.9→1.4 秒，Aggregator 24.2→8.3 秒；4 次 LLM、1 次 RAG |
| 终答流式恢复 | 真实 DeepSeek 流式完整输出；空流单测验证自动降级，部分流中断不会误记成功 |

真实专家报告保存在本地忽略目录 `agent_api/tests/moe/reports/retrieval_policy_balanced_live_20260816_audit/`。明确要求“检索本地兽医知识库并联网核对仍适用指南”的病例中，临床与药学专家均记录 `required_tools=[rag.search, mcp.web_search.web_search]` 和相同顺序的 `attempted_tools`；RAG 分别命中 8 条（最高分 0.9173/0.9138），两次 Web Search 均成功返回 5 条，最终 `pending_tools=[]`。流式 QA 审计同步修复：记录 `id=96` 已落盘 `tools_used=[mcp.web_search.web_search, rag.search]`、`rag_hit_count=16`、`rag_best_score=0.9173`、`used_web_search=1`。普通咨询只记录 `recommended_tools`，专家 1 轮直接 final，证明建议检索不会锁死工具链。强制检索病例的旧风格验收仍因终答标题与测试脚本预设词不完全一致而标记 FAIL，但工具链与结构化专家协议本身已通过，需与内容风格测试分开理解。

Memory 测试必须使用独立测试数据库。若测试与运行中的 Worker 共用 `petmemory_dev`，Worker 会抢先消费测试队列，造成“测试线程只处理 4/5”的假失败。

常用回归：

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m pytest agent_api\tests\platform -q
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m pytest memory_service\tests -q

cd C:\Users\ROG\Animal_detection2\petmindAgentFrontend
pnpm test
pnpm build
pnpm test:e2e
```

启动后检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8300/health
Invoke-RestMethod http://127.0.0.1:8000/health
Invoke-RestMethod http://127.0.0.1:8000/ready
Invoke-WebRequest http://127.0.0.1:5173/chat
```

## 18. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `/api/v1/me` 在 5173 显示 401 | Vite 代理正常；检查 Access Token、Refresh Cookie、用户状态和 API 日志 |
| 刷新后一直显示恢复失败 | sessionStorage run 信息、`Last-Event-ID`、Run 状态轮询和最终消息刷新 |
| 收起侧边栏无变化 | `.workspace.sidebar-collapsed` 是否生效，桌面宽度是否大于 800px |
| API Key 撤销后仍可用 | 数据库 `revoked_at`、中间件 Principal、是否误用旧 `keys.txt` Key |
| Run 长期 queued | Redis、Platform Worker、`petmind:platform:runs` 和 Worker 日志 |
| Worker 重启后重复执行 | Run 状态是否为 queued/retry；检查状态领取和幂等键 |
| SSE 没有实时输出 | Nginx buffering、读取超时、Content-Type 和代理缓存 |
| 没有订阅/积分时报 402 | 用户订阅、过期时间、余额、预占记录 |
| 跨会话失忆 | `AGENT_MEMORY_ENABLED/REQUIRED`、稳定 user ID、Memory 健康和 turn ID |
| Memory Worker 连接池超时 | PostgreSQL 连接上限、Worker 数、长事务和外部 LLM 耗时 |
| WebSearch 未调用 | Tavily Key、MCP 注册、工具白名单和 QA `tools_used` |
| 心率 flag 没有真实值 | VITALS DSN、animal ID、MCP 状态和显式工具禁用语义 |
| RAG 命中旧目录 | taxonomy 中的活动 `index_dir` 和服务器 checkout 路径 |
| Windows CPU 原生崩溃 | 移除 `CUDA_VISIBLE_DEVICES=-1`，只用 `AGENT_WARMUP_DEVICE=cpu` |
| systemd 显示 active 但 Memory 报连接失败 | PostgreSQL DSN、连接池、服务启动顺序和日志完整行 |

## 19. 提交与数据安全

提交前：

1. `git status --short --branch`，确认没有覆盖用户改动。
2. 检查 `.env`、API Key、数据库密码和日志未进入暂存区。
3. 执行平台、Memory、前端定向回归和 `git diff --check`。
4. 检查 Alembic 迁移只操作 `platform_*` 表。
5. 检查前端 `dist/`、`node_modules/`、Playwright 结果未提交。
6. 检查 RAG 索引、模型、OCR 输出和测试数据库未提交。

允许提交：生产源码、规格、迁移、测试源码、启动器、Logo 源图与前端品牌派生资产。

禁止提交：`.env`、`keys.txt`、真实密钥、`agent_api_logs`、SQLite、stdout/stderr、RAG 索引、本地模型、第三方缓存、测试截图和临时数据库。

## 20. 后续技术债

1. 为 Memory API 增加服务间认证、用户记忆导出/删除与审计。
2. 将 Memory Worker 的外部 LLM 计算移出长事务，并做版本条件提交。
3. 为 Redis Run 队列补充租约、可见性超时和更完整的重试/死信策略。
4. 当前平台专项测试数量较少，需要补齐认证重放、跨用户越权、真实 PostgreSQL 迁移和 Redis 故障恢复测试。
5. 接入真实邮件适配器，当前开发令牌只允许在非生产环境暴露。
6. 接入真实支付渠道前实现渠道适配器、退款和对账任务。
7. 对中文搜索建立并验证 `pg_trgm` 索引与真实数据性能基线。
8. 多实例前验证 Redis 限流、SSE 连接数和 100 并发连接背压。
9. 将 RAG taxonomy 活动路径完全改为仓库相对配置。
10. 建立前端视觉回归、无障碍检查和生产错误监控。

维护本系统时始终坚持：**身份来自认证上下文、完整会话归生产数据库、长期记忆按用户隔离、患者事实与医学证据分离、外部告警必须核实、所有任务先持久化再排队、任何密钥和健康数据都不进入 Git。**
