# PetMind Agent / RAG / Memory 维护交接文档

> 最后核对：2026-08-05
> 目标仓库：`C:\Users\ROG\Animal_detection2`
> 目标分支：`test-agent`
> 核对基线：`9659682 feat(agent): migrate production agent stack`

本文是 `agentAndRag` 当前实现的零上下文维护入口。旧的 `agent_api/README.md` 含有部分迁移前的扁平目录、旧工具和旧会话描述；发生冲突时，以本文和实际代码为准。

## 1. 接手先看这里

1. 工作目录固定为 `C:\Users\ROG\Animal_detection2\agentAndRag`。
2. Python 建议使用 `C:\Users\ROG\anaconda3\envs\RAG\python.exe`。
3. 本机索引、模型、密钥和 `.env` 是忽略文件，不在 Git 中；换机器或重新 clone 后必须单独恢复。
4. Windows CPU 启动只设置 `AGENT_WARMUP_DEVICE=cpu`，**绝对不要设置 `CUDA_VISIBLE_DEVICES=-1`**。
5. 先看 `/health` 判断进程存活，再看 `/ready` 判断 RAG 预热及工具注册状态。
6. 生产接口 `/v1/chat/completions` 是无状态的；调用方必须保存并在下一次请求中重发完整 `messages`。
7. 只有内置测试页 `/chat-moe` 使用 Agent 侧 SQLite 会话缓存，不可把它误当成生产会话服务。
8. MoE 内 `rag.search` 的 `query` 必须是英语；用户输入和最终回答仍可使用中文。
9. `memory_service` 是独立的 PostgreSQL + pgvector 服务，当前没有自动注入 Agent 提示词。
10. 修改后至少验证三种模型、RAG、MCP、会话恢复和无关话题门禁。

## 2. 总体架构

```mermaid
flowchart TD
    CLIENT["业务后端或测试网页"] --> API["FastAPI Agent API :8000"]
    API --> AUTH["认证、限流、请求上下文"]
    AUTH --> OPENAI["POST /v1/chat/completions"]
    AUTH --> CHAT["POST /chat-moe/completions"]

    OPENAI --> MODE{"model 路由"}
    MODE --> PLAN["Plan-and-Solve"]
    MODE --> MULTI["Multi-Turn Agent"]
    MODE --> MOE["MoE Orchestrator"]

    CHAT --> SESSION["测试会话管理器"]
    SESSION --> SQLITE["petmind_sessions.db"]
    SESSION --> MOE

    PLAN --> REGISTRY["ToolRegistry"]
    MULTI --> REGISTRY
    MOE --> ROUTER["Router"]
    ROUTER --> EXPERTS["独立并发专家子代理"]
    EXPERTS --> BROKER["Request-scoped Tool Broker"]
    BROKER --> REGISTRY
    REGISTRY --> RAG["本地 RAG 索引"]
    REGISTRY --> MCP["MCP: Web Search / Nutrition / Vitals"]
    REGISTRY --> SQL["MySQL: sql.search / vitals.summary"]
    EXPERTS --> CRITIC["Critic"]
    CRITIC --> AGG["Aggregator 流式终答"]

    API -. "尚未直接接入" .-> MEMORY["Memory Service :8300"]
    MEMORY --> PG["PostgreSQL + pgvector"]
    MEMORY --> WORKERS["异步记忆整理 Worker"]
```

### 2.1 关键边界

- Agent API 负责推理编排、工具访问、流式输出、测试会话和审计。
- RAG 是本地只读检索层，不是病例数据库；检索结果是医学证据，不是当前患者事实。
- MCP 通过 stdio 启动子进程，提供网络搜索、营养计算和生命体征告警。
- PetMind MySQL 工具只读业务数据，并依赖请求中的 `animal_id`。
- Memory Service 管理跨会话长期记忆，但目前与 Agent API 解耦，接入前不能声称 Agent 已具备生产长期记忆。

## 3. 目录与职责

```text
agentAndRag/
|-- agent.md                              # 本文，当前维护入口
|-- .env.example                         # Agent 环境变量样例
|-- start_agent.bat                      # Windows GPU 快捷启动
|-- start_agent.sh                       # Linux/systemd 场景启动
|-- agent-rag.service                    # systemd 示例
|-- agent_api/
|   |-- app/main.py                      # FastAPI 生命周期、工具注册、RAG 预热
|   |-- app/routers/routes_openai.py     # OpenAI-compatible API
|   |-- app/routers/routes_chat_ui.py    # /chat-moe 测试页与测试会话接口
|   |-- app/services/agent_execution.py  # 三种架构的统一分派
|   |-- app/services/moe/                # Router、Experts、Broker、Critic、Aggregator
|   |-- app/prompts/                     # 三种架构全部 LLM 提示词、MoE 各阶段与请求级动态注入
|   |-- app/tools/                       # ToolRegistry、RAG、MCP、内置工具
|   |-- app/concurrency/                 # 单进程全局资源限流
|   |-- app/persistence/                 # QA、trace、测试会话 SQLite
|   |-- app/lifecycle_tasks/             # 定时清理测试会话
|   |-- app/sql_search/                  # PetMind MySQL 只读工具
|   |-- tests/                           # Agent 单元、集成和真实链路测试源码
|   |-- scripts/run_cpu_rag_server.py    # Windows CPU 安全启动辅助脚本
|   |-- scripts/*test*.py                # API smoke / integration 测试入口
|   |-- mcp_servers.json                 # MCP 服务注册配置
|   `-- keys.txt                         # 本地 API key，禁止提交
|-- RAG/
|   |-- simple_rag/                      # embedding、dense/BM25、重排、分类索引
|   |-- tests/                           # RAG 单元测试源码
|   |-- experiments/                     # 检索评测和性能脚本，源码需要提交
|   |-- query.py                         # 检索 CLI
|   |-- ingest.py                        # 索引构建入口
|   `-- data/                            # 本地索引与 taxonomy，禁止提交
|-- mcp_servers/
|   |-- web_search/                      # Tavily 网络搜索
|   |-- nutritional_planner/             # 营养与运动计算
|   `-- vitals_alert/                    # PostgreSQL 生命体征告警
|-- memory_service/
|   |-- app/main.py                      # 记忆服务入口，默认 :8300
|   |-- app/memory/                      # 短期、中期、长期、检索、队列
|   |-- app/worker.py                    # 后台整理任务
|   |-- scripts/init_local_db.py         # 初始化 PostgreSQL + pgvector
|   |-- scripts/run_worker.py            # 独立 worker
|   |-- tests/                           # Memory Service 测试源码
|   `-- sql/001_schema.sql               # 记忆表和 vector(384)
|-- models/                              # 本地 HF 模型，禁止提交
`-- agent_api_logs/                      # SQLite、trace、运行日志，禁止提交
```

## 4. Agent API 与三种执行模式

统一入口是 `POST /v1/chat/completions`，请求格式兼容 OpenAI Chat Completions。

| `model` | 执行方式 | 用途 |
|---|---|---|
| `agent-plan-solve` | 一次规划，再执行工具并生成答案 | 兼容旧链路 |
| `agent-multi-turn` | 单 Agent 多轮“决策 -> 工具 -> 观察” | 一般工具型问答 |
| `agent-moe` | Router -> 独立专家 -> Critic -> Aggregator | 兽医复杂问诊主路径 |

无法识别的模型名当前会默认进入 MoE，而不是报错。调用方必须使用明确模型名，避免拼写错误导致架构静默变化。

请求中的工具语义：

- 未提供 `tools` 或为 `null`：按该架构使用默认可用工具。
- 显式 `tools=[]`：无工具可用。
- `tool_choice="none"`：禁止工具调用。
- 请求体 `animal_id` 或请求头 `X-Animal-Id` 会建立请求级动物上下文。

生产 `/v1/chat/completions` 不写入 `/chat-moe` 会话。业务服务器应保存完整轮次，并把 `system/user/assistant` 历史随每个请求重发；不要依赖 Agent 进程内状态。

## 5. MoE 实际调用链

```mermaid
sequenceDiagram
    participant U as "调用方"
    participant O as "Orchestrator"
    participant R as "Router"
    participant E as "专家子代理"
    participant B as "Tool Broker"
    participant T as "RAG / MCP / SQL"
    participant C as "Critic"
    participant A as "Aggregator"

    U->>O: "query + role + messages + expert history"
    O->>R: "路由提示词与带事实状态的历史"
    R-->>O: "选择专家、权重、门禁判断"
    par "专家上下文相互独立"
        O->>E: "临床专家循环"
        O->>E: "药理专家循环"
        O->>E: "营养专家循环"
        O->>E: "行为专家循环"
    end
    loop "每个专家最多 6 轮"
        E->>B: "自主提出工具调用"
        B->>T: "执行或复用结果"
        T-->>B: "证据或业务数据"
        B-->>E: "观察结果"
    end
    E-->>O: "独立专家结论及工具记录"
    O->>C: "专家意见、证据和历史"
    C-->>O: "安全性与约束审查"
    O->>A: "汇总上下文"
    A-->>U: "直接 SSE 流式输出"
```

### 5.1 Router

Router 接收当前用户问题、用户角色、完整会话历史和已保存的专家历史。它负责：

- 判断整体话题是否仍与动物健康、兽医或合理延伸信息相关；
- 结合历史处理简短追问，避免只看当前一句造成误拦截；
- 选择临床、营养、药理、行为专家及权重；
- 对真正无关的主题执行门禁。

### 5.2 独立专家子代理

每个专家拿到“当前 query + 对应专家系统提示词 + 带事实状态的主历史 + 本专家历史”，然后独立进行多轮工具决策。不存在一个总 Planner 预先替四位专家查完再喂固定结果的流程。

专家默认循环控制：

| 变量 | 默认值 | 含义 |
|---|---:|---|
| `MOE_EXPERT_MAX_ROUNDS` | 6 | 专家最大 LLM 决策轮数 |
| `MOE_EXPERT_MAX_TOOL_CALLS` | 4 | 单专家最大工具调用数 |
| `MOE_EXPERT_TIMEOUT_SEC` | 60 | 单专家总超时 |
| `MOE_EXPERT_FINALIZE_RESERVE_SEC` | 12 | 为最终结论保留的时间 |
| `MOE_EXPERT_MAX_REPEATED_CALLS` | 1 | 同参数重复调用上限 |

倒数第二轮会提示专家下一轮必须输出结论；达到轮数、工具数或时间边界后仍会保留最终返回机会。临床与药理专家在高影响、证据敏感场景且工具可用时，提示词会倾向同时使用 RAG 和 Web Search，但不强制每个常识问题都检索。

### 5.3 Tool Broker 与两阶段检索

Broker 是**单次 MoE 请求作用域**的协调器：

- 专家并发思考并分别提出工具请求；不要求先完成的专家等待所有专家“统一提交问题”后才能继续。
- `rag.search` 进入 FIFO 队列串行执行，避免本地向量模型和索引同时争用内存/GPU。
- 规范化后完全相同的 RAG 请求共享同一个结果。
- Web Search、营养、SQL 等非 RAG 工具可并行执行，但仍受进程级 MCP/LLM 限流。
- `MOE_RAG_BROKER_TIMEOUT_SEC` 默认 120 秒。
- MoE 的 RAG 查询必须是英语；Broker 会拒绝含中日韩字符的 `arguments.query`。

这解决的是请求内 RAG 协调；跨请求还会经过全局 `AGENT_RAG_MAX_CONCURRENCY` 限流。

### 5.4 Critic 与 Aggregator

Critic 检查安全性、证据一致性、禁忌联用、替代方案和回答约束。Aggregator 接收专家意见、Critic 结果、工具证据和事实状态历史，直接流式生成最终回答。

必须保持以下约束：

- 只有用户明确报告、检查结果或数据库返回的患者数据可作为确定患者事实。
- 先前 assistant/expert 的诊断建议仍是推断，除非用户随后明确确认。
- RAG/Web Search 只能证明一般医学知识，不能证明当前患者患有某病。
- 引用必须来自实际检索证据，不得生成不存在的来源、标题或 URL。
- 已查明存在重要禁忌或相互作用时，需要给出可讨论的替代路径或下一步处置，不能只说“不能同用”。
- 面向宠物主的非急症首轮问题应优先补充关键追问和观察点，避免在信息不足时直接渲染严重氛围；明确急症信号仍应及时分诊。
- Aggregator 后没有第二道内容门控，因为终答占主要输出并需直接流式返回。提示词质量和上游证据边界因此尤其重要。

MoE 最终答案预算由 `MOE_FINAL_ANSWER_MAX_TOKENS` 统一封顶，默认 2500。模型因 token 上限结束时，`finish_reason` 归一化为 `truncated`。

## 6. 历史、事实状态与测试会话

### 6.1 事实状态

历史构造逻辑位于 `agent_api/app/services/moe/history_context.py`。历史被标为：

- `user_report`：用户提供的信息，仍需区分主观描述与已完成检查；
- `assistant_inference`：主 Agent 先前推断或建议；
- `expert_inference`：专家先前推断。

这些标签分别注入 Router、每个专家、Critic 和 Aggregator。维护提示词时不得删除或弱化标签，否则“可能 MMVD”在下一轮被当作“既往已确诊 MMVD”的问题会回归。

### 6.2 `/chat-moe` 测试会话

测试页使用：

- `POST /chat-moe/sessions` 创建 session；
- `POST /chat-moe/completions` 携带 `session_id` 对话；
- SQLite：`agent_api_logs/petmind_sessions.db`；
- 代码：`app/persistence/session_manager.py`；
- 定时清理：`app/lifecycle_tasks/session_cleanup.py`。

它保存完整轮次、工具结果和专家上下文，并用每 session 锁防止同一会话并发写乱。后端重启后可从 SQLite 恢复。测试网页在一次页面生命周期内复用同一 session；刷新页面会创建新的 session id，因此不能把刷新后的新会话误判为持久化失败。

默认配置：

| 变量 | 默认值 |
|---|---:|
| `AGENT_SESSION_TTL_SEC` | 3600 |
| `AGENT_SESSION_MAX` | 10000 |
| `AGENT_SESSION_CONTEXT_MAX_TURNS` | 24 |
| `AGENT_SESSION_CONTEXT_MAX_CHARS` | 48000 |
| `AGENT_SESSION_CLEANUP_INTERVAL_SEC` | 300 |
| `AGENT_SESSION_DB_PATH` | `agent_api_logs/petmind_sessions.db` |

上下文按**完整轮次**截断，不能从一轮中间切断 tool/result 或 user/assistant 对。清理任务跳过正在使用的 session，按 TTL 和容量删除过期/最旧记录。

### 6.3 两个 SQLite 不等于业务数据库

- `petmind_sessions.db`：仅 `/chat-moe` 测试会话。
- `petmind_qa.db`：QA、反馈和知识缺口等观测数据。
- PetMind 业务数据由 MySQL 提供。
- Memory Service 使用 PostgreSQL + pgvector。

快速查看 SQLite：

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
C:\Users\ROG\anaconda3\envs\RAG\python.exe -c "import sqlite3; p='agent_api_logs/petmind_sessions.db'; c=sqlite3.connect(p); c.row_factory=sqlite3.Row; r=c.execute('select * from agent_sessions order by last_active desc limit 1').fetchone(); print(dict(r) if r else 'empty')"
```

## 7. 工具体系

### 7.1 ToolRegistry

内置工具包括：

- `rag.search`
- `rag.reindex`
- `sql.search`
- `vitals.summary`
- `debug.echo`

MoE 专家当前暴露所有非管理工具，静态专家工具列表只是偏好，不是硬性权限边界。`rag.reindex` 和 `debug.echo` 不向专家开放。

### 7.2 MCP

配置文件为 `agent_api/mcp_servers.json`，当前服务：

- `web_search`：Tavily 网络搜索；
- `nutritional_planner`：营养和运动规划；
- `vitals_alert`：从 PetHealth PostgreSQL 读取生命体征并告警。

配置中的 `command: null` 会使用当前 `sys.executable` 启动 MCP，避免迁移后仍指向旧 Python。`AGENT_ENABLE_MCP=0` 可禁用全部 MCP。Web Search 需要 `TAVILY_API_KEY`；缺失时工具可能注册但调用失败，验证时必须看实际 tool result，不能只看 `/tools`。

### 7.3 SQL 与动物上下文

`sql.search` 和 `vitals.summary` 只在请求包含 `animal_id` 或 `X-Animal-Id` 时对 Agent 暴露，防止无动物范围查询。MySQL 连接使用 `PETMIND_MYSQL_*`。`vitals_alert` MCP 则使用独立的 `VITALS_DB_DSN` PostgreSQL 连接，二者不要混淆。

请求示例：

```json
{
  "model": "agent-moe",
  "stream": true,
  "animal_id": 123,
  "messages": [
    {"role": "user", "content": "结合它最近的健康记录分析食欲下降。"}
  ]
}
```

## 8. RAG 系统

### 8.1 当前索引

- Embedding：`multilingual-e5-small`
- 向量维度：384
- 主索引：166,990 chunks，3 个主文件
- 分类索引：138 个文件
- taxonomy：46 个分类
- 主目录：`RAG/data/rag_index_e5`
- 分类目录：`RAG/data/rag_index_e5_by_cat`
- 分类配置：`RAG/data/category_taxonomy.json`

典型流程：英文 query -> query embedding -> dense/BM25 候选 -> 邻居扩展 -> 可选 CrossEncoder rerank -> 返回命中、分数与来源元数据。

### 8.2 预热策略

- `AGENT_WARMUP_RAG=1`：后台预热，`/health` 可先响应，完成后 `/ready` 才报告 ready。
- `AGENT_WARMUP_BM25=1`：启动时构建 166,990 文档的 BM25，内存峰值较高。
- `AGENT_WARMUP_RERANKER=1`：预加载 CrossEncoder；CPU 上很慢。
- `AGENT_WARMUP_CATEGORIES=1`：预热专家分类索引，会增加启动时间和内存。
- 保守 CPU 启动建议三项都设为 `0`，只预热 dense 主索引；需要时再按机器资源打开。

### 8.3 迁移路径陷阱

`category_taxonomy.json` 的分类 `index_dir` 当前保存绝对路径。此次迁移后 46 个分类路径已改为 `Animal_detection2` 并验证存在，但顶层 `source_index`、`out_root` 元数据仍可能显示旧工程路径。未来再次迁移时：

1. 搜索 taxonomy 中是否还有旧根目录；
2. 验证每个 `index_dir` 都存在；
3. 发起一次真实分类检索，检查返回的实际 index path；
4. 不要仅凭索引能加载就认为路径正确，它可能静默读取另一份旧 checkout。

命中元数据中的 `source_path` 可能是原始 OCR/建库文件位置，它是引用来源元数据，不代表当前活动索引目录。

## 9. 全局并发与超时

`agent_api/app/concurrency/resource_limits.py` 提供单 Python 进程共享的资源限流：

| 资源 | 默认并发 | 变量 |
|---|---:|---|
| LLM | 4 | `AGENT_LLM_MAX_CONCURRENCY` |
| RAG | 1 | `AGENT_RAG_MAX_CONCURRENCY` |
| MCP | 4 | `AGENT_MCP_MAX_CONCURRENCY` |
| 获取槽位超时 | 30 秒 | `AGENT_RESOURCE_ACQUIRE_TIMEOUT_SEC` |

`AGENT_RESOURCE_LIMITS_ENABLED=0` 可关闭，但生产不建议。同步和异步调用共享底层计数，超时会抛出 `RESOURCE_BUSY`；指标可在 `/ready` 查看。

**限制只在单进程内全局。** 启动多个 Uvicorn worker 后，每个进程各有一套 semaphore，例如 4 workers、每进程 RAG=1，实际最多可能同时 4 个 RAG。当前设计没有 Redis/数据库式跨进程全局限流。未完成外部限流前，建议 Agent 使用单 worker，由上游业务服务做请求排队与会话管理。

## 10. Windows 启动

### 10.1 必需的本地资产

确认以下内容存在：

- `.env` 或已经导出的环境变量；
- `agent_api/keys.txt` 或 `AGENT_API_KEYS`；
- `RAG/data/rag_index_e5`；
- `RAG/data/rag_index_e5_by_cat` 和 taxonomy；
- 本地模型目录或可用的 Hugging Face cache；
- LLM API key；需要 Web Search 时还要 Tavily key。

Python 入口在 Windows 上**不会自动读取 `.env`**。`start_agent.sh` 会 source `.env`，但直接运行 Uvicorn 前必须在当前 PowerShell 设置变量，或由进程管理器注入。

### 10.2 推荐 CPU 启动

在 PowerShell 中：

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
$env:AGENT_WARMUP_DEVICE = 'cpu'
$env:AGENT_WARMUP_RAG = '1'
$env:AGENT_WARMUP_BM25 = '0'
$env:AGENT_WARMUP_RERANKER = '0'
$env:AGENT_WARMUP_CATEGORIES = '0'
$env:AGENT_ENABLE_MCP = '1'
$env:AGENT_HF_OFFLINE = '1'
$env:OMP_NUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'
$env:OPENBLAS_NUM_THREADS = '1'
$env:TOKENIZERS_PARALLELISM = 'false'
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000
```

也可以用专用辅助脚本，它会在主线程先完成 CPU RAG 预热，并显式规避 CUDA 环境变量崩溃：

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
$env:AGENT_WARMUP_BM25 = '0'
$env:AGENT_ENABLE_MCP = '1'
$env:AGENT_HF_OFFLINE = '1'
C:\Users\ROG\anaconda3\envs\RAG\python.exe agent_api\scripts\run_cpu_rag_server.py --host 127.0.0.1 --port 8000
```

### 10.3 CUDA 的关键警告

**不要设置 `$env:CUDA_VISIBLE_DEVICES='-1'`。** 当前 Windows 环境使用 CUDA build 的 PyTorch，`-1` 会被该构建/依赖路径当成不兼容设备配置，曾在 RAG warmup 后触发原生层 `ACCESS_VIOLATION`，退出码为 `0xC0000005` / `3221225477`。这不是普通 Python 异常，日志可能在正常预热输出后突然终止。

要在 CPU 上跑 RAG，只设置：

```powershell
$env:AGENT_WARMUP_DEVICE = 'cpu'
```

若当前终端以前设置过 `CUDA_VISIBLE_DEVICES=-1`，先移除：

```powershell
Remove-Item Env:CUDA_VISIBLE_DEVICES -ErrorAction SilentlyContinue
```

GPU 启动使用：

```powershell
$env:AGENT_WARMUP_DEVICE = 'cuda'
```

`start_agent.bat` 默认选择 CUDA。开启 reranker 和分类索引预热前先确认显存，否则可能启动慢、OOM 或与正在运行的任务争抢资源。

### 10.4 认证

- 默认从 `agent_api/keys.txt` 或 `AGENT_API_KEYS` 加载 Bearer key。
- 没有 key 且 `AGENT_DISABLE_AUTH` 未开启时，服务会启动失败。
- `AGENT_DISABLE_AUTH=1` 仅用于本地测试。
- 文档、日志、提交信息中都不要写真实 key。

## 11. 启动后验证

### 11.1 存活与就绪

```powershell
Invoke-RestMethod http://127.0.0.1:8000/health
Invoke-RestMethod http://127.0.0.1:8000/ready
Invoke-RestMethod http://127.0.0.1:8000/tools -Headers @{Authorization = 'Bearer <key>'}
```

`/health` 返回成功只说明 FastAPI 进程活着；RAG 仍可能在预热。以 `/ready` 的 `ready`、warmup 信息和资源指标为准。

### 11.2 直接验证 RAG

```powershell
$headers = @{Authorization = 'Bearer <key>'; 'Content-Type' = 'application/json'}
$body = @{query = 'canine degenerative mitral valve disease syncope differential diagnosis'; top_k = 3} | ConvertTo-Json
Invoke-RestMethod http://127.0.0.1:8000/tools/rag/search -Method Post -Headers $headers -Body $body
```

### 11.3 验证三种架构

```powershell
$headers = @{Authorization = 'Bearer <key>'; 'Content-Type' = 'application/json'}
$models = @('agent-plan-solve', 'agent-multi-turn', 'agent-moe')
foreach ($model in $models) {
    $body = @{
        model = $model
        stream = $false
        max_tokens = 800
        messages = @(@{role = 'user'; content = 'A 12-year-old small dog coughs after drinking and excitement. Resting respiratory rate is 24. What should be clarified and considered?'})
    } | ConvertTo-Json -Depth 6
    Invoke-RestMethod http://127.0.0.1:8000/v1/chat/completions -Method Post -Headers $headers -Body $body
}
```

全链路验证时不要传 `tools=[]`，否则会误以为 Agent 不愿调用 RAG/MCP。检查 trace 中是否确实出现 `rag.search` 与 `mcp.web_search.*`，不要只根据最终文本猜测。

### 11.4 多轮事实边界回归

至少包含以下三轮：

1. 老年小型犬咳嗽，询问鉴别；
2. 补充饮水/兴奋后咳、睡眠呼吸 24 次/分、无晕倒；
3. 补充突然倒地、舌头发紫、数秒恢复。

第三轮应把心源性晕厥、心律失常、气道/肺源性缺氧等作为鉴别和急症风险，但不得把前两轮“可能 MMVD”写成“既往已存在/已确诊 MMVD”。

### 11.5 `/chat-moe` 会话回归

验证点：

- 同 session 的第二轮能引用第一轮用户事实；
- 专家意见、RAG 和 Web Search 记录进入 `expert_contexts/tool_results`；
- 重启后端后，用原 session id 仍能继续；
- 页面刷新会创建新 session id；
- 与既有兽医上下文相关的简短查询可通过门禁；
- 明确转向无关编程问题应被拒绝；
- 调用生产 `/v1/chat/completions` 不应修改测试 session。

## 12. Memory Service

### 12.1 当前定位

Memory Service 是独立服务，入口 `memory_service/app/main.py`，默认端口 8300。搜索当前代码没有发现 Agent API 主链路调用它，因此它的 profile、knowledge 或 context **不会自动进入 Router/Experts/Critic/Aggregator**。后续接入需明确用户/宠物身份、读取时机、事实可信度和隐私边界。

### 12.2 数据模型

PostgreSQL + pgvector 表：

- `memory_short_term`：快速写入的短期消息；
- `memory_segments`：中期分段与摘要；
- `memory_pages`：分段内页面及向量；
- `memory_profiles`：用户/宠物结构化画像；
- `memory_knowledge`：长期知识条目；
- `memory_tasks`：异步整理任务队列。

写入路径先保存短期消息并排队，LLM/embedding 整理在 worker 中执行。多个 worker 使用 `FOR UPDATE SKIP LOCKED` 避免重复出队，并用 PostgreSQL advisory lock 保证同一用户串行处理；启动时会重排长时间卡在 running 的任务。

### 12.3 初始化和启动

默认 DSN：`postgresql://postgres:postgres@127.0.0.1:5432/petmemory_dev`。先安装 PostgreSQL 的 pgvector 扩展，再执行：

```powershell
cd C:\Users\ROG\Animal_detection2\agentAndRag
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m memory_service.scripts.init_local_db --seed
C:\Users\ROG\anaconda3\envs\RAG\python.exe -m memory_service.app.main
```

独立 worker：

```powershell
C:\Users\ROG\anaconda3\envs\RAG\python.exe memory_service\scripts\run_worker.py --workers 4
```

主要接口：

- `POST /v1/memory/messages`
- `POST /v1/memory/context`
- `GET /v1/memory/profile/{user_id}`
- `GET /v1/memory/stats/{user_id}`
- `GET /health`

默认 `MEMORY_WORKER_CONCURRENCY=2`，embedding 默认 CPU。数据库 schema 使用 `vector(384)`，更换 embedding 模型时必须同时迁移向量列与索引，否则写入会维度不匹配。

## 13. 日志、Trace 与性能定位

主要运行数据在 `agent_api_logs/`：

- `trace.jsonl` 或按 trace 配置生成的请求追踪；
- `petmind_sessions.db`；
- `petmind_qa.db`；
- 手工启动时重定向的 stdout/stderr。

排查一次慢请求时按时间线拆分：Router LLM、每位专家每轮 LLM、Broker 排队、RAG、Web Search、Critic LLM、Aggregator 首 token 与总生成时间。MoE 的总时长更接近最慢专家加 Critic/Aggregator，而不是所有专家时长简单相加；RAG 被串行化后，多个不同查询会累积排队。

已验证的迁移基线：

- Agent API：215 tests passed；
- RAG：10 tests passed；
- Memory Service：153 tests passed；
- 合计：378 tests passed；
- 125 个生产 Python 文件完成解析/导入检查；
- CPU dense RAG、Tavily、三种 Agent 和 `/chat-moe` 重启恢复均做过真实链路验证。

一次真实环境参考耗时：Plan-Solve 50.8 秒、Multi-Turn 61.7 秒、MoE 39.4 秒。该数字受 LLM 服务、搜索网络、工具数量和缓存冷热影响，只能作为基线，不能当 SLA。

## 14. 常见故障与容易踩坑的地方

| 现象 | 优先检查 |
|---|---|
| 预热看似成功后进程突然退出，码 `3221225477` | 是否设置了 `CUDA_VISIBLE_DEVICES=-1`；移除它，只用 `AGENT_WARMUP_DEVICE=cpu` |
| `/health` 正常但请求慢或 RAG 不可用 | 查看 `/ready`，确认 warmup 状态，不要把 liveness 当 readiness |
| CPU 启动内存很高 | 设 `AGENT_WARMUP_BM25=0`、`AGENT_WARMUP_CATEGORIES=0` |
| CPU 检索数秒甚至更久 | 检查是否启用 CrossEncoder reranker；CPU reranker 是主要慢点 |
| 直接 Uvicorn 找不到 key/model | Windows Python 不自动加载 `.env`，需在当前进程导出环境变量 |
| RAG 命中来自旧工程 | 检查 `category_taxonomy.json` 的绝对 `index_dir` 和实际解析路径 |
| MoE 的 RAG 调用报参数错误 | `rag.search.arguments.query` 必须全部用英语 |
| 工具列表有 Web Search，但调用失败 | 检查 `TAVILY_API_KEY` 和 MCP stderr；注册成功不等于外部 API 可用 |
| SQL 工具没有出现在专家工具中 | 请求是否携带 `animal_id` 或 `X-Animal-Id` |
| 多轮生产请求“失忆” | `/v1` 本来就是无状态；业务后端必须重发完整 `messages` |
| `/chat-moe` 刷新后历史消失 | 刷新会创建新 session id；原 session 仍可按旧 id 从 SQLite 恢复 |
| 旧推断变成已确诊病史 | 检查 fact-state 标签是否仍注入所有阶段，尤其 Aggregator |
| 引用看起来完整但不存在 | 引用只能来自实际 RAG/Web 结果；Aggregator 后没有额外事实门控 |
| `finish_reason=truncated` | 终答达到 token 上限；检查 `MOE_FINAL_ANSWER_MAX_TOKENS` 和回答冗长度 |
| 多 worker 后 RAG 并发超预期 | limiter 仅单进程全局；worker 数会乘大实际并发 |
| 以为 Memory 已改善 Agent 长期记忆 | 当前两服务未接线，需要显式调用与提示词注入设计 |
| SQLite 被误认为 PetMind 数据库 | sessions/qa 是 Agent 本地库；业务是 MySQL，Memory 是 PostgreSQL |

## 15. 安全维护流程

修改前：

1. `git status --short --branch`，确认用户已有变更，不要覆盖。
2. 核对分支为 `test-agent`。
3. 检查本地 `.env`、keys、索引和模型是否存在，但不要加入 Git。
4. 先读实际调用链再改提示词；同一约束可能分别注入 Router、Expert、Critic、Aggregator。

修改后：

1. 跑对应单元测试和静态导入检查。
2. 用 CPU 真实启动后端，等待 `/ready`。
3. 分别测试 `agent-plan-solve`、`agent-multi-turn`、`agent-moe`。
4. MoE 至少验证一次本地 RAG 和一次真实 Web Search。
5. 验证多轮事实边界、相关话题门禁和无关话题拒绝。
6. 验证 `/chat-moe` 同 session、重启恢复、专家/tool 历史以及生产接口不污染测试 session。
7. 停止后台进程并确认 8000/8300 端口释放。
8. 提交前逐个检查差异文件及 `.gitignore`。

禁止提交：

- `.env`、`agent_api/keys.txt`、任何真实密钥；
- `RAG/data` 索引、`models`；
- `agent_api_logs`、SQLite、stdout/stderr；
- `outputs`、评测 reports/results、测试缓存、临时运行目录和生成制品；
- 未经明确要求的大型 OCR 数据或第三方缓存。

测试源码不是运行结果，必须随生产代码一起回迁和提交，包括 `agent_api/tests`、`RAG/tests`、`memory_service/tests`、`RAG/experiments`、`agent_api/scripts` 中的 smoke/integration 测试脚本，以及测试直接 import 的支撑脚本（当前包括 `DeepSeek-OCR-master/DeepSeek-OCR-hf/batch_ocr_to_vllm_layout.py` 和 `memory_service/scripts/simulate_usage.py`）。只排除这些测试产生的 `reports`、结果 JSON、日志、缓存和临时数据库。

## 16. 下一位维护者的优先技术债

1. 把 taxonomy 的活动索引路径改为相对仓库路径，消除迁移后静默读取旧 checkout 的风险。
2. 若要多 worker/多实例部署，把 LLM/RAG/MCP 限流迁移到 Redis、队列或统一网关。
3. 明确 Memory Service 接入契约：身份键、检索时机、事实状态、冲突处理、删除与隐私策略。
4. 更新或替换旧 `agent_api/README.md`，避免新维护者照旧目录修改错误文件。
5. 为真实链路建立固定回归集和阶段耗时指标，尤其监控专家超时、Broker 等待和 Aggregator 截断率。

维护时最重要的三个原则：**生产状态归调用方、患者事实与医学证据严格分离、任何并发结论都要明确其进程边界。**
