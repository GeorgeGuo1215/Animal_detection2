# PetMind Agent API

PetMind 仅保留 `agent-moe` 执行架构。生产部署把业务 API 与 GPU Agent Runtime 分离，避免 8002 重复加载 RAG、Reranker 和 MCP。

## 生产拓扑

```text
Browser / API client
        │
        ▼
8002  Gateway / Platform Backend
      认证、RBAC、套餐与积分、会话和消息、Run、SSE、PostgreSQL、Redis
        │  internal HTTP + AGENT_WORKER_TOKEN
        ▼
8102  Agent Worker
      agent-moe、Task Policy、Router、专家、Critic、Aggregator、RAG、MCP
        │
        ├── Redis Run queue
        └── 8300 Memory Service
```

- `8002` 是生产前端唯一访问的后端，不加载推理模型。
- `8102` 是 Agent 本体，只绑定回环地址或 GPU 私网，不开放公网。
- `8300` 是长期记忆服务。Gateway 管理业务身份，实际会诊时由 Worker 读取和写回记忆。
- `/v1/chat/completions`、`/chat-moe/completions` 和平台 Run 最终都由 8102 的同一套 MoE 执行。
- 已删除 Plan-and-Solve、旧 Multi-Turn Agent、公开 `/tools/*`、`/agent/plan_and_solve`、旧 `/chat`、旧 `/admin` 和旧 `/sessions`。
- `agent_api/app/tools/` 仍然保留，因为它是 MoE 内部 RAG/MCP 工具注册与调度层，不是公开工具 API。

## 启动

开发单进程：

```bash
cd agentAndRag
python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8002
```

生产：

```bash
export AGENT_PLATFORM_ENV=production
export AGENT_PORT=8002
export AGENT_WORKER_HOST=127.0.0.1
export AGENT_WORKER_PORT=8102
export AGENT_WORKER_URL=http://127.0.0.1:8102
export AGENT_WORKER_TOKEN='replace-with-at-least-32-random-bytes'
bash start_agent.sh cuda
```

启动器同时运行：

- `agent_api.app.main:app`：8002 Gateway。
- `agent_api.scripts.run_platform_worker`：8102 Worker HTTP 与 Redis Run consumer。
- `memory_service.app.main`：8300 Memory Service。

## Agent API

### `GET /v1/models`

只返回：

```json
{
  "object": "list",
  "data": [{"id": "agent-moe", "object": "model", "owned_by": "petmind"}]
}
```

### `POST /v1/chat/completions`

请求：

```json
{
  "model": "agent-moe",
  "stream": true,
  "temperature": 0.3,
  "max_tokens": 2500,
  "user_role": "veterinarian",
  "messages": [
    {"role": "user", "content": "猫频繁进出猫砂盆，如何排急症？"}
  ]
}
```

`model` 只能是 `agent-moe`。旧模型名会返回 422，不再静默映射到其他管线。

常用扩展字段：

| 字段 | 说明 |
| --- | --- |
| `user_role` | `pet_owner` 或 `veterinarian` |
| `animal_id` | 限定宠物数据工具的请求作用域 |
| `pethealth_server` | 外部心率异常核实上下文 |
| `tools` | 请求级可用工具白名单；`[]` 禁用工具 |
| `tool_choice` | `auto`、`none` 或指定工具 |
| `memory_session_id` | 上游会话 ID，仅用于记忆元数据 |
| `memory_turn_id` | 记忆写入幂等键 |

流式响应遵循 OpenAI SSE chunk，并通过 `agent_status/agent_detail` 暴露脱敏阶段。最终只流式输出 Aggregator 答案，不暴露系统提示词或内部推理。

### `/chat-moe`

保留 MoE 联调控制台及：

- `POST /chat-moe/sessions`
- `POST /chat-moe/completions`

该入口持有浏览器测试 session，并持久化专家上下文；生产网页的正式会话仍使用 `/api/v1/conversations` 与平台 Run API。

## 平台 API

8002 还提供生产前端需要的 `/api/v1`：

- 邀请、登录、Refresh Token、API Key、RBAC。
- 会话、消息、全文搜索。
- SSE、同步和异步 Agent Run。
- 套餐、订阅、订单、积分账本。
- 用户、订单、Run、审计等后台管理接口。

平台 Run 先写 PostgreSQL并预占积分，再写 Redis 队列；8102 Worker 领取任务、运行 MoE、持久化事件和终答并结算积分。

## 内部工具

MoE 通过 Tool Registry 使用内部能力：

- `rag.search`
- `sql.search`
- `vitals.summary`
- `mcp.vitals_alert.check_vitals`
- `mcp.web_search.web_search`
- `mcp.web_search.ingredient_check`
- 营养与运动 MCP 工具

这些工具不提供独立 HTTP 直调端点。索引导入使用受控的 integration/ingest 流程或离线 ingest 脚本。

## 健康检查

```bash
curl -fsS http://127.0.0.1:8102/health
curl -fsS http://127.0.0.1:8102/ready
curl -fsS http://127.0.0.1:8002/ready
curl -fsS http://127.0.0.1:8300/health
```

生产 8002 的 `/ready` 会同时探测 8102；Worker 不可用时 Gateway readiness 返回 503。

## 安全边界

- 8102 仅允许 8002 使用 `AGENT_WORKER_TOKEN` 调用。
- 正式用户身份来自 JWT 或数据库 API Key，不信任 body 中的内部用户 ID。
- 浏览器不直接访问 8102/8300。
- 正式环境关闭公开 OpenAPI 文档并限制 CORS、Host 和代理头。
- LLM、Web Search、数据库和 Worker 密钥只放 `.env` 或服务环境，禁止提交。

## 回归

```bash
python -m pytest agent_api/tests/moe agent_api/tests/platform agent_api/tests/mcp_servers -q
python agent_api/scripts/test_petmind_moe.py --base http://127.0.0.1:8002
```

检查路由收敛时，OpenAPI 应包含 `/v1/chat/completions`，且不应出现 `/tools`、`/agent/plan_and_solve`、`/chat`、`/admin` 或顶层 `/sessions`。
