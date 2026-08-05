# PetHealthAI Agent API

基于 FastAPI 的本地 Agent 服务，集成 RAG 知识库检索、MCP 工具扩展（比价 / 成分分析 / 营养运动计划），支持 OpenAI-compatible 流式接口、Plan-and-Solve 与多轮决策两种 Agent 架构。

## 架构总览

```
用户 (Web / curl / n8n)
  │  POST /v1/chat/completions (SSE stream)
  ▼
FastAPI 中间件链
  ├─ RateLimitMiddleware   (令牌桶限流, 30 req/min)
  └─ APIKeyAuthMiddleware  (Bearer token 校验)
  ▼
routes_openai.py
  ├─ agent-plan-solve   → 单轮: Plan → Tool Calls → Answer
  └─ agent-multi-turn   → 多轮: 循环决策 (最多5轮 tool call)
       │
      ├─ rag.search                          (内置, RAG 检索)
      ├─ mcp.web_search.web_search           (MCP, Tavily 实时网络搜索)
      ├─ mcp.web_search.ingredient_check     (MCP, 成分禁忌分析)
      ├─ mcp.nutritional_planner.calculate_meal_plan   (MCP, 热量/喂食计算)
      └─ mcp.nutritional_planner.generate_exercise_plan (MCP, 运动建议)
```

## 快速开始

### 1. 安装依赖

```bash
cd agentAndRag
pip install -r agent_api/requirements.txt
pip install -r RAG/requirements.txt
```

### 2. 配置环境变量

```bash
# LLM API (必填)
set OPENAI_BASE_URL=https://api.deepseek.com
set OPENAI_API_KEY=sk-your-key
set OPENAI_MODEL=deepseek-chat

# 认证 (可选, 默认从 agent_api/keys.txt 读取)
# set AGENT_DISABLE_AUTH=1
```

### 3. 启动服务

```bash
python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000
```

启动时自动执行：
- 加载 API Keys (`keys.txt`)
- 注册内置工具 (rag.search, rag.reindex, debug.echo)
- 注册 MCP 工具 (price_watcher, nutritional_planner)
- **预热 RAG 缓存** (向量库 / Embedder / BM25 倒排索引 / Reranker / SourceIndex)

健康检查：

```bash
curl http://127.0.0.1:8000/health
```

### 4. HTTPS 启动 (可选)

```bash
set AGENT_SSL_CERTFILE=fullchain.pem
set AGENT_SSL_KEYFILE=privkey.pem
python -m agent_api.app.serve
```

## API 接口

### `POST /v1/chat/completions` — OpenAI-compatible 聊天 (推荐入口)

完全兼容 OpenAI Chat Completions API 格式，支持流式 SSE 和非流式两种模式。

#### 请求格式

```json
{
  "model": "agent-multi-turn",
  "stream": true,
  "temperature": 0.2,
  "max_tokens": 768,
  "messages": [
    {"role": "system", "content": "你是一个宠物健康助手"},
    {"role": "user", "content": "我的狗12kg，今天吃了200kcal，术后恢复期，帮我算一下还需要喂多少"}
  ],
  "tools": null
}
```

#### 请求参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `model` | string | 否 | `agent-plan-solve` | `agent-plan-solve` (单轮 Plan→Solve) 或 `agent-multi-turn` (多轮决策循环) |
| `messages` | array | 是 | - | OpenAI 标准 messages，支持 `system` / `user` / `assistant` role |
| `stream` | bool | 否 | `false` | `true` 返回 SSE 流，`false` 返回完整 JSON |
| `temperature` | float | 否 | `0.2` | LLM 温度 |
| `max_tokens` | int | 否 | `768` | 最终回答最大 token 数 |
| `tools` | array | 否 | `null` | 限制可用工具列表（OpenAI function 格式），`null` 使用默认全部工具 |
| `tool_choice` | string | 否 | `auto` | `auto` / `none` |

#### 认证

请求头必须携带 Agent API Key（来自 `keys.txt`）：

```
Authorization: Bearer sk-pethealthai-default-key-2026
```

#### 流式响应 (stream=true)

返回 SSE 事件流，每个 chunk 格式：

```json
{
  "id": "chatcmpl-...",
  "object": "chat.completion.chunk",
  "model": "agent-multi-turn",
  "choices": [{"delta": {"content": "..."}, "finish_reason": null}],
  "agent_status": "thinking",
  "agent_detail": {"message": "思考中...", "round": 1}
}
```

`agent_status` 状态流转：`thinking` → `tool_calling` → `tool_complete` → `generating` → `streaming` → `stop`

#### 非流式响应 (stream=false)

```json
{
  "id": "chatcmpl-...",
  "object": "chat.completion",
  "model": "agent-plan-solve",
  "choices": [{"message": {"role": "assistant", "content": "..."}, "finish_reason": "stop"}],
  "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
  "plan": [{"type": "tool", "tool_name": "rag.search", "arguments": {...}}, {"type": "final"}],
  "tool_results": [{"step": 0, "tool_name": "rag.search", "result": {...}}]
}
```

非流式模式额外返回 `plan`（Agent 执行计划）和 `tool_results`（各步骤工具调用结果）。

#### curl 示例

```bash
curl -X POST http://127.0.0.1:8000/v1/chat/completions ^
  -H "Content-Type: application/json" ^
  -H "Authorization: Bearer sk-pethealthai-default-key-2026" ^
  -d "{\"model\":\"agent-multi-turn\",\"stream\":true,\"messages\":[{\"role\":\"user\",\"content\":\"我的狗12kg，今天吃了200kcal，术后恢复期，帮我算一下还需要喂多少\"}]}"
```

#### `GET /v1/models`

列出可用模型：

```bash
curl http://127.0.0.1:8000/v1/models -H "Authorization: Bearer sk-pethealthai-default-key-2026"
```

### Plan-and-Solve Agent (REST)

```bash
curl -X POST http://127.0.0.1:8000/agent/plan_and_solve ^
  -H "Content-Type: application/json" ^
  -H "Authorization: Bearer sk-pethealthai-default-key-2026" ^
  -d "{\"query\":\"犬猫腹泻常见原因有哪些？\",\"temperature\":0.2,\"max_tokens\":800}"
```

### RAG 检索

```bash
curl -X POST http://127.0.0.1:8000/tools/rag/search ^
  -H "Content-Type: application/json" ^
  -H "Authorization: Bearer sk-pethealthai-default-key-2026" ^
  -d "{\"query\":\"What is BRDC?\",\"top_k\":5,\"multi_route\":true,\"rerank\":true,\"expand_neighbors\":1}"
```

### 重建索引

```bash
curl -X POST http://127.0.0.1:8000/tools/rag/reindex ^
  -H "Content-Type: application/json" ^
  -H "Authorization: Bearer sk-pethealthai-default-key-2026" ^
  -d "{\"batch_size\":32}"
```

### 工具列表 / 通用调用

```bash
# 列出所有已注册工具
curl http://127.0.0.1:8000/tools -H "Authorization: Bearer sk-pethealthai-default-key-2026"

# 通用工具调用
curl -X POST http://127.0.0.1:8000/tools/call ^
  -H "Content-Type: application/json" ^
  -H "Authorization: Bearer sk-pethealthai-default-key-2026" ^
  -d "{\"tool_name\":\"rag.search\",\"arguments\":{\"query\":\"canine parvovirus\",\"top_k\":3}}"
```

### 会话管理

```bash
# 创建会话
curl -X POST http://127.0.0.1:8000/sessions -H "Authorization: Bearer ..."
# 查询会话
curl http://127.0.0.1:8000/sessions/{session_id} -H "Authorization: Bearer ..."
# 删除会话
curl -X DELETE http://127.0.0.1:8000/sessions/{session_id} -H "Authorization: Bearer ..."
```

## MCP 工具扩展

配置文件：`agent_api/mcp_servers.json`

```json
{
  "servers": [
    {
      "name": "price_watcher",
      "transport": "stdio",
      "command": "python",
      "args": ["-m", "mcp_servers.price_watcher_pro"],
      "enabled": true
    },
    {
      "name": "nutritional_planner",
      "transport": "stdio",
      "command": "python",
      "args": ["-m", "mcp_servers.nutritional_planner"],
      "enabled": true
    }
  ]
}
```

MCP 工具以 `mcp.{server}.{tool}` 命名注册到 ToolRegistry，Agent 自动识别调用。

### Price_Watcher_Pro

| 工具 | 功能 |
|------|------|
| `price_compare` | 跨平台比价 (DuckDuckGo + LLM 提取结构化价格) |
| `ingredient_check` | 成分禁忌分析 (匹配健康状况 → 返回冲突或 `INSUFFICIENT_DATA`) |

### Nutritional_Planner

| 工具 | 功能 |
|------|------|
| `calculate_meal_plan` | 计算 RER/MER、热量平衡、下一餐克数 (支持 `FEEDING_INQUIRY_NEEDED` / `OVERFED_WARNING`) |
| `generate_exercise_plan` | 根据热量差和医嘱生成运动建议 |

## 环境变量参考

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `OPENAI_BASE_URL` | `https://api.deepseek.com` | LLM API 地址 |
| `OPENAI_API_KEY` | - | LLM API Key |
| `OPENAI_MODEL` | `deepseek-chat` | 模型名 |
| `AGENT_DISABLE_AUTH` | `0` | 设为 `1` 跳过 API Key 校验 |
| `AGENT_ENABLE_MCP` | `1` | 设为 `0` 禁用 MCP 工具 |
| `AGENT_ENABLE_CORS` | `0` | 设为 `1` 开启 CORS |
| `AGENT_RATE_LIMIT` | `30` | 每分钟请求上限 (per key) |
| `AGENT_WARMUP_RAG` | `1` | 启动时预热 RAG 缓存 |
| `AGENT_WARMUP_BM25` | `1` | 预热 BM25 倒排索引 |
| `AGENT_WARMUP_RERANKER` | `1` | 预热 CrossEncoder 重排模型 |
| `AGENT_WARMUP_DEVICE` | `cpu` | 预热设备 (`cpu` / `cuda`) |
| `AGENT_TRACE_DIR` | `agent_api_logs/` | Trace 日志目录 |

## RAG 检索性能

166,990 chunks 索引下的基准测试（`RAG/experiments/bench_rag_latency.py`，RTX 3080 Ti）：

| 环节 | CPU | GPU (CUDA) |
|------|-----|------------|
| Query Embedding | ~10ms | ~9ms |
| Dense Search (numpy dot) | ~6ms | ~6ms |
| BM25 Retrieve (倒排索引) | **~0.8ms** | **~0.8ms** |
| Neighbor Context (预建索引) | **~0.02ms** | **~0.02ms** |
| Reranker 10 passages | ~6.5s | **~346ms** |
| Reranker 20 passages | - | **~784ms** |

不含 Reranker 的热路径总延迟：**~17ms**。
含 Reranker (GPU) 全路径：**~362ms**。
重复查询命中 LRU 缓存：**~11ms**。

> GPU 重排加速比：**19×**（6.5s → 346ms）。推荐设置 `AGENT_WARMUP_DEVICE=cuda`。

## 项目结构

```
agentAndRag/
├── agent_api/
│   ├── app/
│   │   ├── main.py              # FastAPI 入口, 启动预热, 路由注册
│   │   ├── routes_openai.py     # /v1/chat/completions (SSE 流式)
│   │   ├── plan_and_solve.py    # Plan-and-Solve / Async Agent 核心
│   │   ├── rag_tools.py         # RAG 检索封装 (缓存/优化层)
│   │   ├── tool_registry.py     # 统一工具注册与派发 (sync/async)
│   │   ├── tools_builtin.py     # 内置工具注册
│   │   ├── tools_mcp.py         # MCP 工具自动发现与注册
│   │   ├── mcp_client.py        # MCP stdio 客户端 (async)
│   │   ├── mcp_config.py        # MCP 服务器配置加载
│   │   ├── llm_client.py        # LLM 客户端 (sync + async, 连接池)
│   │   ├── llm_client_stream.py # LLM 流式客户端
│   │   ├── auth.py              # API Key 认证中间件
│   │   ├── rate_limit.py        # 令牌桶限流中间件
│   │   ├── session_manager.py   # 会话管理 (TTL, 内存)
│   │   ├── schemas.py           # REST 端点 Pydantic schemas
│   │   └── schemas_openai.py    # OpenAI-compatible schemas
│   ├── mcp_servers.json         # MCP 服务器配置
│   ├── keys.txt                 # API Keys
│   └── requirements.txt
├── mcp_servers/
│   ├── price_watcher_pro/       # 比价 + 成分分析 MCP Server
│   └── nutritional_planner/     # 营养 + 运动计划 MCP Server
└── RAG/
    ├── simple_rag/              # 检索核心 (embeddings, retrieval, reranker, context)
    ├── experiments/             # 评测 + 性能基准脚本
    └── data/                    # 索引数据
```

## Trace 审计

所有请求自动写入 `agent_api_logs/trace.jsonl`，包含 trace_id、工具名、请求参数、响应摘要。可通过 `AGENT_TRACE_DIR` 更改路径。
