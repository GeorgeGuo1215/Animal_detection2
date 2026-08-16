# PetMind MoE 统一任务策略真实回归报告（2026-08-16）

## 结论

统一任务策略已通过 24 个真实 DeepSeek API 混合病例回归。D1～D8 各 3 题，意图、输出变体、领域格式、检索要求和工具执行均为 24/24 通过，可以替代旧的“关键词规则 + 独立意图分类器 + Router LLM”链路。

## 测试设计

- 每个领域包含 1 个无需检索、1 个本地知识/药学资料检索、1 个当前资料 Web Search 场景；D6 因专业资料核验需要没有设置纯无工具题。
- 混合问题允许产生 secondary intents，但主交付结构必须严格服从 primary intent。
- 必须检索的题目校验 `required_tools`、`attempted_tools`、`successful_tools` 和真实检索结果；无须检索的题目校验不会被规则强制调用工具。
- D1～D8 分别校验 SOAP/问题列表、鉴别诊断、检查规划、报告解读、治疗用药、专业快答、多轮管理和安全边界的标题、顺序与关键语义。

## 结果

| 指标 | 结果 |
| --- | ---: |
| 真实病例 | 24 |
| 分类与输出契约通过 | 24/24 |
| RAG 命中病例 | 10 |
| Web Search 成功病例 | 8 |
| 正常无工具病例 | 7 |
| 总 Token | 449,000 |
| 平均全链路时延 | 25.27 秒 |
| 最小时延 / 最大时延 | 13.71 / 35.42 秒 |

工具调用只由统一策略的证据任务触发：`rag.search` 成功 10 次，`mcp.web_search.web_search` 成功 8 次。没有出现“所有专家必须先检索才能 final”的锁死行为。

## 分领域统计

| 领域 | 题数 | Token | RAG | Web | 无工具 |
| --- | ---: | ---: | ---: | ---: | ---: |
| D1 | 3 | 45,146 | 1 | 1 | 1 |
| D2 | 3 | 50,811 | 1 | 1 | 1 |
| D3 | 3 | 48,684 | 1 | 1 | 1 |
| D4 | 3 | 53,117 | 1 | 1 | 1 |
| D5 | 3 | 65,065 | 2 | 1 | 1 |
| D6 | 3 | 59,837 | 2 | 1 | 0 |
| D7 | 3 | 66,944 | 1 | 1 | 1 |
| D8 | 3 | 59,396 | 1 | 1 | 1 |

## 自动化回归

- MoE + MCP：203 passed。
- 可复现脚本：`agent_api/tests/moe/run_unified_policy_regression_live.py`。
- 原始答复和逐题断言：`agent_api/tests/moe/reports/unified_policy_regression_full_final_20260816/`（本地审计目录，不提交 Git）。

## `/chat-moe` 与 `/chat` 真实对接

在本机启动真实后端并连接 DeepSeek API 后完成入口级联调：

| 检查项 | 结果 |
| --- | --- |
| `GET /chat` 与页面内 `/v1/chat/completions` 对接 | 200，通过 |
| `GET /chat-moe` 与页面内 `/chat-moe/completions` 对接 | 200，通过 |
| `/chat-moe/sessions` 创建 SQLite 会话 | 通过 |
| `/chat-moe` 第一轮完成并提交 | 694 字，正常 stop |
| 同一会话第二轮恢复 | 加载 1 个完整轮次、1 个专家上下文轮次 |
| 历史语义验证 | 正确回忆第一轮猫名“豆包” |
| `/v1/chat/completions` API Key + SSE | 229 字，正常 stop |
| error / busy 事件 | 0 |

部署时需允许后端访问上游 LLM；如操作系统配置了代理但代理不可用，可设置 `HTTPX_TRUST_ENV=0` 使用直连。该项只影响后端到模型服务的出站连接，不影响两个页面自身的路由对接。

## 架构确认

生产链路固定为：统一任务策略 LLM → 确定性门控 → 专家及按需工具 → Critic → Aggregator。旧意图分类器、旧 Router LLM、关键词检索决策及其提示词已经移除；D1～D8 边界与 `routing_guidance` 继续作为统一策略的完整输入。
