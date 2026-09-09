# 回答评价与统一 HTTP 限流：实施与验收

日期：2026-09-09。基线：`7729a6c`。本次完成代码、迁移、测试和本机验收；不部署生产、不实现 RLHF 训练流水线。

## 1. 最终实现

### 单条回答评价

- `platform_messages.feedback_is_good BOOLEAN NULL` 是唯一持久化评价字段。`true / false / NULL` 分别表示好、坏、未评价或取消。取消时评价时间清空。
- 保持 `PUT /api/v1/messages/{message_id}/feedback` 的 `{rating: "up" | "down" | null}`、返回的 `rating`，以及消息列表的 `feedback_rating` 接口兼容。ORM 只提供旧字段的只读转换属性，没有双写。
- 仅会话所有者可以评价已完成的 assistant 消息。消息行 `FOR UPDATE OF platform_messages` 锁贯穿读取、更新、审计和提交；同值请求不更新时间、不新增审计。审计记录前值、后值及旧接口值。
- 新回答、分支复制回答均不继承评价。备份输出 v2；读取 v1 时校验原校验和，再转换旧字符串字段。所有记录先校验再开始覆盖恢复，`false` 不会丢失。
- 工具栏保留复制、赞、踩、分支；支持选中、切换、取消、Tab/空格和焦点轮廓。同步 ref 防重覆盖 React 尚未完成渲染的间隙。429 读取 `Retry-After`、显示剩余秒数、禁用评价，到期后允许手动重试；不自动重放。计时器在卸载时清理。

主要文件：`platform/message_feedback.py`、`platform/models.py`、`platform/user_backup.py`、`routers/routes_platform_conversation_actions.py`，以及前端 `MessageActions.tsx`、`api.ts`、`styles.css`。

### 一个 HTTP 限流服务

删除原 `middleware/rate_limit.py` 与 `middleware/platform_rate_limit.py` 两套实现，合并到 `app/http_limits/`：

| 文件 | 职责 |
|---|---|
| `policies.py` | 配额默认值、配置解析、HTTP 方法与路由模板的显式策略表 |
| `backend.py` | Redis Lua 原子令牌桶/固定窗口、有界开发回退、连接关闭 |
| `service.py` | 稳定身份、生成套餐缓存、组合额度、统一错误和监控 |
| `integration.py` | 鉴权前 IP 保护、鉴权后路由保护，SSE 按请求建连计数 |

请求先消耗 IP 基础额度；鉴权成功后按 `user:<稳定用户ID>` 聚合。换 JWT、IP、同用户数据库 API Key 不重置用户额度。旧静态 key 仅在校验成功后以完整 SHA-256 指纹作为身份；未经验证的凭据不能创建新额度。平台鉴权结果复用到请求状态，认证有效性不跨请求缓存。

只有生成策略读取套餐；套餐缓存最多 1,024 项，30 秒到期。轻量读写不查询套餐。套餐中 `rate_limit_per_minute / rate_limit_burst` 可覆盖生成默认值，原有 1～10,000 范围保留。会员变更最多延迟 30 秒影响生成配额，账号停用和凭据失效下一请求即重新验证。

一个路由所需的主额度、评价额外额度、敏感操作额度由一次 Lua 同时检查并扣减；任意一个不满足，不扣减其他用户子额度。IP 保护位于鉴权前，仍会计数。Lua 使用 Redis 时钟并原子设置到期时间；通过 `register_script` 使用 EVALSHA，并支持缓存丢失后重新加载。开发回退最多 10,000 桶，有 TTL 和淘汰，每秒最多一次有界过期扫描。

生产 Redis 缺失/故障返回 503 和 `Retry-After: 5`；超额返回 429、`Retry-After`、`X-RateLimit-Limit/Remaining` 和请求 ID。平台与旧接口保留各自错误外层格式。管理员原 `/api/v1/admin/rate-limits` 保留旧字段，附加策略、进程计数和缓存大小；复用既有 Redis 客户端。

LLM/RAG/MCP 并发槽保持独立。取消、运行状态、SSE 建连及续传使用 control 策略，生成额度耗尽不占用它们。健康检查、开发文档和 OPTIONS 豁免。可信代理继续由既有 Uvicorn `AGENT_TRUST_PROXY_HEADERS / AGENT_FORWARDED_ALLOW_IPS` 配置管理，限流器不直接信任任意请求头。

## 2. 集中配置

默认值及示例位于 `app/http_limits/policies.py`、`agentAndRag/.env.platform.example`。

| 策略 | RATE（每分钟补充量） | BURST（最大积累令牌数） |
|---|---:|---:|
| IP | 600 | 120 |
| AUTH | 60 | 20 |
| READ | 300 | 60 |
| WRITE | 120 | 20 |
| FEEDBACK（同时消耗 WRITE） | 30 | 6 |
| GENERATION | 60，套餐可覆盖 | 20，套餐可覆盖 |
| CONTROL | 120 | 20 |
| LEGACY（非生成旧路由） | 30 | 30 |

配置名为 `AGENT_HTTP_<策略>_RATE` 和 `AGENT_HTTP_<策略>_BURST`，必须为正整数。它们描述令牌补充速度，不是“任意连续60秒绝对不超过 RATE 次”的滑动窗口。

`AGENT_PLATFORM_RATE_LIMIT / AGENT_PLATFORM_RATE_BURST` 是未设置新生成配置时的兼容默认值；`AGENT_RATE_LIMIT / AGENT_RATE_BURST` 对应旧路由组。显式新配置优先。所有实例必须使用相同 Redis、`AGENT_HTTP_RATE_PREFIX` 和策略配置。

原敏感操作附加固定窗口保持：兑换激活码用户 5/600秒、IP 20/600秒；删除单条记忆用户 5/60秒、IP 30/60秒；清空记忆用户 6/3600秒、IP 20/3600秒。不同消息 ID 不产生独立评价额度。

当前后端针对单 Redis 主节点部署；未实现 Redis Cluster 跨槽脚本支持。监控计数按进程，实际额度在 Redis 共享。新增 HTTP 路由必须注册策略，测试和安装过程都会拒绝漏配置。

## 3. 数据库迁移与回退

新版本 `20260909_0006`，前置版本 `20260905_0005`。在目标环境配置数据库 URL 后执行：

```powershell
python -m alembic -c agentAndRag/agent_api/alembic.ini upgrade head
# 回退本次数据库结构；随后必须运行对应旧版应用代码
python -m alembic -c agentAndRag/agent_api/alembic.ini downgrade 20260905_0005
```

升级先检查全部历史字符串。存在非 `up/down/NULL` 值时在 DDL 前拒绝，需查明并修正数据再重试；不会将异常值静默变成 NULL。正常升级保留 `feedback_updated_at`，转换后删除字符串列。降级将布尔值反向转换，并移除布尔列。

旧初始迁移使用当前 ORM metadata 建表，因此空库升级时可能已存在空布尔列；新迁移兼容此情况。若混合结构中已存在非空布尔评价，则拒绝覆盖，要求先核对来源。完整空库升级、降级本次版本、再次升级已在真实 PostgreSQL 验证。

上线前应先备份、暂停写入，再迁移并启动匹配代码；不可让新旧 ORM 同时写同一个已切换结构。v2 快照由新代码恢复，回退旧代码时不能直接交给旧版 v1 恢复器。本次未连接或修改生产数据库。

## 4. 自动化验收

| 验证项 | 结果 |
|---|---|
| platform、concurrency、architecture 后端测试（启用真实 PG/Redis 及双进程） | **81 通过**；4 条已有 FastAPI on_event 弃用警告 |
| 前端 Vitest | **24 通过**，6 个测试文件 |
| 前端已有 Playwright 回归 | **33 通过、3 按视口条件跳过** |
| ESLint、TypeScript、Vite 生产构建 | **通过** |
| PostgreSQL 完整迁移链 | **通过** |

新增覆盖包括三态严格转换、旧备份非空评价恢复、并发相同/不同评价与审计、归属和未完成回答拒绝、分页回读、账号停用即时生效、固定窗口和令牌补充、组合额度、套餐缓存上限/到期、路由覆盖、SSE建连/续传计数、真实 Redis TTL/连接关闭、两进程共享身份和额度。

持有全部 LLM/RAG/MCP 资源槽时，经真实 PostgreSQL/Redis 的 HTTP 评价和取消仍成功，资源计数未增加；测试同时禁止调用模型编排器。真实 Redis 容器停止后两个 API 均返回 503，health 均为 200。

为稳定重复验收修复了旧测试的两个问题：分页测试固定主键跨轮冲突；取消测试依赖50ms睡眠，改为等待编排器进入执行的事件。跨进程读取额度测试改用专用低额度，避免默认补充速度使顺序请求无法稳定达到边界。

### 复现命令

从仓库根目录执行，先选择独立测试密码，不使用生产连接串：

```powershell
$env:PYTHONPATH="$PWD/agentAndRag"
$env:PETMIND_TEST_DB_PASSWORD='<本次独立测试密码>'
docker compose -p petmind-feedback-test -f agentAndRag/agent_api/tests/platform/feedback.compose.yml up -d --wait
$env:PETMIND_TEST_POSTGRES_URL="postgresql+asyncpg://feedback_test:$($env:PETMIND_TEST_DB_PASSWORD)@127.0.0.1:15439/feedback_test"
$env:PETMIND_TEST_REDIS_URL='redis://127.0.0.1:16389/0'
python -m pytest agentAndRag/agent_api/tests/platform agentAndRag/agent_api/tests/concurrency agentAndRag/agent_api/tests/architecture -q --basetemp=.test-tmp/feedback-check
```

双进程测试另需两个运行中的 API。二者连接同一独立数据库/Redis/限流前缀，测试配置 `FEEDBACK=30/2`、`READ=1/2`；通过 `PETMIND_TEST_API_URLS=http://127.0.0.1:18092,http://127.0.0.1:18093`、`PETMIND_TEST_PROCESS_DB_URL=<该测试库URL>` 指定。然后运行 `test_http_process_integration.py`。本次全部81项的最终日志已启用这些变量，没有以跳过真实测试代替验收。

```powershell
cd petmindAgentFrontend
pnpm test
pnpm lint
pnpm build
pnpm test:e2e
```

手工浏览器数据可通过 `tests/platform/feedback_browser_fixture.py seed` 重建，需设置 `AGENT_PLATFORM_DB_URL`、`AGENT_PLATFORM_REDIS_URL`、`PETMIND_TEST_BROWSER_PASSWORD`。在迁移并配置完测试库后运行；输出两账号及会话 ID，首账号144条消息。`worker` 子命令提供合成流式模型适配器，仅供本机验收，不导入生产应用。真实 API 可用 `python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 18092` 启动，第二实例使用18093。关闭测试环境的 RAG预热/MCP/记忆，分别设置 `AGENT_WARMUP_RAG=0`、`AGENT_ENABLE_MCP=0`、`AGENT_MEMORY_ENABLED=0`，并为每实例设置独立 `QA_DB_PATH`。

## 5. 实机浏览器矩阵

环境：Edge；生产 `dist` 静态构建；真实后端、PostgreSQL 16、Redis 7；前端回环14179，API18092/18093，数据库15439，Redis16389。使用专用账号与合成内容。浏览器阶段仅将评价突发容量从默认6降为2，速率仍30/分钟；双进程测试阶段另将读取改为1/分钟、突发2。均为临时进程配置，默认产品配置未降低。

| 计划项 | 最终状态 | 实操与核对 |
|---|---|---|
| 1 登录、已有回答及操作位置 | 通过 | 登录专用账号；复制/赞/踩/分支顺序及提示，见01 |
| 2 点赞刷新、离开返回 | 通过 | 刷新、第二标签页重新读取；`true`与选中一致 |
| 3 点踩、再次取消 | 通过 | `false`及新时间；取消后布尔/时间均NULL，审计保留 |
| 4 快速连续点击 | 通过 | 双击和快速切换；无重复同值审计；单位测试另覆盖同一渲染周期防重 |
| 5 评价限流 | 通过 | 第三次快速评价真实429；禁用与倒计时；到期手动提交成功，见02 |
| 6 服务中断与恢复 | 通过 | 停止专用API；错误提示、选中不变；重启后再次保存成功，见03 |
| 7 两账号、两标签页 | 通过 | 第二账号列表仅自身会话；直接访问他人URL显示not found；双标签重新读取一致，见09 |
| 8 长历史虚拟列表 | 通过 | 加载至144条、`data-virtualized=true`；评价后滚至约第60条再返回144条，刷新仍保持，见06 |
| 9 桌面/移动/键盘及相邻功能 | 通过 | 390×844移动视口实际点踩；Tab/空格；复制成功；分支评价为空；重写完成和流式停止，见04/05/07/08 |

浏览器矩阵无未执行项。首次合成 Worker 缺少 `last_finish_reason` 导致重写失败，补齐后重测完成和取消路径；该失败记录保留在证据中，不隐去。模型输出来自合成适配器；Run队列、持久化、计费、SSE、取消及前端均为真实实现，本次不声称进行了真实大模型效果评测。

## 6. 性能对比

Windows同机、真实 PostgreSQL/Redis，完整应用 ASGI 请求链。每场景24次预热、400次测量、并发4；48条合成消息；评价在4条消息上交替好/坏，确保测的是更新，不是同值快捷返回。两版本测量时都将桶配置提高至10,000，避免限流掩盖请求处理开销。基线为修改前完整 `app` 快照。

| 场景 | SQL/请求：基线→本次 | 首轮 p95 ms：基线→本次 | 复测 p95 ms：基线→本次 |
|---|---|---|---|
| 消息读取 | 4→3 | 40.103→33.537 | 34.396→33.936 |
| 评价写入 | 5→4 | 47.163→37.062 | 36.193→31.326 |

两轮均未出现超过5%的p95退化。减少查询主要来自轻量请求不再查询套餐；鉴权只在请求内复用。指标不包含浏览器渲染、公网、TLS或外部模型开销，也不是生产容量承诺。

复现脚本 `tests/platform/benchmark_http_limits.py` 接收 `--package agent_api.app --output result.json`。配置独立 `AGENT_PLATFORM_DB_URL/REDIS_URL`。基线使用另一空测试库，将指定基线的原始 `app` 目录放在可导入的 `baseline_app` 包路径，通过 `--package baseline_app` 运行。脚本不会调用模型，结束后清除自己创建的用户/数据和本次限流命名空间。

## 7. 证据、清理与后续方向

证据目录：[feedback-20260909](../docs/acceptance/feedback-20260909/)。包含9张关键截图、数据库三态/审计核对、Run与分支核对、Redis故障响应、测试和构建日志、迁移链以及两轮性能JSON。不保存密码、JWT或API Key。证据沿用仓库现有规则保存在本地忽略目录；本实施文档与测试源码可纳入版本管理。

本次专用测试账号、会话、API Key、快照数据、Redis键及临时配置随专用容器卷/验收进程清理；清理结果另见 `cleanup.json`。没有连接生产数据库或部署服务。

后续建议：

1. 在部署入口串行执行迁移/初始化，并将遗留QA审计从共享SQLite迁移到统一存储；本次发现两个API同时首次初始化同一QA文件可能出现database locked，验收使用独立文件绕开，业务PG与限流Redis仍共享。
2. 将新增限流监控计数接入现有监控系统，观察各策略429/503与生成套餐命中率；需要Redis Cluster时再设计同槽键和敏感IP额度的一致性方案。
3. 冻结旧初始Alembic迁移的schema定义，逐步替换动态引用当前ORM metadata的历史做法；迁移和回退更可预测。
4. 后续建设评价导出和样本审核时，关联问题、答案版本、模型/提示词版本与证据来源，再进行匿名化、去重和质量复核；单个赞踩不能直接当作可靠训练标签。本次不扩展训练流水线。

设计依据：[Redis Lua原子执行与脚本缓存](https://redis.io/docs/latest/develop/programmability/eval-intro/)；[PostgreSQL行级锁](https://www.postgresql.org/docs/current/explicit-locking.html)。具体行为以本次代码与真实集成证据为准。
