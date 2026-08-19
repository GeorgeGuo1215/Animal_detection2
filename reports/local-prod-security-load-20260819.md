# PetMind 本地生产拓扑安全与容量评估

- 时间：2026-08-19（Asia/Shanghai）
- 范围：本机授权评估，仅 `127.0.0.1` / 本机 Docker
- 拓扑：Nginx `:5173`（`pnpm build` dist）→ Gateway `:8002` → Worker `:8102`（CUDA）+ Memory `:8300` + PG `:15433` + Redis `:16379` + Memory PG `:5432`
- 方法：防御性探测 + 有界阶梯压测。未编写利用代码或洪水工具。
- 原始数据：`reports/local-prod-assess-raw.json`、`reports/local-prod-load.json`、`reports/_load_probe.py`（评估后可回收）

## 1. 部署快照

| 项 | 结果 |
| --- | --- |
| GPU | RTX 3080 Ti Laptop 16GB。启动前 Cursor 占用约 175MiB / 18%。Worker 热加载后约 3019MiB。双 Run 后 3321MiB / 27% |
| Docker | 评估开始时引擎未运行，已拉起 Desktop。`petmind-nginx`、platform PG/Redis、`petmind-memory-pg`（pgvector） |
| 配置偏差 | 仓库无现成 `agentAndRag/.env`，按 production 生成（gitignored）。本地 HTTP 将 `COOKIE_SECURE=0`。Gateway `AGENT_HOST=0.0.0.0` 以便容器反代 |
| Alembic | `upgrade head` 成功 |
| 管理员 | `admin@local.test` / SUPER_ADMIN（密码仅在本机 `.env`） |

就绪探针（启动完成后）：

| URL | 结果 |
| --- | --- |
| `8300/health` | 200 |
| `8102/health`、`8102/ready` | 200，`queue_consumer=true`，RAG 111244 chunks / 43 categories / reranker |
| `8002/health`、`8002/ready` | 200，`role=gateway` |
| `5173/healthz`、`5173/ready`、`5173/chat` | 200，静态资源带内容哈希 |

监听地址：

| 端口 | 绑定 | 含义 |
| --- | --- | --- |
| 5173 | `0.0.0.0` | Docker Nginx，预期对外入口 |
| 8002 | `0.0.0.0` | Gateway，供 `host.docker.internal` 反代 |
| 8102 | `127.0.0.1` | Worker，符合规格 |
| 8300 | `127.0.0.1` | Memory，符合规格 |
| 15433 / 16379 / 5432 | `0.0.0.0` | 数据面误暴露 |

## 2. 安全发现摘要

生产 CORS 白名单、TrustedHost、JWT 登录、Admin 无 token 401、OpenAPI 在 Gateway 关闭，这些是有效控制。主要缺口在**数据面绑定、限流身份、以及仍匿名开放的开发入口**。

### 2.1 2026-08-20 代码复核与整改状态

本节是对8月19日现场快照的增量复核；原问题表保留用于追踪，不代表当前代码仍存在全部缺口。

| 项 | 当前状态 |
| --- | --- |
| P01 / 平台侧 P02 | 已整改：仓库 compose 将 Redis、平台 PostgreSQL 映射限定为 `127.0.0.1`；Memory PostgreSQL 仍需在服务器编排和防火墙中同样核验 |
| P03 | 已整改：Uvicorn 只接受 `AGENT_FORWARDED_ALLOW_IPS` 中的可信反代头，限流键同时包含连接 IP 与认证主体 |
| P04 | 已整改：生产 `/chat-moe*` 要求 `SUPER_ADMIN` JWT 或内部 Worker Token，并继续转发到8102 |
| P05 | 已整改：`/integration`、n8n Webhook及相关依赖已从应用和仓库删除 |
| P07 | 已整改：生产 Gateway `/ready` 仅返回 `{"ready": boolean}` |
| P09 | 已整改：Nginx统一加入 CSP、frame、nosniff、Referrer、Permissions、COOP和HSTS头 |
| P12 / P17 | 代码已强制生产 Secure Cookie 和 HTTPS Origin；证书、443监听及公网防火墙仍属于部署必做项 |
| P14 | 已整改：关闭兼容开关后不再读取或载入 `keys.txt/AGENT_API_KEYS` |
| P18 | 已整改：确认 Refresh 重放时递增用户 `token_version`，现有 Access JWT随即失效 |
| 新增授权复核 | 已整改：数据库 API Key不能访问`/api/v1`个人/记忆/后台接口；调用方不能用`X-User-Id`或body伪造记忆主体；SUPPORT_ADMIN不能邀请管理员或付费套餐 |

仍需部署侧确认：公网只开放5173/443，8002只供反代访问，8102/8300/数据库/Redis不可公网路由；正式域名证书启用后再开启真实业务流量。

## 3. 容量拐点

控制面阶梯（每档一次有界并发波，非持续洪水）：

| 并发 | Nginx `/healthz` | Gateway `/health` | `/api/v1/me`（期望 401） | `/api/v1/plans` via 5173 |
| --- | --- | --- | --- | --- |
| 10 | 200×10，p50 40.3s | 200×10，p50 43.0s | 401×3 / 429×7 | 200×6 / 429×4 |
| 50 | 200×50，p50 29.9s | 200×50，p50 84.9s | **502×50** | **502×50** |
| 100 | 200×100，p50 83.2s | 200×100，p50 39.8s | 401×36 / 429×64 | 200×41 / 429×59 |
| 200 | 200×200，p50 29.5s | 199×200 + 1×502 | 401×108 / 429×80 / 503×9 / 502×3 | 200×86 / 429×114 |

解读：

- 平台令牌桶有效：对 `/api/v1/plans` 串行探测，**第 21 次出现 429**，与 `RATE_BURST=20` 一致。
- `X-Forwarded-For` 换成 40 个不同地址后 **40/40 均为 200**，限流身份被拆开。
- Nginx 静态 `/healthz` 在 200 并发下仍全 200，但延迟已到数十秒量级；业务 API 在并发 50 时出现整波 502。
- 单机控制面饱和点：**业务 API 约 50 并发开始失败**；限流在此之前就会 429。有效吞吐约 2 rps 量级（含排队）。

执行面（两路同时 SSE，单 GPU Worker 规格并发 1）：

| Run | 首事件 | 结束 | 耗时 |
| --- | --- | --- | --- |
| a | 60ms | completed | 26.6s |
| b | 61ms | completed | 51.1s |

两路都完成，第二路近似排队多等一轮。显存 3028MiB → 3321MiB，未顶满 16GB。Gateway `/ready` 在双 Run 期间仍 200。

## 4. 问题清单

| ID | 级别 | 资产 | 现象与复现 | 影响 | 建议 |
| --- | --- | --- | --- | --- | --- |
| P01 | Critical | Redis `:16379` | `0.0.0.0:16379`，`redis-cli ping` → `PONG`，无密码 | 队列、限流状态可被局域网读写 | 绑定 127.0.0.1 或 Docker 内网；`requirepass`；防火墙 |
| P02 | Critical | PG `:15433`、Memory PG `:5432` | 均 `0.0.0.0`，compose 默认口令即可 `select 1` | 用户、JWT 族、积分、记忆可被拖库 | 改随机口令；只映射到 localhost；拆开 Memory/平台凭据 |
| P03 | High | Gateway 限流 | `TRUST_PROXY_HEADERS=1` 下 40 个不同 `X-Forwarded-For` 全部 200 | 绕过 60/min 桶，放大控制面负载 | 生产仅信任已知反代；或忽略来自容器网段的伪造 XFF；限流键加入真实套接字 IP |
| P04 | High | `:8002` `/chat-moe*` | 无认证 GET `/chat-moe` 200（含 jsDelivr 脚本）；空 POST `/sessions`、`/completions` 为 422 而非 401 | 任何人可走测试会诊入口打到 8102 MoE | production 卸掉 `chat_moe_router` 或强制 JWT |
| P05 | High | `:8002` `/integration/ingest` | 无认证 POST `{}` → 422 | 匿名即可打到校验层，有效 payload 可进 n8n/ingest | 生产关闭或要求签名/API Key |
| P06 | Medium | Gateway `:8002` | 监听 `0.0.0.0` | 绕过 Nginx 直连 API、开发页、`/ready` | 仅 Docker 网络可达，或本机防火墙只放行 5173 |
| P07 | Medium | `/ready` | 匿名返回 `index_dir` 绝对路径、Memory URL、资源限额 | 便于侦察 | 对外只保留 `ready: true/false` |
| P08 | Medium | Worker `:8102` | `/health`、`/ready` 无 `x-petmind-worker-token` 也 200。`/v1/*` 为 401 | 回环可缓解；路径与内部拓扑仍泄露 | health 也要求 token，或 Unix socket |
| P09 | Medium | Nginx `:5173` | `/chat` 无 CSP / `X-Frame-Options` / `X-Content-Type-Options` / HSTS | 点击劫持与 MIME sniff | 在 `nginx.conf` 加安全头 |
| P10 | Medium | Nginx↔Gateway | 并发 50 时 `/api/v1/me` 与 `/plans` **整波 502**；200 并发出现 503 | 单机可用性先于 GPU 崩 | 反代超时/连接池；uvicorn workers；把 502 与限流区分 |
| P11 | Medium | Gateway `/ready` | 空载探测约 5.6s（健康检查 20s 窗口） | 编排误判未就绪 | 缓存 worker 探测；缩短依赖链 |
| P12 | Low | Cookie | `HttpOnly` + `SameSite=strict` 有效；`Secure` 因本地 HTTP 关闭 | HTTPS 部署若忘记打开会明文 Cookie | 生产强制 `COOKIE_SECURE=1` |
| P13 | Low | Nginx | `/v1/*` 未反代，落到 SPA `index.html` 200 | 第三方 SDK 若打 5173 会拿到 HTML | 需要兼容时增加 `/v1/` `proxy_pass` |
| P14 | Low | Auth | `AGENT_LEGACY_API_KEYS_ENABLED=0` 仍加载 `keys.txt`（日志 “Loaded 1 API key”） | 遗留密钥面仍存在 | 生产删除 `keys.txt` 或启动期拒绝遗留文件 |
| P15 | Info | SPA | `/docs` `/admin` `/openapi.json` 在 5173 为前端 HTML 200；Gateway `/docs` 404 | 不是 Swagger 泄露 | 可忽略；勿与 API 文档混淆 |
| P16 | Info | `/chat-moe` | 页面加载 `cdn.jsdelivr.net/npm/marked` | 额外供应链 | 随 P04 一并下线 |
| P17 | High | TLS | `https://127.0.0.1:5173` / `:8002` 均为明文 HTTP（SSL WRONG_VERSION_NUMBER）；本机无 `:443`；`nginx.conf` 无证书；响应无 HSTS | 局域网可窃听 JWT / Cookie / 病历 | 终止 TLS 于 Nginx；只对 443 开放；HSTS；`COOKIE_SECURE=1` |
| P18 | Medium | Refresh 族 | 重放旧 refresh → 401 `reuse detected`；新 refresh 随后也 401（族撤销有效）。**旋转后的 access JWT 仍 200** | 窃得的 Access Token 在 15 分钟内仍能用 | 重放时递增 `token_version` 或维护 access jti 黑名单 |

## 6. 补测（2026-08-19 晚）

原始数据：`reports/local-prod-followup.json`、`reports/_followup_probe.py`。

### 6.1 TLS / 证书

未终止 TLS。对 5173/8002 做 TLS 握手得到 `WRONG_VERSION_NUMBER`（对端是 HTTP）。无 443 监听，无 HSTS。这与规格「反向代理启用 TLS」不一致。见 P17。

### 6.2 支付 Webhook HMAC

`POST /api/v1/payments/test-webhook`（无 JWT，只验 HMAC）：

| 用例 | 结果 |
| --- | --- |
| 缺签名 | 401 invalid webhook signature |
| 错误签名 | 401 |
| 时间戳超过 300s | 401 |
| 正确 HMAC + 不存在的 order_id | **404 order not found**（签名已通过） |

HMAC + 5 分钟窗 + `compare_digest` 行为正确。未发现可匿名改订单状态。

### 6.3 Refresh 重放与族撤销

登录 → 旋转得到 cookie B → 重放 cookie A → 401 reuse detected → 再用 cookie B → 401 reuse detected。族撤销在生产库上成立。缺口见 P18：Access JWT 未随族一起作废。

### 6.4 SQL / MCP（无害标记，无注入字符串）

未发送 SQL 拼接、命令注入或 exploit。做法：

- 表名白名单拒绝 `information_schema` / `mysql` / `pg_catalog` / 未登记表
- `LIKE` 条件编译为 `` report_text LIKE %s ``，标记 `PETMIND_CANARY_VALUE` 只出现在参数列表，不进 SQL 文本
- 带着假 `animal_id=PETMIND_CANARY_ANIMAL` 调用 `sql.search`：工具返回 `ok=true, row_count=0`（MySQL 可达，0 行，没有把标记插进语句）
- MCP：`vitals_alert` 使用 `%(pet_id)s` 绑定；`web_search` 把 query 当 JSON 字段发给 Tavily，不是 shell

结论：当前 sql.search 是编译器 + 白名单 + 占位符，假 payload 不会变成可执行 SQL。真攻击载荷没有发，也不应作为回归手段。

### 6.5 跨主机扫描 / SYN 洪水

未对局域网其他主机发包，也未做 SYN/UDP/内核洪水。本机通配绑定仍是 P01/P02/P06/P17：其它机器只要能路由到这台 Windows，就可以打到 Redis/PG/5173/8002。要用防火墙或改 `127.0.0.1` 端口映射，而不是用洪水来验证。

## 7. 结论

本地生产拓扑**可以按文档拉起**：5173 dist + 8002 控制面 + 8102 CUDA Worker。鉴权主路径（JWT、Admin 401、TrustedHost、生产 CORS、Worker `/v1` token）可用。

8月20日代码复核已收口P01（仓库编排）、平台侧P02、P03–P05、P07、P09、P14和P18，并补上 API Key、记忆身份与邀请角色三类授权边界。上线前剩余的硬门槛主要是部署侧：Memory PostgreSQL绑定核验、正式域名TLS/443、防火墙只公开入口端口，以及P10/P11容量与就绪延迟的服务器复测。容量上单 GPU 两路 SSE可排队完成，控制面仍应把对外并发预算放在限流（burst 20）以内，而不是50+业务并发。
