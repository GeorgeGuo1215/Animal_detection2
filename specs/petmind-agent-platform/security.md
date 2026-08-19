# 安全模型

- 密码：Argon2id；邀请、刷新、重置令牌均只保存 SHA-256 摘要。
- Access JWT：15分钟，校验 issuer、audience、jti、角色和用户状态。
- Refresh Token：30天、单次轮换、重放后撤销整个 token family。
- API Key：一次显示，支持作用域、过期、撤销和审计；旧文件 Key 仅保留迁移期。
- 生产关闭旧 Key 后不读取 `keys.txt/AGENT_API_KEYS`，且禁止 `AGENT_ALLOW_INSECURE_DEFAULT_KEY=1`。
- 浏览器 `/api/v1/*` 只接受 JWT 登录会话；数据库 API Key 仅允许 `/v1/models` 与 `/v1/chat/completions`，不能访问个人资料、记忆或管理接口。
- 管理路由使用显式 RBAC，不依赖隐藏按钮。
- `SUPPORT_ADMIN` 只能邀请试用套餐的普通兽医账号，不能创建或撤销管理员邀请；用户身份只取认证上下文，忽略 body、OpenAI `user` 与 `X-User-Id` 的身份声明。
- 生产 CORS 只允许 HTTPS 配置域名；诊断 `/chat-moe` 仅允许超级管理员 JWT 或内部 Worker Token，`/integration` 与 n8n 代码已移除。
- Memory 管理和快照接口必须携带独立 `MEMORY_MANAGEMENT_TOKEN`；未配置时返回 503，不允许 fail-open。
- 用户 Markdown 必须净化；日志不得写入密码、令牌、完整API Key或原始工具载荷。
- Redis 限流按 IP、主体、路由联合计算；额度扣减在 PostgreSQL 事务内完成。
- 生产启动必须配置 Secure Cookie、HTTPS 前端 Origin、独立 Worker/Memory/Webhook 强密钥；就绪接口只返回布尔状态，不暴露路径和内部容量。
