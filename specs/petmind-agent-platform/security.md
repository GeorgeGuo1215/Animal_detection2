# 安全模型

- 密码：Argon2id；邀请、刷新、重置令牌均只保存 SHA-256 摘要。
- Access JWT：15分钟，校验 issuer、audience、jti、角色和用户状态。
- Refresh Token：30天、单次轮换、重放后撤销整个 token family。
- API Key：一次显示，支持作用域、过期、撤销和审计；旧文件 Key 仅保留迁移期。
- 管理路由使用显式 RBAC，不依赖隐藏按钮。
- 生产 CORS 只允许配置域名，禁用匿名测试页、调试路由和错误堆栈。
- 用户 Markdown 必须净化；日志不得写入密码、令牌、完整API Key或原始工具载荷。
- Redis 限流按 IP、主体、路由联合计算；额度扣减在 PostgreSQL 事务内完成。
