# 生产部署

1. PostgreSQL 与 Memory 数据库使用不同数据库或不同账号；平台迁移只包含 `platform_*` 表。
2. Redis 必须开启持久化并限制网络访问，API 和 Worker 使用同一 Redis URL。
3. 从 `.env.platform.example` 生成仅服务器可读的 `.env`，JWT 和支付 Webhook 使用不同随机密钥。
4. 执行 `alembic upgrade head` 后再启动服务；生产严禁 `AUTO_CREATE_SCHEMA=1`。
5. 首个管理员通过 CLI 初始化，密码不得出现在 unit 文件、Shell 历史或日志中。
6. 单 GPU Worker 并发保持 1。扩容 API 不等于扩容推理，禁止多个 Worker 争用同一张卡。
7. 反向代理启用 TLS、合理的请求体限制、SSE `proxy_buffering off` 和不少于 130 秒的读取超时。
8. 前端与 API 分域时，Refresh Cookie 仍需满足 Secure/SameSite 策略；只配置精确 CORS Origin 和 Host。
9. 备份 PostgreSQL、Redis AOF 与 Memory 数据库；恢复演练必须覆盖未完成 Run 的重新入队。
10. 迁移期观察 `legacy_api_key.used` 审计事件，确认调用方切换后设置 `AGENT_LEGACY_API_KEYS_ENABLED=0`。

systemd 的系统级 unit 由 root 管理并可在无人登录时启动；用户级 unit 随用户会话/linger 运行。不要同时启用两份监听相同端口的 unit。`agent-rag.service` 是模板，复制到 `/etc/systemd/system` 前必须核对 Linux 用户、工作目录、Python 环境及 EnvironmentFile。
