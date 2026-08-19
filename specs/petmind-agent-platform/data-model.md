# 数据与迁移

Agent 平台使用独立数据库；生产连接必须是 PostgreSQL。SQLite 仅用于本地开发和单元测试。

## 表组

- 身份：`platform_users`、`platform_invitations`、`platform_refresh_tokens`、`platform_password_reset_tokens`、`platform_api_keys`。
- 权限：`platform_roles`、`platform_permissions`、`platform_role_permissions`。
- 对话：`platform_conversations`、`platform_messages`、`platform_agent_runs`、`platform_run_events`、`platform_expert_consultations`。
- 商业化：`platform_plans`、`platform_subscriptions`、`platform_orders`、`platform_payment_events`。
- 额度：`platform_credit_accounts`、`platform_credit_ledger`、`platform_credit_reservations`、`platform_usage_records`。
- 运维：`platform_audit_logs`、`platform_outbox_events`。
- 用户设置：`platform_user_preferences`、`platform_common_phrases`、`platform_feedback`。
- 权益激活：`platform_activation_codes`、`platform_activation_redemptions`。
- 合规：`platform_legal_acceptances`。

Memory PostgreSQL 使用 `memory_derivations` 保存中期主题段到长期知识/画像的生成依赖。它不记录原始会话 turn；会话隐藏与记忆删除是两套独立生命周期。

## 迁移规则

- Alembic 是生产迁移的唯一入口；应用进程不得在生产调用 `create_all`。
- 迁移必须可重复检查、支持回滚，并先在数据库副本验证。
- 不删除或覆盖 PetHealth、Memory、QA、Trace 和 `/chat-moe` SQLite 数据。
- PostgreSQL 启用 `pg_trgm`，对会话标题和消息正文建立搜索索引。
