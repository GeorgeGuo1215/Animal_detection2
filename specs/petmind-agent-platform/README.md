# PetMind 兽医 Agent 生产平台规格

本目录是生产平台的实现契约。接口、迁移、前端和测试均以此处为准；`/chat-moe` 继续作为开发测试入口，不是生产会话服务。

## 交付边界

- 个人兽医账号，由管理员邀请注册；不开放匿名注册。
- Agent 专属 PostgreSQL 持久化身份、会话、任务、套餐、订单和积分账本。
- Redis 提供跨进程限流和任务队列；运行事件持久化到 PostgreSQL 并可按事件 ID 续传，生产环境禁止静默退化。
- Memory Service 继续保存跨会话长期记忆，平台 `user.id` 是稳定记忆主体 ID。
- 首期不实现宠物档案、PetHealth 绑定、组织租户或真实支付渠道。
- 前端采用 React/Vite/TypeScript，使用 PetMind 原创暖色铅绘视觉。

## 状态机

```text
Invitation: pending -> accepted | revoked | expired
User:       active -> suspended -> deleted
Run:        queued -> running -> completed | failed | cancel_requested -> cancelled
Order:      pending_payment -> paid -> fulfilled | cancelled | expired | refunded
Subscription: active -> expired | suspended | cancelled
```

## 不变量

1. 用户身份来自 JWT 或数据库 API Key，不能来自请求体。
2. 所有会话、消息、任务和搜索查询必须包含 `user_id` 所有权条件。
3. `client_message_id`、`Idempotency-Key`、支付事件 ID 和 Memory `turn_id` 均需幂等。
4. 积分采用预占—结算；失败且无可核实用量时不扣费。
5. 对话最后活跃365天后软删除，30天宽限期后物理清理。
6. 订单、积分账本和审计记录不随聊天清理。
7. 生产事件只暴露脱敏阶段，不暴露系统提示词、内部推理或原始工具载荷。

## 文档索引

- [API契约](api-contract.md)
- [数据与迁移](data-model.md)
- [安全模型](security.md)
- [界面状态](ui-state-spec.md)
- [验收标准](acceptance.md)
- [生产部署](deployment.md)
