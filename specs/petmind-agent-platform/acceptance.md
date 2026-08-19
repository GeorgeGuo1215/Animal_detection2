# 验收标准

- 邀请令牌只能使用一次；暂停用户不能登录或继续使用旧Access Token。
- Refresh Token轮换和重放检测有效；API Key完整值不落库。
- 任意跨用户会话、消息、Run、搜索和订单访问均失败。
- 重复消息、Run、支付回调和Memory写入不产生重复记录或重复扣费。
- SSE断线后按事件 ID 续传，最终回答只保存一次。
- API、Worker或Redis重启后，已持久化任务可恢复或进入明确失败状态。
- 并发积分预占不会产生负余额。
- 365天清理只影响对话数据，不删除账本和审计。
- 生产配置缺少PostgreSQL、Redis、HTTPS/Secure Cookie或任一独立强密钥时启动失败。
- 数据库 API Key 访问个人资料/记忆/后台返回403；伪造 `X-User-Id` 或 body 用户标识不能改变记忆归属。
- SUPPORT_ADMIN不能创建管理员邀请或向邀请附加付费套餐。
- Memory管理令牌缺失或错误时分别返回503/401，不能匿名删除或恢复数据。
- 现有 MoE、RAG、MCP、Memory 和 `/v1/chat/completions` 回归测试通过。
