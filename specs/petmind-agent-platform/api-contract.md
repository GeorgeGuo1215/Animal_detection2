# API 契约

所有平台响应携带 `X-Request-Id`。错误统一为：

```json
{"code":"string","message":"string","request_id":"string","details":{}}
```

## 认证

- `POST /api/v1/auth/invitations/accept`
- `POST /api/v1/auth/login`
- `POST /api/v1/auth/refresh`
- `POST /api/v1/auth/logout`
- `POST /api/v1/auth/password/forgot`
- `POST /api/v1/auth/password/reset`
- `GET /api/v1/me`
- `GET|POST|DELETE /api/v1/me/api-keys`
- `GET|PATCH /api/v1/me/preferences`
- `GET|POST /api/v1/me/common-phrases`
- `PATCH|DELETE /api/v1/me/common-phrases/{id}`
- `GET|DELETE /api/v1/me/memories`
- `DELETE /api/v1/me/memories/{id}`
- `POST /api/v1/activation-codes/redeem`
- `POST /api/v1/feedback`
- `GET /api/v1/legal/{terms|privacy}`
- `POST /api/v1/me/legal-acceptances`

Access JWT 通过 `Authorization: Bearer` 发送；Refresh Token 只使用 HttpOnly Cookie。上述 `/api/v1` 用户与管理接口不接受数据库 API Key；API Key 仅用于 OpenAI 兼容接口。服务端不采信 body、OpenAI `user` 或 `X-User-Id` 作为记忆归属。

## 会话与任务

- `POST|GET /api/v1/conversations`
- `GET|PATCH|DELETE /api/v1/conversations/{id}`
- `GET /api/v1/conversations/{id}/messages`
- `GET /api/v1/conversations/search?q=`
- `POST /api/v1/conversations/{id}/runs`
- `GET|DELETE /api/v1/runs/{id}`
- `GET /api/v1/runs/{id}/events`

创建 Run 的 `delivery` 为 `sse|sync|async`。SSE事件包含 `id/event/data`，事件 ID 单调递增并支持 `Last-Event-ID`。

## 套餐、订单和后台

- `GET /api/v1/plans`
- `POST|GET /api/v1/orders`
- `GET /api/v1/subscription`
- `GET /api/v1/credits`
- `POST /api/v1/payments/test-webhook`

管理员用户数据备份：

- `GET /api/v1/admin/users/{user_id}/data-snapshot`：导出会话、Run、专家意见及全部记忆层。
- `POST /api/v1/admin/users/{user_id}/data-snapshot/restore-file`：上传 `application/gzip` 快照覆盖恢复；要求 `X-Restore-Confirmation: OVERWRITE_USER_DATA`。
- 普通 JSON 接口仍限制 1 MiB；压缩快照限制 8 MiB，解压后限制 64 MiB。
- `/api/v1/admin/*`：邀请、用户、套餐、订单、积分、任务和审计。

收费页只展示 `active=true` 的套餐。测试支付回调使用时间戳、事件 ID 和 HMAC 签名，并按事件 ID 防重。

邀请码本期没有管理 API；记录由受控数据库流程预置，兑换接口只返回统一的“无效或不可用”错误，避免枚举。

### 用户数据测试快照（仅 SUPER_ADMIN）

- `GET /api/v1/admin/users/{user_id}/data-snapshot`
- `POST /api/v1/admin/users/{user_id}/data-snapshot/restore`

快照覆盖对话、消息、Agent Run、运行事件、专家意见、用量记录、短中长期记忆、用户画像和来源边。恢复为覆盖式操作，仅允许 `SUPER_ADMIN`，并要求固定确认文本、用户 ID 一致、版本一致和 SHA-256 完整性校验；不迁移财务账本或认证凭据。
