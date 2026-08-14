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

Access JWT 通过 `Authorization: Bearer` 发送；Refresh Token 只使用 HttpOnly Cookie。

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
- `/api/v1/admin/*`：邀请、用户、套餐、订单、积分、任务和审计。

收费页只展示 `active=true` 的套餐。测试支付回调使用时间戳、事件 ID 和 HMAC 签名，并按事件 ID 防重。
