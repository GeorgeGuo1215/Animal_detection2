# PetMind Agent Frontend

面向兽医个人账号的生产 Web 客户端。界面采用暖米色纸张、陶土色和铅绘线条；借鉴成熟聊天产品的信息密度与键盘流程，不复制第三方品牌资产。

## 页面

- 邀请制登录、接受邀请、找回和重置密码。
- 会话历史、全文搜索、Markdown 答复、SSE 续传、停止生成。
- 套餐、订单、会员和积分入口。
- 设置、帮助、会员计划、API Key 悬浮菜单。
- 管理邀请、用户、订单、任务和审计的后台入口。

## 开发

```bash
pnpm install
pnpm dev
pnpm test
pnpm build
pnpm test:e2e
```

`pnpm dev` 将 `/api` 代理到 `http://127.0.0.1:8002`。部署到独立 API 域名时设置 `VITE_API_ROOT`。Access Token 只保存在内存，Refresh Token 由后端写入安全 HttpOnly Cookie。

## 生产（Docker Nginx）

对外仍使用 **5173**。镜像内执行 `pnpm build`，由 Nginx 托管静态页并把 `/api`、`/health`、`/ready` 反代到宿主机 Agent（默认 `8002`）。

```bash
docker compose up -d --build
```

本机开发仍可用 `pnpm dev`；不要和 compose 同时占用 5173。

品牌源文件是仓库根目录 `logo.jpg`，禁止覆盖。`public/brand/` 是经裁边、透明化和多尺寸缩放后的确定性派生资产。
