# PetMind Agent Frontend

面向兽医个人账号的生产 Web 客户端。界面采用暖米色纸张、陶土色和铅绘线条；借鉴成熟聊天产品的信息密度与键盘流程，不复制第三方品牌资产。

## 页面

- 邀请制登录、接受邀请、找回和重置密码。
- 会话历史、全文搜索、Markdown 答复、SSE 续传、停止生成。
- 套餐、订单、会员和积分入口。
- 设置、帮助、会员计划、API Key 悬浮菜单；设置中心覆盖账号、主题、常用语、邀请码、反馈、协议与分层记忆管理。
- 管理邀请、用户、订单、任务和审计的后台入口。

## 开发

```bash
pnpm install
pnpm dev
pnpm lint
pnpm test
pnpm build
pnpm test:e2e
```

`pnpm dev` 将 `/api` 代理到 `http://127.0.0.1:8002`。Access Token 只保存在内存，Refresh Token 由后端写入安全 HttpOnly Cookie。

## 生产（Docker Nginx）

对外默认使用 **5173**。镜像内执行 `pnpm build`，由 Nginx 托管带内容哈希的静态资源，并把 `/api`、`/health`、`/ready` 反代到宿主机 Agent（默认 `8002`）。这会避免生产浏览器逐个加载 Vite 的 `/src` 和 `/node_modules/.vite/deps` 开发模块。

```bash
cd /path/to/Animal_detection2
docker compose up -d --build
docker compose ps
docker compose logs --tail=100 web
```

可选部署变量：

```bash
# Agent 不在宿主机 8002 时覆盖；不要在末尾添加 /。
AGENT_UPSTREAM=http://host.docker.internal:8000 docker compose up -d --build

# 修改 Web 暴露端口。
PETMIND_WEB_PORT=8080 docker compose up -d --build

# 仅当浏览器需要直连独立 API 域名时在构建期设置；常规同源部署留空。
VITE_API_ROOT=https://agent-api.example.com docker compose up -d --build
```

Linux 需要 Docker 20.10+ 才支持 compose 中的 `host-gateway`。若 Agent 也容器化，应把两个服务加入同一 Docker 网络，并将 `AGENT_UPSTREAM` 改为 Agent 的服务名，例如 `http://agent:8002`。

启动后验证：

```bash
curl -fsS http://127.0.0.1:5173/healthz
curl -fsS http://127.0.0.1:5173/health
curl -fsS http://127.0.0.1:5173/ready
curl -I http://127.0.0.1:5173/chat
docker inspect --format '{{.State.Health.Status}}' petmind-nginx
```

`/healthz` 只证明 Nginx 和静态站点存活，`/ready` 才证明 Agent 已完成热启动。发布新前端必须重新执行 `docker compose up -d --build`；只重启旧容器不会重新构建资源。

镜像默认返回 CSP、`X-Frame-Options: DENY`、`X-Content-Type-Options: nosniff`、Referrer/Permissions Policy、COOP 和 HSTS。正式域名必须使用 HTTPS，并让后端设置 `AGENT_PLATFORM_COOKIE_SECURE=1`；生产配置使用 HTTP Origin 或非 Secure Cookie 会直接启动失败。本机 HTTP 验收必须使用 development 环境。

本机开发仍可用 `pnpm dev`；不要和 compose 同时占用 5173。生产环境不要使用 `pnpm dev` 或 `vite --host` 对外服务。

品牌源文件是仓库根目录 `logo.jpg`，禁止覆盖。`public/brand/` 是经裁边、透明化和多尺寸缩放后的确定性派生资产。
