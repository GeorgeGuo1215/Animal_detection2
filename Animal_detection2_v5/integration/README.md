## Integration（对接层）

目标：作为前端/桌面端与 n8n/MySQL 的统一接入层。

### 1) 安装依赖

```bash
python -m pip install -r integration/requirements.txt
```

### 2) 启动

```bash
python -m uvicorn integration.api.main:app --host 127.0.0.1 --port 9001
```

### 3) 配置 n8n Webhook

设置环境变量：

```bash
set N8N_WEBHOOK_URL=http://localhost:5678/webhook/your-id
```

### 4) 发送事件

向 `POST /ingest` 发送统一事件 JSON，即可转发到 n8n。

