from __future__ import annotations

import os

import uvicorn


if __name__ == "__main__":
    os.environ["AGENT_EXECUTION_ROLE"] = "worker"
    uvicorn.run(
        "agent_api.app.worker_main:app",
        host=os.getenv("AGENT_WORKER_HOST", "127.0.0.1"),
        port=int(os.getenv("AGENT_WORKER_PORT", "8102")),
        proxy_headers=False,
        access_log=os.getenv("AGENT_WORKER_ACCESS_LOG", "0") == "1",
    )
