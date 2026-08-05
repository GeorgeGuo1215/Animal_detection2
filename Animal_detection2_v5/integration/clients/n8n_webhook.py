from __future__ import annotations

import httpx
import logging

from ..config import N8N_TIMEOUT_SEC, N8N_WEBHOOK_URL


async def send_to_n8n(payload: dict) -> dict:
    if not N8N_WEBHOOK_URL:
        raise ValueError("N8N_WEBHOOK_URL 未配置")

    timeout = httpx.Timeout(N8N_TIMEOUT_SEC)
    async with httpx.AsyncClient(timeout=timeout) as client:
        logging.info("POST n8n webhook: %s", N8N_WEBHOOK_URL)
        try:
            resp = await client.post(N8N_WEBHOOK_URL, json=payload)
        except Exception as exc:  # noqa: BLE001
            logging.error("n8n webhook request failed: %s", exc)
            raise

        text = (resp.text or "").strip()
        preview = text[:1000] + ("...(truncated)" if len(text) > 1000 else "")
        meta = {
            "status_code": resp.status_code,
            "content_type": resp.headers.get("content-type"),
            "content_length": resp.headers.get("content-length"),
        }

        if resp.status_code >= 400:
            logging.error("n8n webhook error %s meta=%s body=%s", resp.status_code, meta, preview)
            raise RuntimeError(f"n8n error {resp.status_code}: {preview}")

        logging.info("n8n webhook ok %s meta=%s", resp.status_code, meta)
        return {"status_code": resp.status_code, "body_preview": preview, "meta": meta}
