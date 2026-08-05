from __future__ import annotations

from fastapi import APIRouter, HTTPException
import logging
import uuid

from ..clients.n8n_webhook import send_to_n8n
from ..schemas.event import Event, normalize_event


router = APIRouter()


@router.post("/ingest")
async def ingest(event: Event) -> dict:
    req_id = uuid.uuid4().hex[:8]
    payload = normalize_event(event)
    logging.info("[%s] ingest event_id=%s animal_id=%s device_id=%s", req_id, payload.get("event_id"), payload.get("animal", {}).get("animal_id"), payload.get("device", {}).get("device_id"))
    try:
        forwarded = await send_to_n8n(payload)
    except ValueError as exc:
        logging.error("[%s] bad request: %s", req_id, exc)
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:  # noqa: BLE001
        logging.error("[%s] n8n failed: %s", req_id, exc)
        raise HTTPException(status_code=502, detail=f"n8n webhook failed: {exc}") from exc
    return {"ok": True, "request_id": req_id, "forwarded": forwarded}




@router.post("/debug/echo")
async def debug_echo(payload: dict) -> dict:
    req_id = uuid.uuid4().hex[:8]
    logging.info("[%s] debug echo keys=%s", req_id, list(payload.keys()))
    return {"ok": True, "request_id": req_id, "payload": payload}


@router.get("/debug/ping_n8n")
async def debug_ping_n8n() -> dict:
    req_id = uuid.uuid4().hex[:8]
    logging.info("[%s] ping n8n", req_id)
    try:
        forwarded = await send_to_n8n({"ping": True, "request_id": req_id})
    except Exception as exc:  # noqa: BLE001
        logging.error("[%s] ping n8n failed: %s", req_id, exc)
        raise HTTPException(status_code=502, detail=f"n8n ping failed: {exc}") from exc
    return {"ok": True, "request_id": req_id, "forwarded": forwarded}
