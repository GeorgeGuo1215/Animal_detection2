from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from .routes_ingest import router as ingest_router


logging.basicConfig(level=logging.INFO, format="[integration] %(levelname)s %(message)s")

app = FastAPI(title="Animal Detection Integration API", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(ingest_router)


@app.get("/health")
def health() -> dict:
    return {"ok": True}
