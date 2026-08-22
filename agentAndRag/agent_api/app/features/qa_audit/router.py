"""QA 审计查询与回答反馈路由。"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional

from fastapi import APIRouter, Depends, HTTPException, Query, Request

from .repository import (
    get_feedback_stats,
    get_knowledge_gaps,
    get_qa_stats,
    query_qa_history,
    submit_feedback,
)

router = APIRouter(prefix="/qa", tags=["qa-audit"])


async def _require_admin(request: Request) -> None:
    """校验独立 QA 管理令牌；不受开发认证开关影响。"""
    expected = os.getenv("QA_ADMIN_TOKEN", "")
    if not expected:
        raise HTTPException(status_code=403, detail="QA admin token not configured on server")
    if request.headers.get("X-Admin-Token", "").strip() != expected:
        raise HTTPException(status_code=403, detail="Invalid or missing admin token")


@router.get("/history", dependencies=[Depends(_require_admin)])
async def qa_history(
    page: int = Query(1, ge=1),
    page_size: int = Query(20, ge=1, le=100),
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
    keyword: Optional[str] = Query(None),
) -> Dict[str, Any]:
    """分页查询问答历史。"""
    data = await query_qa_history(
        page=page, page_size=page_size, date_from=date_from, date_to=date_to, keyword=keyword,
    )
    return {"ok": True, **data}


@router.get("/stats", dependencies=[Depends(_require_admin)])
async def qa_stats(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
) -> Dict[str, Any]:
    """查询问答统计。"""
    return {"ok": True, **await get_qa_stats(date_from=date_from, date_to=date_to)}


@router.get("/knowledge-gaps", dependencies=[Depends(_require_admin)])
async def qa_knowledge_gaps(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
    min_occurrences: int = Query(1, ge=1),
    limit: int = Query(50, ge=1, le=200),
) -> Dict[str, Any]:
    """查询潜在知识库缺口问题。"""
    data = await get_knowledge_gaps(
        date_from=date_from, date_to=date_to, min_occurrences=min_occurrences, limit=limit,
    )
    return {"ok": True, **data}


@router.post("/feedback")
async def qa_feedback(request: Request) -> Dict[str, Any]:
    """为某条回答提交一次 1～5 分反馈。"""
    try:
        body = await request.json()
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail="Invalid JSON body") from exc
    request_id = str(body.get("request_id") or "").strip()
    rating = body.get("rating")
    comment = str(body.get("comment") or "").strip()
    if not request_id:
        raise HTTPException(status_code=400, detail="request_id is required")
    if not isinstance(rating, int) or not 1 <= rating <= 5:
        raise HTTPException(status_code=400, detail="rating must be an integer between 1 and 5")
    if not await submit_feedback(request_id=request_id, rating=rating, comment=comment):
        raise HTTPException(status_code=404, detail="Record not found or already rated")
    return {"ok": True}


@router.get("/feedback-stats", dependencies=[Depends(_require_admin)])
async def qa_feedback_stats(
    date_from: Optional[str] = Query(None, description="YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="YYYY-MM-DD"),
) -> Dict[str, Any]:
    """查询反馈统计。"""
    return {"ok": True, **await get_feedback_stats(date_from=date_from, date_to=date_to)}
