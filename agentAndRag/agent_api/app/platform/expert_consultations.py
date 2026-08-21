from __future__ import annotations

from collections import defaultdict
from typing import Any, Iterable

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .database import platform_session
from .models import AgentRun, ExpertConsultation, RunEvent


def consultation_payload(item: ExpertConsultation) -> dict[str, Any]:
    """将专家会诊记录序列化为对终端用户安全的公开结构。"""
    return {
        "expert": item.expert_key,
        "name": item.expert_name,
        "status": item.status,
        "task": item.task,
        "required_tools": list(item.required_tools or []),
        "recommended_tools": list(item.recommended_tools or []),
        "tools": list(item.tool_summaries or []),
        "opinion": {
            "conclusion": item.conclusion,
            "evidence": list(item.evidence or []),
            "risks": list(item.risks or []),
            "confidence": float(item.confidence or 0.0),
        },
        "execution": item.execution,
    }


def _trace_values(trace: dict[str, Any]) -> dict[str, Any]:
    """从公开专家轨迹中提取可写入 ``ExpertConsultation`` 的字段。"""
    opinion = trace.get("opinion") if isinstance(trace.get("opinion"), dict) else {}
    return {
        "expert_name": str(trace.get("name") or "专家")[:100],
        "status": str(trace.get("status") or "completed")[:24],
        "task": str(trace.get("task") or ""),
        "required_tools": list(trace.get("required_tools") or []),
        "recommended_tools": list(trace.get("recommended_tools") or []),
        "tool_summaries": list(trace.get("tools") or []),
        "conclusion": str(opinion.get("conclusion") or ""),
        "evidence": [str(item) for item in opinion.get("evidence") or []],
        "risks": [str(item) for item in opinion.get("risks") or []],
        "confidence": float(opinion.get("confidence") or 0.0),
        "execution": str(trace.get("execution") or "single_pass")[:32],
    }


async def persist_expert_consultation(run_id: str, trace: dict[str, Any]) -> None:
    """幂等持久化一次已完成的专家意见；同一 Run + 专家键则更新已有行。"""
    expert_key = str(trace.get("expert") or "")[:40]
    if not expert_key:
        return
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        if run is None:
            return
        item = await session.scalar(select(ExpertConsultation).where(
            ExpertConsultation.run_id == run_id,
            ExpertConsultation.expert_key == expert_key,
        ))
        values = _trace_values(trace)
        if item is None:
            item = ExpertConsultation(
                run_id=run_id,
                conversation_id=run.conversation_id,
                expert_key=expert_key,
                **values,
            )
            session.add(item)
        else:
            for key, value in values.items():
                setattr(item, key, value)
        await session.commit()


def _event_trace(payload: Any) -> dict[str, Any] | None:
    """从 Run 状态事件载荷中提取已完成的专家轨迹；结构不符则返回 ``None``。"""
    if not isinstance(payload, dict):
        return None
    expert = payload.get("expert")
    if not isinstance(expert, dict) or expert.get("status") != "completed" or not expert.get("expert"):
        return None
    return expert


async def consultations_by_run(
    session: AsyncSession,
    run_ids: Iterable[str | None],
) -> dict[str, list[dict[str, Any]]]:
    """按 Run ID 批量加载专家会诊；表中没有记录时回退到历史 RunEvent。

    事件回退用于在迁移完成前立即展示旧数据；Alembic 迁移也会永久回填该表。
    """
    ids = {run_id for run_id in run_ids if run_id}
    if not ids:
        return {}
    rows = list((await session.scalars(
        select(ExpertConsultation)
        .where(ExpertConsultation.run_id.in_(ids))
        .order_by(ExpertConsultation.created_at.asc())
    )).all())
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in rows:
        grouped[item.run_id].append(consultation_payload(item))

    missing = ids.difference(grouped)
    if missing:
        events = list((await session.scalars(
            select(RunEvent)
            .where(RunEvent.run_id.in_(missing), RunEvent.event_type == "status")
            .order_by(RunEvent.run_id.asc(), RunEvent.sequence.asc())
        )).all())
        seen: set[tuple[str, str]] = set()
        for event in events:
            trace = _event_trace(event.payload)
            if trace is None:
                continue
            key = (event.run_id, str(trace["expert"]))
            if key in seen:
                continue
            seen.add(key)
            grouped[event.run_id].append(trace)
    return dict(grouped)
