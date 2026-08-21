from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import math
from datetime import timezone
from typing import Any, AsyncIterator

from sqlalchemy import func, select

from ..memory import load_user_memory, write_user_memory
from ..services.agent_execution import build_moe_orchestrator, public_moe_allowed_tools
from ..services.moe import MoETrace
from ..tools.tool_registry import get_registry
from .config import get_platform_settings
from .database import platform_session
from .expert_consultations import persist_expert_consultation
from .models import AgentRun, Conversation, Message, RunEvent, UsageRecord, UserPreference, utcnow
from .services import settle_credits


logger = logging.getLogger(__name__)

TERMINAL_RUN_STATES = {"completed", "failed", "cancelled"}
PUBLIC_PHASES = {
    "intent_classifying": ("understanding", "正在理解临床问题"),
    "intent_classified": ("understanding", "已识别任务类型"),
    "routing": ("routing", "正在组织会诊路径"),
    "expert_calling": ("consulting", "专家会诊中"),
    "expert_complete": ("consulting", "专家意见已返回"),
    "tool_complete": ("consulting", "临床资料核对完成"),
    "reviewing": ("reviewing", "正在进行安全复核"),
    "generating": ("generating", "正在整理答复"),
}

_CAPABILITY_TOOL = {
    "local_knowledge": "rag.search",
    "medication_reference": "rag.search",
    "current_web": "mcp.web_search.web_search",
    "patient_vitals": "mcp.vitals_alert.check_vitals",
}


def _public_source_label(value: Any) -> str:
    """仅保留书籍相对标签或文件名，绝不暴露服务器绝对路径。"""
    text = str(value or "").strip().replace("\\", "/")
    if not text:
        return ""
    marker = "/books/"
    lowered = text.lower()
    if marker in lowered:
        return "books/" + text[lowered.index(marker) + len(marker):].split("/")[0]
    if text.lower().startswith("books/"):
        return "/".join(text.split("/")[:2])
    if not text.startswith("/") and not (len(text) > 2 and text[1] == ":"):
        return text[:160]
    return text.rsplit("/", 1)[-1][:160]


def _trace_node(
    node_id: str,
    node_type: str,
    status: str,
    *,
    parent_id: str = "",
    wave: int = 0,
    goal_id: str = "",
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """构造版本化、可公开和可按 node_id 原地更新的 trace 节点。"""
    return {
        "version": 1,
        "node_id": node_id[:160],
        "parent_id": parent_id[:160],
        "node_type": node_type[:40],
        "status": status[:24],
        "wave": max(0, int(wave)),
        "goal_id": goal_id[:160],
        "details": dict(details or {}),
    }


def _goal_id(expert: str, capability: str, query: str, index: int = 0) -> str:
    # reason/query goal is stable across bilingual and category-expanded calls.
    raw = f"{expert}|{query or capability}".encode("utf-8")
    return "goal:" + hashlib.sha1(raw).hexdigest()[:12]


def _query_node_id(
    expert: str,
    tool_name: str,
    query: str,
    wave: int,
    scope: str = "expert",
) -> str:
    """按专家、工具、查询、波次与范围生成稳定节点，避免互补工具互相覆盖。"""
    raw = f"{expert}|{tool_name}|{query}|{wave}|{scope}".encode("utf-8")
    return f"query:{hashlib.sha1(raw).hexdigest()[:12]}"


def _public_tool_summary(record: dict[str, Any]) -> dict[str, Any]:
    """将工具调用记录压缩为不含 prompt 的摘要，供 Web UI 安全展示。"""
    tool_name = str(record.get("tool_name") or "")
    result = record.get("result") if isinstance(record.get("result"), dict) else {}
    summary: dict[str, Any] = {
        "kind": "tool",
        "tool_name": tool_name,
        "ok": bool(record.get("ok")),
        "latency_ms": float(record.get("latency_ms") or 0.0),
        "query": str((record.get("arguments") or {}).get("query") or "")[:500],
        "goal": str(record.get("evidence_goal") or "")[:500],
        "wave": max(1, int(record.get("round") or 1)),
        "scope": str(record.get("scope") or "expert")[:24],
    }
    sufficiency = record.get("sufficiency")
    if isinstance(sufficiency, dict):
        summary["sufficiency"] = {
            "status": str(sufficiency.get("status") or "unknown")[:24],
            "reason": str(sufficiency.get("reason") or "")[:500],
        }
    if tool_name == "rag.search":
        hits = result.get("hits") if isinstance(result.get("hits"), list) else []
        summary["result"] = {
            "hits": len(hits),
            "sources": [
                _public_source_label(hit.get("source_path") or hit.get("source"))
                for hit in hits[:3]
                if isinstance(hit, dict)
            ],
        }
    elif "web_search" in tool_name:
        rows = result.get("results") if isinstance(result.get("results"), list) else []
        summary["result"] = {
            "results": len(rows),
            "titles": [str(row.get("title") or "")[:160] for row in rows[:3] if isinstance(row, dict)],
        }
    else:
        summary["result"] = {
            key: result.get(key)
            for key in ("code", "status", "alert_level")
            if result.get(key) is not None
        }
    if record.get("error"):
        summary["error"] = str(record["error"])[:300]
    return summary


def _public_expert_trace(opinion: dict[str, Any]) -> dict[str, Any]:
    """对外暴露专家工作产物，不包含系统 prompt 或隐藏模型轨迹。"""
    return {
        "expert": str(opinion.get("expert") or ""),
        "name": str(opinion.get("name_zh") or "专家"),
        "status": "completed",
        "task": str(opinion.get("retrieval_reason") or "根据统一任务策略形成专业意见"),
        "required_tools": list(opinion.get("required_tools") or []),
        "recommended_tools": list(opinion.get("recommended_tools") or []),
        "tools": [_public_tool_summary(row) for row in opinion.get("tool_results") or [] if isinstance(row, dict)],
        "opinion": {
            "conclusion": str(opinion.get("conclusion") or ""),
            "evidence": [str(item) for item in opinion.get("evidence") or []],
            "risks": [str(item) for item in opinion.get("risks") or []],
            "confidence": float(opinion.get("confidence") or 0.0),
        },
        "execution": "single_pass",
    }


def _trace_nodes_for_agent_event(
    agent_status: str,
    detail: dict[str, Any],
    *,
    public_expert: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """把内部阶段事件转换为不含提示词/思维链的公开任务节点。"""
    nodes: list[dict[str, Any]] = []
    if agent_status == "intent_classified":
        intent_id = str(detail.get("intent_id") or "")
        nodes.append(_trace_node(
            "decision", "decision", "completed",
            details={
                "intent_id": intent_id,
                "intent_name": str(detail.get("name") or "")[:120],
                "output_variant": str(detail.get("output_variant") or "default")[:60],
                "selected_experts": list(detail.get("selected_experts") or []),
                "emergency": bool(detail.get("emergency")),
            },
        ))
        for task in detail.get("evidence_tasks") or []:
            if not isinstance(task, dict):
                continue
            owner = str(task.get("owner") or "clinical")
            reason = str(task.get("reason") or "核对外部证据")[:500]
            capability = str(task.get("capability") or "")
            queries = [str(value)[:500] for value in task.get("queries") or [] if str(value).strip()]
            if not queries and task.get("query"):
                queries = [str(task["query"])[:500]]
            gid = _goal_id(owner, capability, reason)
            nodes.append(_trace_node(
                gid, "goal", "pending", parent_id="decision", goal_id=gid,
                details={
                    "owner": owner,
                    "capability": capability,
                    "requirement": str(task.get("requirement") or "recommended"),
                    "goal": reason,
                    "queries": queries,
                },
            ))
    elif agent_status == "expert_calling":
        expert = str(detail.get("expert") or "")
        nodes.append(_trace_node(
            f"expert:{expert}", "expert", "running", parent_id="decision",
            details={"expert": expert, "name": str(detail.get("name_zh") or "专家")[:100]},
        ))
        for task in detail.get("evidence_tasks") or []:
            if not isinstance(task, dict):
                continue
            reason = str(task.get("reason") or "核对外部证据")[:500]
            capability = str(task.get("capability") or "")
            gid = _goal_id(expert, capability, reason)
            queries = [str(value)[:500] for value in task.get("queries") or [] if str(value).strip()]
            if not queries and task.get("query"):
                queries = [str(task["query"])[:500]]
            for query in queries:
                tool_name = _CAPABILITY_TOOL.get(capability, capability)
                nodes.append(_trace_node(
                    _query_node_id(expert, tool_name, query, 1), "query", "running",
                    parent_id=f"expert:{expert}", wave=1, goal_id=gid,
                    details={
                        "query": query,
                        "tool_name": tool_name,
                        "scope": "expert",
                    },
                ))
    elif agent_status == "expert_complete" and public_expert:
        expert = str(public_expert.get("expert") or "")
        opinion = public_expert.get("opinion") if isinstance(public_expert.get("opinion"), dict) else {}
        nodes.append(_trace_node(
            f"expert:{expert}", "expert", "completed", parent_id="decision",
            details={
                "expert": expert,
                "name": str(public_expert.get("name") or "专家")[:100],
                "confidence": float(opinion.get("confidence") or 0.0),
            },
        ))
        for tool in public_expert.get("tools") or []:
            if not isinstance(tool, dict):
                continue
            query = str(tool.get("query") or "")[:500]
            goal = str(tool.get("goal") or "核对外部证据")[:500]
            wave = max(1, int(tool.get("wave") or 1))
            tool_name = str(tool.get("tool_name") or "")
            scope = str(tool.get("scope") or "expert")
            gid = _goal_id(expert, "", goal)
            nodes.append(_trace_node(
                _query_node_id(expert, tool_name, query, wave, scope), "query",
                "completed" if tool.get("ok") else "failed",
                parent_id=f"expert:{expert}", wave=wave, goal_id=gid,
                details={
                    "query": query,
                    "tool_name": tool_name,
                    "latency_ms": float(tool.get("latency_ms") or 0.0),
                    "scope": scope,
                    "result": tool.get("result") or {},
                    "sufficiency": tool.get("sufficiency"),
                    "error": str(tool.get("error") or "")[:300],
                },
            ))
    elif agent_status == "reviewing":
        completed = bool(detail.get("verdict"))
        nodes.append(_trace_node(
            "review", "review", "completed" if completed else "running",
            details={
                "verdict": str(detail.get("verdict") or "")[:40],
                "issues": [str(item)[:300] for item in detail.get("issues") or []],
            },
        ))
    elif agent_status == "generating":
        nodes.append(_trace_node(
            "answer", "answer", "running",
            details={"message": str(detail.get("message") or "正在整理答复")[:200]},
        ))
    return nodes


async def append_run_event(run_id: str, event_type: str, payload: dict[str, Any]) -> int:
    """追加一条 Run 事件并返回新的序号。"""
    async with platform_session() as session:
        last = await session.scalar(select(func.max(RunEvent.sequence)).where(RunEvent.run_id == run_id))
        sequence = int(last or 0) + 1
        session.add(RunEvent(run_id=run_id, sequence=sequence, event_type=event_type, payload=payload))
        await session.commit()
        return sequence


async def _run_cancel_requested(run_id: str) -> bool:
    """查询该 Run 是否已被请求取消；记录不存在时视为已取消。"""
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        return run is None or run.cancel_requested


async def enqueue_run(run_id: str) -> None:
    """将 Run 推入 Redis 队列；无 Redis 时在进程内创建后台任务执行。

    生产环境 Redis 失败会向上抛出，开发环境则回退到本地 ``execute_run``。
    """
    settings = get_platform_settings()
    if settings.redis_url:
        try:
            from redis.asyncio import Redis

            redis = Redis.from_url(settings.redis_url, decode_responses=True)
            await redis.rpush("petmind:platform:runs", run_id)
            await redis.aclose()
            return
        except Exception:
            if settings.production:
                raise
    asyncio.create_task(execute_run(run_id), name=f"platform-run-{run_id}")


async def execute_run(run_id: str) -> None:
    """执行一次排队中的 Agent Run：流式会诊、落库答复、结算积分并写记忆。

    仅认领 ``queued`` / ``retry`` 状态。取消、失败或终答生成异常会更新 Run 状态并释放预留积分。
    """
    async with platform_session() as session:
        run = await session.get(AgentRun, run_id)
        if run is None or run.status not in {"queued", "retry"}:
            return
        run.status = "running"
        run.started_at = utcnow()
        conversation = await session.get(Conversation, run.conversation_id)
        history_rows = list((await session.scalars(
            select(Message).where(
                Message.conversation_id == run.conversation_id,
                Message.status == "complete",
            ).order_by(Message.created_at.asc()).limit(48)
        )).all())
        await session.commit()
        query = run.query
        user_id = run.user_id
        conversation_id = run.conversation_id
        parameters = dict(run.parameters or {})
        preference = await session.get(UserPreference, run.user_id)
        memory_recall_enabled = preference.memory_recall_enabled if preference else True
        memory_write_enabled = preference.memory_write_enabled if preference else True
        user_role = str(parameters.get("user_role") or "veterinarian")
        if user_role not in {"pet_owner", "veterinarian"}:
            user_role = "veterinarian"
        conversation_history = [
            {"role": item.role, "content": item.content}
            for item in history_rows
            if item.id != run.user_message_id and item.role in {"user", "assistant"}
        ]

    await append_run_event(run_id, "status", {"phase": "queued", "message": "任务已进入会诊队列"})
    answer_parts: list[str] = []
    expert_calls = 0
    tool_calls = 0
    current_phase = "queued"
    try:
        memory_injection = ""
        if memory_recall_enabled:
            memory_injection, _ = await load_user_memory(user_id=user_id, query=query, pet_id=None)
        registry = get_registry()
        allowed_tools = public_moe_allowed_tools(tool.name for tool in registry.list_tools())
        orchestrator = build_moe_orchestrator(
            registry=registry,
            temperature=float(parameters.get("temperature", 0.3)),
            max_tokens=int(parameters.get("max_tokens", 2500)),
            user_role=user_role,
            allowed_tools=allowed_tools,
        )
        trace = MoETrace(question=query, user_role=user_role)
        async for event in orchestrator.stream(
            query=query,
            system_context=(
                "你正在为已认证兽医提供临床决策支持。"
                if user_role == "veterinarian"
                else "你正在用宠物主容易理解的方式提供健康信息、风险分级与就医建议。"
            ),
            conversation_history=conversation_history,
            user_memory=memory_injection,
            recorder=trace,
        ):
            if await _run_cancel_requested(run_id):
                raise asyncio.CancelledError
            status = str(event.get("status") or "")
            content = str(event.get("content") or "")
            if status == "streaming" and content:
                answer_parts.append(content)
                await append_run_event(run_id, "delta", {"content": content})
            elif status == "answer_reset":
                answer_parts.clear()
                detail = event.get("detail") if isinstance(event.get("detail"), dict) else {}
                reset_reason = str(detail.get("reason") or "answer_replaced")
                await append_run_event(run_id, "reset", {
                    "reason": reset_reason,
                    "message": (
                        "证据复核后已收缩未核实的具体结论"
                        if reset_reason == "evidence_safety_repair"
                        else "终答连接中断，已自动重新生成完整答复"
                    ),
                })
            elif status in PUBLIC_PHASES:
                phase, message = PUBLIC_PHASES[status]
                current_phase = phase
                if status == "expert_calling":
                    expert_calls += 1
                if status == "tool_complete":
                    tool_calls += 1
                payload: dict[str, Any] = {"phase": phase, "message": message, "agent_status": status}
                detail = event.get("detail") if isinstance(event.get("detail"), dict) else {}
                public_expert: dict[str, Any] | None = None
                if status == "expert_calling":
                    payload["expert"] = {
                        "expert": str(detail.get("expert") or ""),
                        "name": str(detail.get("name_zh") or "专家"),
                        "status": "running",
                    }
                elif status == "expert_complete" and isinstance(detail.get("opinion"), dict):
                    tool_calls += len(detail["opinion"].get("tool_results") or [])
                    public_expert = _public_expert_trace(detail["opinion"])
                    payload["expert"] = public_expert
                    await persist_expert_consultation(run_id, payload["expert"])
                await append_run_event(run_id, "status", payload)
                for trace_node in _trace_nodes_for_agent_event(
                    status,
                    detail,
                    public_expert=public_expert,
                ):
                    await append_run_event(run_id, "trace", trace_node)

        answer = "".join(answer_parts).strip()
        if not answer:
            raise RuntimeError("Agent completed without a final answer")
        credits = max(1, math.ceil((len(query) + len(answer)) / 1000) + expert_calls * 2 + tool_calls)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is None:
                return
            assistant = Message(
                conversation_id=conversation_id,
                run_id=run_id,
                role="assistant",
                content=answer,
                status="complete",
            )
            session.add(assistant)
            await session.flush()
            run.assistant_message_id = assistant.id
            run.response = answer
            run.status = "completed"
            run.finished_at = utcnow()
            conversation = await session.get(Conversation, conversation_id)
            if conversation is not None:
                conversation.last_active_at = utcnow()
                if conversation.title == "新会诊":
                    conversation.title = query[:60]
            billed_credits = await settle_credits(session, run_id=run_id, actual_amount=credits)
            run.actual_credits = billed_credits
            session.add(UsageRecord(
                run_id=run_id,
                user_id=user_id,
                expert_calls=expert_calls,
                tool_calls=tool_calls,
                credits=billed_credits,
                details={"finish_reason": orchestrator.last_finish_reason, "calculated_credits": credits},
            ))
            await session.commit()
        memory_synced = True
        try:
            if memory_write_enabled:
                await write_user_memory(
                    user_id=user_id,
                    query=query,
                    answer=answer,
                    pet_id=None,
                    session_id=conversation_id,
                    turn_id=run_id,
                )
        except Exception as exc:  # noqa: BLE001
            # 回答已持久化。记忆同步失败不得把成功会诊事后改成失败。
            memory_synced = False
            logger.exception("memory sync failed after completed run run_id=%s", run_id)
            await append_run_event(run_id, "warning", {
                "code": "memory_sync_failed",
                "message": "回答已完成，但长期记忆同步失败，系统将稍后重试",
            })
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "completed",
            details={"finish_reason": orchestrator.last_finish_reason},
        ))
        await append_run_event(run_id, "completed", {
            "finish_reason": orchestrator.last_finish_reason,
            "credits": billed_credits,
            "memory_synced": memory_synced,
        })
    except asyncio.CancelledError:
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is not None:
                run.status = "cancelled"
                run.finished_at = utcnow()
                await settle_credits(session, run_id=run_id, actual_amount=0)
                await session.commit()
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "cancelled", details={"message": "任务已取消"},
        ))
        await append_run_event(run_id, "cancelled", {"message": "任务已取消"})
    except Exception as exc:  # noqa: BLE001
        logger.exception("platform run failed run_id=%s phase=%s", run_id, current_phase)
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is not None:
                run.status = "failed"
                run.error_code = "agent_run_failed"
                run.error_message = str(exc)[:1000]
                run.finished_at = utcnow()
                await settle_credits(session, run_id=run_id, actual_amount=0)
                await session.commit()
        await append_run_event(run_id, "trace", _trace_node(
            "answer", "answer", "failed",
            details={"message": "终答生成失败" if current_phase == "generating" else "会诊执行失败"},
        ))
        await append_run_event(run_id, "failed", {
            "code": "agent_run_failed",
            "message": "终答生成失败，请重试" if current_phase == "generating" else "会诊任务执行失败",
            "phase": current_phase,
            "retryable": current_phase == "generating",
        })


async def run_event_stream(run_id: str, *, after_sequence: int = 0) -> AsyncIterator[str]:
    """以 SSE 文本帧推送指定序号之后的 Run 事件，直到进入终态。"""
    sequence = max(0, after_sequence)
    while True:
        async with platform_session() as session:
            rows = list((await session.scalars(select(RunEvent).where(
                RunEvent.run_id == run_id,
                RunEvent.sequence > sequence,
            ).order_by(RunEvent.sequence.asc()))).all())
            run = await session.get(AgentRun, run_id)
        for row in rows:
            sequence = row.sequence
            data = json.dumps(row.payload, ensure_ascii=False, default=str)
            yield f"id: {row.sequence}\nevent: {row.event_type}\ndata: {data}\n\n"
        if run is None or (run.status in TERMINAL_RUN_STATES and not rows):
            break
        yield ": keep-alive\n\n"
        await asyncio.sleep(0.5)


async def wait_for_run(run_id: str, timeout_seconds: int) -> AgentRun | None:
    """轮询直到 Run 进入终态或超时；超时返回 ``None``，记录不存在则立即返回。"""
    deadline = asyncio.get_running_loop().time() + timeout_seconds
    while asyncio.get_running_loop().time() < deadline:
        async with platform_session() as session:
            run = await session.get(AgentRun, run_id)
            if run is None or run.status in TERMINAL_RUN_STATES:
                return run
        await asyncio.sleep(0.4)
    return None


async def worker_forever() -> None:
    """生产 Worker：从 Redis 阻塞弹出 Run 并执行，启动时回收未完成任务。

    需要配置 ``AGENT_PLATFORM_REDIS_URL``。失败的 Run 会写入死信队列。
    """
    settings = get_platform_settings()
    if not settings.redis_url:
        raise RuntimeError("AGENT_PLATFORM_REDIS_URL is required for the production worker")
    from redis.asyncio import Redis

    # BLPOP 在空队列时会一直等待。显式关闭 socket 读超时，
    # 避免 redis-py 8 把正常的空轮询当成 Worker 失败。
    redis = Redis.from_url(settings.redis_url, decode_responses=True, socket_timeout=None, health_check_interval=30)
    try:
        # 在 API/Redis/Worker 重启后回收已持久化的任务。
        # 重复 ID 无害，因为 execute_run 只会认领 queued/retry 状态。
        async with platform_session() as session:
            recoverable = list((await session.scalars(select(AgentRun).where(
                AgentRun.status.in_(["queued", "running", "retry"])
            ))).all())
            for run in recoverable:
                if run.status == "running":
                    run.status = "retry"
                await redis.rpush("petmind:platform:runs", run.id)
            await session.commit()
        while True:
            item = await redis.blpop("petmind:platform:runs", timeout=5)
            if item:
                run_id = item[1]
                await execute_run(run_id)
                async with platform_session() as session:
                    completed = await session.get(AgentRun, run_id)
                    if completed is not None and completed.status == "failed":
                        await redis.rpush("petmind:platform:runs:dead", run_id)
    finally:
        await redis.aclose()
