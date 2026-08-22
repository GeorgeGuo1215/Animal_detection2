"""平台 Run 的公开 Trace 脱敏与稳定节点映射。"""

from __future__ import annotations

import hashlib
from typing import Any

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
