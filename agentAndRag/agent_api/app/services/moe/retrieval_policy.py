"""语义证据任务的确定性解析器。LLM 任务策略阶段决定为何需要证据；本模块只把已校验的能力映射到注册工具名，并分配给一名入选专家，不做查询关键词匹配。"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


RAG_TOOL = "rag.search"
WEB_SEARCH_TOOL = "mcp.web_search.web_search"
VITALS_TOOL = "mcp.vitals_alert.check_vitals"

CAPABILITY_TO_TOOL: Mapping[str, str] = {
    "local_knowledge": RAG_TOOL,
    "medication_reference": RAG_TOOL,
    "current_web": WEB_SEARCH_TOOL,
    "patient_vitals": VITALS_TOOL,
}

CAPABILITY_DEFAULT_OWNER: Mapping[str, str] = {
    "local_knowledge": "clinical",
    "medication_reference": "pharmacy",
    "current_web": "clinical",
    "patient_vitals": "clinical",
}


@dataclass(frozen=True)
class EvidenceTask:
    """一条证据任务：能力、负责人、必要性与查询。"""
    capability: str
    owner: str
    requirement: str
    reason: str
    web_fallback_on_weak_local: bool = False
    query: str = ""

    def as_dict(self) -> Dict[str, Any]:
        """转为可序列化字典。"""
        return {
            "capability": self.capability,
            "owner": self.owner,
            "requirement": self.requirement,
            "reason": self.reason,
            "web_fallback_on_weak_local": self.web_fallback_on_weak_local,
            "query": self.query,
        }


@dataclass(frozen=True)
class RetrievalRequirement:
    """某专家须执行的检索工具策略。"""
    required_tools: Tuple[str, ...]
    recommended_tools: Tuple[str, ...]
    require_web_on_rag_failure: bool
    reason: str
    tool_queries: Tuple[Tuple[str, str], ...] = ()
    tool_query_goals: Tuple[Tuple[str, str, str], ...] = ()

    @property
    def required(self) -> bool:
        """是否存在必做检索工具。"""
        return bool(self.required_tools)


def assign_evidence_tasks(
    tasks: Iterable[EvidenceTask],
    selected_experts: Sequence[str],
) -> Tuple[EvidenceTask, ...]:
    """确保每条证据任务恰好有一名在场专家负责人。"""
    selected = [str(key) for key in selected_experts if str(key)]
    if not selected:
        return ()
    assigned = []
    for task in tasks:
        preferred = task.owner or CAPABILITY_DEFAULT_OWNER.get(task.capability, "clinical")
        if preferred not in selected:
            default = CAPABILITY_DEFAULT_OWNER.get(task.capability, "clinical")
            preferred = default if default in selected else selected[0]
        assigned.append(EvidenceTask(
            capability=task.capability,
            owner=preferred,
            requirement=task.requirement,
            reason=task.reason,
            web_fallback_on_weak_local=task.web_fallback_on_weak_local,
            query=task.query,
        ))
    return tuple(assigned)


def resolve_retrieval_requirement(
    *,
    expert_key: str,
    evidence_tasks: Sequence[EvidenceTask] = (),
) -> RetrievalRequirement:
    """把分配给某专家的证据任务映射为必需/建议工具及逐任务查询。

    同一工具的不同查询会全部保留，只去除完全相同的 tool/query 对；建议工具不会与
    必需工具重复。RAG 任务还可携带弱本地证据时的 Web 补证标志与证据目标。
    """
    required = []
    recommended = []
    reasons = []
    web_fallback = False
    # 每个证据任务保留一条查询。不同任务可能映射到同一工具，不得互相覆盖；
    # 完全重复的调用稍后由 ToolBroker 按工具名 + 规范化参数去重。
    tool_queries = []
    tool_query_goals = []
    for task in evidence_tasks:
        if task.owner != expert_key:
            continue
        tool_name = CAPABILITY_TO_TOOL.get(task.capability)
        if not tool_name:
            continue
        target = required if task.requirement == "required" else recommended
        if tool_name not in target:
            target.append(tool_name)
        if task.reason and task.reason not in reasons:
            reasons.append(task.reason)
        query_pair = (tool_name, task.query)
        if task.query and query_pair not in tool_queries:
            tool_queries.append(query_pair)
            tool_query_goals.append((tool_name, task.query, task.reason))
        if task.web_fallback_on_weak_local and tool_name == RAG_TOOL:
            web_fallback = True

    recommended = [tool for tool in recommended if tool not in required]
    return RetrievalRequirement(
        required_tools=tuple(required),
        recommended_tools=tuple(recommended),
        require_web_on_rag_failure=web_fallback,
        reason="；".join(reasons) or "统一任务策略未分配外部证据任务，由专家自主判断",
        tool_queries=tuple(tool_queries),
        tool_query_goals=tuple(tool_query_goals),
    )


def rag_requires_web_fallback(result: Dict[str, Any], *, ok: bool) -> bool:
    """判断强制本地证据尝试是否需要外部回退。"""
    if not ok or not isinstance(result, dict):
        return True
    hits = result.get("hits")
    if not isinstance(hits, list):
        return True
    try:
        min_hits = max(1, int(os.getenv("RAG_WEB_FALLBACK_MIN_HITS", "2") or 2))
    except (TypeError, ValueError):
        min_hits = 2
    try:
        threshold = float(os.getenv("RAG_RELEVANCE_THRESHOLD", "0.90") or 0.90)
    except (TypeError, ValueError):
        threshold = 0.90
    best_score = max(
        (float(hit.get("score", 0.0)) for hit in hits if isinstance(hit, dict)),
        default=0.0,
    )
    return len(hits) < min_hits or best_score < threshold


__all__ = [
    "CAPABILITY_DEFAULT_OWNER",
    "CAPABILITY_TO_TOOL",
    "EvidenceTask",
    "RAG_TOOL",
    "RetrievalRequirement",
    "WEB_SEARCH_TOOL",
    "VITALS_TOOL",
    "assign_evidence_tasks",
    "rag_requires_web_fallback",
    "resolve_retrieval_requirement",
]
