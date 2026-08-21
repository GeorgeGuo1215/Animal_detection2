"""MoE 下游专家委员会：任务驱动工具执行与单轮结构化意见。"""
from __future__ import annotations

import asyncio
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from ...context.request_context import ANIMAL_REQUIRED_TOOLS, get_request_animal_id
from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...prompts.moe_experts import (
    AUDIENCE_OWNER,
    AUDIENCE_VET,
    EXPERT_PERSONAS,
    FORCE_FINAL_REMINDER,
    IMPORTANT_RETRIEVAL_POLICY,
    OUTPUT_CONTRACT,
    SPECIES_BREED_GUARD,
    SPECIES_GUARD,
    build_expert_system_prompt,
)
from ...tools.rag_query import is_english_rag_query
from ...tools.tool_registry import ToolRegistry
from ..structured_output import safe_json_loads
from ..tool_call_utils import canonical_tool_call
from .evidence_sufficiency import (
    EvidenceSufficiencyAssessment,
    EvidenceSufficiencyItem,
    assess_evidence_sufficiency,
    evidence_sufficiency_enabled,
)
from .tool_broker import ToolBroker, ToolRequest, ToolResult
from .history_context import build_fact_state_history
from .retrieval_policy import (
    RAG_TOOL,
    RetrievalRequirement,
    WEB_SEARCH_TOOL,
    rag_requires_web_fallback,
    resolve_retrieval_requirement,
)
from .trace import MoETrace, extract_usage


# 是否给每个专家暴露 registry 中「全部可用工具」（排除管理类）。
# True：每个专家 Subagent 都能看到 rag/sql/vitals/web_search/nutritional_planner 等
#       全部已注册工具，最大化功能覆盖（各 ExpertConfig.allowed_tools 退化为“偏好提示”，
#       不再作为硬门控）；未注册的工具（如未配置 TAVILY 时的 web_search）自然不会出现。
# False：回退到各专家静态 allowed_tools 子集。
EXPOSE_ALL_EXPERT_TOOLS = True

# 管理/重活类工具：即使全量暴露也不交给专家（避免误触发重建索引等）。
_ADMIN_TOOLS = frozenset({"rag.reindex", "debug.echo"})


@dataclass(frozen=True)
class ExpertConfig:
    """单个专家的 persona、工具偏好与 RAG 类目。"""
    key: str
    name_zh: str
    persona: str
    allowed_tools: List[str] = field(default_factory=list)
    rag_query_hint: str = ""
    # rag.search 的次级知识类目（精确 id 或 prefix.* 通配）。
    rag_categories: List[str] = field(default_factory=list)


_CLINICAL_RAG_CATEGORIES = [
    "basic.*",
    "clinical.*",
    "diagnostics.*",
    "clinical_skills.*",
    "integrative.*",
    "anesthesia.default",
    "immunology.default",
    "reproduction.default",
    "infectious.*",
    "exotic.default",
    "equine.*",
    "individual.*",
    "guidelines.*",
    "zoonosis.toxoplasmosis",
]

_NUTRITION_RAG_CATEGORIES = [
    "nutrition.*",
    "equine.nutrition",
    "integrative.general",
]

_PHARMACY_RAG_CATEGORIES = [
    "pharmacy.*",
    "basic.pharmacology_fundamentals",
    "anesthesia.default",
]

_BEHAVIOR_RAG_CATEGORIES = [
    "behavior.*",
]


_SPECIES_GUARD = SPECIES_GUARD
_SPECIES_BREED_GUARD = SPECIES_BREED_GUARD
_OUTPUT_CONTRACT = OUTPUT_CONTRACT
_AUDIENCE_OWNER = AUDIENCE_OWNER
_AUDIENCE_VET = AUDIENCE_VET
_IMPORTANT_RETRIEVAL_POLICY = IMPORTANT_RETRIEVAL_POLICY


EXPERTS: Dict[str, ExpertConfig] = {
    "clinical": ExpertConfig(
        key="clinical",
        name_zh="兽医临床专家",
        persona=EXPERT_PERSONAS["clinical"],
        allowed_tools=[
            "rag.search",
            "sql.search",
            "vitals.summary",
            "mcp.web_search.web_search",
        ],
        rag_query_hint="clinical signs differential diagnosis treatment",
        rag_categories=list(_CLINICAL_RAG_CATEGORIES),
    ),
    "nutrition": ExpertConfig(
        key="nutrition",
        name_zh="兽医营养专家",
        persona=EXPERT_PERSONAS["nutrition"],
        allowed_tools=[
            "rag.search",
            "sql.search",
            "mcp.nutritional_planner.calculate_meal_plan",
            "mcp.nutritional_planner.generate_exercise_plan",
            "mcp.web_search.ingredient_check",
            "mcp.web_search.web_search",
        ],
        rag_query_hint="nutrition diet calorie requirement feeding",
        rag_categories=list(_NUTRITION_RAG_CATEGORIES),
    ),
    "pharmacy": ExpertConfig(
        key="pharmacy",
        name_zh="兽医药剂师",
        persona=EXPERT_PERSONAS["pharmacy"],
        allowed_tools=["rag.search", "mcp.web_search.web_search"],
        rag_query_hint="drug dosage contraindication toxicity interaction",
        rag_categories=list(_PHARMACY_RAG_CATEGORIES),
    ),
    "behavior": ExpertConfig(
        key="behavior",
        name_zh="行为安抚老师",
        persona=EXPERT_PERSONAS["behavior"],
        allowed_tools=["rag.search", "mcp.web_search.web_search"],
        rag_query_hint="animal behavior anxiety stress training",
        rag_categories=list(_BEHAVIOR_RAG_CATEGORIES),
    ),
}


def _rag_metrics(result: Dict[str, Any]) -> tuple[int, float]:
    """从 RAG 结果提取命中数与最高分。"""
    hits = result.get("hits") if isinstance(result, dict) else None
    if not isinstance(hits, list) or not hits:
        return 0, 0.0
    best = max((float(h.get("score", 0.0)) for h in hits if isinstance(h, dict)), default=0.0)
    return len(hits), best


def _build_evidence_block(result: Dict[str, Any], max_chars: int = 2400) -> str:
    """把 rag.search 命中拼成精简证据块，供专家阅读。"""
    if not isinstance(result, dict):
        return "（无检索结果）"
    hits = result.get("hits") or []
    if not hits:
        return "（知识库未命中相关内容）"
    parts: List[str] = []
    for i, h in enumerate(hits, start=1):
        if not isinstance(h, dict):
            continue
        src = h.get("source_path") or "unknown"
        text = (h.get("text") or "").strip()
        parts.append(f"[{i}] 来源: {src}\n{text}")
    block = "\n\n".join(parts)
    return block[:max_chars]


def _env_int(name: str, default: int) -> int:
    """读取正整数环境变量。"""
    try:
        value = int(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default
    return max(1, value)


def _env_nonnegative_int(name: str, default: int) -> int:
    """读取非负整数环境变量。"""
    try:
        value = int(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default
    return max(0, value)


def _env_float(name: str, default: float) -> float:
    """读取浮点环境变量。"""
    try:
        value = float(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default
    return max(0.1, value)


@dataclass(frozen=True)
class ExpertLoopConfig:
    """专家循环的超时与轮次配置。"""
    timeout_s: float = field(default_factory=lambda: _env_float("MOE_EXPERT_TIMEOUT_SEC", 120.0))
    final_max_tokens: int = field(
        default_factory=lambda: _env_int("MOE_EXPERT_FINAL_MAX_TOKENS", 1400)
    )
    repair_attempts: int = field(
        default_factory=lambda: _env_nonnegative_int("MOE_EXPERT_FORMAT_REPAIR_ATTEMPTS", 1)
    )
    finalize_reserve_s: float = field(
        default_factory=lambda: _env_float("MOE_EXPERT_FINALIZE_RESERVE_SEC", 12.0)
    )


def _tool_result_context(result: ToolResult, max_chars: int = 4000) -> str:
    """将工具结果格式化为上下文文本。"""
    if result.tool_name == "rag.search" and isinstance(result.result, dict):
        content = _build_evidence_block(result.result, max_chars=max_chars)
    else:
        try:
            content = json.dumps(result.result, ensure_ascii=False)
        except (TypeError, ValueError):
            content = str(result.result)
        content = content[:max_chars]
    envelope = {
        "tool_name": result.tool_name,
        "ok": result.ok,
        "error": result.error,
        "shared_result": result.shared,
        "content": content,
    }
    return json.dumps(envelope, ensure_ascii=False)


def _tool_feedback_message(content: str) -> Dict[str, str]:
    """构造工具执行反馈消息。"""
    return {
        "role": "user",
        "content": f"TOOL_RESULT\n{content}\n请基于该结果决定下一步动作。",
    }


class ExpertAgentSession:
    """单个专家 Subagent 会话：检索状态、工具请求与最终意见。"""

    def __init__(
        self,
        *,
        expert: ExpertConfig,
        query: str,
        weight: float,
        registry: ToolRegistry,
        llm: AsyncOpenAIClient,
        request_allowed_tools: Optional[Sequence[str]] = None,
        rag_top_k: int = 5,
        device: Optional[str] = None,
        species_en: Optional[str] = None,
        species_zh: Optional[str] = None,
        breed: Optional[str] = None,
        user_role: str = "pet_owner",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: Optional[str] = None,
        intent_id: str = "",
        emergency: bool = False,
        retrieval_requirement: Optional[RetrievalRequirement] = None,
        recorder: Optional[MoETrace] = None,
        loop_config: Optional[ExpertLoopConfig] = None,
    ) -> None:
        """初始化专家身份、上下文、检索任务、可见工具和整轮超时预算。

        工具集合先排除管理类，再依次应用专家偏好开关、请求白名单和 animal_id 条件；
        必需但不可见的工具会单独记录。构造出的首条用户载荷包含事实状态历史与当前
        retrieval_state，后续工具结果和最终意见都在该独立会话内累积。
        """
        self.expert = expert
        self.query = query
        self.weight = float(weight)
        self.registry = registry
        self.llm = llm
        self.rag_top_k = int(rag_top_k)
        self.device = device
        self.species_en = species_en
        self.species_zh = species_zh
        self.breed = breed
        self.user_role = user_role
        self.intent_id = str(intent_id or "").strip().upper()
        self.emergency = bool(emergency)
        self.recorder = recorder
        self.loop_config = loop_config or ExpertLoopConfig()
        self.started_at = time.monotonic()
        self.rounds = 0
        self.tool_results: List[Dict[str, Any]] = []
        self.plan_steps: List[Dict[str, Any]] = []
        self.tools_used: List[str] = []
        self.attempted_tools: List[str] = []
        self.completed_tools: List[str] = []
        self.successful_tools: List[str] = []
        self.required_tools: List[str] = []
        self.recommended_tools: List[str] = []
        self.unavailable_required_tools: List[str] = []
        self._attempted_request_keys: set[str] = set()
        self._required_request_tools: Dict[str, str] = {}
        self._completed_request_keys: set[str] = set()
        self._successful_request_keys: set[str] = set()
        self.tool_timeout_occurred = False
        self.last_output = ""
        self.completed = False

        available = [tool for tool in registry.list_tools() if tool.name not in _ADMIN_TOOLS]
        if not EXPOSE_ALL_EXPERT_TOOLS:
            preferred = set(expert.allowed_tools)
            available = [tool for tool in available if tool.name in preferred]
        if request_allowed_tools is not None:
            allowed = set(request_allowed_tools)
            available = [tool for tool in available if tool.name in allowed]
        if not get_request_animal_id():
            available = [tool for tool in available if tool.name not in ANIMAL_REQUIRED_TOOLS]
        self.available_tools = {tool.name: tool for tool in available}

        retrieval = retrieval_requirement or resolve_retrieval_requirement(
            expert_key=expert.key,
            evidence_tasks=(),
        )
        self.retrieval_required = retrieval.required
        self.retrieval_reason = retrieval.reason
        self.require_web_on_rag_failure = retrieval.require_web_on_rag_failure
        self.tool_queries: Dict[str, List[str]] = {}
        self.tool_query_goals: Dict[tuple[str, str], str] = {}
        for tool_name, tool_query in retrieval.tool_queries:
            queries = self.tool_queries.setdefault(tool_name, [])
            if tool_query and tool_query not in queries:
                queries.append(tool_query)
        for tool_name, tool_query, evidence_goal in retrieval.tool_query_goals:
            if tool_query and evidence_goal:
                self.tool_query_goals[(tool_name, tool_query)] = evidence_goal
        for tool_name in retrieval.required_tools:
            if tool_name in self.available_tools:
                self.required_tools.append(tool_name)
            else:
                self.unavailable_required_tools.append(tool_name)
        self.recommended_tools = [
            tool_name
            for tool_name in retrieval.recommended_tools
            if tool_name in self.available_tools and tool_name not in self.required_tools
        ]
        self._register_required_request_keys()

        system_prompt = build_expert_system_prompt(
            persona=expert.persona,
            expert_key=expert.key,
            user_role=user_role,
        )
        payload: Dict[str, Any] = {
            "user_question": query,
            "user_role": user_role,
        }
        if species_zh:
            payload["species"] = species_zh
        if breed:
            payload["breed"] = breed
        history_context = build_fact_state_history(
            conversation_history,
            expert_context_history,
            user_memory=user_memory,
        )
        if history_context:
            payload["history_context"] = history_context
        payload["retrieval_policy"] = self.retrieval_state()
        self.messages: List[Dict[str, Any]] = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
        ]

    @property
    def remaining_timeout_s(self) -> float:
        """剩余超时秒数。"""
        elapsed = time.monotonic() - self.started_at
        return max(0.0, self.loop_config.timeout_s - elapsed)

    @property
    def finalize_reserve_s(self) -> float:
        """终答预留秒数。"""
        return min(self.loop_config.finalize_reserve_s, self.loop_config.timeout_s * 0.25)

    @property
    def tool_wait_timeout_s(self) -> float:
        """单次工具等待超时。"""
        return max(0.001, self.remaining_timeout_s - self.finalize_reserve_s)

    @property
    def pending_required_tools(self) -> List[str]:
        """尚未完成的必做工具。"""
        pending = [
            tool_name
            for request_key, tool_name in self._required_request_tools.items()
            if request_key not in self._completed_request_keys
        ]
        return list(dict.fromkeys(pending))

    def _queries_for_tool(self, tool_name: str) -> List[str]:
        """某工具对应的查询列表。"""
        return list(self.tool_queries.get(tool_name) or [""])

    def _register_required_request_keys(self) -> None:
        """登记必做请求键。"""
        for tool_name in self.required_tools:
            for assigned_query in self._queries_for_tool(tool_name):
                arguments = self._normalize_tool_arguments(
                    tool_name, {}, assigned_query=assigned_query
                )
                self._required_request_tools[
                    canonical_tool_call(tool_name, arguments)
                ] = tool_name

    def retrieval_state(self) -> Dict[str, Any]:
        """当前检索状态快照。"""
        return {
            "retrieval_required": self.retrieval_required,
            "reason": self.retrieval_reason,
            "required_tools": list(self.required_tools),
            "recommended_tools": list(self.recommended_tools),
            "attempted_tools": list(self.attempted_tools),
            "completed_tools": list(self.completed_tools),
            "successful_tools": list(self.successful_tools),
            "pending_tools": self.pending_required_tools,
            "unavailable_required_tools": list(self.unavailable_required_tools),
        }

    def _retrieval_state_message(self) -> Dict[str, str]:
        """将检索状态格式化为提示消息。"""
        return {
            "role": "user",
            "content": (
                "RETRIEVAL_STATE\n"
                + json.dumps(self.retrieval_state(), ensure_ascii=False)
                + "\n只有 pending_tools 非空时才必须按顺序调用对应工具并禁止 final；"
                "recommended_tools 只是质量建议，可基于现有证据直接 final。"
            ),
        }

    def _normalize_tool_arguments(
        self,
        tool_name: str,
        arguments: Dict[str, Any],
        *,
        assigned_query: str = "",
    ) -> Dict[str, Any]:
        """规范化工具参数。"""
        normalized = dict(arguments or {})
        if tool_name == "rag.search":
            species_term = f" {self.species_en}" if self.species_en else ""
            breed_term = f" {self.breed}" if is_english_rag_query(self.breed) else ""
            default_query = f"{self.expert.rag_query_hint}{species_term}{breed_term}".strip()
            assigned_query = str(assigned_query or "").strip()
            normalized.setdefault(
                "query",
                assigned_query if is_english_rag_query(assigned_query) else default_query,
            )
            normalized.setdefault("top_k", self.rag_top_k)
            if self.device is not None:
                normalized.setdefault("device", self.device)
            if self.expert.rag_categories:
                normalized["category"] = list(self.expert.rag_categories)
            normalized.setdefault("rerank", True)
        elif tool_name == "mcp.web_search.web_search":
            normalized.setdefault("query", assigned_query or self.query)
            normalized.setdefault("max_results", 5)
        return normalized

    def prepare_tool_requests(self) -> List[ToolRequest]:
        """把必需/建议工具及每个证据任务展开为独立的首轮 ToolRequest。

        仅生成当前专家可见的工具；同一工具的不同查询均予保留，完全重复调用交由
        ToolBroker 按工具名和规范化参数去重。
        """
        self.rounds = 1
        planned = list(dict.fromkeys([*self.required_tools, *self.recommended_tools]))
        requests: List[ToolRequest] = []
        for tool_name in planned:
            if tool_name not in self.available_tools:
                continue
            for assigned_query in self._queries_for_tool(tool_name):
                arguments = self._normalize_tool_arguments(
                    tool_name, {}, assigned_query=assigned_query
                )
                request_key = canonical_tool_call(tool_name, arguments)
                if request_key in self._attempted_request_keys:
                    continue
                self._attempted_request_keys.add(request_key)
                if tool_name in self.required_tools:
                    self._required_request_tools[request_key] = tool_name
                self.attempted_tools.append(tool_name)
                self.plan_steps.append({
                    "type": "tool",
                    "tool_name": tool_name,
                    "arguments": dict(arguments),
                    "note": self.retrieval_reason,
                    "round": 1,
                    "required": tool_name in self.required_tools,
                })
                requests.append(ToolRequest(
                    expert=self.expert.key,
                    tool_name=tool_name,
                    arguments=arguments,
                ).with_request_id())
        return requests

    def evidence_sufficiency_item(
        self,
        result: ToolResult,
    ) -> Optional[EvidenceSufficiencyItem]:
        """构造证据充分性审计条目。"""
        if (
            result.tool_name != RAG_TOOL
            or not self.require_web_on_rag_failure
            or rag_requires_web_fallback(result.result, ok=result.ok)
            or not isinstance(result.result, dict)
        ):
            return None
        hits = result.result.get("hits")
        if not isinstance(hits, list):
            return None
        return EvidenceSufficiencyItem(
            id=result.request_id,
            expert=self.expert.key,
            evidence_query=str(result.arguments.get("query") or ""),
            evidence_goal=self.tool_query_goals.get(
                (RAG_TOOL, str(result.arguments.get("query") or "")),
                self.retrieval_reason,
            ),
            hits=tuple(hit for hit in hits if isinstance(hit, dict)),
        )

    def _parse_opinion(self, obj: Dict[str, Any]) -> Dict[str, Any]:
        """校验并规范化 ``action=final`` 的专家 JSON 意见。

        统一结论、证据、风险、置信度和检索状态，并依据实际成功工具修正模型自报的
        ``tools_used``；缺少结论或仍有必需工具未完成时拒绝作为有效终答。
        """
        opinion = obj.get("opinion") if isinstance(obj.get("opinion"), dict) else obj
        conclusion = str(opinion.get("conclusion") or opinion.get("answer") or "").strip()
        evidence = opinion.get("evidence")
        risks = opinion.get("risks")
        evidence_items = [str(item) for item in evidence] if isinstance(evidence, list) else []
        risk_items = [str(item) for item in risks] if isinstance(risks, list) else []
        has_rag = RAG_TOOL in self.successful_tools
        has_web = WEB_SEARCH_TOOL in self.successful_tools
        guarded_evidence: List[str] = []
        removed_source_claim = False
        for item in evidence_items:
            lowered = item.lower()
            claims_rag = any(token in lowered for token in ("本地知识库", "rag", "书籍页码"))
            claims_web = any(token in lowered for token in (
                "网络证据", "网络检索", "网络来源", "网页来源", "http://", "https://",
            ))
            claims_external_source = any(token in lowered for token in (
                "来源：", "来源:", "指南", "共识", "文献", "研究显示",
            ))
            if (claims_rag and not has_rag) or (claims_web and not has_web) or (
                claims_external_source and not (has_rag or has_web)
            ):
                removed_source_claim = True
                continue
            guarded_evidence.append(item)
        if removed_source_claim:
            risk_items.append("已移除未由本轮工具结果支撑的外部来源声明")
        try:
            confidence = max(0.0, min(1.0, float(opinion.get("confidence", 0.0))))
        except (TypeError, ValueError):
            confidence = 0.0
        return {
            "conclusion": conclusion or self.last_output[:800] or "（专家未生成有效结论）",
            "evidence": guarded_evidence,
            "risks": risk_items,
            "confidence": round(confidence, 3),
        }

    async def generate_final_opinion(self) -> Dict[str, Any]:
        """生成 action=final 的结构化专家意见。"""
        self.rounds = 1
        messages = [*self.messages, self._retrieval_state_message(), {
            "role": "user",
            "content": FORCE_FINAL_REMINDER,
        }]
        attempts = 1 + self.loop_config.repair_attempts
        for attempt in range(attempts):
            started = time.perf_counter()
            response: Dict[str, Any] = {}
            try:
                response = await self.llm.chat(
                    messages=messages,
                    temperature=0.2,
                    max_tokens=self.loop_config.final_max_tokens,
                    response_format={"type": "json_object"},
                    thinking=False,
                )
                text = extract_text(response)
            except Exception as exc:  # noqa: BLE001
                text = ""
                error = str(exc)
            else:
                error = ""
            self.last_output = text
            if self.recorder is not None:
                self.recorder.record_llm(
                    stage=(
                        f"expert:{self.expert.key}:final"
                        if attempt == 0
                        else f"expert:{self.expert.key}:format_repair:{attempt}"
                    ),
                    model=getattr(self.llm, "model", ""),
                    messages=messages,
                    output=text,
                    latency_ms=(time.perf_counter() - started) * 1000.0,
                    usage=extract_usage(response),
                    meta={
                        "weight": self.weight,
                        "single_pass": True,
                        "format_repair": attempt > 0,
                        "error": error,
                        "retrieval_state": self.retrieval_state(),
                    },
                )
            obj, _parse_error = safe_json_loads(text)
            if (
                isinstance(obj, dict)
                and str(obj.get("action") or "").strip().lower() == "final"
                and isinstance(obj.get("opinion"), dict)
            ):
                self.completed = True
                return self._parse_opinion(obj)
            if attempt + 1 < attempts:
                messages = [*messages]
                if text:
                    messages.append({"role": "assistant", "content": text})
                messages.append({
                    "role": "user",
                    "content": (
                        "FORMAT_REPAIR：上一次输出为空或不符合协议。不要解释、不要调用工具，"
                        "只返回 {\"action\":\"final\",\"opinion\":{...}} 的完整 JSON。"
                    ),
                })
        self.completed = True
        return self.fallback_opinion()

    def apply_tool_result(
        self,
        result: ToolResult,
        *,
        sufficiency: Optional[EvidenceSufficiencyAssessment] = None,
        semantic_assessment_required: bool = False,
    ) -> None:
        """记录工具反馈，并更新成功工具、请求键、RAG 指标与补证状态。

        对本地 RAG 结果可结合语义充分性评估决定是否追加 Web 补证；失败、超时和
        低相关结果均保留在结构化工具记录中，供最终意见如实说明证据限制。
        """
        self.messages.append(_tool_feedback_message(_tool_result_context(result)))
        self.tools_used.append(result.tool_name)
        result_code = result.result.get("code") if isinstance(result.result, dict) else ""
        if result_code == "EXPERT_TOOL_TIMEOUT":
            self.tool_timeout_occurred = True
        validation_failure = result_code in {"RAG_QUERY_MUST_BE_ENGLISH", "TOOL_NOT_ALLOWED"}
        request_key = canonical_tool_call(result.tool_name, result.arguments)
        if not validation_failure:
            self._completed_request_keys.add(request_key)
            if result.tool_name not in self.completed_tools:
                self.completed_tools.append(result.tool_name)
            if result.ok:
                self._successful_request_keys.add(request_key)
                if result.tool_name not in self.successful_tools:
                    self.successful_tools.append(result.tool_name)

        numeric_weak = (
            result.tool_name == RAG_TOOL
            and self.require_web_on_rag_failure
            and rag_requires_web_fallback(result.result, ok=result.ok)
        )
        semantic_weak = (
            semantic_assessment_required
            and (sufficiency is None or not sufficiency.supported)
        )
        if (
            result.tool_name == RAG_TOOL
            and self.require_web_on_rag_failure
            and (numeric_weak or semantic_weak)
            and WEB_SEARCH_TOOL not in self.required_tools
        ):
            if WEB_SEARCH_TOOL in self.available_tools:
                self.required_tools.append(WEB_SEARCH_TOOL)
                self.recommended_tools = [
                    name for name in self.recommended_tools if name != WEB_SEARCH_TOOL
                ]
                self._register_required_request_keys()
            elif WEB_SEARCH_TOOL not in self.unavailable_required_tools:
                self.unavailable_required_tools.append(WEB_SEARCH_TOOL)
        sufficiency_record: Optional[Dict[str, Any]] = None
        if result.tool_name == RAG_TOOL and self.require_web_on_rag_failure:
            if numeric_weak:
                sufficiency_record = {
                    "status": "unsupported",
                    "reason": "本地命中数量或相关性分数未通过数值门槛",
                    "matched_hit_ids": [],
                    "method": "numeric_gate",
                }
            elif semantic_assessment_required and sufficiency is None:
                sufficiency_record = {
                    "status": "unknown",
                    "reason": "语义充分性判断超时、失败或未返回该证据项，按未核实处理",
                    "matched_hit_ids": [],
                    "method": "semantic_llm",
                }
            elif sufficiency is not None:
                sufficiency_record = {**sufficiency.as_dict(), "method": "semantic_llm"}
        record = {
            "step": len(self.tool_results),
            "round": self.rounds,
            "tool_name": result.tool_name,
            "arguments": dict(result.arguments),
            "result": result.result,
            "ok": result.ok,
            "latency_ms": result.latency_ms,
            "error": result.error,
            "shared": result.shared,
            "sufficiency": sufficiency_record,
        }
        self.tool_results.append(record)

        if self.recorder is None:
            return
        if result.tool_name == "rag.search" and isinstance(result.result, dict):
            hits_count, best_score = _rag_metrics(result.result)
            self.recorder.record_rag(
                stage=f"expert:{self.expert.key}:round:{self.rounds}",
                query=str(result.arguments.get("query") or ""),
                hits_count=hits_count,
                best_score=best_score,
                latency_ms=result.latency_ms,
            )
        self.recorder.record_tool(
            stage=f"expert:{self.expert.key}:round:{self.rounds}",
            tool_name=result.tool_name,
            arguments=result.arguments,
            ok=result.ok,
            latency_ms=result.latency_ms,
            error=result.error,
        )

    def fallback_opinion(self) -> Dict[str, Any]:
        """超时或失败时的兜底意见。"""
        pending = self.pending_required_tools
        retrieval_risk = (
            f"必需检索尚未完成：{', '.join(pending)}；不得把该意见作为已核验结论"
            if pending
            else "专家单轮结构化输出为空、格式无效或超时，结论不可作为主要决策依据"
        )
        return {
            "conclusion": "（专家未生成有效结构化结论）",
            "evidence": [],
            "risks": [retrieval_risk],
            "confidence": 0.0,
        }

    def build_result(self, opinion: Dict[str, Any]) -> Dict[str, Any]:
        """合并专家意见、工具审计、检索指标和执行状态，生成运行结果。

        RAG 命中数与最高分由真实工具结果计算，工具使用记录不采信模型自报值；结果
        同时携带 required/recommended/pending 状态供 Critic、Aggregator 与持久化层使用。
        """
        hits_count = 0
        best_score = 0.0
        for result in self.tool_results:
            if result.get("tool_name") != "rag.search" or not isinstance(result.get("result"), dict):
                continue
            count, score = _rag_metrics(result["result"])
            hits_count += count
            best_score = max(best_score, score)
        risks = list(opinion.get("risks") or [])
        failed_required = list(dict.fromkeys(
            tool_name
            for request_key, tool_name in self._required_request_tools.items()
            if request_key in self._completed_request_keys
            and request_key not in self._successful_request_keys
        ))
        if self.retrieval_required and self.unavailable_required_tools:
            risks.append(
                "外部证据核验工具不可用：" + ", ".join(self.unavailable_required_tools)
            )
        if failed_required:
            risks.append("外部证据核验调用失败：" + ", ".join(failed_required))
        return {
            "expert": self.expert.key,
            "name_zh": self.expert.name_zh,
            "weight": round(self.weight, 4),
            "conclusion": str(opinion.get("conclusion") or ""),
            "evidence": list(opinion.get("evidence") or []),
            "risks": risks,
            "confidence": round(float(opinion.get("confidence") or 0.0), 3),
            "rag_hits": hits_count,
            "rag_best_score": round(best_score, 4),
            "tools_used": sorted(set(self.tools_used)),
            "retrieval_required": self.retrieval_required,
            "retrieval_reason": self.retrieval_reason,
            "required_tools": list(self.required_tools),
            "recommended_tools": list(self.recommended_tools),
            "attempted_tools": list(self.attempted_tools),
            "completed_tools": list(self.completed_tools),
            "successful_tools": list(self.successful_tools),
            "pending_tools": self.pending_required_tools,
            "unavailable_required_tools": list(self.unavailable_required_tools),
            "plan_steps": list(self.plan_steps),
            "tool_results": list(self.tool_results),
            "evidence_sufficiency": [
                result["sufficiency"]
                for result in self.tool_results
                if isinstance(result.get("sufficiency"), dict)
            ],
            "rounds": self.rounds,
        }


async def run_expert_sessions(
    *,
    sessions: Sequence[ExpertAgentSession],
    broker: ToolBroker,
) -> List[Dict[str, Any]]:
    """并行完成多名专家的工具阶段与单轮结构化终答。

    工具最多执行“已分配任务”和“弱本地证据触发 Web 补证”两波；随后各专家只做
    一次最终意见生成。超时或异常按专家隔离并转为安全兜底，不启动旧式多轮循环。
    """
    async def _execute_assigned_tools() -> None:
        # At most two deterministic waves: assigned tasks, then weak-local web fallback.
        """批量执行首波证据任务，并在需要时执行一次 Web 补证波次。"""
        for _wave in range(2):
            ownership: Dict[str, ExpertAgentSession] = {}
            requests: List[ToolRequest] = []
            timeouts: Dict[str, float] = {}
            for session in sessions:
                for request in session.prepare_tool_requests():
                    ownership[request.request_id] = session
                    requests.append(request)
                    timeouts[request.request_id] = session.tool_wait_timeout_s
            if not requests:
                break
            results = await broker.execute_batch(requests, timeouts=timeouts)
            semantic_candidates: Dict[str, EvidenceSufficiencyItem] = {}
            if evidence_sufficiency_enabled():
                for request in requests:
                    result = results.get(request.request_id)
                    if result is None:
                        continue
                    item = ownership[request.request_id].evidence_sufficiency_item(result)
                    if item is not None:
                        semantic_candidates[request.request_id] = item

            assessments: Dict[str, EvidenceSufficiencyAssessment] = {}
            if semantic_candidates:
                candidate_sessions = [ownership[item_id] for item_id in semantic_candidates]
                timeout_s = min(session.tool_wait_timeout_s for session in candidate_sessions)
                first_session = candidate_sessions[0]
                assessments = await assess_evidence_sufficiency(
                    case_question=first_session.query,
                    items=semantic_candidates.values(),
                    llm=first_session.llm,
                    recorder=next(
                        (session.recorder for session in candidate_sessions if session.recorder),
                        None,
                    ),
                    timeout_s=timeout_s,
                )
            for request in requests:
                result = results.get(request.request_id)
                if result is not None:
                    ownership[request.request_id].apply_tool_result(
                        result,
                        sufficiency=assessments.get(request.request_id),
                        semantic_assessment_required=request.request_id in semantic_candidates,
                    )

    await _execute_assigned_tools()

    async def _finalize(session: ExpertAgentSession) -> Dict[str, Any]:
        """收尾并产出最终意见。"""
        try:
            opinion = await asyncio.wait_for(
                session.generate_final_opinion(),
                timeout=max(0.001, session.remaining_timeout_s),
            )
        except asyncio.TimeoutError:
            opinion = session.fallback_opinion()
        except Exception as exc:  # noqa: BLE001
            session.last_output = f"（专家单轮生成失败：{exc}）"
            opinion = session.fallback_opinion()
        return session.build_result(opinion)

    return list(await asyncio.gather(*(_finalize(session) for session in sessions)))


async def run_expert(
    *,
    expert: ExpertConfig,
    query: str,
    weight: float,
    registry: ToolRegistry,
    llm: AsyncOpenAIClient,
    rag_top_k: int = 5,
    device: Optional[str] = None,
    species_en: Optional[str] = None,
    species_zh: Optional[str] = None,
    breed: Optional[str] = None,
    user_role: str = "pet_owner",
    conversation_history: Optional[List[Dict[str, str]]] = None,
    expert_context_history: Optional[List[Dict[str, Any]]] = None,
    user_memory: Optional[str] = None,
    intent_id: str = "",
    emergency: bool = False,
    retrieval_requirement: Optional[RetrievalRequirement] = None,
    recorder: Optional[MoETrace] = None,
    request_allowed_tools: Optional[Sequence[str]] = None,
    loop_config: Optional[ExpertLoopConfig] = None,
) -> Dict[str, Any]:
    """兼容入口：构造单个 ExpertAgentSession 与请求级 ToolBroker 后执行会诊。

    新编排器通常使用 ``run_expert_sessions`` 批量并发；本函数保留旧调用方所需的
    单专家返回结构，但仍遵循同一单轮终答、工具去重和安全兜底规则。
    """
    session = ExpertAgentSession(
        expert=expert,
        query=query,
        weight=weight,
        registry=registry,
        llm=llm,
        request_allowed_tools=request_allowed_tools,
        rag_top_k=rag_top_k,
        device=device,
        species_en=species_en,
        species_zh=species_zh,
        breed=breed,
        user_role=user_role,
        conversation_history=conversation_history,
        expert_context_history=expert_context_history,
        user_memory=user_memory,
        intent_id=intent_id,
        emergency=emergency,
        retrieval_requirement=retrieval_requirement,
        recorder=recorder,
        loop_config=loop_config,
    )
    broker = ToolBroker(registry=registry, allowed_tools=request_allowed_tools)
    return (await run_expert_sessions(sessions=[session], broker=broker))[0]
