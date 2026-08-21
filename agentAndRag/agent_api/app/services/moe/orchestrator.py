"""MoE 编排器：统一任务策略 → 确定性门控 → 并行专家 → Critic → 融合生成。

两个入口共享同一组内部阶段函数：
- `stream(...)`  生产路径：异步产出事件 dict（content/status/detail/finish），仅最终答案 token 流式；
- `run(...)`     评测路径：非流式跑完，返回 (final_answer, MoETrace)。
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import time
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator, Dict, List, Optional, Tuple

from ...concurrency import ResourceBusyError
from ...llm.llm_client import AsyncOpenAIClient, extract_text, get_shared_async_client
from ...llm.llm_client_stream import AsyncOpenAIStreamClient, get_shared_async_stream_client
from ...prompts.moe import (
    PETHEALTH_VITALS_TOOL,
    build_pethealth_vitals_injection,
    inject_prompt,
)
from ...prompts.moe_aggregator import build_aggregator_prompt
from ...prompts.moe_answer_safety import build_answer_safety_messages
from ...prompts.intent_contracts import (
    INTENT_SPECS,
    build_intent_aggregator_injection,
)
from ...tools.tool_registry import ToolRegistry, get_registry
from ...prompts.solve import build_solve_prompt
from .critic import CriticResult, review
from .experts import EXPERTS, ExpertAgentSession, ExpertLoopConfig, run_expert_sessions
from .history_context import fact_state_history_text
from .router import RouterConfig, RouterDecision
from .retrieval_policy import EvidenceTask, resolve_retrieval_requirement
from .task_policy import IntentDecision, TaskPolicyDecision, decide_task_policy
from .tool_broker import ToolBroker
from .trace import MoETrace, extract_usage
from ...sql_search import fetch_animal_profile, species_label
from ...context.request_context import get_request_animal_id


logger = logging.getLogger(__name__)


_OUT_OF_SCOPE_TEXT = (
    "抱歉，这个问题似乎与宠物的健康、养护、营养、用药或行为无关，"
    "我是宠物健康助手，暂时无法回答。如果你有关于猫狗等宠物健康的问题，欢迎随时问我～"
)

_BLOCK_FALLBACK_TEXT_OWNER = (
    "出于安全考虑，我不能直接给出该建议。你描述的情况可能涉及较高风险，"
    "强烈建议尽快联系或前往专业兽医进行线下评估与处理。\n\n"
    "**免责声明**：以上内容仅供健康管理参考，不能替代执业兽医的诊断与治疗。"
)

_BLOCK_FALLBACK_TEXT_VET = (
    "出于安全边界考虑，当前草案触及不可自动放行的用药/处置红线，"
    "不宜在此直接给出可执行方案。\n\n"
    "请结合体格检查、实验室与影像结果，按院内急症流程再评估；"
    "若涉及高风险药物，请核对物种特异性毒性、剂量与监测后再处方。\n\n"
    "**说明**：本系统以 AI 助手身份提供临床参考，不能替代现场诊疗决策。"
)

_DEFAULT_FINAL_ANSWER_MAX_TOKENS = 2500

_HIGH_RISK_SPECIFIC_RE = re.compile(
    r"(?ix)(?:"
    r"\b\d+(?:\.\d+)?\s*(?:mg|mcg|µg|μg|g|ml|mL|IU|U)\s*(?:/\s*(?:kg|day|d|h))?"
    r"|\bq\s*\d+(?:[-–]\d+)?\s*h\b"
    r"|(?:洗脱|减量|剂量|阈值|复查|监测|给药).{0,48}?\d+(?:\.\d+)?(?:\s*[-–]\s*\d+(?:\.\d+)?)?\s*(?:天|日|周|月|小时|h)"
    r")"
)

_EXPLICIT_NO_FIXED_RE = re.compile(
    r"(?:请勿|不要|不得|避免|不应|无需).{0,12}(?:猜|给出|提供|编造|使用)?.{0,12}(?:固定|具体|精确).{0,8}(?:剂量|数值|天数|时长|时间|阈值|频率)",
    flags=re.IGNORECASE,
)
_HIGH_RISK_VALUE_RE = re.compile(
    r"(?ix)(?:"
    r"\b\d+(?:\.\d+)?\s*(?:mg|mcg|µg|μg|g|ml|mL|IU|U)\s*(?:/\s*(?:kg|day|d|h))?"
    r"|\bq\s*\d+(?:[-–]\d+)?\s*h\b"
    r"|\d+(?:\.\d+)?(?:\s*[-–—]\s*\d+(?:\.\d+)?)?\s*(?:天|日|周|月|小时|h)"
    r")"
)
_HIGH_RISK_CONTEXT_RE = re.compile(
    r"洗脱|减量|剂量|阈值|复查|监测|给药|联用|合用|同用|切换|停药|停用|禁忌|间隔|频率",
    flags=re.IGNORECASE,
)


def final_answer_max_tokens() -> int:
    """MoE 终答 token 预算：既作为默认值，也作为显式请求的封顶值。

    经 `MOE_FINAL_ANSWER_MAX_TOKENS` 覆盖，是该预算的唯一来源；路由层与编排器都
    必须取自此处，避免同一数字在多个文件里各写一份。
    """
    raw = (os.getenv("MOE_FINAL_ANSWER_MAX_TOKENS") or "").strip()
    if not raw:
        return _DEFAULT_FINAL_ANSWER_MAX_TOKENS
    try:
        return max(1, int(raw))
    except ValueError:
        return _DEFAULT_FINAL_ANSWER_MAX_TOKENS


def normalize_finish_reason(reason: Optional[str]) -> str:
    """将上游 finish_reason 规范为 stop/truncated 等。"""
    normalized = str(reason or "stop").strip().lower()
    if normalized in {"length", "max_tokens", "token_limit", "truncated"}:
        return "truncated"
    if normalized in {"stop", "tool_calls", "content_filter"}:
        return normalized
    return "stop"


def _response_finish_reason(response: Dict[str, Any]) -> str:
    """从 LLM 响应提取 finish_reason。"""
    choices = response.get("choices") if isinstance(response, dict) else None
    choice = choices[0] if isinstance(choices, list) and choices else {}
    reason = choice.get("finish_reason") if isinstance(choice, dict) else None
    return normalize_finish_reason(reason)


def _block_fallback_text(user_role: str) -> str:
    """按用户角色返回安全兜底话术。"""
    if user_role == "veterinarian":
        return _BLOCK_FALLBACK_TEXT_VET
    return _BLOCK_FALLBACK_TEXT_OWNER


def _collect_retrieved_sources(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从成功的 RAG/Web 工具结果提取、去重并编号可引用来源。

    仅保留可公开的标题、路径、URL、页码和截断摘录；相同来源不会因被多个
    专家复用而重复进入最终引用列表。
    """
    sources: List[Dict[str, Any]] = []
    seen = set()
    counts = {"rag": 0, "web": 0}

    for opinion in opinions or []:
        for tool_result in opinion.get("tool_results") or []:
            if not isinstance(tool_result, dict) or not tool_result.get("ok"):
                continue
            tool_name = str(tool_result.get("tool_name") or "")
            result = tool_result.get("result")
            if not isinstance(result, dict):
                continue
            if tool_name == "rag.search":
                items = result.get("hits") or []
                source_type = "rag"
                prefix = "R"
                sufficiency = (
                    tool_result.get("sufficiency")
                    if isinstance(tool_result.get("sufficiency"), dict)
                    else None
                )
                evidence_status = str(
                    (sufficiency or {}).get("status") or "unassessed"
                ).strip().lower()
                # A successful tool call is not automatically usable evidence.
                # Explicitly weak/unknown results are hidden from the citation
                # catalogue so the aggregator cannot accidentally cite them.
                if evidence_status in {"unsupported", "unknown"}:
                    continue
                matched_indexes: Optional[set[int]] = None
                if evidence_status == "partial":
                    matched_indexes = set()
                    for hit_id in (sufficiency or {}).get("matched_hit_ids") or []:
                        value = str(hit_id).strip().lower()
                        if value.startswith("h") and value[1:].isdigit():
                            matched_indexes.add(int(value[1:]) - 1)
            elif tool_name.startswith("mcp.web_search"):
                items = result.get("results") or result.get("hits") or []
                source_type = "web"
                prefix = "W"
                sufficiency = (
                    tool_result.get("sufficiency")
                    if isinstance(tool_result.get("sufficiency"), dict)
                    else None
                )
                evidence_status = str(
                    (sufficiency or {}).get("status") or "unknown"
                ).strip().lower()
                if evidence_status in {"unsupported", "unknown"}:
                    continue
                matched_indexes = None
                if evidence_status == "partial":
                    matched_indexes = set()
                    for hit_id in (sufficiency or {}).get("matched_hit_ids") or []:
                        value = str(hit_id).strip().lower()
                        if value.startswith("h") and value[1:].isdigit():
                            matched_indexes.add(int(value[1:]) - 1)
            else:
                continue
            if not isinstance(items, list):
                continue

            for item_index, item in enumerate(items):
                if not isinstance(item, dict):
                    continue
                if matched_indexes is not None and item_index not in matched_indexes:
                    continue
                metadata = item.get("metadata") if isinstance(item.get("metadata"), dict) else {}
                source_path = str(item.get("source_path") or metadata.get("source_path") or "").strip()
                title = str(item.get("title") or metadata.get("title") or source_path).strip()
                url = str(item.get("url") or metadata.get("url") or "").strip()
                page = item.get("page") or metadata.get("page") or metadata.get("page_number")
                excerpt = str(item.get("text") or item.get("content") or item.get("snippet") or "").strip()
                key = (source_type, source_path, title, url, str(page or ""), excerpt[:1200])
                if key in seen or not any((source_path, title, url, excerpt)):
                    continue
                seen.add(key)
                counts[source_type] += 1
                source = {
                    "id": f"{prefix}{counts[source_type]}",
                    "type": source_type,
                    "title": title,
                    "source_path": source_path,
                    "url": url,
                    "excerpt": excerpt[:1800],
                    "evidence_status": evidence_status,
                }
                if page not in (None, ""):
                    source["page"] = page
                sources.append(source)
    return sources


def _synthesis_opinions(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """为融合阶段整理专家意见。"""
    fields = (
        "expert", "name_zh", "weight", "conclusion", "evidence", "risks", "confidence",
        "required_tools", "attempted_tools", "successful_tools", "pending_tools",
        "unavailable_required_tools", "evidence_sufficiency",
    )
    return [{field: opinion.get(field) for field in fields} for opinion in opinions or []]


def _evidence_audit_summary(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """按证据目标合并多波审计，供终答作唯一的最终充分性判断。"""
    grouped: Dict[Tuple[str, str], Dict[str, Any]] = {}
    status_rank = {"unsupported": 0, "unknown": 1, "partial": 2, "supported": 3}
    for opinion in opinions or []:
        expert = str(opinion.get("expert") or "")
        for result in opinion.get("tool_results") or []:
            if not isinstance(result, dict):
                continue
            sufficiency = result.get("sufficiency")
            if not isinstance(sufficiency, dict):
                continue
            query = str((result.get("arguments") or {}).get("query") or "")[:500]
            goal = str(result.get("evidence_goal") or query)[:500]
            status = str(sufficiency.get("status") or "unknown").strip().lower()
            if status not in status_rank:
                status = "unknown"
            key = (expert, goal)
            item = grouped.setdefault(key, {
                "expert": expert,
                "goal": goal,
                "status": "unsupported",
                "reason": "",
                "matched_hit_ids": [],
                "attempts": [],
            })
            attempt = {
                "tool_name": str(result.get("tool_name") or ""),
                "query": query,
                "scope": str(result.get("scope") or "expert"),
                "status": status,
                "reason": str(sufficiency.get("reason") or "")[:500],
            }
            item["attempts"].append(attempt)
            if status_rank[status] >= status_rank.get(str(item["status"]), 0):
                item["status"] = status
                item["reason"] = attempt["reason"]
                item["matched_hit_ids"] = list(sufficiency.get("matched_hit_ids") or [])
    return list(grouped.values())


def _opinions_used_web(opinions: List[Dict[str, Any]]) -> bool:
    """判断意见是否使用了网络搜索。"""
    if any(source.get("type") == "web" for source in _collect_retrieved_sources(opinions)):
        return True
    return any(
        str(tool_name).startswith("mcp.web_search")
        for opinion in opinions or []
        for tool_name in opinion.get("tools_used") or []
    )


def _answer_requires_evidence_repair(
    answer: str,
    evidence_audit: List[Dict[str, Any]],
) -> bool:
    """仅当存在弱证据目标且草案含高风险具体数值时触发一次安全编辑。"""
    weak = any(
        str(item.get("status") or "unknown").lower() in {"partial", "unsupported", "unknown"}
        for item in evidence_audit
        if isinstance(item, dict)
    )
    return weak and bool(_HIGH_RISK_SPECIFIC_RE.search(str(answer or "")))


def _explicit_forbidden_claims(query: str, answer: str) -> List[str]:
    """提取用户明确禁止、且不是病例原始数据的新增高风险数值片段。"""
    if not _EXPLICIT_NO_FIXED_RE.search(str(query or "")):
        return []
    query_values = {
        re.sub(r"\s+", "", match.group(0)).lower()
        for match in _HIGH_RISK_VALUE_RE.finditer(str(query or ""))
    }
    claims: List[str] = []
    for line in re.split(r"(?<=[。！？；;])|\n", str(answer or "")):
        if not _HIGH_RISK_CONTEXT_RE.search(line):
            continue
        for match in _HIGH_RISK_VALUE_RE.finditer(line):
            normalized = re.sub(r"\s+", "", match.group(0)).lower()
            if normalized in query_values:
                continue
            snippet = line.strip()
            if snippet and snippet not in claims:
                claims.append(snippet[:500])
    return claims


def _remove_forbidden_claims(answer: str, forbidden_claims: List[str]) -> str:
    """在安全编辑仍保留明确禁用值时，删除对应句并给出保守占位说明。"""
    forbidden_values = {
        re.sub(r"\s+", "", match.group(0)).lower()
        for claim in forbidden_claims
        for match in _HIGH_RISK_VALUE_RE.finditer(claim)
    }
    if not forbidden_values:
        return answer
    replacement = "相关具体剂量、固定时长或数值阈值缺乏充分证据，需结合患者资料和可靠依据个体化核定。"
    # 仅在句末标点后切分；换行本身留在后续片段中，避免安全删句把
    # Markdown 标题、列表与段落压成一行。
    parts = re.split(r"(?<=[。！？；;])", str(answer or ""))
    revised: List[str] = []
    replaced = False
    for part in parts:
        values = {
            re.sub(r"\s+", "", match.group(0)).lower()
            for match in _HIGH_RISK_VALUE_RE.finditer(part)
        }
        if values & forbidden_values and _HIGH_RISK_CONTEXT_RE.search(part):
            last_newline = part.rfind("\n")
            if last_newline >= 0:
                revised.append(part[:last_newline + 1])
            if not replaced:
                revised.append(replacement)
                replaced = True
            continue
        revised.append(part)
    return "".join(revised).strip()


def _strip_markdown_envelope(text: str) -> str:
    """仅移除包住整篇回答的 Markdown 围栏。"""
    value = str(text or "").strip()
    match = re.fullmatch(r"```(?:markdown|md)?\s*\n([\s\S]*?)\n```", value, flags=re.IGNORECASE)
    return match.group(1).strip() if match else value


# 向后兼容别名（宠主文案）
_BLOCK_FALLBACK_TEXT = _BLOCK_FALLBACK_TEXT_OWNER


@dataclass
class OrchestratorConfig:
    """编排器温度、token、角色与工具白名单配置。"""
    router: RouterConfig = field(default_factory=RouterConfig)
    rag_top_k: int = 5
    temperature: float = 0.3
    max_tokens: int = field(default_factory=final_answer_max_tokens)
    user_role: str = "pet_owner"
    device: Optional[str] = None
    animal_id: Optional[str] = None
    allowed_tools: Optional[List[str]] = None
    pethealth_server: Optional[Dict[str, Any]] = None
    expert_loop: ExpertLoopConfig = field(default_factory=ExpertLoopConfig)


def _event(content: str = "", status: Optional[str] = None,
           detail: Optional[Dict[str, Any]] = None, finish: Optional[str] = None) -> Dict[str, Any]:
    """构造流式事件字典。"""
    return {"content": content, "status": status, "detail": detail, "finish": finish}


class MoEOrchestrator:
    """MoE 主编排器。"""
    def __init__(
        self,
        *,
        registry: Optional[ToolRegistry] = None,
        llm: Optional[AsyncOpenAIClient] = None,
        stream_llm: Optional[AsyncOpenAIStreamClient] = None,
        config: Optional[OrchestratorConfig] = None,
    ) -> None:
        """绑定工具注册表与编排配置。"""
        self.registry = registry or get_registry()
        self.llm = llm or get_shared_async_client()
        self.stream_llm = stream_llm or get_shared_async_stream_client()
        self.config = config or OrchestratorConfig()
        self.last_finish_reason = "stop"
        self.last_run_context: Dict[str, Any] = {}
        self._active_conversation_history: Optional[List[Dict[str, str]]] = None
        self._active_expert_context_history: Optional[List[Dict[str, Any]]] = None
        self._active_user_memory: str = ""
        self._active_intent_decision: Optional[IntentDecision] = None
        self._active_task_policy: Optional[TaskPolicyDecision] = None
        self._active_evidence_tasks: Tuple[EvidenceTask, ...] = ()

    # ------------------------------------------------------------------ stages
    def _resolve_species(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        # Resolve (species_en, species_zh, breed) for the request-scoped animal.
        """解析请求物种（画像或文本）。"""
        animal_id = self.config.animal_id or get_request_animal_id()
        if not animal_id:
            return None, None, None
        profile = fetch_animal_profile(animal_id)
        if not profile:
            return None, None, None
        breed = profile.get("breed")
        breed_s = str(breed).strip() if breed else None
        return (profile.get("species") or None), species_label(profile), (breed_s or None)

    def _aggregator_max_tokens(self, query: str) -> int:
        """融合器 max_tokens。"""
        return max(1, min(int(self.config.max_tokens), final_answer_max_tokens()))

    def _pethealth_server_context(self) -> Optional[Dict[str, Any]]:
        """读取 PetHealth_Server 上下文。"""
        ctx = self.config.pethealth_server if isinstance(self.config.pethealth_server, dict) else None
        if not ctx or not bool(ctx.get("heart_rate_abnormal")):
            return None
        animal_id = str(ctx.get("animal_id") or "").strip()
        if not animal_id:
            return None
        try:
            hours = int(ctx.get("vitals_window_hours") or 24)
        except (TypeError, ValueError):
            hours = 24
        return {
            "animal_id": animal_id,
            "heart_rate_abnormal": True,
            "vitals_window_hours": max(1, min(720, hours)),
        }

    def _pethealth_prompt_injection(self, stage: str) -> str:
        """构造体征提示注入。"""
        ctx = self._pethealth_server_context()
        if not ctx:
            return ""
        return build_pethealth_vitals_injection(
            animal_id=ctx["animal_id"],
            heart_rate_abnormal=True,
            vitals_window_hours=ctx["vitals_window_hours"],
            stage=stage,
        )

    def _intent_prompt_injection(self, stage: str) -> str:
        """构造意图输出契约注入。"""
        decision = self._active_intent_decision
        if decision is None:
            return ""
        if stage == "aggregator":
            return build_intent_aggregator_injection(
                decision.intent_id,
                decision.confidence,
                decision.output_variant,
            )
        return ""

    def _request_prompt_injection(self, stage: str) -> str:
        """合并本请求全部提示注入。"""
        prompt = inject_prompt("", self._intent_prompt_injection(stage))
        return inject_prompt(prompt, self._pethealth_prompt_injection(stage))

    async def _decide_task_policy(
        self,
        query: str,
        recorder: Optional[MoETrace],
    ) -> TaskPolicyDecision:
        """调用统一任务策略分类。"""
        _, species_zh, breed = self._resolve_species()
        return await decide_task_policy(
            query=query,
            user_role=self.config.user_role,
            llm=self.llm,
            config=self.config.router,
            species_zh=species_zh,
            breed=breed,
            conversation_history=self._active_conversation_history,
            expert_context_history=self._active_expert_context_history,
            user_memory=self._active_user_memory,
            prompt_injection=self._request_prompt_injection("router"),
            recorder=recorder,
        )

    async def _prepare_request_policy(
        self,
        query: str,
        recorder: Optional[MoETrace],
    ) -> RouterDecision:
        """准备本请求的意图、路由与证据策略。"""
        self._active_task_policy = await self._decide_task_policy(query, recorder)
        if self.config.user_role == "veterinarian":
            self._active_intent_decision = self._active_task_policy.as_intent_decision()
        decision = self._active_task_policy.as_router_decision(self.config.router)
        self._active_evidence_tasks = self._active_task_policy.assigned_tasks(
            decision.selected_experts
        )

        if recorder is not None:
            recorder.intent_decision = (
                self._active_intent_decision.as_dict()
                if self._active_intent_decision is not None else None
            )
            recorder.router_decision = {
                "scores": decision.scores,
                "raw_weights": decision.raw_weights,
                "weights": decision.weights,
                "selected_experts": decision.selected_experts,
                "emergency": decision.emergency,
                "emergency_rule_hit": False,
                "out_of_scope": decision.out_of_scope,
                "reason": decision.reason,
                "source": "unified_task_policy",
                "config": vars(self.config.router),
            }
        return decision

    @staticmethod
    def _unwrap_mcp_json_content(result: Any) -> Any:
        """解开 MCP JSON 内容包装。"""
        if not isinstance(result, dict):
            return result
        content = result.get("content")
        if not isinstance(content, list) or not content:
            return result
        first = content[0]
        if not isinstance(first, dict) or not isinstance(first.get("text"), str):
            return result
        try:
            parsed = json.loads(first["text"])
        except (TypeError, ValueError):
            return result
        return parsed if isinstance(parsed, dict) else result

    async def _check_pethealth_vitals(
        self,
        recorder: Optional[MoETrace],
    ) -> Optional[Dict[str, Any]]:
        """按需调用 PetHealth 体征核验工具。"""
        ctx = self._pethealth_server_context()
        if not ctx:
            return None

        arguments = {
            "pet_id": ctx["animal_id"],
            "hours": ctx["vitals_window_hours"],
        }
        started = time.perf_counter()
        ok = False
        error = ""
        result: Any

        if self.config.allowed_tools is not None and PETHEALTH_VITALS_TOOL not in self.config.allowed_tools:
            result = {
                "status": "TOOL_NOT_ALLOWED",
                "message": f"{PETHEALTH_VITALS_TOOL} is not allowed by this request.",
            }
            error = "tool not allowed"
        elif self.registry.get(PETHEALTH_VITALS_TOOL) is None:
            result = {
                "status": "TOOL_NOT_AVAILABLE",
                "message": f"{PETHEALTH_VITALS_TOOL} is not registered.",
            }
            error = "tool not registered"
        else:
            try:
                result = await self.registry.call(PETHEALTH_VITALS_TOOL, arguments)
                result = self._unwrap_mcp_json_content(result)
                ok = True
            except ResourceBusyError as exc:
                result = exc.as_dict()
                error = str(exc)
            except Exception as exc:  # noqa: BLE001
                result = {"status": "TOOL_ERROR", "message": str(exc)}
                error = str(exc)

        latency_ms = round((time.perf_counter() - started) * 1000.0, 1)
        record = {
            "tool_name": PETHEALTH_VITALS_TOOL,
            "arguments": arguments,
            "ok": ok,
            "latency_ms": latency_ms,
            "error": error,
            "result": result,
        }
        if recorder is not None:
            recorder.record_tool(
                stage="pethealth_vitals",
                tool_name=PETHEALTH_VITALS_TOOL,
                arguments=arguments,
                ok=ok,
                latency_ms=latency_ms,
                error=error,
            )
        return record

    async def _run_experts(
        self,
        query: str,
        decision: RouterDecision,
        recorder: Optional[MoETrace],
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """为路由入选专家创建独立会话，并通过同一 ToolBroker 并发执行。

        会话共享请求级工具去重与 RAG 串行门控，但各自保留权重、历史、记忆和
        结构化意见；单个专家失败由专家层生成安全兜底，不中断其他专家。
        """
        conversation_history = conversation_history or self._active_conversation_history
        expert_context_history = expert_context_history or self._active_expert_context_history
        memory_text = self._active_user_memory if user_memory is None else user_memory
        species_en, species_zh, breed = self._resolve_species()
        intent_id = (
            self._active_intent_decision.intent_id
            if self._active_intent_decision is not None
            else ""
        )
        sessions: List[ExpertAgentSession] = []
        for key in decision.selected_experts:
            expert = EXPERTS.get(key)
            if expert is None:
                continue
            weight = decision.weights.get(key, 0.0)
            retrieval_requirement = resolve_retrieval_requirement(
                expert_key=key,
                evidence_tasks=self._active_evidence_tasks,
            )
            sessions.append(
                ExpertAgentSession(
                    expert=expert,
                    query=query,
                    weight=weight,
                    registry=self.registry,
                    llm=self.llm,
                    request_allowed_tools=self.config.allowed_tools,
                    rag_top_k=self.config.rag_top_k,
                    device=self.config.device,
                    species_en=species_en,
                    species_zh=species_zh,
                    breed=breed,
                    user_role=self.config.user_role,
                    conversation_history=conversation_history,
                    expert_context_history=expert_context_history,
                    user_memory=memory_text,
                    intent_id=intent_id,
                    emergency=decision.emergency,
                    retrieval_requirement=retrieval_requirement,
                    recorder=recorder,
                    loop_config=self.config.expert_loop,
                )
            )
        if not sessions:
            return []
        broker = ToolBroker(registry=self.registry, allowed_tools=self.config.allowed_tools)
        opinions = await run_expert_sessions(sessions=sessions, broker=broker)
        opinions.sort(key=lambda o: o.get("weight", 0.0), reverse=True)
        if recorder is not None:
            recorder.expert_opinions = opinions
        return opinions

    async def _critique(
        self,
        query: str,
        opinions: List[Dict[str, Any]],
        emergency: bool,
        recorder: Optional[MoETrace],
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: Optional[str] = None,
    ) -> CriticResult:
        """调用 Critic 审核。"""
        conversation_history = conversation_history or self._active_conversation_history
        expert_context_history = expert_context_history or self._active_expert_context_history
        memory_text = self._active_user_memory if user_memory is None else user_memory
        return await review(
            query=query,
            expert_opinions=opinions,
            emergency=emergency,
            llm=self.llm,
            user_role=self.config.user_role,
            conversation_history=conversation_history,
            expert_context_history=expert_context_history,
            user_memory=memory_text,
            recorder=recorder,
        )

    def _build_synthesis_messages(
        self,
        *,
        query: str,
        opinions: List[Dict[str, Any]],
        critic: CriticResult,
        decision: RouterDecision,
        system_context: str = "",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: Optional[str] = None,
        pethealth_vitals_result: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, str]]:
        """组装最终 Aggregator 的 system/user 消息。

        输入包含公开化专家意见、Critic 结论、会话历史、用户记忆、物种与外部
        心率核验约束，并附带去重后的来源目录；不会直接注入原始工具载荷。
        """
        retrieved_sources = _collect_retrieved_sources(opinions)
        has_web = any(source.get("type") == "web" for source in retrieved_sources)
        base = build_solve_prompt(
            user_role=self.config.user_role,
            has_web_search=has_web,
            query=query,
            max_tokens=self._aggregator_max_tokens(query),
        )
        aggregator_prompt = build_aggregator_prompt(
            base_prompt=base,
            user_role=self.config.user_role,
            has_retrieved_sources=bool(retrieved_sources),
            emergency=decision.emergency,
            critic_constraints=critic.constraints,
        )
        sys_prompt = inject_prompt(
            aggregator_prompt,
            self._request_prompt_injection("aggregator"),
        )
        if system_context:
            sys_prompt = f"{system_context}\n\n{sys_prompt}"

        payload = {
            "query": query,
            "user_role": self.config.user_role,
            "router": {"weights": decision.weights, "emergency": decision.emergency},
            "expert_opinions": _synthesis_opinions(opinions),
            "retrieved_sources": retrieved_sources,
            "evidence_audit": _evidence_audit_summary(opinions),
            "critic_verdict": critic.verdict,
            # Critic output is a safety constraint, not clinical evidence.  D8
            # synthesis needs the flags in order to explain why the requested
            # unsafe content is withheld instead of falling back to a generic
            # message that loses the intent contract.
            "critic_issues": critic.issues,
            "critic_constraints": critic.constraints,
        }
        if self._active_intent_decision is not None:
            payload["intent"] = self._active_intent_decision.as_dict()
        if self._active_task_policy is not None:
            payload["task_policy"] = self._active_task_policy.as_dict()
            payload["assigned_evidence_tasks"] = [
                task.as_dict() for task in self._active_evidence_tasks
            ]
        pethealth_ctx = self._pethealth_server_context()
        if pethealth_ctx:
            payload["pethealth_server"] = pethealth_ctx
            payload["pethealth_vitals_result"] = pethealth_vitals_result
        parts: List[str] = []
        memory_text = self._active_user_memory if user_memory is None else user_memory
        history_text = fact_state_history_text(
            conversation_history,
            expert_context_history,
            user_memory=memory_text,
        )
        if history_text:
            parts.append(history_text)
        parts.append(json.dumps(payload, ensure_ascii=False))
        return [
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": "\n\n".join(parts)},
        ]

    def _requires_terminal_block(self, critic: CriticResult) -> bool:
        """判断是否必须走终端安全阻断。"""
        intent_id = getattr(self._active_intent_decision, "intent_id", "")
        return critic.blocked and intent_id not in INTENT_SPECS

    async def _repair_answer_if_needed(
        self,
        *,
        query: str,
        answer: str,
        opinions: List[Dict[str, Any]],
        recorder: Optional[MoETrace],
    ) -> Tuple[str, bool]:
        """对弱证据下的高风险具体数值做一次有界安全编辑。"""
        audit = _evidence_audit_summary(opinions)
        if not _answer_requires_evidence_repair(answer, audit):
            return answer, False
        forbidden_claims = _explicit_forbidden_claims(query, answer)
        messages = build_answer_safety_messages(
            query=query,
            answer=answer,
            evidence_audit=audit,
            intent_contract=self._intent_prompt_injection("aggregator"),
            user_role=self.config.user_role,
            forbidden_claims=forbidden_claims,
        )
        started = time.perf_counter()
        response: Dict[str, Any] = {}
        revised = ""
        error = ""
        try:
            response = await self.llm.chat(
                messages=messages,
                temperature=0.0,
                max_tokens=self._aggregator_max_tokens(query),
                thinking=False,
            )
            revised = _strip_markdown_envelope(extract_text(response))
            if revised and forbidden_claims:
                revised = _remove_forbidden_claims(revised, forbidden_claims)
            if _response_finish_reason(response) == "truncated":
                error = "safety repair was truncated"
                revised = ""
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
            logger.warning("answer evidence safety repair failed: %s", exc, exc_info=True)
        if recorder is not None:
            recorder.record_llm(
                stage="aggregator:evidence_safety_repair",
                model=getattr(self.llm, "model", ""),
                messages=messages,
                output=revised,
                latency_ms=(time.perf_counter() - started) * 1000.0,
                usage=extract_usage(response),
                meta={"applied": bool(revised), "error": error, "audit_items": len(audit)},
            )
        return (revised, True) if revised else (answer, False)

    # ------------------------------------------------------------------ stream
    async def stream(
        self,
        *,
        query: str,
        system_context: str = "",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: str = "",
        recorder: Optional[MoETrace] = None,
    ) -> AsyncGenerator[Dict[str, Any], None]:
        # 1) 统一任务策略（意图、专家路由、证据任务）
        """按统一策略、专家会诊、Critic 与 Aggregator 顺序流式产出 MoE 事件。

        该异步生成器发布阶段状态、专家结果和终答增量；终答流中断时发送
        ``answer_reset`` 并用非流式调用生成完整替代答案。安全阻断与工具核验失败
        也会形成可持久化终态事件，不暴露系统提示词或内部推理。
        """
        self.last_run_context = {}
        self._active_conversation_history = conversation_history
        self._active_expert_context_history = expert_context_history
        self._active_user_memory = str(user_memory or "").strip()
        self._active_intent_decision = None
        self._active_task_policy = None
        self._active_evidence_tasks = ()
        yield _event(
            status="intent_classifying",
            detail={"message": "正在统一识别任务、专家与证据需求…"},
        )
        decision = await self._prepare_request_policy(query, recorder)
        if self._active_intent_decision is not None:
            intent_payload = self._active_intent_decision.as_dict()
            intent_payload["selected_experts"] = list(decision.selected_experts)
            intent_payload["emergency"] = bool(decision.emergency)
            intent_payload["evidence_tasks"] = [
                task.as_dict() for task in self._active_evidence_tasks
            ]
            self.last_run_context["intent"] = intent_payload
            yield _event(status="intent_classified", detail=intent_payload)
        if self._active_task_policy is not None:
            policy_payload = self._active_task_policy.as_dict()
            policy_payload["assigned_evidence_tasks"] = [
                task.as_dict() for task in self._active_evidence_tasks
            ]
            policy_payload["architecture"] = "unified_task_policy"
            self.last_run_context["task_policy"] = policy_payload

        # 2) 确定性路由门控
        yield _event(status="routing", detail={"message": "统一策略已完成，正在应用专家门控…"})
        self.last_run_context["router"] = {
            "weights": decision.weights,
            "selected_experts": decision.selected_experts,
            "emergency": decision.emergency,
            "reason": decision.reason,
        }

        if decision.out_of_scope:
            if recorder is not None:
                recorder.out_of_scope = True
                recorder.final_answer = _OUT_OF_SCOPE_TEXT
                recorder.finalize()
            yield _event(
                status="routing",
                detail={"out_of_scope": True, "reason": decision.reason, "scores": decision.scores},
            )
            yield _event(content=_OUT_OF_SCOPE_TEXT, status="streaming")
            return

        yield _event(
            content=f"\n**路由完成**：{', '.join(EXPERTS[k].name_zh for k in decision.selected_experts)}\n",
            status="routing",
            detail={
                "weights": decision.weights,
                "raw_weights": decision.raw_weights,
                "scores": decision.scores,
                "selected_experts": decision.selected_experts,
                "emergency": decision.emergency,
                "reason": decision.reason,
            },
        )

        pethealth_vitals_result = await self._check_pethealth_vitals(recorder)
        if pethealth_vitals_result is not None:
            self.last_run_context["pethealth_vitals"] = pethealth_vitals_result
            result = pethealth_vitals_result.get("result")
            alert_level = result.get("alert_level") if isinstance(result, dict) else None
            yield _event(
                content="\n**PetHealth 心率核实完成**\n",
                status="tool_complete",
                detail={
                    "tool_name": pethealth_vitals_result["tool_name"],
                    "arguments": pethealth_vitals_result["arguments"],
                    "ok": pethealth_vitals_result["ok"],
                    "alert_level": alert_level,
                    "error": pethealth_vitals_result["error"],
                },
            )

        # 3) 并行专家会诊
        for key in decision.selected_experts:
            assigned_tasks = [
                task.as_dict() for task in self._active_evidence_tasks
                if task.owner == key
            ]
            yield _event(
                content=f"\n**{EXPERTS[key].name_zh} 会诊中**（权重 {decision.weights.get(key, 0):.2f}）\n",
                status="expert_calling",
                detail={
                    "expert": key,
                    "name_zh": EXPERTS[key].name_zh,
                    "weight": decision.weights.get(key, 0),
                    "evidence_tasks": assigned_tasks,
                },
            )
        opinions = await self._run_experts(query, decision, recorder)
        self.last_run_context["experts"] = opinions
        for o in opinions:
            _tu = o.get("tools_used") or []
            _tools_txt = f"，工具 {', '.join(_tu)}" if _tu else ""
            yield _event(
                content=f"   {o['name_zh']}：置信度 {o['confidence']:.2f}，RAG 命中 {o['rag_hits']}{_tools_txt}\n",
                status="expert_complete",
                detail={
                    "expert": o["expert"],
                    "name_zh": o["name_zh"],
                    "weight": o.get("weight"),
                    "confidence": o["confidence"],
                    "hits_count": o["rag_hits"],
                    "best_score": o["rag_best_score"],
                    "tools_used": o.get("tools_used", []),
                    "required_tools": o.get("required_tools", []),
                    "attempted_tools": o.get("attempted_tools", []),
                    "pending_tools": o.get("pending_tools", []),
                    "opinion": o,
                },
            )

        # 4) Critic 审核
        yield _event(content="\n**边界审核中…**\n", status="reviewing", detail={"message": "安全与边界校验"})
        critic = await self._critique(query, opinions, decision.emergency, recorder)
        self.last_run_context["critic"] = {
            "verdict": critic.verdict,
            "issues": critic.issues,
            "constraints": critic.constraints,
            "reason": critic.reason,
        }
        yield _event(
            status="reviewing",
            detail={"verdict": critic.verdict, "issues": critic.issues, "reason": critic.reason},
        )

        if self._requires_terminal_block(critic):
            block_text = _block_fallback_text(self.config.user_role)
            if recorder is not None:
                recorder.blocked = True
                recorder.final_answer = block_text
                recorder.finalize()
            yield _event(content="\n**生成回答…**\n\n", status="generating")
            yield _event(content=block_text, status="streaming")
            return

        # 5) 流式融合生成
        yield _event(content="\n**生成回答…**\n\n", status="generating")
        messages = self._build_synthesis_messages(
            query=query, opinions=opinions, critic=critic, decision=decision,
            system_context=system_context, conversation_history=conversation_history,
            expert_context_history=expert_context_history,
            pethealth_vitals_result=pethealth_vitals_result,
        )
        max_tokens = self._aggregator_max_tokens(query)
        collected: List[str] = []
        finish_reason = "stop"
        stream_error = ""
        fallback_response: Dict[str, Any] = {}
        fallback_used = False
        t0 = time.perf_counter()
        try:
            stream_events = getattr(self.stream_llm, "chat_stream_events", None)
            if callable(stream_events):
                async for item in stream_events(
                    messages=messages,
                    temperature=self.config.temperature,
                    max_tokens=max_tokens,
                    thinking=False,
                ):
                    piece = str(item.get("content") or "")
                    if piece:
                        collected.append(piece)
                        yield _event(content=piece, status="streaming")
                    if item.get("finish_reason"):
                        finish_reason = normalize_finish_reason(item["finish_reason"])
            else:
                async for piece in self.stream_llm.chat_stream(
                    messages=messages,
                    temperature=self.config.temperature,
                    max_tokens=max_tokens,
                    thinking=False,
                ):
                    collected.append(piece)
                    yield _event(content=piece, status="streaming")
        except Exception as exc:  # noqa: BLE001
            stream_error = str(exc)
            logger.warning("aggregator stream failed before completion: %s", exc, exc_info=True)

        if stream_error or not collected:
            fallback_used = True
            yield _event(
                status="generating",
                detail={
                    "message": "终答流中断，正在自动重试" if stream_error else "流式响应为空，正在自动重试终答",
                    "retry": 1,
                    "stream_error": bool(stream_error),
                },
            )
            try:
                fallback_response = await self.llm.chat(
                    messages=messages,
                    temperature=self.config.temperature,
                    max_tokens=max_tokens,
                    thinking=False,
                )
                fallback_answer = extract_text(fallback_response).strip()
            except Exception as exc:  # noqa: BLE001
                fallback_answer = ""
                stream_error = "; ".join(part for part in (stream_error, str(exc)) if part)
                logger.warning("aggregator non-stream fallback failed: %s", exc, exc_info=True)
            if not fallback_answer:
                reason = stream_error or "empty response without an upstream error"
                raise RuntimeError(f"aggregator returned no visible content after fallback: {reason}")
            if collected:
                # 调用方可能已渲染或持久化了部分增量。持久 reset 事件让第一方消费者
                # 替换这段不完整文本，而不是把第二份答案追加在后面。
                yield _event(
                    status="answer_reset",
                    detail={"reason": "aggregator_stream_interrupted", "replacement": True},
                )
                collected.clear()
            finish_reason = _response_finish_reason(fallback_response)
            for offset in range(0, len(fallback_answer), 96):
                piece = fallback_answer[offset:offset + 96]
                collected.append(piece)
                yield _event(content=piece, status="streaming")
        latency = (time.perf_counter() - t0) * 1000.0
        draft_answer = "".join(collected)
        self.last_finish_reason = finish_reason
        if recorder is not None:
            recorder.record_llm(
                stage="aggregator",
                model=getattr(self.stream_llm, "model", ""),
                messages=messages,
                output=draft_answer,
                latency_ms=latency,
                usage=extract_usage(fallback_response),
                meta={
                    "streamed": not fallback_used,
                    "fallback_used": fallback_used,
                    "stream_error": stream_error,
                    "max_tokens": max_tokens,
                    "finish_reason": finish_reason,
                },
            )
        final_answer, repaired = await self._repair_answer_if_needed(
            query=query,
            answer=draft_answer,
            opinions=opinions,
            recorder=recorder,
        )
        if repaired:
            yield _event(
                status="reviewing",
                detail={
                    "verdict": "revised",
                    "issues": ["已移除未获充分证据支持的具体剂量、时长或阈值"],
                    "message": "证据安全复核已修订终答",
                },
            )
            yield _event(
                status="answer_reset",
                detail={"reason": "evidence_safety_repair", "replacement": True},
            )
            collected.clear()
            for offset in range(0, len(final_answer), 96):
                piece = final_answer[offset:offset + 96]
                collected.append(piece)
                yield _event(content=piece, status="streaming")
            self.last_finish_reason = "stop"
            finish_reason = "stop"
        if recorder is not None:
            recorder.final_answer = final_answer
            recorder.finalize()
        yield _event(finish=finish_reason)

    # --------------------------------------------------------------- non-stream
    async def run(
        self,
        *,
        query: str,
        system_context: str = "",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        user_memory: str = "",
        recorder: Optional[MoETrace] = None,
    ) -> Tuple[str, Optional[MoETrace]]:
        """消费完整编排链路，非流式返回唯一终答与最终化 Trace。

        主要供评测和非流式兼容调用使用；它不会另行执行专家或工具，并在收到
        ``answer_reset`` 后丢弃此前部分增量，只保留完整替代答案。
        """
        self.last_run_context = {}
        self._active_conversation_history = conversation_history
        self._active_expert_context_history = expert_context_history
        self._active_user_memory = str(user_memory or "").strip()
        self._active_intent_decision = None
        self._active_task_policy = None
        self._active_evidence_tasks = ()
        decision = await self._prepare_request_policy(query, recorder)
        if self._active_intent_decision is not None:
            self.last_run_context["intent"] = self._active_intent_decision.as_dict()
        if self._active_task_policy is not None:
            policy_payload = self._active_task_policy.as_dict()
            policy_payload["assigned_evidence_tasks"] = [
                task.as_dict() for task in self._active_evidence_tasks
            ]
            policy_payload["architecture"] = "unified_task_policy"
            self.last_run_context["task_policy"] = policy_payload
        self.last_run_context["router"] = {
            "weights": decision.weights,
            "selected_experts": decision.selected_experts,
            "emergency": decision.emergency,
            "reason": decision.reason,
        }

        if decision.out_of_scope:
            if recorder is not None:
                recorder.out_of_scope = True
                recorder.final_answer = _OUT_OF_SCOPE_TEXT
                recorder.finalize()
            return _OUT_OF_SCOPE_TEXT, recorder

        pethealth_vitals_result = await self._check_pethealth_vitals(recorder)
        if pethealth_vitals_result is not None:
            self.last_run_context["pethealth_vitals"] = pethealth_vitals_result

        opinions = await self._run_experts(query, decision, recorder)
        self.last_run_context["experts"] = opinions
        critic = await self._critique(query, opinions, decision.emergency, recorder)
        self.last_run_context["critic"] = {
            "verdict": critic.verdict,
            "issues": critic.issues,
            "constraints": critic.constraints,
            "reason": critic.reason,
        }

        if self._requires_terminal_block(critic):
            block_text = _block_fallback_text(self.config.user_role)
            if recorder is not None:
                recorder.blocked = True
                recorder.final_answer = block_text
                recorder.finalize()
            return block_text, recorder

        messages = self._build_synthesis_messages(
            query=query, opinions=opinions, critic=critic, decision=decision,
            system_context=system_context, conversation_history=conversation_history,
            expert_context_history=expert_context_history,
            pethealth_vitals_result=pethealth_vitals_result,
        )
        max_tokens = self._aggregator_max_tokens(query)
        t0 = time.perf_counter()
        resp = await self.llm.chat(
            messages=messages,
            temperature=self.config.temperature,
            max_tokens=max_tokens,
            thinking=False,
        )
        latency = (time.perf_counter() - t0) * 1000.0
        draft_answer = extract_text(resp)
        self.last_finish_reason = _response_finish_reason(resp)
        if recorder is not None:
            recorder.record_llm(
                stage="aggregator",
                model=getattr(self.llm, "model", ""),
                messages=messages,
                output=draft_answer,
                latency_ms=latency,
                usage=extract_usage(resp),
                meta={
                    "streamed": False,
                    "max_tokens": max_tokens,
                    "finish_reason": self.last_finish_reason,
                },
            )
        final_answer, repaired = await self._repair_answer_if_needed(
            query=query,
            answer=draft_answer,
            opinions=opinions,
            recorder=recorder,
        )
        if repaired:
            self.last_finish_reason = "stop"
        if recorder is not None:
            recorder.final_answer = final_answer
            recorder.finalize()
        return final_answer, recorder
