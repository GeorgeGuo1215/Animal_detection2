"""MoE 下游专家委员会：独立多轮专家、工具执行与结构化意见。"""
from __future__ import annotations

import asyncio
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from ...context.request_context import ANIMAL_REQUIRED_TOOLS, get_request_animal_id
from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ...tools.rag_query import is_english_rag_query
from ...tools.tool_registry import ToolRegistry
from ..tool_call_utils import canonical_tool_call
from ..plan_and_solve import _safe_json_loads
from .tool_broker import ToolBroker, ToolRequest, ToolResult
from .history_context import build_fact_state_history
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
    key: str
    name_zh: str
    persona: str
    allowed_tools: List[str] = field(default_factory=list)
    rag_query_hint: str = ""
    # Secondary knowledge categories for rag.search (exact ids or prefix.* wildcards).
    rag_categories: List[str] = field(default_factory=list)


_CLINICAL_RAG_CATEGORIES = [
    "basic.anatomy",
    "basic.terminology",
    "clinical.*",
    "diagnostics.*",
    "clinical_skills.*",
    "integrative.general",
    "anesthesia.default",
    "immunology.default",
    "reproduction.default",
    "infectious.placeholder",
    "exotic.default",
    "zoonosis.toxoplasmosis",
]

_NUTRITION_RAG_CATEGORIES = [
    "nutrition.placeholder",
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


_SPECIES_GUARD = (
    "严格遵守物种安全：犬、猫的生理与药理差异巨大，禁止把某一物种的方案直接用于另一物种；"
    "若用户未说明物种，需提示该差异并按通用/谨慎口径作答。"
)

_SPECIES_BREED_GUARD = (
    "物种/品种特异化保障：结合用户叙述与注入的 species/breed（若有），提炼该个体相关的特异风险与注意事项，"
    "写入 risks 与 conclusion，禁止套用『泛犬/泛猫』方案而忽略品种差异。"
    "示例：短吻犬（斗牛/巴哥等）需强调运动强度、热耐受、呼吸道与麻醉风险；"
    "英短等品种在泌尿/结石倾向上需结合主诉谨慎提示。"
    "若用户文本已点名品种但 payload 无 breed，仍须从问题中识别并特异化作答。"
)

_OUTPUT_CONTRACT = (
    "你必须只输出严格 JSON（不要任何额外文字、不要 markdown 代码块），结构如下：\n"
    "{\n"
    '  "conclusion": "你的核心结论（中文，2-4 句）",\n'
    '  "evidence": ["支撑该结论的依据，尽量引用检索内容里的来源/页码"],\n'
    '  "risks": ["与本专业相关的风险提示或禁忌"],\n'
    '  "confidence": 0.0\n'
    "}\n"
    "confidence 为 0~1 的自评置信度：检索证据充分且与问题高度相关时高，证据不足时低。\n"
    "禁忌联用硬约束：若可靠证据已确认两种药物或药物类别属于禁忌联用，结论必须明确不得同时使用；"
    "替代方案必须移除或替换至少一种冲突药物或药物类别。降低剂量、错峰给药、缩短重叠期、换用同类中"
    "所谓低风险药物、增加支持性用药或仅加强监测，都不能解除禁忌，也不得表述为可以继续联用的方案。"
    "支持性处理只能作为意外暴露后的风险处置，不能作为计划性联用的许可。切换、减停和洗脱必须依据具体"
    "药名、剂量、疗程、器官功能及可靠来源；信息不足时不得编造统一天数，也不得要求长期用药患者擅自"
    "骤停，而应说明需要处方兽医协调的调整原则与给药前风险控制。若问题是在计划给药且尚未发生意外暴露，"
    "conclusion、evidence、risks 的任何字段都不得加入『若必须/若不得已/可短期重叠』后继续联合给药的例外；"
    "输出前必须删除任何允许该禁忌组合重叠的句子。若用户未提供具体药名、剂量和疗程，任何字段都禁止"
    "给出数字洗脱天数、半衰期示例或按假定药物推算间隔。替代路径必须与当前适应证相符，且不能换成仍属于"
    "冲突类别的药物；适应证尚未明确时，应给出按病因选择治疗类别的原则，而不是编造具体替代处方。若已经"
    "意外重叠且疗程未知，不得无条件指定立即停用其中某一种药，也不得要求患者自行选择停药对象；应说明在"
    "下一次计划给药前由处方兽医根据现有疗程决定暂停、减停或替换对象，对需要渐减的长期用药避免骤停。"
)

_AUDIENCE_OWNER = (
    "【读者身份】当前提问者是宠物主（pet_owner）。"
    "用通俗可执行语气写 conclusion/risks；可保留就医时机与安全提醒（如勿自行使用人用止痛药）。"
    "若疾病症状描述非特异、信息不足且用户未报告明确当前红旗，优先列出 3~6 个会改变风险判断的关键"
    "追问与短时观察项；不要因严重疾病存在于鉴别范围就把它写成当前患者事实、突出最坏情况或无条件要求"
    "立即就医。此类首轮可说明常见原因和风险类别，但不要主动点名、展开尚无个体证据的罕见严重疾病；"
    "用户明确询问鉴别诊断、已有相关证据或报告红旗时仍应正常说明。若本轮输入已包含相关补充信息，应"
    "利用这些信息推进鉴别和建议，不要机械重复追问。"
    "若用户已报告明确当前红旗，则直接说明紧急程度与行动，不得为了补全信息而延误。"
    "本条优先于上方 persona 中『面向兽医用户 / 勿写就医口号』的表述。"
)

_AUDIENCE_VET = (
    "【读者身份】当前提问者是执业兽医（veterinarian）。"
    "你是 AI 临床助手：用专业、结构化语气写 conclusion/risks；急症写处置与检查优先级，"
    "不要写『请立即就医/线下就诊』等宠主话术，也不要自称『同事』。"
)


_IMPORTANT_RETRIEVAL_POLICY: Dict[str, str] = {
    "clinical": (
        "【重要场景双检索】在诊断或鉴别诊断、急症风险判断、会显著影响处置优先级的治疗决策、"
        "复杂或少见病例等高影响临床场景中，不能只凭模型记忆直接返回 final。若当前可用工具同时包含"
        " rag.search 与 mcp.web_search.web_search，本次专家任务应先后调用二者并综合证据，再返回最终意见："
        "RAG 用于核对本地专业资料，Web Search 用于核对近期指南、共识或外部证据。每轮仍只调用一个工具，"
        "因此应在连续轮次中完成；首次检索结果不足或未命中，不是跳过另一类检索的理由。只有普通低风险"
        "养护/沟通问题、用户信息不足到无法形成有效检索问题、或相应工具未提供时，才可不做双检索。"
    ),
    "pharmacy": (
        "【重要场景双检索】在具体药物剂量、联合用药与相互作用、禁忌、物种毒性、不良反应、停换药/"
        "洗脱方案，以及会显著影响用药安全的特殊个体场景中，不能只凭模型记忆直接返回 final。若当前"
        "可用工具同时包含 rag.search 与 mcp.web_search.web_search，本次专家任务应先后调用二者并综合"
        "证据，再返回最终意见：RAG 用于核对本地药理资料，Web Search 用于核对近期药品资料、指南或"
        "外部安全证据。每轮仍只调用一个工具，因此应在连续轮次中完成；首次检索结果不足或未命中，不是"
        "跳过另一类检索的理由。只有不涉及具体用药安全的普通问题、信息不足到无法形成有效检索问题、或"
        "相应工具未提供时，才可不做双检索。"
    ),
}


EXPERTS: Dict[str, ExpertConfig] = {
    "clinical": ExpertConfig(
        key="clinical",
        name_zh="兽医临床专家",
        persona=(
            "你是一位经验丰富的兽医临床专家，为执业兽医用户提供 AI 助手式临床参考："
            "症状评估、鉴别诊断排序与检查/处置优先级。"
            "你基于循证兽医学谨慎推断，区分『直接证据』与『临床经验推断』，"
            "避免绝对确诊措辞；急症写清处置与监护要点，不要写『请线下就医』等宠主话术，不要自称同事。"
        ),
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
        persona=(
            "你是一位兽医营养专家，负责膳食配方、体重与慢病饮食管理。"
            "你依据 NRC/AAFCO 等营养标准给出热量与配方建议，关注个体体重与病史。"
        ),
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
        persona=(
            "你是一位兽医药剂师，负责用药安全、剂量、相互作用与禁忌。"
            "你尤其关注犬猫物种特异性毒性，对剂量与禁忌保持高度谨慎；"
            "评估联合用药时，若可靠证据已确认两种药物属于禁忌联用，不能只给出停用或禁用结论；"
            "必须说明禁忌原因，并给出至少一条可执行的替代路径，包括优先替换哪一种药、可选替代药物或"
            "非药物方案，以及必要的切换/洗脱和监测边界。替代方案必须符合当前物种、适应证、器官功能"
            "和既往用药；若患者信息或证据不足以安全指定具体替代药，不得编造，应给出替代选择原则、"
            "临时风险控制措施和需要补充的信息。替代路径必须真正消除禁忌组合，不能把换用同类低风险药、"
            "减量、错峰、缩短重叠、增加支持性治疗或加强监测写成允许继续联用的条件。不得在 risks 或其他字段"
            "中重新加入『若必须短期联用』之类的例外。未提供具体药名、剂量和疗程时，不得列举数字洗脱期或"
            "半衰期示例；替代药物不得仍属于冲突类别，也不得脱离当前适应证。意外重叠且疗程未知时，不得无条件"
            "要求立即停用其中某一种药，应由处方兽医在下一次给药前决定具体暂停、减停或替换对象。"
            "面向兽医用户写处方边界与监测要点，不要写『勿自行给人药/请遵医嘱就医』等宠主口号，不要自称同事。"
        ),
        allowed_tools=["rag.search", "mcp.web_search.web_search"],
        rag_query_hint="drug dosage contraindication toxicity interaction",
        rag_categories=list(_PHARMACY_RAG_CATEGORIES),
    ),
    "behavior": ExpertConfig(
        key="behavior",
        name_zh="行为安抚老师",
        persona=(
            "你是一位动物行为与安抚专家，负责焦虑/应激识别、行为矫正与医患沟通要点。"
            "给出可执行的训练与安抚建议；若需向宠主传达，写成『可告知宠主…』的沟通建议，"
            "不要直接用『请立即就医』等对宠主喊话的口吻作为结论。"
        ),
        allowed_tools=["rag.search", "mcp.web_search.web_search"],
        rag_query_hint="animal behavior anxiety stress training",
        rag_categories=list(_BEHAVIOR_RAG_CATEGORIES),
    ),
}


def _rag_metrics(result: Dict[str, Any]) -> tuple[int, float]:
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
    try:
        value = int(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default
    return max(1, value)


def _env_float(name: str, default: float) -> float:
    try:
        value = float(os.getenv(name, "") or default)
    except (TypeError, ValueError):
        return default
    return max(0.1, value)


@dataclass(frozen=True)
class ExpertLoopConfig:
    max_rounds: int = field(default_factory=lambda: _env_int("MOE_EXPERT_MAX_ROUNDS", 6))
    max_tool_calls: int = field(default_factory=lambda: _env_int("MOE_EXPERT_MAX_TOOL_CALLS", 4))
    timeout_s: float = field(default_factory=lambda: _env_float("MOE_EXPERT_TIMEOUT_SEC", 60.0))
    finalize_reserve_s: float = field(
        default_factory=lambda: _env_float("MOE_EXPERT_FINALIZE_RESERVE_SEC", 12.0)
    )
    max_repeated_calls: int = field(
        default_factory=lambda: _env_int("MOE_EXPERT_MAX_REPEATED_CALLS", 1)
    )


@dataclass(frozen=True)
class ExpertAction:
    kind: str
    request: Optional[ToolRequest] = None
    opinion: Optional[Dict[str, Any]] = None


def _tool_result_context(result: ToolResult, max_chars: int = 4000) -> str:
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
    return {
        "role": "user",
        "content": f"TOOL_RESULT\n{content}\n请基于该结果决定下一步动作。",
    }


class ExpertAgentSession:
    """Independent expert context that can iteratively call tools until final."""

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
        recorder: Optional[MoETrace] = None,
        loop_config: Optional[ExpertLoopConfig] = None,
    ) -> None:
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
        self.recorder = recorder
        self.loop_config = loop_config or ExpertLoopConfig()
        self.started_at = time.monotonic()
        self.rounds = 0
        self.tool_call_count = 0
        self.call_counts: Dict[str, int] = {}
        self.tool_results: List[Dict[str, Any]] = []
        self.plan_steps: List[Dict[str, Any]] = []
        self.tools_used: List[str] = []
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

        audience = _AUDIENCE_VET if user_role == "veterinarian" else _AUDIENCE_OWNER
        retrieval_policy = _IMPORTANT_RETRIEVAL_POLICY.get(expert.key, "")
        tool_brief = [
            {
                "name": tool.name,
                "description": tool.description,
                "input_schema": tool.input_schema,
            }
            for tool in available
        ]
        system_prompt = (
            f"{expert.persona}\n{_SPECIES_GUARD}\n{_SPECIES_BREED_GUARD}\n{audience}\n"
            f"{retrieval_policy}\n\n"
            "你是独立运行的专家 Subagent。你拥有自己的上下文，必须根据用户问题和本上下文中的"
            "工具结果逐轮决定下一步；不要预先生成固定多步计划。每轮只能执行一个动作。\n"
            "需要外部证据时返回 action=tool；证据充分或无需工具时返回 action=final。"
            "不要为了形式调用工具，也不要重复相同调用。你必须只输出严格 JSON。\n"
            "调用 rag.search 时，arguments.query 必须完全使用英语，不得包含中文、日文或韩文字符。\n"
            "工具动作格式："
            '{"action":"tool","tool_name":"<name>","arguments":{},"reason":"..."}\n'
            "最终意见格式："
            '{"action":"final","opinion":{"conclusion":"...","evidence":[],"risks":[],"confidence":0.0}}\n'
            f"最终 opinion 约束：{_OUTPUT_CONTRACT}\n"
            f"当前可用工具：{json.dumps(tool_brief, ensure_ascii=False)}"
        )
        payload: Dict[str, Any] = {
            "user_question": query,
            "user_role": user_role,
        }
        if species_zh:
            payload["species"] = species_zh
        if breed:
            payload["breed"] = breed
        history_context = build_fact_state_history(conversation_history, expert_context_history)
        if history_context:
            payload["history_context"] = history_context
        self.messages: List[Dict[str, Any]] = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
        ]

    @property
    def remaining_timeout_s(self) -> float:
        elapsed = time.monotonic() - self.started_at
        return max(0.0, self.loop_config.timeout_s - elapsed)

    @property
    def finalize_reserve_s(self) -> float:
        return min(self.loop_config.finalize_reserve_s, self.loop_config.timeout_s * 0.25)

    @property
    def tool_wait_timeout_s(self) -> float:
        return max(0.001, self.remaining_timeout_s - self.finalize_reserve_s)

    def _normalize_tool_arguments(self, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        normalized = dict(arguments or {})
        if tool_name == "rag.search":
            species_term = f" {self.species_en}" if self.species_en else ""
            breed_term = f" {self.breed}" if is_english_rag_query(self.breed) else ""
            default_query = f"{self.expert.rag_query_hint}{species_term}{breed_term}".strip()
            normalized.setdefault("query", default_query)
            normalized.setdefault("top_k", self.rag_top_k)
            if self.device is not None:
                normalized.setdefault("device", self.device)
            if self.expert.rag_categories:
                normalized["category"] = list(self.expert.rag_categories)
        elif tool_name == "mcp.web_search.web_search":
            normalized.setdefault("query", self.query)
            normalized.setdefault("max_results", 5)
        return normalized

    def _parse_opinion(self, obj: Dict[str, Any]) -> Dict[str, Any]:
        opinion = obj.get("opinion") if isinstance(obj.get("opinion"), dict) else obj
        conclusion = str(opinion.get("conclusion") or opinion.get("answer") or "").strip()
        evidence = opinion.get("evidence")
        risks = opinion.get("risks")
        try:
            confidence = max(0.0, min(1.0, float(opinion.get("confidence", 0.0))))
        except (TypeError, ValueError):
            confidence = 0.0
        return {
            "conclusion": conclusion or self.last_output[:800] or "（专家未生成有效结论）",
            "evidence": [str(item) for item in evidence] if isinstance(evidence, list) else [],
            "risks": [str(item) for item in risks] if isinstance(risks, list) else [],
            "confidence": round(confidence, 3),
        }

    async def next_action(self) -> ExpertAction:
        if self.completed:
            return ExpertAction(kind="final", opinion=self.fallback_opinion())

        self.rounds += 1
        force_final = (
            self.rounds >= self.loop_config.max_rounds
            or self.tool_call_count >= self.loop_config.max_tool_calls
            or not self.available_tools
            or self.remaining_timeout_s <= self.finalize_reserve_s
        )
        round_messages = list(self.messages)
        if force_final:
            round_messages.append({
                "role": "user",
                "content": "循环预算已到或没有可用工具。本轮必须返回 action=final，不得再调用工具。",
            })
        elif self.rounds == self.loop_config.max_rounds - 1:
            round_messages.append({
                "role": "user",
                "content": (
                    "这是倒数第二轮。你本轮仍可按需调用一次工具，但下一轮必须返回 "
                    "action=final 的结构化最终意见。请避免发起无法在下一轮完成归纳的检索。"
                ),
            })

        started = time.perf_counter()
        resp = await self.llm.chat(
            messages=round_messages,
            temperature=0.2,
            max_tokens=700,
            response_format={"type": "json_object"},
        )
        latency_ms = (time.perf_counter() - started) * 1000.0
        text = extract_text(resp)
        self.last_output = text
        if self.recorder is not None:
            self.recorder.record_llm(
                stage=f"expert:{self.expert.key}:round:{self.rounds}",
                model=getattr(self.llm, "model", ""),
                messages=round_messages,
                output=text,
                latency_ms=latency_ms,
                usage=extract_usage(resp),
                meta={"weight": self.weight, "force_final": force_final},
            )

        obj, _error = _safe_json_loads(text)
        if not isinstance(obj, dict):
            self.completed = True
            return ExpertAction(kind="final", opinion=self.fallback_opinion())

        action = str(obj.get("action") or "").strip().lower()
        if action in ("final", "final_answer") or "conclusion" in obj or isinstance(obj.get("opinion"), dict):
            self.completed = True
            return ExpertAction(kind="final", opinion=self._parse_opinion(obj))

        if force_final:
            self.completed = True
            return ExpertAction(
                kind="final",
                opinion={
                    "conclusion": "（专家在最终轮未按要求返回结构化最终意见）",
                    "evidence": [],
                    "risks": ["最终轮输出不合规，已禁止继续调用工具；该专家意见不应作为主要决策依据"],
                    "confidence": 0.0,
                },
            )

        if action not in ("tool", "call_tool"):
            self.completed = True
            return ExpertAction(kind="final", opinion=self._parse_opinion(obj))

        tool_name = str(obj.get("tool_name") or "").strip()
        arguments = obj.get("arguments") if isinstance(obj.get("arguments"), dict) else {}
        arguments = self._normalize_tool_arguments(tool_name, arguments)
        self.messages.append({"role": "assistant", "content": text})

        if tool_name not in self.available_tools:
            self.messages.append(_tool_feedback_message(json.dumps({
                "code": "TOOL_NOT_ALLOWED",
                "tool_name": tool_name,
            }, ensure_ascii=False)))
            return ExpertAction(kind="continue")

        call_key = canonical_tool_call(tool_name, arguments)
        seen = self.call_counts.get(call_key, 0)
        if seen >= self.loop_config.max_repeated_calls:
            self.messages.append(_tool_feedback_message(json.dumps({
                "code": "REPEATED_TOOL_CALL",
                "tool_name": tool_name,
            }, ensure_ascii=False)))
            return ExpertAction(kind="continue")

        self.call_counts[call_key] = seen + 1
        self.tool_call_count += 1
        self.plan_steps.append({
            "type": "tool",
            "tool_name": tool_name,
            "arguments": dict(arguments),
            "note": str(obj.get("reason") or ""),
            "round": self.rounds,
        })
        return ExpertAction(
            kind="tool",
            request=ToolRequest(
                expert=self.expert.key,
                tool_name=tool_name,
                arguments=arguments,
            ).with_request_id(),
        )

    def apply_tool_result(self, result: ToolResult) -> None:
        self.messages.append(_tool_feedback_message(_tool_result_context(result)))
        self.tools_used.append(result.tool_name)
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
        else:
            self.recorder.record_tool(
                stage=f"expert:{self.expert.key}:round:{self.rounds}",
                tool_name=result.tool_name,
                arguments=result.arguments,
                ok=result.ok,
                latency_ms=result.latency_ms,
                error=result.error,
            )

    def fallback_opinion(self) -> Dict[str, Any]:
        return {
            "conclusion": self.last_output[:800] or "（专家在循环预算内未生成有效结论）",
            "evidence": [],
            "risks": ["专家会话达到轮数、工具次数或超时限制，结论可能不完整。但具备一定参考价值"],
            "confidence": 0.0,
        }

    def build_result(self, opinion: Dict[str, Any]) -> Dict[str, Any]:
        hits_count = 0
        best_score = 0.0
        for result in self.tool_results:
            if result.get("tool_name") != "rag.search" or not isinstance(result.get("result"), dict):
                continue
            count, score = _rag_metrics(result["result"])
            hits_count += count
            best_score = max(best_score, score)
        return {
            "expert": self.expert.key,
            "name_zh": self.expert.name_zh,
            "weight": round(self.weight, 4),
            "conclusion": str(opinion.get("conclusion") or ""),
            "evidence": list(opinion.get("evidence") or []),
            "risks": list(opinion.get("risks") or []),
            "confidence": round(float(opinion.get("confidence") or 0.0), 3),
            "rag_hits": hits_count,
            "rag_best_score": round(best_score, 4),
            "tools_used": sorted(set(self.tools_used)),
            "plan_steps": list(self.plan_steps),
            "tool_results": list(self.tool_results),
            "rounds": self.rounds,
        }


async def run_expert_sessions(
    *,
    sessions: Sequence[ExpertAgentSession],
    broker: ToolBroker,
) -> List[Dict[str, Any]]:
    async def _run_session(session: ExpertAgentSession) -> Dict[str, Any]:
        opinion: Optional[Dict[str, Any]] = None
        while opinion is None:
            remaining = session.remaining_timeout_s
            if remaining <= 0:
                opinion = session.fallback_opinion()
                break
            try:
                action = await asyncio.wait_for(session.next_action(), timeout=remaining)
            except asyncio.TimeoutError:
                opinion = session.fallback_opinion()
                break
            except Exception as exc:  # noqa: BLE001
                session.last_output = f"（专家会话失败：{exc}）"
                opinion = session.fallback_opinion()
                break

            if action.kind == "final":
                session.completed = True
                opinion = action.opinion or session.fallback_opinion()
                break
            elif action.kind == "tool" and action.request is not None:
                results = await broker.execute_batch(
                    [action.request],
                    timeouts={action.request.request_id: session.tool_wait_timeout_s},
                )
                result = results.get(action.request.request_id)
                if result is not None:
                    session.apply_tool_result(result)

            if session.rounds >= session.loop_config.max_rounds or session.remaining_timeout_s <= 0:
                opinion = session.fallback_opinion()

        return session.build_result(opinion or session.fallback_opinion())

    return list(await asyncio.gather(*(_run_session(session) for session in sessions)))


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
    recorder: Optional[MoETrace] = None,
    request_allowed_tools: Optional[Sequence[str]] = None,
    loop_config: Optional[ExpertLoopConfig] = None,
) -> Dict[str, Any]:
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
        recorder=recorder,
        loop_config=loop_config,
    )
    broker = ToolBroker(registry=registry, allowed_tools=request_allowed_tools)
    return (await run_expert_sessions(sessions=[session], broker=broker))[0]
