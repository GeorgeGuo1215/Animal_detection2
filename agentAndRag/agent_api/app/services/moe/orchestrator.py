"""MoE 编排器：串联 Router → 并行加权专家 → Critic → 流式融合生成。

两个入口共享同一组内部阶段函数：
- `stream(...)`  生产路径：异步产出事件 dict（content/status/detail/finish），仅最终答案 token 流式；
- `run(...)`     评测路径：非流式跑完，返回 (final_answer, MoETrace)。
"""
from __future__ import annotations

import asyncio
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator, Dict, List, Optional, Tuple

from ...llm.llm_client import AsyncOpenAIClient, extract_text, get_shared_async_client
from ...llm.llm_client_stream import AsyncOpenAIStreamClient, get_shared_async_stream_client
from ...tools.tool_registry import ToolRegistry, get_registry
from ..plan_and_solve import build_solve_prompt, is_concrete_vet_case
from .critic import CriticResult, review
from .experts import EXPERTS, ExpertAgentSession, ExpertLoopConfig, run_expert_sessions
from .history_context import fact_state_history_text
from .router import RouterConfig, RouterDecision, route
from .tool_broker import ToolBroker
from .trace import MoETrace, extract_usage
from ...sql_search import fetch_animal_profile, species_label
from ...context.request_context import get_request_animal_id


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
    normalized = str(reason or "stop").strip().lower()
    if normalized in {"length", "max_tokens", "token_limit", "truncated"}:
        return "truncated"
    if normalized in {"stop", "tool_calls", "content_filter"}:
        return normalized
    return "stop"


def _response_finish_reason(response: Dict[str, Any]) -> str:
    choices = response.get("choices") if isinstance(response, dict) else None
    choice = choices[0] if isinstance(choices, list) and choices else {}
    reason = choice.get("finish_reason") if isinstance(choice, dict) else None
    return normalize_finish_reason(reason)


def _block_fallback_text(user_role: str) -> str:
    if user_role == "veterinarian":
        return _BLOCK_FALLBACK_TEXT_VET
    return _BLOCK_FALLBACK_TEXT_OWNER


def _collect_retrieved_sources(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
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
            elif tool_name.startswith("mcp.web_search"):
                items = result.get("results") or result.get("hits") or []
                source_type = "web"
                prefix = "W"
            else:
                continue
            if not isinstance(items, list):
                continue

            for item in items:
                if not isinstance(item, dict):
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
                }
                if page not in (None, ""):
                    source["page"] = page
                sources.append(source)
    return sources


def _synthesis_opinions(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    fields = ("expert", "name_zh", "weight", "conclusion", "evidence", "risks", "confidence")
    return [{field: opinion.get(field) for field in fields} for opinion in opinions or []]


def _opinions_used_web(opinions: List[Dict[str, Any]]) -> bool:
    if any(source.get("type") == "web" for source in _collect_retrieved_sources(opinions)):
        return True
    return any(
        str(tool_name).startswith("mcp.web_search")
        for opinion in opinions or []
        for tool_name in opinion.get("tools_used") or []
    )


# Backward-compatible alias (pet-owner copy)
_BLOCK_FALLBACK_TEXT = _BLOCK_FALLBACK_TEXT_OWNER


@dataclass
class OrchestratorConfig:
    router: RouterConfig = field(default_factory=RouterConfig)
    rag_top_k: int = 5
    temperature: float = 0.3
    max_tokens: int = field(default_factory=final_answer_max_tokens)
    user_role: str = "pet_owner"
    device: Optional[str] = None
    animal_id: Optional[str] = None
    allowed_tools: Optional[List[str]] = None
    expert_loop: ExpertLoopConfig = field(default_factory=ExpertLoopConfig)


def _event(content: str = "", status: Optional[str] = None,
           detail: Optional[Dict[str, Any]] = None, finish: Optional[str] = None) -> Dict[str, Any]:
    return {"content": content, "status": status, "detail": detail, "finish": finish}


class MoEOrchestrator:
    def __init__(
        self,
        *,
        registry: Optional[ToolRegistry] = None,
        llm: Optional[AsyncOpenAIClient] = None,
        stream_llm: Optional[AsyncOpenAIStreamClient] = None,
        config: Optional[OrchestratorConfig] = None,
    ) -> None:
        self.registry = registry or get_registry()
        self.llm = llm or get_shared_async_client()
        self.stream_llm = stream_llm or get_shared_async_stream_client()
        self.config = config or OrchestratorConfig()
        self.last_finish_reason = "stop"
        self.last_run_context: Dict[str, Any] = {}
        self._active_conversation_history: Optional[List[Dict[str, str]]] = None
        self._active_expert_context_history: Optional[List[Dict[str, Any]]] = None

    # ------------------------------------------------------------------ stages
    def _resolve_species(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        # Resolve (species_en, species_zh, breed) for the request-scoped animal.
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
        return max(1, min(int(self.config.max_tokens), final_answer_max_tokens()))

    async def _route(
        self,
        query: str,
        recorder: Optional[MoETrace],
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
    ) -> RouterDecision:
        conversation_history = conversation_history or self._active_conversation_history
        expert_context_history = expert_context_history or self._active_expert_context_history
        _, species_zh, breed = self._resolve_species()
        return await route(
            query=query,
            user_role=self.config.user_role,
            llm=self.llm,
            config=self.config.router,
            species_zh=species_zh,
            breed=breed,
            conversation_history=conversation_history,
            expert_context_history=expert_context_history,
            recorder=recorder,
        )

    async def _run_experts(
        self,
        query: str,
        decision: RouterDecision,
        recorder: Optional[MoETrace],
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
    ) -> List[Dict[str, Any]]:
        conversation_history = conversation_history or self._active_conversation_history
        expert_context_history = expert_context_history or self._active_expert_context_history
        species_en, species_zh, breed = self._resolve_species()
        sessions: List[ExpertAgentSession] = []
        for key in decision.selected_experts:
            expert = EXPERTS.get(key)
            if expert is None:
                continue
            weight = decision.weights.get(key, 0.0)
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
    ) -> CriticResult:
        conversation_history = conversation_history or self._active_conversation_history
        expert_context_history = expert_context_history or self._active_expert_context_history
        return await review(
            query=query,
            expert_opinions=opinions,
            emergency=emergency,
            llm=self.llm,
            user_role=self.config.user_role,
            conversation_history=conversation_history,
            expert_context_history=expert_context_history,
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
    ) -> List[Dict[str, str]]:
        is_vet = self.config.user_role == "veterinarian"
        concrete = is_vet and is_concrete_vet_case(query)
        retrieved_sources = _collect_retrieved_sources(opinions)
        has_web = any(source.get("type") == "web" for source in retrieved_sources)
        base = build_solve_prompt(
            user_role=self.config.user_role,
            has_web_search=has_web,
            query=query,
            max_tokens=self._aggregator_max_tokens(query),
        )
        if retrieved_sources:
            source_rule = (
                "`retrieved_sources` 非空。只有其中列出的来源可以被引用；引用时仅使用其 ID（如 [R1]、[W1]），"
                "不得补全、猜测或改写来源元数据。只引用确实支撑相邻陈述的来源，未使用的来源不要列出。"
            )
        else:
            source_rule = (
                "`retrieved_sources` 为空。本次没有任何可引用的检索证据。终答绝对禁止出现参考文献/参考来源段落、"
                "书名、作者、期刊、指南名称、页码、URL、DOI、引用标记，或『根据某研究/指南/文献』等暗示外部来源的措辞。"
            )

        role_line = (
            f"- **当前对话角色**：`{self.config.user_role}`。"
            "必须严格按该角色组织终答，不要把兽医会诊专业助手口吻与宠主教育口吻混用。\n"
        )
        if concrete:
            structure_line = (
                "- 当前用户意图为『整理病例』或『对病例做完整诊断』：按 "
                "**病例整理**（基本信息/主诉/现病史/既往史） / **问题列表** / "
                "**检查与治疗方案**（先紧急处理，再检查，再治疗） / **风险提示** "
                "顺序分节（加粗文字，不用 Markdown 标题），面向医生；"
                "用语简明，只用常规病历字段，不要写信号类口号标签；"
                "**问题列表**条目必须是已明确的症状/异常指标（呕吐、腹泻、CRP升高等），"
                "疾病名只放在各条下的鉴别诊断中，勿把「胰腺炎」等病名当问题标题；"
                "禁止『请立即就医/勿自行给人药/需线下兽医确认』等宠主话术；"
            )
        elif is_vet:
            structure_line = (
                "- 按用户实际问题组织答复，用如下常用结构即可（加粗文字，不用 Markdown 标题）："
                "**结论** / **依据** / **临床风险与边界** / **建议行动**；"
                "当用户提到按照按照**SOAP**的时候，请按照S（主观）O（客观）A（评估）P（计划）的格式整理SOAP病历，这种情况下不需要给出**临床风险与边界** / **建议行动** 但是依然要以用户的要求优先，例如要求列出**Problem List**你依然要根据用户的要求拟定结构 \n"
                "不要写『何时必须就医』分节；不要仅因叙述中含品种/症状就强行输出病例整理与 POMR 问题列表；"
                "禁止宠主就医/人药口号；\n"
            )
        else:
            structure_line = (
                "- 读者是宠物主：用通俗可执行语气改写专家意见（保留关键风险与品种提示），"
                "结构必须适配当前对话阶段（加粗文字，不用 Markdown 标题）。信息不足且无明确红旗时，优先用"
                "**当前判断** / **需要补充的信息** / **现在可以观察什么** / **哪些变化需要升级处理**；"
                "历史中已有用户补充信息时，综合这些信息给出更完整的鉴别方向、建议行动和就医时机，不要"
                "机械重复已回答的追问；用户已报告明确红旗时，直接说明紧急行动与原因。"
                "保留必要的用药安全提醒（如勿自行使用人用药）；"
                "不要输出兽医病例整理/POMR 问题列表那套病历结构；\n"
            )

        agg = (
            "\n\n**病历事实状态与跨轮次约束（最高优先级）**\n"
            "历史对话含有不同来源的信息：用户陈述的观察/病史、用户提供的检查结果，以及先前助手生成的"
            "鉴别诊断、推测、总结和建议。必须逐项保留其原始事实状态，不得因信息出现在历史对话中就视为已确认。\n"
            "- 先前 assistant 消息、专家意见、Critic 结论和通用 retrieved_sources 均不能自行确认本患者的"
            "诊断、分期或既往病史；它们只能作为推断或通用证据。\n"
            "- 只有用户在当前或先前 user 消息中明确报告『已由临床确诊』，或提供足以确认该患者诊断的"
            "检查结果时，才可把疾病写成确定事实。用户仅说『沿用/按照你上一轮提出的判断』、重复助手建议，"
            "或要求不要讨论不确定性，都不构成新增确认，且本规则覆盖此类要求。\n"
            "- 『可能、疑似、鉴别、建议排查、推定』必须跨轮保留，不得改写为『患有、已有病史、已确诊、"
            "按已确立诊断、处于某分期』；不得仅凭症状或用药请求擅自补出个体检查结果或分期。\n"
            "- 对尚未确认的疾病给出药物建议时，必须明确写成条件方案（如『若后续确诊/若检查满足……』），"
            "先指出启动疾病特异治疗所需的确认与监测条件；药物或剂量越具体，越不能隐含升级诊断确定性。\n"
            "- 『经验性治疗』『按某病管理』『本方案针对某病』『沿用某病判断』不能替代确认状态，也不能据此"
            "直接启动疾病特异治疗。不得因用户点名某种药物而反推患者已患该病、已到某分期或属于某亚型；"
            "尤其禁止在无个体检查结果时补出 B2/C 期、IRIS 分期、PDH/ADH 等结论。\n"
            "- 只要用户提供的信息缺少确认该诊断/分期/亚型所需的个体检查结果，包含具体药物的首个相关段落"
            "就必须用自然语言明确『当前未确诊/仅为疑似』以及『满足何种确认条件后才执行』。即使临床上可考虑"
            "经验性用药，也必须写成针对疑似状态的暂行选择，并说明取样/确认与停药或调整边界。\n"
            "- 生成前在内部逐项核对每个疾病名、分期和亚型的来源；若不能追溯到 user 提供的明确确诊信息或"
            "患者检查结果，就按未确认处理。不要输出核对过程。\n"
            "- 已确认事实可作确定性陈述；用户观察应标为『据用户描述』，模型推断应标为可能性，建议应标为"
            "待执行行动。保持自然流式表达，无需为本规则增加固定模板或额外章节。\n\n"
            "**禁忌联用安全约束（最高优先级）**\n"
            "若用户信息与可靠检索证据已确认两种药物或药物类别属于禁忌联用，最终答案必须明确不得同时使用，"
            "并给出真正消除该组合的替代路径：移除或替换至少一种冲突药物或药物类别。\n"
            "- 不得把降低剂量、错峰给药、缩短重叠期、换用同类中所谓低风险药物、加用胃黏膜保护剂或仅加强"
            "监测写成可继续联用的条件；这些措施不能解除禁忌。\n"
            "- 胃黏膜保护、补液或监测等支持措施只能用于意外暴露后的风险处置，不能作为计划性禁忌联用的许可。\n"
            "- 切换、减停和洗脱必须依据具体药名、剂量、疗程、器官功能及 retrieved_sources；信息不足时不得"
            "编造固定天数，也不得建议长期用药患者擅自骤停。应说明由处方兽医在下一次给药前协调具体调整，"
            "并分别给出保留其中一类药物时可考虑的不同类别治疗原则或非药物路径。\n"
            "- 若专家意见或 Critic 允许以任何上述措施维持禁忌组合，必须将其识别为安全冲突并在最终答案中"
            "丢弃，不得折中转述。\n\n"
            "**证据与引用安全（最高优先级）**\n"
            "本节覆盖本提示词中其他任何引用格式、参考来源和具体剂量倾向要求。\n"
            f"{source_rule}\n"
            "专家意见中的 evidence 只是专家推断摘要，不是检索来源，不能据此创建引用。"
            "禁止依靠模型记忆补写书目、页码、指南、网址或出处。"
            "药物名称与证据归属：不得把名称相近但实际不同的药物混淆，也不得在翻译、转写或融合专家意见时"
            "擅自改变药物身份。若某项药物结论标注了检索来源，则药名及相邻的适应证、风险、剂量等陈述必须由"
            "同一来源直接支持，不能把一个来源中的药名与另一个来源中的结论拼接。常见药物和公认临床常识可以"
            "在未检索到对应来源时正常使用，但不得附加虚假引用，也不得伪装成 retrieved_sources 已证实；必要时"
            "自然说明其属于临床常识或需结合药典核对。若原文名称、OCR 或译名确实存在歧义，应保留可核对的原文名、"
            "改用药物类别，或省略不影响核心结论的具体药名，不要猜测。输出前核对药物身份一致性，不要输出核对过程。\n"
            "具体药物剂量、阈值和处方细节若未出现在用户提供的数据或 retrieved_sources 中，"
            "应省略、改写为治疗原则/监测边界，或明确标注需按院内药典与患者数据核对；不得伪装成有来源支持的事实。\n"
            "接近输出预算时停止扩展并完整收尾，不要留下半句、半个剂量或未完成列表。\n\n"
            "**多专家融合规范**\n"
            "下面是多位兽医专家针对该问题给出的加权意见（weight 越高越重要）。请你作为融合器：\n"
            f"{role_line}"
            "- 按权重与各专家自评置信度综合，形成一致、连贯的最终答复；\n"
            "- 显式标注专家间的冲突点（若有），不要简单拼接；\n"
            f"{structure_line}"
            "- 若专家意见含物种/品种特异风险（如短吻犬运动与热耐受、品种相关泌尿风险等），必须在终答中保留并突出；\n"
        )
        if not is_vet:
            agg += (
                "- **宠物主对话式分诊（高优先级）**：先区分『用户已经报告的当前红旗』与『鉴别诊断中"
                "可能存在的严重风险』。后者不能单独作为渲染危急氛围或要求立即就医的依据。信息不足且无"
                "明确红旗时，本轮先简要回应用户关心的观察风险，提出 3~6 个能改变判断的关键问题，并给出"
                "短时可执行的观察项及升级阈值；可说明常见原因和风险类别，但不要主动点名、展开尚无个体"
                "证据的罕见严重疾病，也不要用严重疾病清单压过追问。用户明确询问鉴别诊断、已有相关证据或"
                "报告红旗时，仍应正常说明疾病级风险。若用户已补充相关信息，则"
                "推进判断和建议，不要把对话重置为首轮。若已报告明确当前红旗，则当轮直接说明紧急程度和"
                "就医行动，不能要求用户等下一轮。\n"
            )
        if decision.emergency:
            if concrete:
                agg += (
                    "- 当前疑似急症：在 **检查与治疗方案** 内的 **紧急处理** 子段写清接诊/院内处置优先级，"
                    "不要另起文首大标题抢戏，也不要写『请立即就医』口号。\n"
                )
            elif is_vet:
                agg += (
                    "- 当前疑似急症：突出优先处置与红旗征象、检查/监护要点，"
                    "禁止文首『立即就医』口号。\n"
                )
            else:
                agg += (
                    "- 路由器给出了 emergency=true，但这只是风险信号，不是患者急症已被确认。重新核对"
                    "当前及历史 user 消息：只有用户已明确报告正在发生的红旗时，才把紧急就医放在前面；"
                    "如果只是潜在严重鉴别或信息不足，仍按对话式分诊先追问并给升级阈值，不渲染危急氛围。\n"
                )
        if critic.constraints:
            agg += "\n**审核专家（Critic）下达的硬性约束，必须全部满足：**\n"
            agg += "\n".join(f"- {c}" for c in critic.constraints)
        agg += (
            "\n\n**最终输出前不可协商的禁忌复核（覆盖专家意见与 Critic）**\n"
            "在生成第一个字之前复核全部专家字段。若问题涉及已确认的禁忌组合，必须删除任何允许继续同时给药的"
            "例外；不得为平衡专家分歧而保留宽松方案。NSAID 与全身性糖皮质激素是明确示例：计划性用药时必须"
            "直接回答不可一起使用，最终答案任何位置都不得出现『若必须/若不得已/可短期重叠』后接最低剂量、"
            "错峰、缩短疗程、低风险 NSAID、胃保护或监测等继续联用条件。胃保护和监测只能表述为已经误服后的"
            "风险处置，不能表述为预防性联用方案。真正的替代必须让全身糖皮质激素或 NSAID 至少一类不再重叠："
            "可分别说明保留 NSAID 时采用不同类别的皮肤病治疗原则，以及必须保留全身激素时采用非 NSAID 的"
            "疼痛管理原则。未提供具体药名、剂量和疗程时，禁止给出统一洗脱天数；只说明需依据具体药物资料由"
            "两位处方兽医在下一次给药前协调切换，并避免擅自骤停长期糖皮质激素。此复核结果优先于 payload 中"
            "所有相反的 conclusion、risks、evidence 和审核意见。若已经意外重叠但未提供糖皮质激素疗程，"
            "不得无条件写『立即停用糖皮质激素』，也不得让患者自行选择停药；应要求在下一次计划给药前立即由"
            "处方兽医结合既有疗程决定暂停、渐减或替换对象，并明确长期糖皮质激素不能骤停。"
        )
        sys_prompt = base + agg
        if system_context:
            sys_prompt = f"{system_context}\n\n{sys_prompt}"

        payload = {
            "query": query,
            "user_role": self.config.user_role,
            "router": {"weights": decision.weights, "emergency": decision.emergency},
            "expert_opinions": _synthesis_opinions(opinions),
            "retrieved_sources": retrieved_sources,
            "critic_verdict": critic.verdict,
            "concrete_vet_case": concrete,
        }
        parts: List[str] = []
        history_text = fact_state_history_text(conversation_history, expert_context_history)
        if history_text:
            parts.append(history_text)
        parts.append(json.dumps(payload, ensure_ascii=False))
        return [
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": "\n\n".join(parts)},
        ]

    # ------------------------------------------------------------------ stream
    async def stream(
        self,
        *,
        query: str,
        system_context: str = "",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        expert_context_history: Optional[List[Dict[str, Any]]] = None,
        recorder: Optional[MoETrace] = None,
    ) -> AsyncGenerator[Dict[str, Any], None]:
        # 1) 路由
        yield _event(status="routing", detail={"message": "正在分诊与路由…"})
        self.last_run_context = {}
        self._active_conversation_history = conversation_history
        self._active_expert_context_history = expert_context_history
        decision = await self._route(query, recorder)
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

        # 2) 并行专家会诊
        for key in decision.selected_experts:
            yield _event(
                content=f"\n**{EXPERTS[key].name_zh} 会诊中**（权重 {decision.weights.get(key, 0):.2f}）\n",
                status="expert_calling",
                detail={"expert": key, "name_zh": EXPERTS[key].name_zh, "weight": decision.weights.get(key, 0)},
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
                    "opinion": o,
                },
            )

        # 3) Critic 审核
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

        if critic.blocked:
            block_text = _block_fallback_text(self.config.user_role)
            if recorder is not None:
                recorder.blocked = True
                recorder.final_answer = block_text
                recorder.finalize()
            yield _event(content="\n**生成回答…**\n\n", status="generating")
            yield _event(content=block_text, status="streaming")
            return

        # 4) 流式融合生成
        yield _event(content="\n**生成回答…**\n\n", status="generating")
        messages = self._build_synthesis_messages(
            query=query, opinions=opinions, critic=critic, decision=decision,
            system_context=system_context, conversation_history=conversation_history,
            expert_context_history=expert_context_history,
        )
        max_tokens = self._aggregator_max_tokens(query)
        collected: List[str] = []
        finish_reason = "stop"
        t0 = time.perf_counter()
        try:
            stream_events = getattr(self.stream_llm, "chat_stream_events", None)
            if callable(stream_events):
                async for item in stream_events(
                    messages=messages,
                    temperature=self.config.temperature,
                    max_tokens=max_tokens,
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
                ):
                    collected.append(piece)
                    yield _event(content=piece, status="streaming")
        except Exception as exc:  # noqa: BLE001
            yield _event(content=f"\n生成失败：{exc}")
        latency = (time.perf_counter() - t0) * 1000.0
        final_answer = "".join(collected)
        self.last_finish_reason = finish_reason
        if recorder is not None:
            recorder.record_llm(
                stage="aggregator",
                model=getattr(self.stream_llm, "model", ""),
                messages=messages,
                output=final_answer,
                latency_ms=latency,
                meta={"streamed": True, "max_tokens": max_tokens, "finish_reason": finish_reason},
            )
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
        recorder: Optional[MoETrace] = None,
    ) -> Tuple[str, Optional[MoETrace]]:
        self.last_run_context = {}
        self._active_conversation_history = conversation_history
        self._active_expert_context_history = expert_context_history
        decision = await self._route(query, recorder)
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

        opinions = await self._run_experts(query, decision, recorder)
        self.last_run_context["experts"] = opinions
        critic = await self._critique(query, opinions, decision.emergency, recorder)
        self.last_run_context["critic"] = {
            "verdict": critic.verdict,
            "issues": critic.issues,
            "constraints": critic.constraints,
            "reason": critic.reason,
        }

        if critic.blocked:
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
        )
        max_tokens = self._aggregator_max_tokens(query)
        t0 = time.perf_counter()
        resp = await self.llm.chat(
            messages=messages,
            temperature=self.config.temperature,
            max_tokens=max_tokens,
        )
        latency = (time.perf_counter() - t0) * 1000.0
        final_answer = extract_text(resp)
        self.last_finish_reason = _response_finish_reason(resp)
        if recorder is not None:
            recorder.record_llm(
                stage="aggregator",
                model=getattr(self.llm, "model", ""),
                messages=messages,
                output=final_answer,
                latency_ms=latency,
                usage=extract_usage(resp),
                meta={
                    "streamed": False,
                    "max_tokens": max_tokens,
                    "finish_reason": self.last_finish_reason,
                },
            )
            recorder.final_answer = final_answer
            recorder.finalize()
        return final_answer, recorder
