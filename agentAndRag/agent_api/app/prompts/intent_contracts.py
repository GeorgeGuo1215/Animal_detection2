"""Doctor-side intent catalogue and output contracts for MoE.

The classifier, Router injection, Aggregator injection and evaluation harness all
consume this registry so adding or revising an intent does not require editing
the orchestration code.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple


DEFAULT_INTENT_ID = "D2"

INTENT_VARIANTS: Tuple[str, ...] = (
    "default",
    "soap",
    "problem_list",
    "summary",
    "emr",
    "dose",
    "contraindication",
    "interaction",
    "guideline",
    "sop",
    "reference",
    "concept",
    "prognosis",
    "species_difference",
)


INTENT_CLASSIFICATION_BOUNDARIES: Tuple[str, ...] = (
    "以用户本轮最希望获得的主要交付物确定 primary_intent，不能按医学关键词或风险词机械分类。",
    "D1 的核心是把已有材料忠实转换为 SOAP、Problem List、摘要或 EMR；即使材料包含诊断、检查或用药内容，只要主要要求仍是整理病历，就选 D1。",
    "D2 负责病例鉴别、病因排序和风险判断；当主要交付物变成检查路径、既有报告解释或治疗方案时，分别选择 D3、D4、D5。",
    "D3 规划尚未完成的检查；D4 解读已经给出的具体报告、数值、单位或影像描述。泛问某指标或概念的含义属于 D6。",
    "D5 面向具体患者制定治疗、用药与监测决策；D6 是脱离具体患者决策的知识卡、指南、剂量、概念、参考范围、预后或物种差异查询。",
    "D7 必须依赖既往病例状态、复诊变化或新旧计划比较；只有单轮材料、没有历史状态变化的完整病例仍按 D1-D5 分类。",
    "D8 仅在主要交付物是急症安全处置、危险操作或越权请求拦截、证据/工具失败降级时选择；普通病例存在高风险鉴别不自动改为 D8。",
    "复合问题允许 secondary_intents；primary_intent 只保留一个并决定最终回答结构，次意图用于补充专家与证据任务，不能覆盖主意图输出契约。",
)


@dataclass(frozen=True)
class IntentPromptSpec:
    intent_id: str
    name: str
    description: str
    routing_guidance: str
    output_contract: str
    required_sections: Tuple[str, ...]


INTENT_SPECS: Dict[str, IntentPromptSpec] = {
    "D1": IntentPromptSpec(
        intent_id="D1",
        name="病历结构化",
        description="整理问诊材料，生成 SOAP、Problem List、病例摘要或 EMR 草稿；核心是忠实映射事实。",
        routing_guidance="临床专家为主，关注事实抽取、事实归属和时间线；不要把整理任务误作全面诊断。",
        output_contract=(
            "使用固定病历字段：**主诉**、**现病史**、**既往史**、**客观检查**、**评估**、"
            "**计划**、**待确认项**。原始事实与临床推断必须分开；材料没有的内容写“未提供/待确认”，"
            "不得补写。按时间顺序整理，保留用药、过敏和既往史的事实归属。"
        ),
        required_sections=("主诉", "现病史", "既往史", "客观检查", "评估", "计划", "待确认项"),
    ),
    "D2": IntentPromptSpec(
        intent_id="D2",
        name="临床问题分析",
        description="病例鉴别诊断、病因排序、风险判断及下一步临床推理。",
        routing_guidance="优先激活临床专家；涉及用药、营养或行为病因时再提高相应专家权重。",
        output_contract=(
            "依次输出 **问题表示**、**鉴别诊断**、**高风险项**、**信息缺口**、**下一步验证**。"
            "鉴别诊断按优先级列 Top-N；每项分别写支持证据、反对证据和缺失信息。"
            "高风险疾病不得遗漏，但无依据时不得写成确诊。"
        ),
        required_sections=("问题表示", "鉴别诊断", "高风险项", "信息缺口", "下一步验证"),
    ),
    "D3": IntentPromptSpec(
        intent_id="D3",
        name="检查规划",
        description="制定首诊或复诊检查路径，在时效、成本和侵入性之间分层。",
        routing_guidance="以临床专家为主；要求专家把每项检查对应到具体临床问题，避免检查清单堆砌。",
        output_contract=(
            "依次输出 **检查目标**、**必要检查**、**可选检查**、**暂缓检查**、"
            "**优先级与触发条件**。必要/可选/暂缓清单中的每一项均使用“检查：…；目的：…；"
            "能回答的问题：…；优先级：…”的明确字段，不能只罗列检查名称；"
            "可选/暂缓项说明成本、侵入性、替代方案或何时升级。"
        ),
        required_sections=("检查目标", "必要检查", "可选检查", "暂缓检查", "优先级与触发条件"),
    ),
    "D4": IntentPromptSpec(
        intent_id="D4",
        name="报告解读",
        description="解读 CBC、生化、尿检、PCR、影像或其他报告，联合解释异常并识别危急值。",
        routing_guidance="优先激活临床专家；要求严格复述数值和单位，并区分报告事实、可能机制和临床结论。",
        output_contract=(
            "依次输出 **异常摘要**、**模式识别**、**临床关联**、**危急项**、**下一步建议**。"
            "先准确复述异常数值与单位，再解释组合模式；明确区分事实、可能机制与患者结论。"
            "危急值或需立即复核项必须单列；没有危急项时明确写“未发现明确危急值”。"
        ),
        required_sections=("异常摘要", "模式识别", "临床关联", "危急项", "下一步建议"),
    ),
    "D5": IntentPromptSpec(
        intent_id="D5",
        name="治疗与用药安全",
        description="治疗路径、支持疗法、药物建议、相互作用、监测和复查计划。",
        routing_guidance="临床与药理专家优先；涉及饮食治疗时提高营养专家权重。必须核对禁忌和相互作用。",
        output_contract=(
            "依次输出 **治疗目标**、**方案分层**、**用药核对**、**监测与复查**、**禁忌与红旗**。"
            "治疗按优先级和前提条件分层；具体药物需覆盖药名、适应证、剂量依据、途径、频次/疗程，"
            "但条件不足或无可靠来源时不得编造剂量。明确物种禁忌、相互作用和可执行复查节点。"
        ),
        required_sections=("治疗目标", "方案分层", "用药核对", "监测与复查", "禁忌与红旗"),
    ),
    "D6": IntentPromptSpec(
        intent_id="D6",
        name="专业知识快答",
        description="面向医生的药物、指南、SOP、概念、参考范围、预后或物种差异精确查询。",
        routing_guidance="按知识主题选择临床/药理/营养/行为专家；高风险事实必须检索可追溯来源。",
        output_contract=(
            "依次输出 **知识卡片**、**适用对象与前置条件**、**核心结论**、**禁忌与例外**、"
            "**来源与版本**、**不确定性**。只填写有事实依据的字段；关键结论给可追溯来源，"
            "来源冲突时展示差异与采用原则。"
        ),
        required_sections=("知识卡片", "适用对象与前置条件", "核心结论", "禁忌与例外", "来源与版本", "不确定性"),
    ),
    "D7": IntentPromptSpec(
        intent_id="D7",
        name="多轮病例管理",
        description="复诊、慢病追踪或新增信息后的病例状态维护、风险更新和计划调整。",
        routing_guidance="优先保留上一轮相关专家并由临床专家统筹；重点比较新旧事实和解释判断变化。",
        output_contract=(
            "依次输出 **病例状态**、**本轮新增/修正/撤回**、**判断更新**、**风险变化**、"
            "**计划变更**、**待跟踪项**。明确哪些事实保持、哪些被新增/修正/撤回，"
            "解释判断和计划为何改变；新增关键风险必须升级。"
        ),
        required_sections=("病例状态", "本轮新增/修正/撤回", "判断更新", "风险变化", "计划变更", "待跟踪项"),
    ),
    "D8": IntentPromptSpec(
        intent_id="D8",
        name="安全与边界控制",
        description="急症、危险操作、越权要求、证据不足、工具失败或冲突指令下的安全处置。",
        routing_guidance="临床专家负责风险分级；涉及药物或毒物时提高药理专家权重。安全响应优先于常规答题。",
        output_contract=(
            "依次输出 **安全判断**、**原因**、**立即行动与时间要求**、**必要追问**、"
            "**可继续提供的安全信息**、**能力边界**。先分级；必要时停止常规方案并进入安全模式。"
            "行动必须具体且有时间边界，不能只说“咨询兽医”；同时说明当前能做和不能做的内容。"
        ),
        required_sections=("安全判断", "原因", "立即行动与时间要求", "必要追问", "可继续提供的安全信息", "能力边界"),
    ),
}


_D1_VARIANTS = {
    "soap": (
        "用户要求 SOAP：严格输出 **S（主观）**、**O（客观）**、**A（评估）**、**P（计划）**、"
        "**待确认项**。S/O 只放原始事实，A 放推断，P 放待执行计划；缺失项写“未提供/待确认”。"
    ),
    "problem_list": (
        "用户要求 Problem List：严格输出 **病例摘要**、**Problem List**、**评估**、**计划**、"
        "**待确认项**。Problem 条目只能是已知症状/异常指标，疾病假设放在评估中。"
    ),
    "summary": (
        "用户要求病例摘要：输出 **基本信息**、**主诉**、**时间线**、**关键阳性/阴性信息**、"
        "**当前评估**、**待确认项**，保持简洁并禁止补写。"
    ),
    "emr": (
        "用户要求 EMR 草稿：使用 **主诉**、**现病史**、**既往史**、**客观检查**、**评估**、"
        "**计划**、**待确认项**，采用可直接编辑入病历的专业短句。"
    ),
}

_D6_VARIANT_FIELDS = {
    "dose": "剂量卡还需覆盖药物、适应证、物种、剂量依据、途径、频次、疗程、禁忌与监测。",
    "contraindication": "禁忌卡还需覆盖适用对象、绝对/相对禁忌、例外条件、替代原则与监测。",
    "interaction": "相互作用卡还需覆盖相互作用机制、严重度、证据等级、规避/替代方案与监测。",
    "guideline": "指南卡还需覆盖指南组织、版本/发布日期、目标人群、推荐等级和适用限制。",
    "sop": (
        "SOP 卡必须在 **核心结论** 内继续使用 **前置准备**、**操作步骤**、**停止条件**、"
        "**并发症与记录** 四个明确子字段；停止条件需给出立即中止操作及后续处置。"
    ),
    "reference": "检验参考卡还需覆盖物种/方法/单位、参考区间、前分析影响和解释限制。",
    "concept": "概念卡还需覆盖定义、临床意义、常见误区和适用边界。",
    "prognosis": "预后卡还需覆盖影响因素、随访节点、结局范围和不确定性。",
    "species_difference": "物种差异卡还需并列说明各物种差异、临床影响、禁忌与证据来源。",
}


def get_intent_spec(intent_id: str) -> IntentPromptSpec:
    return INTENT_SPECS.get(str(intent_id or "").upper(), INTENT_SPECS[DEFAULT_INTENT_ID])


def classification_boundary_catalog() -> str:
    return "\n".join(
        f"{index}. {rule}"
        for index, rule in enumerate(INTENT_CLASSIFICATION_BOUNDARIES, start=1)
    )


def task_policy_catalog() -> str:
    return "\n".join(
        (
            f"- {spec.intent_id} {spec.name}：{spec.description}\n"
            f"  路由指导：{spec.routing_guidance}"
        )
        for spec in INTENT_SPECS.values()
    )


def normalize_variant(intent_id: str, variant: str) -> str:
    value = str(variant or "default").strip().lower()
    if value not in INTENT_VARIANTS:
        return "default"
    if intent_id == "D1" and value not in {"default", *tuple(_D1_VARIANTS)}:
        return "default"
    if intent_id == "D6" and value not in {"default", *tuple(_D6_VARIANT_FIELDS)}:
        return "default"
    if intent_id not in {"D1", "D6"}:
        return "default"
    return value


def intent_output_contract(intent_id: str, variant: str = "default") -> str:
    spec = get_intent_spec(intent_id)
    normalized = normalize_variant(spec.intent_id, variant)
    if spec.intent_id == "D1" and normalized in _D1_VARIANTS:
        return _D1_VARIANTS[normalized]
    if spec.intent_id == "D6" and normalized in _D6_VARIANT_FIELDS:
        return f"{spec.output_contract} {_D6_VARIANT_FIELDS[normalized]}"
    return spec.output_contract


def intent_required_sections(intent_id: str, variant: str = "default") -> Tuple[str, ...]:
    spec = get_intent_spec(intent_id)
    normalized = normalize_variant(spec.intent_id, variant)
    if spec.intent_id == "D1":
        if normalized == "soap":
            return ("S（主观）", "O（客观）", "A（评估）", "P（计划）", "待确认项")
        if normalized == "problem_list":
            return ("病例摘要", "Problem List", "评估", "计划", "待确认项")
        if normalized == "summary":
            return ("基本信息", "主诉", "时间线", "关键阳性/阴性信息", "当前评估", "待确认项")
    if spec.intent_id == "D6" and normalized == "sop":
        return spec.required_sections + ("前置准备", "操作步骤", "停止条件", "并发症与记录")
    return spec.required_sections


def build_intent_aggregator_injection(intent_id: str, confidence: float, variant: str = "default") -> str:
    spec = get_intent_spec(intent_id)
    contract = intent_output_contract(spec.intent_id, variant)
    safety_note = ""
    if spec.intent_id == "D8":
        safety_note = (
            "\n- Critic 的 issues/constraints 只作为安全警示和内容限制，不是临床事实来源；即使其 verdict=block，"
            "也要完成本 D8 安全响应，但不得提供被阻止的危险操作、剂量或越权结论。若 retrieved_sources"
            " 没有直接支持，不得补充精确剂量、毒性阈值、检验阈值等数值；改为说明需由处方兽医结合"
            "患者资料和可追溯来源核定。"
        )
    return (
        "**医生端能力输出契约（最高优先级）**\n"
        f"- 当前主意图：`{spec.intent_id}` {spec.name}；分类置信度：{float(confidence):.2f}。\n"
        f"- 输出契约：{contract}\n"
        "- 分节必须使用加粗文字，不使用 `#` 标题；字段顺序保持不变。用户明确要求的更窄范围优先，"
        "但不得省略安全门槛项。只填写有事实依据的内容，缺失信息按契约标记，不得编造。"
        "\n- Critic 的 `revise/block` 表示必须删去或改写不安全内容，不表示跳过本意图输出契约。"
        "即使需要拒绝某个具体操作、剂量或结论，也必须保留上述分节，在对应字段解释安全边界并给出"
        "仍可提供的安全信息。Critic 的 issues/constraints 不是临床事实来源。"
        f"{safety_note}"
    )


def all_intent_ids() -> Iterable[str]:
    return INTENT_SPECS.keys()
