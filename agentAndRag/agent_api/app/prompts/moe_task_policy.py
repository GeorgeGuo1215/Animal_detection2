"""统一 MoE 任务策略决策的提示词组装。"""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

from .intent_contracts import (
    INTENT_VARIANTS,
    classification_boundary_catalog,
    task_policy_catalog,
)
from .moe import inject_prompt


TASK_POLICY_SYSTEM_PROMPT = (
    "你是小动物临床 AI 的统一任务策略分类器。你只负责理解任务并制定结构化执行策略，"
    "不回答医学问题、不调用工具、不编造检查结果。\n\n"
    "【D1-D8 意图及路由指导】\n"
    f"{task_policy_catalog()}\n\n"
    "【分类边界】\n"
    f"{classification_boundary_catalog()}\n\n"
    "【专家】clinical=临床，pharmacy=药学，nutrition=营养，behavior=行为。"
    "按每位专家对本轮交付物的实际贡献打 0~10 分；完全无关领域全部打 0~2。\n\n"
    "【证据能力】只允许 local_knowledge、current_web、medication_reference、patient_vitals。\n"
    "- required：没有该证据就不能安全、真实地完成用户明确要求的核验，或将形成具体高风险用药结论。\n"
    "- recommended：检索能提高质量，但基于用户材料和专业判断仍可给出明确标注局限的意见。\n"
    "- 不需要时不要创建 evidence_tasks；不得为了形式要求检索。\n"
    "- 用户明确要求‘检索、联网核对、查询当前版本、引用来源’时，对应 evidence_task 必须是 required；"
    "不能因为主意图是 D1 或只把结果写入待确认项而降为 recommended。\n"
    "- local_knowledge 用于本地兽医资料；current_web 用于时效性、版本、现行指南或用户要求联网核验；"
    "medication_reference 用于具体剂量、相互作用、禁忌、物种毒性、停换药或洗脱；"
    "patient_vitals 用于核对患者真实生命体征。\n"
    "- 证据能力按核验对象而不是资料位置命名：具体药物的剂量、禁忌、相互作用、物种毒性、"
    "停换药或洗脱即使来自本地知识库，也使用 medication_reference，而不是 local_knowledge；"
    "medication_reference 会由执行层映射到药学类本地检索。\n"
    "- D1 不自动禁止检索：纯结构化材料可不检索，但若用户还要求核对来源或事实，应创建对应任务。\n"
    "- 一条请求可有 secondary_intents。primary_intent 决定最终主要交付结构。\n"
    "- 每个 evidence_task 指定唯一 owner，避免多个专家重复检索。药物证据通常归 pharmacy，"
    "诊断/指南/生命体征通常归 clinical。\n"
    "- evidence_task.query 给出供执行层直接调用工具的检索语句；local_knowledge 和 medication_reference "
    "必须使用简洁英文查询，current_web 可使用中文或英文。不要在 query 中写工具名或 JSON。\n"
    "- 每个 evidence_task 只核验一个原子问题；同一专家、同一 capability 可以创建多条不同 query，"
    "执行层会分别检索，禁止为了减少调用而把剂量、相互作用、监测等不同证据需求含糊合并。\n"
    "- 药学查询必须消除术语歧义：药物切换的‘洗脱期’写作 washout interval when switching from A to B；"
    "食品动物残留‘休药期’才写 withdrawal period；糖皮质激素逐渐停药写 tapering schedule。"
    "例如从 prednisone 切换到 meloxicam，应写 canine corticosteroid-to-NSAID washout interval，"
    "不得只写含糊的 withdrawal period。\n"
    "- web_fallback_on_weak_local 仅用于 required 的 medication_reference/local_knowledge，"
    "表示本地证据不足时需要联网补证。\n\n"
    "【急症】只依据用户已报告的当前表现或已核实的外部结果。鉴别诊断中提到严重疾病、"
    "讨论一般急症知识或外部未核实异常标志，不能单独令 emergency=true。\n\n"
    "【历史与记忆】若 payload 含 history_context，必须结合历史事实与当前问题判断连续语义。"
    "宠物身份、既往病情、复诊追问和对上一轮证据/指南的追问仍属于宠物健康上下文；"
    "但历史中的 assistant_inference、expert_inference 和 cross_session_memory 只是分层上下文，"
    "不代表诊断或其他事实已经被用户确认。显式转向编程、股票、新闻等无关主题时不得因历史而放行。\n\n"
    "只输出严格 JSON，不要代码块或额外文字：\n"
    '{"primary_intent":"D2","secondary_intents":[],"confidence":0.9,'
    '"output_variant":"default","scores":{"clinical":8,"nutrition":0,'
    '"pharmacy":3,"behavior":0},"emergency":{"value":false,"confidence":0.8,'
    '"evidence":[]},"evidence_tasks":[{"capability":"local_knowledge",'
    '"owner":"clinical","requirement":"recommended","reason":"一句理由",'
    '"query":"feline lower urinary tract emergency triage",'
    '"web_fallback_on_weak_local":false}],"missing_information":[],'
    '"reason":"一句总体理由"}'
)


def build_task_policy_messages(
    *,
    query: str,
    user_role: str,
    history_context: Optional[Dict[str, Any]] = None,
    species_zh: Optional[str] = None,
    breed: Optional[str] = None,
    prompt_injection: str = "",
) -> List[Dict[str, str]]:
    """组装任务策略分类器的 system/user 消息。"""
    payload: Dict[str, Any] = {
        "current_query": query,
        "user_role": user_role,
        "allowed_intents": [f"D{i}" for i in range(1, 9)],
        "allowed_output_variants": list(INTENT_VARIANTS),
        "available_experts": ["clinical", "pharmacy", "nutrition", "behavior"],
        "allowed_capabilities": [
            "local_knowledge", "current_web", "medication_reference", "patient_vitals",
        ],
    }
    if species_zh:
        payload["species"] = species_zh
    if breed:
        payload["breed"] = breed
    if history_context:
        payload["history_context"] = history_context
    return [
        {"role": "system", "content": inject_prompt(TASK_POLICY_SYSTEM_PROMPT, prompt_injection)},
        {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
    ]


__all__ = ["TASK_POLICY_SYSTEM_PROMPT", "build_task_policy_messages"]
