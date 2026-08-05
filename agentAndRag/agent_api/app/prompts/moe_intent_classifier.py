"""Prompt assembly for the doctor-side D1-D8 intent classifier."""
from __future__ import annotations

import json
from typing import Dict, List, Optional

from .intent_contracts import INTENT_VARIANTS, classifier_catalog


INTENT_CLASSIFIER_SYSTEM_PROMPT = (
    "你是小动物临床 AI 的任务分类器。你的唯一任务是把兽医用户当前最主要的交付意图分类到 D1-D8；"
    "不要回答医学问题，也不要输出八类以外的标签。\n\n"
    "**可选意图**\n"
    f"{classifier_catalog()}\n\n"
    "**判定规则**\n"
    "1. 只选择一个 primary_intent，以用户本轮最希望得到的交付物为准，而不是按出现的医学关键词。\n"
    "2. D1 的核心是把已有材料转换成病历；即使材料包含诊断内容，只要主要要求是 SOAP/Problem List/摘要/EMR，仍选 D1。\n"
    "3. D4 必须有待解读的具体报告、数值或检查描述；泛问某指标含义属于 D6。\n"
    "4. D7 必须依赖历史病例状态、复诊变化或新旧计划比较；单次完整病例分析选 D2-D5。\n"
    "5. D8 仅在主要交付物是急症安全处置、危险操作/越权拦截、证据或工具失败降级时选择；"
    "普通病例中顺带存在风险不自动改成 D8。\n"
    "6. D6 是脱离某个具体患者决策的知识卡查询；具体患者的诊断/检查/治疗分别选 D2/D3/D5。\n"
    "7. 同时要求多个结果时，选择占据请求中心、最能决定最终结构的一个；理由中说明取舍。\n"
    "8. output_variant 仅 D1/D6 可使用非 default：D1 可选 soap/problem_list/summary/emr；"
    "D6 可选 dose/contraindication/interaction/guideline/sop/reference/concept/prognosis/species_difference。\n\n"
    "只输出严格 JSON，不要代码块和额外文字：\n"
    '{"primary_intent":"D1","confidence":0.95,"output_variant":"soap","reason":"一句中文理由"}'
)


def build_intent_classifier_messages(
    *,
    query: str,
    conversation_history: Optional[List[Dict[str, str]]] = None,
) -> List[Dict[str, str]]:
    history = []
    for item in (conversation_history or [])[-12:]:
        if not isinstance(item, dict):
            continue
        role = str(item.get("role") or "")
        content = str(item.get("content") or "")
        if role not in {"user", "assistant"} or not content:
            continue
        history.append({"role": role, "content": content[:3000]})
    payload = {
        "current_query": query,
        "conversation_history": history,
        "allowed_intents": [f"D{i}" for i in range(1, 9)],
        "allowed_output_variants": list(INTENT_VARIANTS),
    }
    return [
        {"role": "system", "content": INTENT_CLASSIFIER_SYSTEM_PROMPT},
        {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
    ]
