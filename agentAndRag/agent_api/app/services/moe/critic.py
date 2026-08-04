"""边界审核专家 Critic：召回前对专家草案做安全与边界校验。

审核维度：责任边界 / 事实一致性 / 用药安全 / 数据一致性 / 合规与免责。
输出 verdict：
- pass   : 草案安全，可直接合成；
- revise : 存在可修补问题，把 constraints 注入最终生成 prompt（强制免责/降确定性/标红用药）；
- block  : 命中硬性安全红线，一票否决 → 编排层走安全兜底话术。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ...llm.llm_client import AsyncOpenAIClient, extract_text
from ..plan_and_solve import _safe_json_loads
from .trace import MoETrace, extract_usage
from .history_context import build_fact_state_history


_CRITIC_SYS_OWNER = (
    "你是宠物健康智能体的『边界审核专家』，是医疗安全的最后守门人。"
    "当前用户是宠物主（pet_owner）。从以下维度把关：\n"
    "1) 责任边界：是否用『确诊为…』等绝对措辞替代线下检查，或劝阻就医；\n"
    "2) 事实一致性：结论是否有检索依据支撑、是否疑似幻觉；\n"
    "3) 用药安全：剂量/禁忌/物种特异性毒性（犬猫差异）是否存在危险；\n"
    "4) 数据一致性：建议之间是否相互矛盾；\n"
    "5) 合规与免责：是否需要补充免责声明与紧急情形指引。\n\n"
    "重要区分：\n"
    "- 用户只给出非特异症状、信息不足且未明确报告当前红旗时，草案若过早突出罕见严重疾病、渲染危急"
    "氛围、无条件要求立即就医，或没有提出会改变风险判断的关键追问，应 verdict=revise；constraints 要求"
    "先追问 3~6 个高信息量问题、给短时观察项和明确升级阈值。严重疾病存在于鉴别范围，本身不代表当前急症。\n"
    "- 上述信息不足的首轮若主动点名、展开尚无个体证据的罕见严重疾病，而不是描述宠物主可观察的风险"
    "变化，也应 verdict=revise；但用户明确询问鉴别诊断、已有相关证据或报告红旗时，不得要求隐藏疾病级风险。\n"
    "- 用户已明确报告当前红旗时，专家给出『高度怀疑 / 鉴别诊断排序』并强调立即就医属于正常工作流，"
    "**不得**仅因出现诊断相关表述就 block 或要求延后处置。\n"
    "- block 仅用于硬性红线：危险用药剂量、明确鼓励自行用药替代就医、"
    "或在无任何就医提示下给出绝对确诊并误导宠主。\n\n"
    "判定规则：硬性安全红线 → verdict=block；"
    "仅需补免责/降确定性/补充就医提示等可修补问题 → verdict=revise；"
    "草案安全 → verdict=pass。\n"
    "你必须只输出严格 JSON（无额外文字、无代码块），结构：\n"
    "{\n"
    '  "verdict": "pass | revise | block",\n'
    '  "issues": ["发现的问题"],\n'
    '  "constraints": ["写最终答案时必须遵守的硬性约束，如必须加入的免责/就医提示/用药警示"],\n'
    '  "reason": "简要中文说明"\n'
    "}"
)

_CRITIC_SYS_VET = (
    "你是宠物健康智能体的『边界审核专家』，是医疗安全的最后守门人。"
    "当前用户是执业兽医（veterinarian），终答由 AI 助手给出专业临床参考，不是宠主教育，也不是『同事』口吻。\n"
    "从以下维度把关：\n"
    "1) 责任边界：是否把鉴别诊断写成绝对确诊且无任何不确定性表述；\n"
    "2) 事实一致性：结论是否有检索依据支撑、是否疑似幻觉；\n"
    "3) 用药安全：剂量/禁忌/物种特异性毒性是否存在危险（可要求补物种毒性与监测边界）；\n"
    "4) 数据一致性：建议之间是否相互矛盾；\n"
    "5) 合规：可要求降低过度确定性；**不要**要求补充『立即就医』『线下就诊』"
    "『勿自行给人药』『请联系执业兽医』等宠主话术。\n\n"
    "重要区分：\n"
    "- 用户要求『做出诊断/鉴别诊断/病例整理』时，『高度怀疑 / 鉴别排序 / 优先排除…』属于正常临床工作流，"
    "**不得**仅因出现诊断相关表述就 block。\n"
    "- block 仅用于硬性红线：危险用药剂量、鼓励在无评估下盲目处方致命剂量等。\n\n"
    "判定规则：硬性安全红线 → verdict=block；"
    "仅需降确定性/补用药监测边界等可修补问题 → verdict=revise；"
    "草案安全 → verdict=pass。\n"
    "你必须只输出严格 JSON（无额外文字、无代码块），结构：\n"
    "{\n"
    '  "verdict": "pass | revise | block",\n'
    '  "issues": ["发现的问题"],\n'
    '  "constraints": ["写最终答案时必须遵守的硬性约束（AI 助手面向兽医用户，勿写宠主就医口号、勿自称同事）"],\n'
    '  "reason": "简要中文说明"\n'
    "}"
)

_OWNER_FALLBACK_CONSTRAINT = (
    "区分用户已报告的当前红旗与尚待确认的潜在风险；信息不足且无明确红旗时，先追问关键症状并给出"
    "短时观察要点和清晰的升级处置阈值；已有明确红旗时不得延迟紧急就医建议。"
)
_VET_FALLBACK_CONSTRAINT = "降低过度确定性，必要时补充关键用药的物种毒性与监测边界；勿写宠主就医口号。"


@dataclass
class CriticResult:
    verdict: str = "pass"
    issues: List[str] = field(default_factory=list)
    constraints: List[str] = field(default_factory=list)
    reason: str = ""

    @property
    def blocked(self) -> bool:
        return self.verdict == "block"


async def review(
    *,
    query: str,
    expert_opinions: List[Dict[str, Any]],
    emergency: bool,
    llm: AsyncOpenAIClient,
    user_role: str = "pet_owner",
    conversation_history: Optional[List[Dict[str, str]]] = None,
    expert_context_history: Optional[List[Dict[str, Any]]] = None,
    recorder: Optional[MoETrace] = None,
) -> CriticResult:
    opinions_brief = [
        {
            "expert": o.get("name_zh") or o.get("expert"),
            "weight": o.get("weight"),
            "confidence": o.get("confidence"),
            "conclusion": o.get("conclusion"),
            "risks": o.get("risks"),
        }
        for o in expert_opinions
    ]
    payload = {
        "user_question": query,
        "user_role": user_role,
        "emergency": emergency,
        "expert_opinions": opinions_brief,
    }
    history_context = build_fact_state_history(conversation_history, expert_context_history)
    if history_context:
        payload["history_context"] = history_context
    user_payload = json.dumps(payload, ensure_ascii=False)
    critic_sys = _CRITIC_SYS_VET if user_role == "veterinarian" else _CRITIC_SYS_OWNER
    messages = [
        {"role": "system", "content": critic_sys},
        {"role": "user", "content": user_payload},
    ]

    t0 = time.perf_counter()
    try:
        resp = await llm.chat(
            messages=messages,
            temperature=0.1,
            max_tokens=400,
            response_format={"type": "json_object"},
        )
        latency = (time.perf_counter() - t0) * 1000.0
        text = extract_text(resp)
        if recorder is not None:
            recorder.record_llm(
                stage="critic",
                model=getattr(llm, "model", ""),
                messages=messages,
                output=text,
                latency_ms=latency,
                usage=extract_usage(resp),
            )
        obj, parse_error = _safe_json_loads(text)
        fallback = _VET_FALLBACK_CONSTRAINT if user_role == "veterinarian" else _OWNER_FALLBACK_CONSTRAINT
        result = CriticResult(
            verdict="revise",
            issues=[f"审核输出无法解析：{parse_error or 'missing JSON object'}"],
            constraints=[fallback],
            reason="critic 输出解析失败的安全回退",
        )
        if isinstance(obj, dict):
            verdict = str(obj.get("verdict") or "revise").strip().lower()
            if verdict not in ("pass", "revise", "block"):
                verdict = "revise"
            result = CriticResult(verdict=verdict)
            iss = obj.get("issues")
            result.issues = [str(x) for x in iss] if isinstance(iss, list) else ([str(iss)] if iss else [])
            cons = obj.get("constraints")
            result.constraints = [str(x) for x in cons] if isinstance(cons, list) else ([str(cons)] if cons else [])
            result.reason = str(obj.get("reason") or "")
            if verdict == "revise" and not result.constraints:
                result.constraints = [fallback]
    except Exception as exc:  # noqa: BLE001
        latency = (time.perf_counter() - t0) * 1000.0
        if recorder is not None:
            recorder.record_llm(
                stage="critic",
                model=getattr(llm, "model", ""),
                messages=messages,
                output=f"[error] {exc}",
                latency_ms=latency,
                meta={"error": str(exc)},
            )
        fallback = _VET_FALLBACK_CONSTRAINT if user_role == "veterinarian" else _OWNER_FALLBACK_CONSTRAINT
        result = CriticResult(
            verdict="revise",
            issues=[f"审核 LLM 失败：{exc}"],
            constraints=[fallback],
            reason="critic 调用失败的安全回退",
        )

    if recorder is not None:
        recorder.critic_result = {
            "verdict": result.verdict,
            "issues": result.issues,
            "constraints": result.constraints,
            "reason": result.reason,
        }
    return result
