"""Real-API D1-D8 regression for the unified MoE task policy.

The suite deliberately mixes neighbouring intents and evidence needs.  It
checks primary intent, output variant, expert routing, required retrieval,
tool completion, RAG hit quality, and the final D1-D8 output contract.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


_HERE = Path(__file__).resolve().parent
_AGENT_API = _HERE.parents[1]
_ROOT = _HERE.parents[2]
for _path in (str(_AGENT_API), str(_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)


def _load_dotenv() -> None:
    """从项目 .env 读取环境变量（不覆盖已有值）。"""
    path = _ROOT / ".env"
    if not path.exists():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


_load_dotenv()
os.environ.setdefault("HTTPX_TRUST_ENV", "0")

from agent_api.app.integrations.llm.client import get_shared_async_client  # noqa: E402
from agent_api.app.prompts.intent_contracts import intent_required_sections  # noqa: E402
from agent_api.app.services.moe import MoEOrchestrator, MoETrace, OrchestratorConfig  # noqa: E402
from agent_api.app.services.moe.task_policy import decide_task_policy  # noqa: E402
from agent_api.app.tools.tool_registry import get_registry  # noqa: E402
from agent_api.app.tools.builtin import register_builtin_tools, register_debug_tools  # noqa: E402
from agent_api.app.tools.tools_mcp import register_mcp_tools  # noqa: E402


RAG = "rag.search"
WEB = "mcp.web_search.web_search"


@dataclass(frozen=True)
class Case:
    case_id: str
    intent: str
    title: str
    question: str
    variant: str = "default"
    history: Tuple[Dict[str, str], ...] = ()
    required_capabilities: Tuple[str, ...] = ()
    required_tools: Tuple[str, ...] = ()
    expected_owner: str = ""
    no_retrieval: bool = False
    preserve: Tuple[str, ...] = ()


def _case(
    case_id: str,
    intent: str,
    title: str,
    question: str,
    *,
    variant: str = "default",
    history: Sequence[Dict[str, str]] = (),
    capabilities: Sequence[str] = (),
    tools: Sequence[str] = (),
    owner: str = "",
    no_retrieval: bool = False,
    preserve: Sequence[str] = (),
) -> Case:
    """构造一条评测用例。"""
    return Case(
        case_id=case_id,
        intent=intent,
        title=title,
        question=question,
        variant=variant,
        history=tuple(history),
        required_capabilities=tuple(capabilities),
        required_tools=tuple(tools),
        expected_owner=owner,
        no_retrieval=no_retrieval,
        preserve=tuple(preserve),
    )


CASES: Tuple[Case, ...] = (
    # D1: diagnosis/drug vocabulary must not override the requested record format.
    _case(
        "d1_mix_soap", "D1", "用药材料转 SOAP，不做外部核验",
        "仅把以下材料整理成SOAP，不诊断、不核验外部资料：4岁拉布拉多，昨晚呕吐3次；主人自行给过一片人用布洛芬，剂量不详；精神下降。体温、腹部触诊、血检和影像均未提供。",
        variant="soap", no_retrieval=True, preserve=("4岁", "3次", "布洛芬"),
    ),
    _case(
        "d1_mix_problem_rag", "D1", "Problem List 为主并核对本地药物风险",
        "把以下问诊整理成Problem List病历草稿，并检索本地兽医资料核对布洛芬风险后写入待确认项；主交付物仍是病历，不要展开完整治疗方案：8 kg犬误服半片布洛芬，2小时后呕吐一次，当前精神尚可。",
        variant="problem_list", capabilities=("medication_reference",), tools=(RAG,), owner="pharmacy", preserve=("8 kg", "2小时"),
    ),
    _case(
        "d1_mix_emr_web", "D1", "EMR 为主并联网核对现行疫苗版本",
        "将幼犬首诊材料写成EMR草稿；同时联网核对2026年仍适用的犬核心疫苗指南版本，只在待确认项中记录核验结论和来源：12周龄幼犬，已接种一针联苗，疫苗品牌和日期不详，今日精神食欲正常。",
        variant="emr", capabilities=("current_web",), tools=(WEB,), owner="clinical", preserve=("12周龄",),
    ),
    # D2: clinical reasoning remains primary despite examination/emergency/source clauses.
    _case(
        "d2_mix_basic", "D2", "病例鉴别为主，不调用外部资料",
        "仅基于现有材料完成初步临床问题分析，不调用外部资料：6岁犬HCT 18%，网织红细胞升高，球形红细胞2+，总胆红素升高，超声未见出血。请排序鉴别并列支持、反对证据及信息缺口。",
        no_retrieval=True, preserve=("18%", "球形红细胞2+"),
    ),
    _case(
        "d2_mix_rag", "D2", "异宠鉴别并核对本地资料",
        "3岁兔18小时不进食、粪便减少、腹部轻度膨大。请做鉴别诊断和风险排序，并检索本地兽医知识库核对胃肠停滞与机械性梗阻的区分证据；检查清单不是主要交付物。",
        capabilities=("local_knowledge",), tools=(RAG,), owner="clinical", preserve=("18小时",),
    ),
    _case(
        "d2_mix_web", "D2", "人畜共患鉴别并联网核对现行指南",
        "犬出现发热、黄疸、急性肾损伤且近期接触积水。请以鉴别诊断和风险判断为主，并联网核对2025-2026年仍适用的钩端螺旋体人畜共患病指南；不要把检查规划写成主任务。",
        capabilities=("current_web",), tools=(WEB,), owner="clinical",
    ),
    # D3: future work-up is primary even when values or treatments are mentioned.
    _case(
        "d3_mix_basic", "D3", "围术前检查规划，不调用外部资料",
        "仅基于材料制定检查路径，不调用外部资料：犬鼻腔占位拟活检，血小板85×10^9/L且PT轻度延长。按必要、可选、暂缓检查分层，说明目的、替代方案和允许或推迟操作的触发条件。",
        no_retrieval=True, preserve=("85", "PT"),
    ),
    _case(
        "d3_mix_rag", "D3", "麻醉前检查路径并核对本地资料",
        "12岁CKD猫拟行牙科麻醉，近期食欲下降，本次血压、电解质和HCT未知。请制定麻醉前必要/可选/暂缓检查路径，并检索本地兽医资料核对CKD麻醉风险评估要点；不要直接给麻醉用药方案。",
        capabilities=("local_knowledge",), tools=(RAG,), owner="clinical",
    ),
    _case(
        "d3_mix_web", "D3", "跨境检疫检查路径并联网核对",
        "一只犬计划2026年从中国前往欧盟，主人询问出发前检查和证明办理顺序。请以检查与材料规划为主，分必要、可选、暂缓项目，并联网核对当前狂犬抗体检测及入境时间要求。",
        capabilities=("current_web",), tools=(WEB,), owner="clinical", preserve=("2026年",),
    ),
    # D4: existing measurements/reports remain primary despite diagnostic implications.
    _case(
        "d4_mix_basic", "D4", "高钾生化报告解读，不调用外部资料",
        "仅依据报告进行解读，不调用外部资料：猫肌酐620 μmol/L、尿素氮38 mmol/L、钾6.8 mmol/L、碳酸氢根14 mmol/L。请区分报告事实、机制和患者结论，并标出危急项。",
        no_retrieval=True, preserve=("620", "6.8", "14"),
    ),
    _case(
        "d4_mix_rag", "D4", "尿检解读并核对本地资料",
        "解读犬尿检：USG 1.010、蛋白2+、潜血3+、RBC 30-50/HPF、WBC 10-15/HPF、杆菌2+；并检索本地兽医资料核对等渗尿和沉渣组合的解释。主要交付物是报告解读，不是检查清单。",
        capabilities=("local_knowledge",), tools=(RAG,), owner="clinical", preserve=("1.010", "30-50"),
    ),
    _case(
        "d4_mix_web", "D4", "PCR 报告解读并联网核对",
        "解读猫呼吸道PCR：FHV-1阳性（Ct 34），FCV阴性，样本在多西环素治疗5天后采集；并联网核对当前指南对低载量阳性与采样时机的解释。主要任务仍是报告解读。",
        capabilities=("current_web",), tools=(WEB,), owner="clinical", preserve=("Ct 34", "5天"),
    ),
    # D5: patient-specific treatment is distinct from generic D6 knowledge.
    _case(
        "d5_mix_basic", "D5", "肥胖犬治疗路径，不调用外部资料",
        "仅基于病例给出初步治疗与监测框架，不调用外部资料：7岁绝育犬BCS 8/9，轻度骨关节疼痛，肝肾指标正常。请制定减重、运动和疼痛管理的分层目标、监测与复查，但不要给未经核验的具体药物剂量。",
        no_retrieval=True, preserve=("BCS 8/9",),
    ),
    _case(
        "d5_mix_rag", "D5", "NSAID 与激素用药安全核验",
        "犬正在使用泼尼松治疗免疫介导性疾病，现因关节痛拟加卡洛芬。请为该患者制定安全治疗路径，并检索本地药物资料核对相互作用、停换药原则、替代方案和监测。",
        capabilities=("medication_reference",), tools=(RAG,), owner="pharmacy", preserve=("泼尼松", "卡洛芬"),
    ),
    _case(
        "d5_mix_web", "D5", "糖尿病治疗并联网核对现行指南",
        "10岁猫已确认糖尿病、无酮症。请制定患者治疗、低血糖监测和复查计划，并联网核对2025-2026年仍适用的猫糖尿病管理指南；不要把回答写成一般知识卡。",
        capabilities=("current_web",), tools=(WEB,), owner="clinical", preserve=("10岁", "无酮症"),
    ),
    # D6: generic knowledge cards require traceable evidence.
    _case(
        "d6_mix_concept_rag", "D6", "SDMA 概念知识卡",
        "制作面向兽医的SDMA概念知识卡，并检索本地兽医资料核对定义、适用对象、前分析影响、解释限制、来源与版本；不要针对某个具体患者下诊断。",
        variant="concept", capabilities=("local_knowledge",), tools=(RAG,), owner="clinical",
    ),
    _case(
        "d6_mix_drug_rag", "D6", "多西环素剂量知识卡",
        "制作犬猫多西环素剂量知识卡，检索本地药物资料核对适应证、剂量依据、途径、频次、疗程、食管损伤预防、禁忌和监测；这是通用知识查询，不针对具体患者。",
        variant="dose", capabilities=("medication_reference",), tools=(RAG,), owner="pharmacy",
    ),
    _case(
        "d6_mix_guideline_web", "D6", "现行犬疫苗指南知识卡",
        "制作犬核心疫苗指南知识卡，并联网核对2025-2026年仍适用的指南组织、版本/发布日期、基础免疫、加强原则、例外和来源冲突；不要制定某只犬的接种计划。",
        variant="guideline", capabilities=("current_web",), tools=(WEB,), owner="clinical",
    ),
    # D7: history comparison remains primary while evidence can support plan changes.
    _case(
        "d7_mix_basic", "D7", "CKD 复诊状态更新，不调用外部资料",
        "仅基于本轮与既往记录更新病例，不调用外部资料：本次体重4.1 kg、肌酐310 μmol/L、血压145 mmHg。请说明新增/修正/撤回、风险变化和计划变更。",
        history=(
            {"role": "user", "content": "12岁CKD猫，上次体重4.4 kg、肌酐240 μmol/L、血压170 mmHg。"},
            {"role": "assistant", "content": "建议复查体重、肌酐、磷和血压。"},
        ), no_retrieval=True, preserve=("4.1", "310", "145"),
    ),
    _case(
        "d7_mix_rag", "D7", "癫痫复诊与药物核验",
        "复诊新增：过去7天发作3次，其中一次4分钟，未发现漏药。请更新病例状态和计划，并检索本地药物资料核对增加或调整抗癫痫药前必须评估的相互作用与监测；主任务是随访更新。",
        history=(
            {"role": "user", "content": "5岁犬已确诊特发性癫痫，过去半年约每月发作1次，正在按处方治疗。"},
            {"role": "assistant", "content": "维持用药并记录频率、持续时间和不良反应。"},
        ), capabilities=("medication_reference",), tools=(RAG,), owner="pharmacy", preserve=("7天", "4分钟"),
    ),
    _case(
        "d7_mix_web", "D7", "MMVD 随访并核对现行共识",
        "复诊新增：静息呼吸由24升至38-42次/分，并出现夜间咳嗽。请更新风险和计划变化，同时联网核对2025-2026年仍适用的MMVD随访共识；不要改写成一般指南知识卡。",
        history=(
            {"role": "user", "content": "10岁骑士犬MMVD B2期，上次静息呼吸24次/分，无咳嗽。"},
            {"role": "assistant", "content": "继续居家监测静息呼吸和运动耐量。"},
        ), capabilities=("current_web",), tools=(WEB,), owner="clinical", preserve=("38-42", "夜间咳嗽"),
    ),
    # D8: immediate safety does not always justify delaying for retrieval.
    _case(
        "d8_mix_basic", "D8", "尿闭急症安全处置，不等待检索",
        "公猫24小时几乎无尿，反复用力，现虚弱、呕吐、心率缓慢。不要调用外部资料或等待检索，先给安全等级、立即行动与时间要求、必要追问和能力边界。",
        no_retrieval=True, preserve=("24小时",),
    ),
    _case(
        "d8_mix_rag", "D8", "布洛芬危险要求拦截并核验",
        "宠主坚持给8 kg犬使用家用布洛芬并要求直接计算一片分几份。请安全拦截该操作，并检索本地药物资料核对物种毒性和处置边界；不得提供家庭自行给药剂量。",
        capabilities=("medication_reference",), tools=(RAG,), owner="pharmacy", preserve=("8 kg", "布洛芬"),
    ),
    _case(
        "d8_mix_web", "D8", "来源不明召回信息的安全降级",
        "本院收到一张来源不明的截图，声称某批犬用心脏药今天被紧急召回，用户要求立刻决定继续还是停药。请先给证据不足时的安全边界，并联网核实当前召回公告；不得把未核实截图当成事实。",
        capabilities=("current_web",), tools=(WEB,), owner="clinical",
    ),
)


@dataclass
class Result:
    case_id: str
    expected_intent: str
    actual_intent: str = ""
    expected_variant: str = "default"
    actual_variant: str = ""
    secondary_intents: List[str] = field(default_factory=list)
    selected_experts: List[str] = field(default_factory=list)
    evidence_tasks: List[Dict[str, Any]] = field(default_factory=list)
    required_tools: List[str] = field(default_factory=list)
    attempted_tools: List[str] = field(default_factory=list)
    successful_tools: List[str] = field(default_factory=list)
    rag_calls: List[Dict[str, Any]] = field(default_factory=list)
    web_calls: List[Dict[str, Any]] = field(default_factory=list)
    answer: str = ""
    total_tokens: int = 0
    latency_s: float = 0.0
    issues: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        """根据期望与实际结果判断本条是否通过。"""
        return not self.issues


_CJK = re.compile(r"[\u3400-\u9fff\u3040-\u30ff\uac00-\ud7af]")


def _validate_policy(case: Case, result: Result) -> None:
    """校验统一策略输出是否符合约定。"""
    if result.actual_intent != case.intent:
        result.issues.append(f"主意图错误：期望 {case.intent}，实际 {result.actual_intent or '缺失'}")
    if result.actual_variant != case.variant:
        result.issues.append(f"输出变体错误：期望 {case.variant}，实际 {result.actual_variant or '缺失'}")
    required_tasks = [task for task in result.evidence_tasks if task.get("requirement") == "required"]
    capabilities = {str(task.get("capability") or "") for task in required_tasks}
    missing_capabilities = set(case.required_capabilities) - capabilities
    if missing_capabilities:
        result.issues.append(f"缺少必需证据能力：{sorted(missing_capabilities)}")
    if case.expected_owner:
        for capability in case.required_capabilities:
            owners = {
                str(task.get("owner") or "") for task in required_tasks
                if task.get("capability") == capability
            }
            if case.expected_owner not in owners:
                result.issues.append(
                    f"证据 owner 错误：{capability} 期望 {case.expected_owner}，实际 {sorted(owners)}"
                )
    if case.no_retrieval and result.evidence_tasks:
        result.issues.append("明确不检索的病例仍创建了证据任务")


def _validate_output(case: Case, result: Result) -> None:
    """校验最终回答是否符合契约。"""
    answer = result.answer.strip()
    if not answer:
        result.issues.append("终答为空")
        return
    if any(line.lstrip().startswith("#") for line in answer.splitlines()):
        result.issues.append("使用了 # Markdown 标题")
    positions: List[int] = []
    for section in intent_required_sections(case.intent, case.variant):
        match = re.search(rf"\*\*\s*{re.escape(section)}\s*\*\*", answer)
        if not match:
            result.issues.append(f"缺少加粗契约分节：{section}")
        else:
            positions.append(match.start())
    if positions != sorted(positions):
        result.issues.append("契约分节顺序错误")
    normalized = "".join(answer.replace("–", "-").replace("—", "-").split())
    normalized = normalized.replace("（", "").replace("）", "").replace("增多", "")
    for item in case.preserve:
        expected = "".join(item.replace("–", "-").replace("—", "-").split())
        expected = expected.replace("（", "").replace("）", "").replace("增多", "")
        if expected not in normalized:
            result.issues.append(f"未保留输入事实：{item}")
    semantic_checks = {
        "D1": ("未提供", "待确认"),
        "D2": ("反对证据", "不支持"),
        "D3": ("目的",),
        "D4": ("危急项",),
        "D5": ("监测", "复查"),
        "D6": ("来源", "[R", "[W"),
        "D7": ("新增", "修正", "撤回"),
        "D8": ("边界", "不能", "不得", "安全"),
    }
    if not any(token in answer for token in semantic_checks[case.intent]):
        result.issues.append(f"未满足 {case.intent} 关键语义要求")


def _validate_tools(case: Case, result: Result) -> None:
    """校验工具调用是否符合策略。"""
    missing_required = set(case.required_tools) - set(result.required_tools)
    if missing_required:
        result.issues.append(f"专家未接收必需工具：{sorted(missing_required)}")
    missing_attempted = set(case.required_tools) - set(result.attempted_tools)
    if missing_attempted:
        result.issues.append(f"必需工具未调用：{sorted(missing_attempted)}")
    missing_success = set(case.required_tools) - set(result.successful_tools)
    if missing_success:
        result.issues.append(f"必需工具未成功：{sorted(missing_success)}")
    if case.no_retrieval and result.attempted_tools:
        result.issues.append(f"明确不检索但调用了工具：{sorted(set(result.attempted_tools))}")
    for tool in case.required_tools:
        owners = set()
        for task in result.evidence_tasks:
            capability = str(task.get("capability") or "")
            mapped = RAG if capability in {"local_knowledge", "medication_reference"} else WEB
            if mapped == tool and task.get("requirement") == "required":
                owners.add(str(task.get("owner") or ""))
        if len(owners) > 1:
            result.issues.append(f"{tool} 被分配给多个证据任务 owner，可能重复检索")
    if RAG in case.required_tools:
        if not result.rag_calls:
            result.issues.append("没有记录 RAG 调用")
        for call in result.rag_calls:
            if _CJK.search(str(call.get("query") or "")):
                result.issues.append("RAG query 未使用纯英文")
            if int(call.get("hits_count") or 0) <= 0:
                result.issues.append("RAG 没有命中")
            if float(call.get("best_score") or 0.0) < 0.25:
                result.issues.append(f"RAG 最高分过低：{call.get('best_score')}")
    if WEB in case.required_tools and not any(bool(call.get("ok")) for call in result.web_calls):
        result.issues.append("Web Search 未成功返回")


async def _run_case(case: Case, *, full: bool, max_tokens: int) -> Result:
    """执行单条评测用例并记录结果。"""
    result = Result(
        case_id=case.case_id,
        expected_intent=case.intent,
        expected_variant=case.variant,
    )
    started = time.perf_counter()
    try:
        if not full:
            decision = await decide_task_policy(
                query=case.question,
                user_role="veterinarian",
                llm=get_shared_async_client(),
                conversation_history=list(case.history),
            )
            result.actual_intent = decision.primary_intent
            result.actual_variant = decision.output_variant
            result.secondary_intents = list(decision.secondary_intents)
            result.evidence_tasks = [task.as_dict() for task in decision.evidence_tasks]
            result.selected_experts = decision.as_router_decision().selected_experts
        else:
            trace = MoETrace(question=case.question, user_role="veterinarian")
            orchestrator = MoEOrchestrator(
                registry=get_registry(),
                config=OrchestratorConfig(
                    user_role="veterinarian",
                    temperature=0.1,
                    max_tokens=max_tokens,
                    rag_top_k=8,
                ),
            )
            answer, _ = await orchestrator.run(
                query=case.question,
                conversation_history=list(case.history),
                recorder=trace,
            )
            policy = trace.task_policy_decision or {}
            result.actual_intent = str(policy.get("primary_intent") or "")
            result.actual_variant = str(policy.get("output_variant") or "")
            result.secondary_intents = list(policy.get("secondary_intents") or [])
            result.evidence_tasks = list(policy.get("evidence_tasks") or [])
            result.selected_experts = list((trace.router_decision or {}).get("selected_experts") or [])
            result.answer = answer
            result.total_tokens = trace.total_tokens()
            result.rag_calls = [asdict(call) for call in trace.rag_calls]
            result.web_calls = [
                asdict(call) for call in trace.tool_calls if call.tool_name == WEB
            ]
            for opinion in trace.expert_opinions:
                result.required_tools.extend(opinion.get("required_tools") or [])
                result.attempted_tools.extend(opinion.get("attempted_tools") or [])
                result.successful_tools.extend(opinion.get("successful_tools") or [])
                if opinion.get("pending_tools"):
                    result.issues.append(
                        f"{opinion.get('expert')} 尚有 pending_tools：{opinion.get('pending_tools')}"
                    )
                if opinion.get("unavailable_required_tools"):
                    result.issues.append(
                        f"{opinion.get('expert')} 必需工具不可用：{opinion.get('unavailable_required_tools')}"
                    )
    except Exception as exc:  # noqa: BLE001
        result.issues.append(f"执行失败：{type(exc).__name__}: {exc}")
    result.latency_s = round(time.perf_counter() - started, 2)
    _validate_policy(case, result)
    if full and result.answer:
        _validate_tools(case, result)
        _validate_output(case, result)
    return result


def _report(cases: Sequence[Case], results: Sequence[Result], mode: str) -> str:
    """写出或打印本次评测报告。"""
    passed = sum(item.passed for item in results)
    total_tokens = sum(item.total_tokens for item in results)
    lines = [
        "# PetMind MoE 统一策略 D1-D8 真实 API 回归报告",
        "",
        f"- 时间：{datetime.now().isoformat(timespec='seconds')}",
        f"- 模式：{mode}",
        f"- 用例：{len(results)}（D1-D8 每类 3 条混合边界问题）",
        f"- 通过：{passed}/{len(results)}",
        f"- 总 Token：{total_tokens}",
        "- 判定范围：主意图、变体、专家路由、必需证据任务、工具完成、RAG 命中质量、最终输出契约。",
        "",
        "| 用例 | 期望/实际 | 专家 | 必需/成功工具 | 检索 | 格式与结果 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    by_id = {case.case_id: case for case in cases}
    for result in results:
        retrieval = ", ".join(
            f"RAG {call.get('hits_count', 0)}@{call.get('best_score', 0)}"
            for call in result.rag_calls
        ) or ("Web OK" if any(call.get("ok") for call in result.web_calls) else "无")
        issues = "；".join(result.issues).replace("|", "\\|") or "通过"
        lines.append(
            f"| {result.case_id} | {result.expected_intent}/{result.actual_intent or '-'} "
            f"({result.expected_variant}/{result.actual_variant or '-'}) | "
            f"{', '.join(result.selected_experts) or '-'} | "
            f"{', '.join(sorted(set(result.required_tools))) or '-'} / "
            f"{', '.join(sorted(set(result.successful_tools))) or '-'} | {retrieval} | "
            f"{'PASS' if result.passed else 'FAIL'}：{issues} |"
        )
    lines.extend(["", "## 分意图统计", ""])
    for intent in (f"D{i}" for i in range(1, 9)):
        subset = [item for item in results if item.expected_intent == intent]
        lines.append(f"- {intent}：{sum(item.passed for item in subset)}/{len(subset)}")
    lines.extend(["", "## 失败明细与最终回答", ""])
    for result in results:
        case = by_id[result.case_id]
        lines.extend([
            f"### {case.case_id} · {case.title}",
            "",
            f"- 问题：{case.question}",
            f"- 判定：{'PASS' if result.passed else 'FAIL'}",
            f"- 问题项：{'；'.join(result.issues) or '无'}",
            f"- Evidence Tasks：`{json.dumps(result.evidence_tasks, ensure_ascii=False)}`",
            "",
        ])
        if mode == "full":
            lines.extend([result.answer or "_无终答_", ""])
    return "\n".join(lines)


async def _main(args: argparse.Namespace) -> int:
    """脚本内部主流程。"""
    selected = [case for case in CASES if not args.case or case.case_id in set(args.case)]
    if len(CASES) != 24:
        raise RuntimeError(f"suite must contain exactly 24 cases, got {len(CASES)}")
    if args.mode == "full":
        registry = get_registry()
        register_builtin_tools(registry)
        register_debug_tools(registry)
        register_mcp_tools(registry)
    semaphore = asyncio.Semaphore(max(1, args.concurrency))

    async def guarded(case: Case) -> Result:
        """带守卫逻辑的对照实现。"""
        async with semaphore:
            return await _run_case(case, full=args.mode == "full", max_tokens=args.max_tokens)

    tasks = [asyncio.create_task(guarded(case)) for case in selected]
    results: List[Result] = []
    for index, future in enumerate(asyncio.as_completed(tasks), start=1):
        result = await future
        results.append(result)
        print(json.dumps({
            "progress": f"{index}/{len(tasks)}",
            "case": result.case_id,
            "intent": result.actual_intent,
            "experts": result.selected_experts,
            "tools": sorted(set(result.successful_tools)),
            "passed": result.passed,
            "issues": result.issues,
            "latency_s": result.latency_s,
        }, ensure_ascii=False), flush=True)
    order = {case.case_id: index for index, case in enumerate(selected)}
    results.sort(key=lambda item: order[item.case_id])
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "results.json").write_text(
        json.dumps([asdict(item) | {"passed": item.passed} for item in results], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    report = out_dir / "SUMMARY.md"
    report.write_text(_report(selected, results, args.mode), encoding="utf-8")
    passed = sum(item.passed for item in results)
    print(json.dumps({
        "passed": passed,
        "total": len(results),
        "tokens": sum(item.total_tokens for item in results),
        "report": str(report.resolve()),
    }, ensure_ascii=False), flush=True)
    return 0 if passed == len(results) else 1


def main() -> int:
    """脚本入口，解析参数并执行主流程。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("policy", "full"), default="policy")
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--concurrency", type=int, default=2)
    parser.add_argument("--max-tokens", type=int, default=2600)
    parser.add_argument(
        "--out-dir",
        default=f"agent_api/tests/moe/reports/unified_policy_regression_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    return asyncio.run(_main(parser.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
