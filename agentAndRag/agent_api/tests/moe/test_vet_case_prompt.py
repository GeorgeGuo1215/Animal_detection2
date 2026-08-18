"""离线单测：医生端意图契约与 Aggregator 安全答风。

Run: pytest tests/moe/test_vet_case_prompt.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.prompts.solve import (
    answer_char_budget,
    build_solve_prompt,
)
from app.services.moe.orchestrator import MoEOrchestrator, OrchestratorConfig
from app.services.moe.task_policy import IntentDecision, parse_task_policy
from app.prompts.moe_task_policy import TASK_POLICY_SYSTEM_PROMPT
from app.prompts.intent_contracts import (
    INTENT_SPECS,
    build_intent_aggregator_injection,
    intent_required_sections,
)
from app.services.moe.router import RouterDecision
from app.services.moe.critic import CriticResult, _CRITIC_SYS_OWNER
from app.services.moe.experts import _AUDIENCE_OWNER


_FLUTD_NARRATIVE = (
    "5 岁的英短，公猫，已经绝育。今天从早上开始就一直往猫砂盆跑，差不多十几分钟去一次，"
    "每次蹲很久，但是宠主看不出来到底有没有尿出来。猫砂盆里好像只有一点点尿团，比平时少很多。"
    "它今天不太爱吃东西，平时早上会主动来要罐头，今天只舔了几口。精神也差一些，老是趴着，还会舔下面。"
    "没有明显呕吐，但是刚才好像干呕了一下。昨天晚上还挺正常的。最近没有换粮，喝水感觉和平时差不多。"
    "宠主家里最近来了客人，它有点紧张，躲了两天。疫苗应该是去年打的，驱虫不太记得。"
    "之前有过一次尿血，可能是膀胱炎，吃药后好了。今天还没有去医院，也没有吃药。"
)

_FLUTD_DIAGNOSIS = _FLUTD_NARRATIVE + "\n请对以上病例做出诊断。"
_FLUTD_ORGANIZE = _FLUTD_NARRATIVE + "\n请对以上病例做出整理，输出统一的病例格式。"

_BULLDOG_EXERCISE = (
    "3岁法斗（法国斗牛犬）公犬，已绝育，体重12kg。"
    "主人想每天带它剧烈跑步或追球1小时，最近热天遛弯就张口喘、不愿走。"
    "从兽医角度评估运动建议与风险边界，并说明与普通中型犬的差异。"
)


def test_teacher_rubric_has_exactly_eight_maintainable_intents():
    assert tuple(INTENT_SPECS) == tuple(f"D{i}" for i in range(1, 9))
    assert all(spec.output_contract and spec.routing_guidance for spec in INTENT_SPECS.values())


def test_task_policy_parser_selects_intent_without_regex_gate():
    decision = parse_task_policy(
        '{"primary_intent":"D1","secondary_intents":[],"confidence":0.97,'
        '"output_variant":"soap","scores":{"clinical":8},'
        '"emergency":{"value":false},"evidence_tasks":[],"reason":"要求SOAP"}'
    )
    assert decision.primary_intent == "D1"
    assert decision.output_variant == "soap"
    assert decision.fallback is False


def test_task_policy_parser_falls_back_safely_for_invalid_label():
    decision = parse_task_policy(
        '{"primary_intent":"case_regex","confidence":1,"output_variant":"default","reason":"bad"}'
    )
    assert decision.primary_intent == "D2"
    assert decision.fallback is True


def test_d1_soap_contract_is_registry_driven():
    prompt = build_intent_aggregator_injection("D1", 0.9, "soap")
    assert intent_required_sections("D1", "soap") == (
        "S（主观）", "O（客观）", "A（评估）", "P（计划）", "待确认项"
    )
    assert "S（主观）" in prompt
    assert "缺失项写“未提供/待确认”" in prompt


def test_vet_base_prompt_forbids_owner_tone():
    prompt = build_solve_prompt(user_role="veterinarian", query="犬急性胰腺炎鉴别要点")
    assert "禁止宠主话术" in prompt
    assert "AI 临床助手" in prompt or "AI 助手" in prompt
    assert "请立即就医" in prompt  # listed as forbidden phrase
    assert "布洛芬" in prompt
    assert "不要自称『同事』" in prompt or "不要自称同事" in prompt or "终答不要自称『同事』" in prompt


def test_owner_prompt_still_suggests_vet():
    prompt = build_solve_prompt(user_role="pet_owner", query="狗吐了怎么办")
    assert "建议咨询兽医" in prompt
    assert "禁止宠主话术" not in prompt


def test_owner_prompt_uses_conversational_triage_without_delaying_red_flags():
    prompt = build_solve_prompt(user_role="pet_owner", query="猫突然不吃饭要观察哪些风险？")
    assert "对话式分诊" in prompt
    assert "3~6 个" in prompt
    assert "短时观察" in prompt
    assert "不要主动点名、展开尚无个体证据的罕见" in prompt
    assert "仍应正常说明疾病级风险" in prompt
    assert "不能把『某症状可能由严重疾病引起』等同于患者已经处于急症" in prompt
    assert "不得为了追问而延误" in prompt


def test_owner_expert_and_critic_prompts_enforce_clarification_boundary():
    assert "3~6 个" in _AUDIENCE_OWNER
    assert "无条件要求立即就医" in _AUDIENCE_OWNER
    assert "明确当前红旗" in _AUDIENCE_OWNER
    assert "verdict=revise" in _CRITIC_SYS_OWNER
    assert "严重疾病存在于鉴别范围，本身不代表当前急症" in _CRITIC_SYS_OWNER
    assert "描述宠物主可观察的风险" in _CRITIC_SYS_OWNER
    assert "要求延后处置" in _CRITIC_SYS_OWNER


def test_unified_policy_does_not_equate_serious_differential_with_emergency():
    assert "只依据用户已报告的当前表现或已核实的外部结果" in TASK_POLICY_SYSTEM_PROMPT
    assert "鉴别诊断中提到严重疾病" in TASK_POLICY_SYSTEM_PROMPT
    assert "不能单独令 emergency=true" in TASK_POLICY_SYSTEM_PROMPT


def test_build_solve_prompt_exercise_no_forced_case_structure():
    prompt = build_solve_prompt(user_role="veterinarian", query=_BULLDOG_EXERCISE)
    assert "兽医病例工作流答风" not in prompt
    assert "兽医证据分层规范" in prompt


def test_build_solve_prompt_requires_exact_web_url_copy():
    prompt = build_solve_prompt(
        user_role="veterinarian",
        query=_BULLDOG_EXERCISE,
        has_web_search=True,
    )

    assert "逐字符原样复制" in prompt
    assert "禁止翻译、纠错、补全、缩短、解码后重编码" in prompt


def test_build_solve_prompt_vet_vague_keeps_evidence_layering():
    prompt = build_solve_prompt(user_role="veterinarian", query="猫尿血怎么办")
    assert "病例整理" not in prompt or "兽医病例工作流答风" not in prompt
    assert "兽医证据分层规范" in prompt


def test_build_solve_prompt_owner_no_case_structure():
    prompt = build_solve_prompt(user_role="pet_owner", query=_FLUTD_DIAGNOSIS)
    assert "兽医病例工作流答风" not in prompt


def test_aggregator_synthesis_structure_for_diagnosis():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    orch._active_intent_decision = IntentDecision(
        intent_id="D2", name="临床问题分析", confidence=0.96,
        output_variant="default", reason="鉴别诊断",
    )
    decision = RouterDecision(
        scores={"clinical": 8},
        raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0},
        selected_experts=["clinical"],
        emergency=False,
        out_of_scope=False,
        reason="test",
    )
    msgs = orch._build_synthesis_messages(
        query=_FLUTD_DIAGNOSIS,
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "疑似下尿路问题", "evidence": [], "risks": [], "confidence": 0.7,
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
    )
    sys = msgs[0]["content"]
    assert "D2` 临床问题分析" in sys
    assert "问题表示" in sys
    assert "鉴别诊断" in sys
    assert "支持证据、反对证据和缺失信息" in sys
    assert "高风险项" in sys
    assert "下一步验证" in sys
    assert "宠主就医/人药口号" in sys
    assert "retrieved_sources` 为空" in sys
    assert "绝对禁止出现参考文献/参考来源段落" in sys
    assert "覆盖本提示词中其他任何引用格式" in sys
    assert "药物名称与证据归属" in sys
    assert "不得把名称相近但实际不同的药物混淆" in sys
    assert "同一来源直接支持" in sys
    assert "常见药物和公认临床常识可以" in sys
    assert "不得附加虚假引用" in sys
    assert "原文名称、OCR 或译名确实存在歧义" in sys
    assert "输出前核对药物身份一致性" in sys
    assert "接近输出预算时停止扩展并完整收尾" in sys
    assert "禁忌联用安全约束（最高优先级）" in sys
    assert "真正消除该组合的替代路径" in sys
    assert "换用同类中所谓低风险药物" in sys
    assert "这些措施不能解除禁忌" in sys
    assert "只能用于意外暴露后的风险处置" in sys
    assert "丢弃，不得折中转述" in sys
    assert "最终输出前不可协商的禁忌复核" in sys
    assert "计划性用药时必须直接回答不可一起使用" in sys
    assert "最终答案任何位置都不得出现" in sys
    assert "禁止给出统一洗脱天数" in sys
    assert "不得无条件写『立即停用糖皮质激素』" in sys
    assert "长期糖皮质激素不能骤停" in sys
    assert sys.rfind("最终输出前不可协商的禁忌复核") > sys.rfind("多专家融合规范")
    assert '"retrieved_sources": []' in msgs[1]["content"]


def test_aggregator_vet_non_case_drops_when_to_seek_care():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    orch._active_intent_decision = IntentDecision(
        intent_id="D5", name="治疗与用药安全", confidence=0.91,
        output_variant="default", reason="运动处置建议",
    )
    decision = RouterDecision(
        scores={"clinical": 8},
        raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0},
        selected_experts=["clinical"],
        emergency=True,
        out_of_scope=False,
        reason="test",
    )
    msgs = orch._build_synthesis_messages(
        query=_BULLDOG_EXERCISE,
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "限制剧烈运动", "evidence": [], "risks": ["热射病"], "confidence": 0.8,
            "tools_used": ["mcp.web_search.web_search"],
            "tool_results": [{
                "tool_name": "mcp.web_search.web_search",
                "ok": True,
                "result": {"results": [{
                    "title": "Current guideline",
                    "url": "https://example.test/guideline",
                    "content": "Avoid strenuous exercise in heat.",
                }]},
            }],
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
    )
    sys = msgs[0]["content"]
    assert "治疗目标" in sys
    assert "禁忌与红旗" in sys
    assert "**何时必须就医**" not in sys
    assert "当前疑似急症" in sys
    assert "只有其中列出的来源可以被引用" in sys
    assert '"id": "W1"' in msgs[1]["content"]
    assert "https://example.test/guideline" in msgs[1]["content"]


def test_aggregator_owner_vague_first_turn_clarifies_even_if_router_flags_emergency():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner"))
    decision = RouterDecision(
        scores={"clinical": 8},
        raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0},
        selected_experts=["clinical"],
        emergency=True,
        out_of_scope=False,
        reason="test",
    )
    msgs = orch._build_synthesis_messages(
        query="狗吐了怎么办",
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "观察", "evidence": [], "risks": [], "confidence": 0.5,
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
    )
    sys = msgs[0]["content"]
    assert "需要补充的信息" in sys
    assert "现在可以观察什么" in sys
    assert "宠物主对话式分诊（高优先级）" in sys
    assert "这只是风险信号，不是患者急症已被确认" in sys
    assert "务必把『立即就医』放在最前" not in sys
    assert "当前对话角色" in sys
    assert '"user_role": "pet_owner"' in msgs[1]["content"]


def test_aggregator_owner_followup_advances_and_reported_red_flags_are_not_delayed():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner"))
    decision = RouterDecision(
        scores={"clinical": 10}, raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0}, selected_experts=["clinical"],
        emergency=True, out_of_scope=False, reason="reported red flags",
    )
    history = [
        {"role": "user", "content": "猫咪突然不吃饭，我要观察哪些风险？"},
        {"role": "assistant", "content": "请补充持续时间、饮水、精神及有无呕吐。"},
        {"role": "user", "content": "已超过一天，反复呕吐，精神很差。"},
    ]
    msgs = orch._build_synthesis_messages(
        query=history[-1]["content"],
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "存在明确红旗", "evidence": [], "risks": [], "confidence": 0.9,
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
        conversation_history=history,
    )
    sys = msgs[0]["content"]
    user = msgs[1]["content"]
    assert "综合这些信息给出更完整的鉴别方向" in sys
    assert "若已报告明确当前红旗，则当轮直接说明紧急程度" in sys
    assert "不能要求用户等下一轮" in sys
    assert "反复呕吐，精神很差" in user
    assert "assistant [未验证模型输出" in user


def test_aggregator_preserves_unconfirmed_assistant_hypothesis_across_turns():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    decision = RouterDecision(
        scores={"clinical": 8}, raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0}, selected_experts=["clinical"],
        emergency=False, out_of_scope=False, reason="test",
    )
    history = [
        {"role": "user", "content": "老年犬咳嗽并有心杂音，尚未做超声心动图。"},
        {"role": "assistant", "content": "MMVD是优先鉴别诊断，但目前尚未确诊。"},
        {"role": "user", "content": "沿用上一轮MMVD判断，只回答具体用药。"},
    ]
    msgs = orch._build_synthesis_messages(
        query=history[-1]["content"],
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "考虑MMVD", "evidence": [], "risks": [], "confidence": 0.7,
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
        conversation_history=history,
    )

    sys = msgs[0]["content"]
    assert "病历事实状态与跨轮次约束（最高优先级）" in sys
    assert "先前 assistant 消息" in sys
    assert "不能自行确认本患者的诊断、分期或既往病史" in sys
    assert "沿用/按照你上一轮提出的判断" in sys
    assert "不构成新增确认" in sys
    assert "可能、疑似、鉴别、建议排查、推定" in sys
    assert "若后续确诊/若检查满足" in sys
    assert "经验性治疗" in sys
    assert "不得因用户点名某种药物而反推" in sys
    assert "B2/C 期、IRIS 分期、PDH/ADH" in sys
    assert "当前未确诊/仅为疑似" in sys
    assert "生成前在内部逐项核对" in sys
    assert "无需为本规则增加固定模板或额外章节" in sys
    assert "证据与引用安全（最高优先级）" in sys
    assert "绝对禁止出现参考文献/参考来源段落" in sys
    assert "历史事实状态提示" in msgs[1]["content"]
    assert "assistant [未验证模型输出，不能作为患者事实]" in msgs[1]["content"]
    assert "MMVD是优先鉴别诊断，但目前尚未确诊。" in msgs[1]["content"]


def test_aggregator_epistemic_history_rule_also_applies_to_pet_owner():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner"))
    msgs = orch._build_synthesis_messages(
        query="那就按哮喘给它用药",
        opinions=[],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=RouterDecision(
            scores={"clinical": 8}, raw_weights={"clinical": 1.0},
            weights={"clinical": 1.0}, selected_experts=["clinical"],
            emergency=False, out_of_scope=False, reason="test",
        ),
        conversation_history=[
            {"role": "assistant", "content": "猫哮喘只是鉴别诊断之一。"},
        ],
    )

    assert "先前 assistant 消息" in msgs[0]["content"]
    assert "只回答用药" not in msgs[0]["content"]
    assert "assistant [未验证模型输出，不能作为患者事实]" in msgs[1]["content"]
    assert "猫哮喘只是鉴别诊断之一。" in msgs[1]["content"]


def test_aggregator_exercise_does_not_force_case_sections():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    orch._active_intent_decision = IntentDecision(
        intent_id="D5", name="治疗与用药安全", confidence=0.9,
        output_variant="default", reason="运动处置建议",
    )
    decision = RouterDecision(
        scores={"clinical": 8},
        raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0},
        selected_experts=["clinical"],
        emergency=False,
        out_of_scope=False,
        reason="test",
    )
    msgs = orch._build_synthesis_messages(
        query=_BULLDOG_EXERCISE,
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "限制剧烈运动", "evidence": [], "risks": ["热射病"], "confidence": 0.8,
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
    )
    sys = msgs[0]["content"]
    assert "治疗目标" in sys
    assert "监测与复查" in sys
    assert "病例整理" not in sys
    assert '"intent_id": "D5"' in msgs[1]["content"]


def test_aggregator_rag_evidence_uses_retrieved_source_ledger():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    decision = RouterDecision(
        scores={"clinical": 8}, raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0}, selected_experts=["clinical"],
        emergency=False, out_of_scope=False, reason="test",
    )
    msgs = orch._build_synthesis_messages(
        query="猫尿血怎么办",
        opinions=[{
            "expert": "clinical", "name_zh": "兽医临床专家", "weight": 1.0,
            "conclusion": "下泌尿道问题", "evidence": ["模型自己的判断"], "risks": [], "confidence": 0.8,
            "tool_results": [{
                "tool_name": "rag.search", "ok": True,
                "result": {"hits": [{
                    "source_path": "books/feline_medicine.pdf", "page": 42,
                    "text": "Feline lower urinary tract signs include dysuria and hematuria.",
                }]},
            }],
        }],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=decision,
    )

    assert "只有其中列出的来源可以被引用" in msgs[0]["content"]
    assert "专家意见中的 evidence 只是专家推断摘要，不是检索来源" in msgs[0]["content"]
    assert '"id": "R1"' in msgs[1]["content"]
    assert '"source_path": "books/feline_medicine.pdf"' in msgs[1]["content"]
    assert '"page": 42' in msgs[1]["content"]
    assert '"tool_results"' not in msgs[1]["content"]


def test_answer_char_budget_converts_tokens_and_ignores_missing_budget():
    assert answer_char_budget(2500) == 3000
    assert answer_char_budget(768) == 921
    assert answer_char_budget(None) is None
    assert answer_char_budget(0) is None
    # 极小预算不应折算出模型无法交付的字数
    assert answer_char_budget(10) == 300


def test_solve_prompt_states_char_budget_not_raw_token_count():
    prompt = build_solve_prompt(user_role="veterinarian", query=_FLUTD_DIAGNOSIS, max_tokens=2500)

    assert "篇幅预算" in prompt
    assert "3000 字以内" in prompt
    assert "2500" not in prompt
    assert "token" not in prompt.lower()
    # 篇幅约束不得压过后续注入的意图分节结构要求
    assert "优先保证分节结构完整" in prompt
    assert "兽医病例工作流答风" not in prompt


def test_solve_prompt_omits_budget_clause_when_unspecified():
    assert "篇幅预算" not in build_solve_prompt(user_role="pet_owner", query="狗吐了怎么办")


def test_aggregator_prompt_carries_its_own_resolved_budget(monkeypatch):
    monkeypatch.setenv("MOE_FINAL_ANSWER_MAX_TOKENS", "1500")
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian", max_tokens=1500))
    msgs = orch._build_synthesis_messages(
        query=_FLUTD_DIAGNOSIS,
        opinions=[],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=RouterDecision(
            scores={"clinical": 8}, raw_weights={"clinical": 1.0},
            weights={"clinical": 1.0}, selected_experts=["clinical"],
            emergency=False, out_of_scope=False, reason="test",
        ),
    )

    assert "1800 字以内" in msgs[0]["content"]


def test_aggregator_max_tokens_defaults_and_caps_at_2500():
    orch = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian", max_tokens=900))
    assert orch._aggregator_max_tokens(_FLUTD_DIAGNOSIS) == 900
    assert orch._aggregator_max_tokens(_BULLDOG_EXERCISE) == 900
    assert orch._aggregator_max_tokens("猫尿血怎么办") == 900
    orch2 = MoEOrchestrator(config=OrchestratorConfig(user_role="pet_owner", max_tokens=3000))
    assert orch2._aggregator_max_tokens("狗吐了") == 2500
    assert MoEOrchestrator()._aggregator_max_tokens("狗吐了") == 2500
