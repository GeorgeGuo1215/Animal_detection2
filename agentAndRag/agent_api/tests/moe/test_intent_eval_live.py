from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import sys

_MOE_TESTS = Path(__file__).resolve().parent
_AGENT_API = _MOE_TESTS.parents[1]
sys.path.insert(0, str(_AGENT_API))
sys.path.insert(0, str(_MOE_TESTS))

from app.prompts.intent_contracts import INTENT_SPECS, intent_required_sections  # noqa: E402
from run_intent_eval_live import CASES, EvalResult, revalidate_results, validate  # noqa: E402


def test_live_suite_has_four_distinct_questions_per_intent():
    """验证在线套件每个意图都有四道不同的题。"""
    assert len(CASES) == 32
    counts = Counter(case.intent_id for case in CASES)
    assert counts == {intent_id: 4 for intent_id in INTENT_SPECS}
    assert len({case.case_id for case in CASES}) == 32
    assert len({case.question for case in CASES}) == 32


def test_validator_accepts_contract_complete_answer():
    """验证校验器接受契约完整的回答。"""
    case = next(case for case in CASES if case.case_id == "d3_fever_workup")
    sections = "\n".join(f"**{section}**\n目的与触发条件。" for section in intent_required_sections("D3"))
    answer = f"{sections}\n输入数值39.8至40.2℃。"
    result = EvalResult(
        case_id=case.case_id,
        expected_intent="D3",
        actual_intent="D3",
        expected_variant="default",
        actual_variant="default",
        answer=answer,
        finish_reason="stop",
    )
    validate(case, result)
    assert result.issues == []


def test_validator_rejects_wrong_intent_and_missing_sections():
    """验证校验器拒绝错误意图和缺段。"""
    case = CASES[0]
    result = EvalResult(
        case_id=case.case_id,
        expected_intent=case.intent_id,
        actual_intent="D2",
        expected_variant=case.variant,
        actual_variant="default",
        answer="简短回答",
        finish_reason="stop",
    )
    validate(case, result)
    assert any("意图错误" in issue for issue in result.issues)
    assert any("缺少契约分节" in issue for issue in result.issues)


def test_validator_accepts_equivalent_fact_typography_and_safety_terms():
    """验证校验器接受等价的事实排版和安全术语。"""
    case = next(case for case in CASES if case.case_id == "d1_problem_list_flutd")
    sections = "\n".join(f"**{section}**\n未提供" for section in intent_required_sections("D1", "problem_list"))
    result = EvalResult(
        case_id=case.case_id,
        expected_intent="D1",
        actual_intent="D1",
        expected_variant="problem_list",
        actual_variant="problem_list",
        answer=f"{sections}\n5岁公猫，少量排尿，既往血尿史。",
        finish_reason="stop",
    )
    validate(case, result)
    assert result.issues == []


def test_revalidate_results_preserves_raw_answer_and_writes_verified_report(tmp_path: Path):
    """验证复验会保留原始回答并写出已核验报告。"""
    case = next(case for case in CASES if case.case_id == "d3_fever_workup")
    sections = "\n".join(f"**{section}**\n目的与触发条件。" for section in intent_required_sections("D3"))
    answer = f"{sections}\n输入数值39.8至40.2℃。"
    source = tmp_path / "source.json"
    source.write_text(json.dumps([{
        "case_id": case.case_id,
        "expected_intent": "D3",
        "actual_intent": "D3",
        "expected_variant": "default",
        "actual_variant": "default",
        "answer": answer,
        "finish_reason": "stop",
        "issues": ["旧规则误报"],
    }], ensure_ascii=False), encoding="utf-8")

    passed, total, report = revalidate_results(source, tmp_path / "verified", "default")

    verified = json.loads((report.parent / "results.json").read_text(encoding="utf-8"))
    assert (passed, total) == (1, 1)
    assert verified[0]["answer"] == answer
    assert verified[0]["issues"] == []
    assert "通过：1/1" in report.read_text(encoding="utf-8")
