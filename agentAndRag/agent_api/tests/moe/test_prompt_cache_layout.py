"""Prompt-prefix stability and provider cache accounting regressions."""
from __future__ import annotations

from datetime import date
import json
import re

from agent_api.app.prompts.solve import build_solve_prompt
from agent_api.app.services.moe.critic import CriticResult
from agent_api.app.services.moe.orchestration.service import MoEOrchestrator, OrchestratorConfig
from agent_api.app.services.moe.retrieval_policy import EvidenceTask
from agent_api.app.services.moe.router import RouterDecision
from agent_api.app.services.moe.trace import MoETrace, extract_usage


def _decision() -> RouterDecision:
    return RouterDecision(
        scores={"clinical": 8}, raw_weights={"clinical": 1.0},
        weights={"clinical": 1.0}, selected_experts=["clinical"],
        emergency=False, out_of_scope=False, reason="test",
    )


def _synthesis_payload(orchestrator: MoEOrchestrator) -> tuple[str, dict]:
    messages = orchestrator._build_synthesis_messages(
        query="请核对最新犬疫苗指南。",
        opinions=[],
        critic=CriticResult(verdict="pass", issues=[], constraints=[], reason="ok"),
        decision=_decision(),
    )
    return messages[0]["content"], json.loads(messages[1]["content"])


def test_generic_solve_prompt_has_no_calendar_value_in_stable_prefix() -> None:
    prompt = build_solve_prompt(user_role="veterinarian", query="犬疫苗咨询")
    assert "今天是" not in prompt
    assert re.search(r"\b20\d{2}-\d{2}-\d{2}\b", prompt) is None


def test_as_of_date_is_absent_without_current_web_task() -> None:
    system_prompt, payload = _synthesis_payload(
        MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    )
    assert "as_of_date" not in payload
    assert date.today().isoformat() not in system_prompt


def test_as_of_date_is_last_dynamic_field_for_current_web_task() -> None:
    orchestrator = MoEOrchestrator(config=OrchestratorConfig(user_role="veterinarian"))
    orchestrator._active_evidence_tasks = (
        EvidenceTask(
            capability="current_web", owner="clinical", requirement="required",
            reason="核对当前指南", queries=("犬疫苗现行指南", "current canine vaccination guideline"),
        ),
    )
    system_prompt, payload = _synthesis_payload(orchestrator)
    assert payload["as_of_date"] == date.today().isoformat()
    assert list(payload)[-1] == "as_of_date"
    assert payload["as_of_date"] not in system_prompt


def test_deepseek_cache_usage_is_preserved_and_aggregated() -> None:
    usage = extract_usage({"usage": {
        "prompt_tokens": 100, "completion_tokens": 20, "total_tokens": 120,
        "prompt_cache_hit_tokens": 80, "prompt_cache_miss_tokens": 20,
    }})
    trace = MoETrace()
    trace.record_llm(
        stage="test", model="deepseek-v4-flash", messages=[], output="ok",
        latency_ms=1, usage=usage,
    )
    assert trace.total_prompt_cache_hit_tokens() == 80
    assert trace.total_prompt_cache_miss_tokens() == 20
    assert trace.prompt_cache_hit_rate() == 0.8
