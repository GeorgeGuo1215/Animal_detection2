from agent_api.app.services.moe.router import RouterConfig, resolve_router_decision


def test_emergency_floor_adds_clinical_but_not_unrelated_pharmacy():
    """验证急症保底会加入临床专家，但不会拉上不相关的药房。"""
    decision = resolve_router_decision(
        scores={"clinical": 9, "pharmacy": 2, "nutrition": 0, "behavior": 0},
        emergency=True,
        reason="urinary emergency triage",
        config=RouterConfig(),
    )

    assert decision.selected_experts == ["clinical"]


def test_emergency_keeps_pharmacy_when_semantically_relevant():
    """验证急症在语义相关时仍会保留药房专家。"""
    decision = resolve_router_decision(
        scores={"clinical": 9, "pharmacy": 8, "nutrition": 0, "behavior": 0},
        emergency=True,
        reason="toxin and antidote emergency",
        config=RouterConfig(),
    )

    assert decision.selected_experts == ["clinical", "pharmacy"]
