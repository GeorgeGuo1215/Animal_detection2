from app.services.moe.router import RouterConfig, resolve_router_decision


def test_emergency_floor_adds_clinical_but_not_unrelated_pharmacy():
    decision = resolve_router_decision(
        scores={"clinical": 9, "pharmacy": 2, "nutrition": 0, "behavior": 0},
        emergency=True,
        reason="urinary emergency triage",
        config=RouterConfig(),
    )

    assert decision.selected_experts == ["clinical"]


def test_emergency_keeps_pharmacy_when_semantically_relevant():
    decision = resolve_router_decision(
        scores={"clinical": 9, "pharmacy": 8, "nutrition": 0, "behavior": 0},
        emergency=True,
        reason="toxin and antidote emergency",
        config=RouterConfig(),
    )

    assert decision.selected_experts == ["clinical", "pharmacy"]
