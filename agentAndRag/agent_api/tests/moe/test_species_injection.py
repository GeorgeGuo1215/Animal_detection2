"""Verify species soft-filter plumbing in the MoE layer (no LLM/DB).

Uses an ASCII sentinel as the species label so we only test the wiring, not text.
Run: pytest tests/moe/test_species_injection.py
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.services.moe.router import _build_router_messages


SENTINEL = "SPECIES_SENTINEL_XYZ"


def test_router_user_message_injects_species():
    msgs = _build_router_messages("why is my pet vomiting", "pet_owner", SENTINEL)
    user_content = msgs[1]["content"]
    assert SENTINEL in user_content
    assert "species" in user_content


def test_router_user_message_omits_species_when_unknown():
    msgs = _build_router_messages("why is my pet vomiting", "pet_owner", None)
    user_content = msgs[1]["content"]
    assert SENTINEL not in user_content
    assert "species" not in user_content
