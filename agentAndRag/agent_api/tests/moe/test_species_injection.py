"""Verify species soft-filter plumbing in the MoE layer (no LLM/DB).

Uses an ASCII sentinel as the species label so we only test the wiring, not text.
Run: pytest tests/moe/test_species_injection.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from app.prompts.moe_task_policy import build_task_policy_messages


SENTINEL = "SPECIES_SENTINEL_XYZ"


def test_task_policy_user_message_injects_species():
    msgs = build_task_policy_messages(
        query="why is my pet vomiting", user_role="pet_owner", species_zh=SENTINEL
    )
    user_content = msgs[1]["content"]
    assert SENTINEL in user_content
    assert "species" in user_content


def test_task_policy_user_message_omits_species_when_unknown():
    msgs = build_task_policy_messages(
        query="why is my pet vomiting", user_role="pet_owner", species_zh=None
    )
    user_content = msgs[1]["content"]
    assert SENTINEL not in user_content
    assert "species" not in json.loads(user_content)
