from __future__ import annotations

import os
from enum import Enum
from typing import Any, Dict, Iterable, List, Optional

from ..tools.tool_registry import ToolRegistry, get_registry
from .moe import MoEOrchestrator, OrchestratorConfig, RouterConfig, final_answer_max_tokens


class AgentMode(str, Enum):
    PLAN_AND_SOLVE = "plan_and_solve"
    MULTI_TURN = "multi_turn"
    MOE = "moe"


_PLAN_MODELS = {"agent-plan", "agent-plan-solve", "plan", "plan-and-solve", "plan_and_solve"}
_MULTI_TURN_MODELS = {"agent-multi-turn", "agent-multiturn", "multi-turn"}
_PUBLIC_MOE_DEFAULT_TOOLS = ("rag.search", "mcp.web_search.web_search")


def resolve_agent_mode(model: str) -> AgentMode:
    normalized = str(model or "").strip().lower()
    if normalized in _PLAN_MODELS:
        return AgentMode.PLAN_AND_SOLVE
    if normalized in _MULTI_TURN_MODELS:
        return AgentMode.MULTI_TURN
    return AgentMode.MOE


def build_moe_orchestrator(
    *,
    registry: Optional[ToolRegistry] = None,
    temperature: float = 0.3,
    max_tokens: Optional[int] = None,
    user_role: str = "pet_owner",
    allowed_tools: Optional[List[str]] = None,
    pethealth_server: Optional[Dict[str, Any]] = None,
) -> MoEOrchestrator:
    budget = final_answer_max_tokens()
    requested = budget if max_tokens is None else int(max_tokens)
    return MoEOrchestrator(
        registry=registry or get_registry(),
        config=OrchestratorConfig(
            router=RouterConfig(),
            temperature=float(temperature),
            max_tokens=min(max(1, requested), budget),
            user_role=user_role,
            allowed_tools=allowed_tools,
            pethealth_server=pethealth_server,
        ),
    )


def public_moe_allowed_tools(available_names: Iterable[str]) -> List[str]:
    configured = os.getenv("AGENT_PUBLIC_MOE_ALLOWED_TOOLS", "")
    requested = (
        [name.strip() for name in configured.split(",") if name.strip()]
        if configured.strip()
        else list(_PUBLIC_MOE_DEFAULT_TOOLS)
    )
    available = set(available_names)
    return [name for name in requested if name in available]
