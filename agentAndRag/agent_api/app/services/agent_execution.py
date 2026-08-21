from __future__ import annotations

import os
from typing import Any, Dict, Iterable, List, Optional

from ..tools.tool_registry import ToolRegistry, get_registry
from .moe import MoEOrchestrator, OrchestratorConfig, RouterConfig, final_answer_max_tokens


AGENT_MODEL_ID = "agent-moe"
_PUBLIC_MOE_DEFAULT_TOOLS = ("rag.search", "mcp.web_search.web_search")


def build_moe_orchestrator(
    *,
    registry: Optional[ToolRegistry] = None,
    temperature: float = 0.3,
    max_tokens: Optional[int] = None,
    user_role: str = "pet_owner",
    allowed_tools: Optional[List[str]] = None,
    pethealth_server: Optional[Dict[str, Any]] = None,
) -> MoEOrchestrator:
    """按请求参数构建 MoEOrchestrator，并钳制 max_tokens 到终答预算。"""
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
    """公开 MoE 允许的工具名，与当前 registry 求交。"""
    configured = os.getenv("AGENT_PUBLIC_MOE_ALLOWED_TOOLS", "")
    requested = (
        [name.strip() for name in configured.split(",") if name.strip()]
        if configured.strip()
        else list(_PUBLIC_MOE_DEFAULT_TOOLS)
    )
    available = set(available_names)
    return [name for name in requested if name in available]
