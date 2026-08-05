"""Schemas for the stateful browser-only MoE test interface."""
from __future__ import annotations

from typing import Any, Dict, Literal, Optional

from pydantic import BaseModel, Field


class ChatMoeSessionRequest(BaseModel):
    username: str = Field(
        min_length=1,
        max_length=80,
        description="Tester name used to isolate and restore cross-session memory.",
    )


class ChatMoeSessionResponse(BaseModel):
    ok: bool = True
    session_id: str
    username: str
    memory_user_id: str
    memory: Dict[str, Any] = Field(default_factory=dict)


class ChatMoeCompletionRequest(BaseModel):
    session_id: str = Field(description="Session created by POST /chat-moe/sessions")
    message: str = Field(description="Current user message; history is loaded only for this test session")
    user_role: Literal["pet_owner", "veterinarian"] = "pet_owner"
    response_lang: Literal["zh", "en"] = "zh"
    temperature: float = Field(default=0.3, ge=0.0, le=2.0)
    max_tokens: Optional[int] = Field(default=None, ge=1)
