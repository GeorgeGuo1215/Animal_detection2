"""Schemas for the stateful browser-only MoE test interface."""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field


class ChatMoeSessionResponse(BaseModel):
    ok: bool = True
    session_id: str


class ChatMoeCompletionRequest(BaseModel):
    session_id: str = Field(description="Session created by POST /chat-moe/sessions")
    message: str = Field(description="Current user message; history is loaded only for this test session")
    user_role: Literal["pet_owner", "veterinarian"] = "pet_owner"
    response_lang: Literal["zh", "en"] = "zh"
    temperature: float = Field(default=0.3, ge=0.0, le=2.0)
    max_tokens: Optional[int] = Field(default=None, ge=1)
