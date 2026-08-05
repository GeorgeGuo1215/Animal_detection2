"""
OpenAI-compatible API schemas for /v1/chat/completions endpoint.
"""
from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional, Union

from pydantic import BaseModel, Field


class ChatMessage(BaseModel):
    """OpenAI chat message format."""
    role: Literal["system", "user", "assistant", "tool"]
    content: Optional[str] = None
    name: Optional[str] = None
    tool_calls: Optional[List[Dict[str, Any]]] = None
    tool_call_id: Optional[str] = None


class PetHealthServerContext(BaseModel):
    """Request-scoped signals supplied by PetHealth_Server."""

    animal_id: Optional[str] = Field(
        default=None,
        description="PetHealth Pet.id / PetHealthMetric.petId used by mcp.vitals_alert.check_vitals.",
    )
    heart_rate_abnormal: bool = Field(
        default=False,
        description="True when PetHealth_Server has detected an abnormal heart-rate signal.",
    )
    vitals_window_hours: int = Field(
        default=24,
        ge=1,
        le=720,
        description="Look-back window for PetHealth vitals verification.",
    )


class ChatCompletionRequest(BaseModel):
    """Stateless request; the caller supplies all conversation history in messages."""
    model: str = Field(default="agent-plan-solve", description="Model name (agent-plan-solve for this agent)")
    messages: List[ChatMessage]
    
    # OpenAI standard optional fields
    temperature: Optional[float] = 0.2
    max_tokens: Optional[int] = None
    top_p: Optional[float] = 1.0
    n: Optional[int] = 1  # Number of completions (this agent only supports 1)
    stream: Optional[bool] = False
    stop: Optional[Union[str, List[str]]] = None
    presence_penalty: Optional[float] = 0
    frequency_penalty: Optional[float] = 0
    logit_bias: Optional[Dict[str, float]] = None
    user: Optional[str] = None  # End user identifier

    # User-level cross-session memory. ``user`` remains the OpenAI-standard
    # end-user field; user_id is an explicit alias for internal/PetHealth callers.
    user_id: Optional[str] = Field(
        default=None,
        description="Stable authenticated user id used to isolate cross-session memory.",
    )
    memory_session_id: Optional[str] = Field(
        default=None,
        description="Optional upstream chat session id stored with the memory turn.",
    )
    memory_turn_id: Optional[str] = Field(
        default=None,
        description="Stable idempotency key for this completed user/assistant turn.",
    )
    
    # Extension fields (OpenAI-compatible tool calling)
    tools: Optional[List[Dict[str, Any]]] = None  # Available tools list
    tool_choice: Optional[Union[str, Dict]] = "auto"  # "none", "auto", or specific tool

    # User role: pet_owner (casual) or veterinarian (professional)
    user_role: Optional[Literal["pet_owner", "veterinarian"]] = "pet_owner"

    # Debug: return per-step timing plus raw tool payloads in the response.
    # Off by default: enabling it streams full web_search results back to the client.
    debug_timing: Optional[bool] = False

    # PetMind: scope sql.search to this animal (also accepted as header X-Animal-Id)
    animal_id: Optional[str] = Field(
        default=None,
        description="When set, exposes sql.search to the agent; must match the pet whose daily_reports are queried.",
    )

    # PetHealth_Server: external monitoring signal. Only MoE consumes this; it
    # never makes the flag a patient fact without checking the MCP vitals tool.
    pethealth_server: Optional[PetHealthServerContext] = None


class ChatCompletionChoice(BaseModel):
    """Single choice in chat completion response."""
    index: int = 0
    message: ChatMessage
    finish_reason: Literal["stop", "length", "truncated", "tool_calls", "content_filter"] = "stop"


class UsageInfo(BaseModel):
    """Token usage information."""
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0


class ChatCompletionResponse(BaseModel):
    """OpenAI-compatible /v1/chat/completions response."""
    id: str
    object: Literal["chat.completion"] = "chat.completion"
    created: int
    model: str
    choices: List[ChatCompletionChoice]
    usage: UsageInfo = Field(default_factory=UsageInfo)
    
    # Extension: agent-specific metadata
    plan: Optional[List[Dict[str, Any]]] = None
    tool_results: Optional[List[Dict[str, Any]]] = None
    timing: Optional[List[Dict[str, Any]]] = None
    memory: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Agent memory retrieval/write metadata; memory content itself is not echoed.",
    )


# Streaming response types
class ChatCompletionChunkDelta(BaseModel):
    """Delta content in streaming response."""
    role: Optional[str] = None
    content: Optional[str] = None


class ChatCompletionChunkChoice(BaseModel):
    """Single choice in streaming chunk."""
    index: int = 0
    delta: ChatCompletionChunkDelta
    finish_reason: Optional[Literal["stop", "length", "truncated", "tool_calls", "content_filter"]] = None


class ChatCompletionChunk(BaseModel):
    """SSE chunk for streaming response."""
    id: str
    object: Literal["chat.completion.chunk"] = "chat.completion.chunk"
    created: int
    model: str
    choices: List[ChatCompletionChunkChoice]
    
    # Extension: agent status updates
    agent_status: Optional[str] = None  # "planning", "searching", "generating"
    agent_detail: Optional[Dict[str, Any]] = None
