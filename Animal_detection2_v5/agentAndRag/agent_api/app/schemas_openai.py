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


class ChatCompletionRequest(BaseModel):
    """OpenAI-compatible /v1/chat/completions request."""
    model: str = Field(default="agent-plan-solve", description="Model name (agent-plan-solve for this agent)")
    messages: List[ChatMessage]
    
    # OpenAI standard optional fields
    temperature: Optional[float] = 0.2
    max_tokens: Optional[int] = 768
    top_p: Optional[float] = 1.0
    n: Optional[int] = 1  # Number of completions (this agent only supports 1)
    stream: Optional[bool] = False
    stop: Optional[Union[str, List[str]]] = None
    presence_penalty: Optional[float] = 0
    frequency_penalty: Optional[float] = 0
    logit_bias: Optional[Dict[str, float]] = None
    user: Optional[str] = None  # End user identifier
    
    # Extension fields (OpenAI-compatible tool calling)
    tools: Optional[List[Dict[str, Any]]] = None  # Available tools list
    tool_choice: Optional[Union[str, Dict]] = "auto"  # "none", "auto", or specific tool

    # Debug: return per-step timing in the response
    debug_timing: Optional[bool] = True


class ChatCompletionChoice(BaseModel):
    """Single choice in chat completion response."""
    index: int = 0
    message: ChatMessage
    finish_reason: Literal["stop", "length", "tool_calls", "content_filter"] = "stop"


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


# Streaming response types
class ChatCompletionChunkDelta(BaseModel):
    """Delta content in streaming response."""
    role: Optional[str] = None
    content: Optional[str] = None


class ChatCompletionChunkChoice(BaseModel):
    """Single choice in streaming chunk."""
    index: int = 0
    delta: ChatCompletionChunkDelta
    finish_reason: Optional[Literal["stop", "length", "tool_calls", "content_filter"]] = None


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
