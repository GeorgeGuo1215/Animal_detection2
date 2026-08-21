"""OpenAI 兼容 `/v1/chat/completions` 的请求/响应 Schema。"""
from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional, Union

from pydantic import BaseModel, Field


class ChatMessage(BaseModel):
    """OpenAI 聊天消息格式。"""
    role: Literal["system", "user", "assistant", "tool"]
    content: Optional[str] = None
    name: Optional[str] = None
    tool_calls: Optional[List[Dict[str, Any]]] = None
    tool_call_id: Optional[str] = None


class PetHealthServerContext(BaseModel):
    """PetHealth_Server 提供的请求级信号。"""

    animal_id: Optional[str] = Field(
        default=None,
        description="PetHealth Pet.id / PetHealthMetric.petId，供 mcp.vitals_alert.check_vitals 使用。",
    )
    heart_rate_abnormal: bool = Field(
        default=False,
        description="PetHealth_Server 检测到心率异常信号时为 True。",
    )
    vitals_window_hours: int = Field(
        default=24,
        ge=1,
        le=720,
        description="核实 PetHealth 体征的回看时间窗口（小时）。",
    )


class ChatCompletionRequest(BaseModel):
    """无状态请求；调用方在 messages 中提供全部对话历史。"""
    model: Literal["agent-moe"] = Field(default="agent-moe", description="唯一支持的 Agent 模型")
    messages: List[ChatMessage]
    
    # OpenAI 标准可选字段
    temperature: Optional[float] = 0.2
    max_tokens: Optional[int] = None
    top_p: Optional[float] = 1.0
    n: Optional[int] = 1  # 补全条数（本 Agent 仅支持 1）
    stream: Optional[bool] = False
    stop: Optional[Union[str, List[str]]] = None
    presence_penalty: Optional[float] = 0
    frequency_penalty: Optional[float] = 0
    logit_bias: Optional[Dict[str, float]] = None
    user: Optional[str] = None  # 终端用户标识

    # 用户级跨会话记忆。``user`` 仍是 OpenAI 标准终端用户字段；user_id 是内部/PetHealth 调用方的显式别名。
    user_id: Optional[str] = Field(
        default=None,
        description="用于隔离跨会话记忆的稳定已认证用户 id。",
    )
    memory_session_id: Optional[str] = Field(
        default=None,
        description="可选的上游聊天会话 id，随记忆轮次一并存储。",
    )
    memory_turn_id: Optional[str] = Field(
        default=None,
        description="本轮已完成 user/assistant 对话的稳定幂等键。",
    )
    
    # 扩展字段（OpenAI 兼容 tool calling）
    tools: Optional[List[Dict[str, Any]]] = None  # 可用工具列表
    tool_choice: Optional[Union[str, Dict]] = "auto"  # "none"、"auto" 或指定工具

    # 用户角色：pet_owner（通俗）或 veterinarian（专业）
    user_role: Optional[Literal["pet_owner", "veterinarian"]] = "pet_owner"

    # Debug：在响应中返回逐步耗时及原始工具载荷。
    # 默认关闭：开启后会把完整 web_search 结果回传给客户端。
    debug_timing: Optional[bool] = False

    # PetMind：将 sql.search 限定到该动物（也可通过头 X-Animal-Id 传入）
    animal_id: Optional[str] = Field(
        default=None,
        description="设置后向 Agent 暴露 sql.search；须与查询 daily_reports 的宠物一致。",
    )

    # PetHealth_Server：外部监测信号。MoE 会消费该信号；未经 MCP 体征工具核验前不得当作患者事实。
    pethealth_server: Optional[PetHealthServerContext] = None


class ChatCompletionChoice(BaseModel):
    """聊天补全响应中的单条 choice。"""
    index: int = 0
    message: ChatMessage
    finish_reason: Literal["stop", "length", "truncated", "tool_calls", "content_filter"] = "stop"


class UsageInfo(BaseModel):
    """Token 用量信息。"""
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0


class ChatCompletionResponse(BaseModel):
    """OpenAI 兼容的 `/v1/chat/completions` 响应。"""
    id: str
    object: Literal["chat.completion"] = "chat.completion"
    created: int
    model: str
    choices: List[ChatCompletionChoice]
    usage: UsageInfo = Field(default_factory=UsageInfo)
    
    # 扩展：记忆集成元数据
    memory: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Agent 记忆检索/写入元数据；不回传记忆正文。",
    )


# 流式响应类型
class ChatCompletionChunkDelta(BaseModel):
    """流式响应中的增量内容。"""
    role: Optional[str] = None
    content: Optional[str] = None


class ChatCompletionChunkChoice(BaseModel):
    """流式 chunk 中的单条 choice。"""
    index: int = 0
    delta: ChatCompletionChunkDelta
    finish_reason: Optional[Literal["stop", "length", "truncated", "tool_calls", "content_filter"]] = None


class ChatCompletionChunk(BaseModel):
    """流式响应的 SSE chunk。"""
    id: str
    object: Literal["chat.completion.chunk"] = "chat.completion.chunk"
    created: int
    model: str
    choices: List[ChatCompletionChunkChoice]
    
    # 扩展：Agent 状态更新
    agent_status: Optional[str] = None  # "planning"、"searching"、"generating"
    agent_detail: Optional[Dict[str, Any]] = None
