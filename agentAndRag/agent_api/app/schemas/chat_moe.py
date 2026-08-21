"""仅浏览器、有状态的 MoE 测试界面的 Schema。"""
from __future__ import annotations

from typing import Any, Dict, Literal, Optional

from pydantic import BaseModel, Field


class ChatMoeSessionRequest(BaseModel):
    """创建 Chat-MoE 测试会话的请求。"""

    username: str = Field(
        min_length=1,
        max_length=80,
        description="用于隔离并恢复跨会话记忆的测试者名称。",
    )


class ChatMoeSessionResponse(BaseModel):
    """创建会话后的响应，含 memory_user_id。"""

    ok: bool = True
    session_id: str
    username: str
    memory_user_id: str
    memory: Dict[str, Any] = Field(default_factory=dict)


class ChatMoeCompletionRequest(BaseModel):
    """在已有测试会话上发起一轮补全。"""

    session_id: str = Field(description="由 POST /chat-moe/sessions 创建的会话")
    message: str = Field(description="当前用户消息；历史仅加载该测试会话")
    user_role: Literal["pet_owner", "veterinarian"] = "pet_owner"
    response_lang: Literal["zh", "en"] = "zh"
    temperature: float = Field(default=0.3, ge=0.0, le=2.0)
    max_tokens: Optional[int] = Field(default=None, ge=1)
