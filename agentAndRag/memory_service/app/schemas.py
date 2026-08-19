"""API 的请求与响应模型。

userId 沿用 pet-server 的 cuid 字符串，服务端不校验其格式，只作为外键使用——
用户不存在时数据库的外键约束会直接挡住，不需要在这里重复一遍判断。
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field


class MessageIn(BaseModel):
    """写入一轮完整对话。"""

    user_id: str = Field(..., min_length=1, description="pet-server 的用户 id")
    user_input: str = Field(..., min_length=1, description="用户这一轮说的话")
    agent_response: str = Field("", description="助手的回复")
    pet_id: Optional[str] = Field(None, description="这轮对话关联的宠物，可空")
    session_id: Optional[str] = Field(None, description="pet-server 的会话 id，可空")
    turn_id: Optional[str] = Field(
        None,
        min_length=1,
        max_length=200,
        description="调用方稳定轮次 ID；重复提交同一 user_id + turn_id 不会重复记忆",
    )


class MessageOut(BaseModel):
    id: str
    queued: bool = Field(..., description="是否新入队了提升任务；false 表示已合并进既有任务")
    short_term_size: int
    duplicate: bool = False


class SubjectIn(BaseModel):
    user_id: str = Field(..., min_length=1, max_length=200)
    display_name: Optional[str] = Field(None, max_length=200)
    source: str = Field(default="external", min_length=1, max_length=50)
    metadata: Dict[str, Any] = Field(default_factory=dict)


class SubjectOut(BaseModel):
    user_id: str
    display_name: Optional[str] = None
    source: str
    metadata: Dict[str, Any] = Field(default_factory=dict)
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class ContextIn(BaseModel):
    user_id: str = Field(..., min_length=1)
    query: str = Field("", description="当前用户提问，用于向量检索")
    pet_id: Optional[str] = None
    include_text: bool = Field(
        True, description="是否顺带返回拼好的中文文本块，便于直接塞进 system prompt"
    )


class KnowledgeItem(BaseModel):
    id: str
    content: str
    pet_id: Optional[str] = None
    source: str
    similarity: float


class RelatedPage(BaseModel):
    segment_id: str
    segment_summary: str
    user_input: str
    agent_response: str
    similarity: float
    created_at: Optional[datetime] = None


class DialogueTurn(BaseModel):
    user_input: str
    agent_response: str
    created_at: Optional[datetime] = None


class ContextOut(BaseModel):
    user_id: str
    profile: Dict[str, Any] = Field(default_factory=dict)
    profile_version: int = 0
    knowledge: List[KnowledgeItem] = Field(default_factory=list)
    related_pages: List[RelatedPage] = Field(default_factory=list)
    recent_dialogue: List[DialogueTurn] = Field(default_factory=list)
    text: Optional[str] = None


class ProfileOut(BaseModel):
    user_id: str
    profile: Dict[str, Any] = Field(default_factory=dict)
    version: int = 0
    updated_at: Optional[datetime] = None


class HeatStats(BaseModel):
    n: int
    min: float
    p50: float
    p90: float
    max: float


class StatsOut(BaseModel):
    user_id: str
    short_term: int
    segments: int
    pages: int
    knowledge: int
    heat: HeatStats


class HealthOut(BaseModel):
    status: str
    database: str
    queue: Dict[str, int] = Field(default_factory=dict)
    workers: Dict[str, int] = Field(default_factory=dict)


class MemoryManageItem(BaseModel):
    id: str
    type: str
    label: str
    content: Any
    created_at: Optional[datetime] = None
    source_count: int = 0
    generation_tags: List[str] = Field(default_factory=list)


class MemoryManageOut(BaseModel):
    user_id: str
    items: List[MemoryManageItem] = Field(default_factory=list)


class MemoryDeleteOut(BaseModel):
    deleted: int


class MemoryClearIn(BaseModel):
    scope: Literal["short_term", "knowledge", "profile"]


class MemoryRestoreIn(BaseModel):
    confirmation: str
    snapshot: Dict[str, Any]
