from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field


class ToolError(BaseModel):
    code: str
    message: str
    detail: Optional[Dict[str, Any]] = None


class ToolResponse(BaseModel):
    ok: bool
    trace_id: str
    data: Optional[Dict[str, Any]] = None
    error: Optional[ToolError] = None


class RagSearchRequest(BaseModel):
    query: str
    top_k: int = 5
    index_dir: Optional[str] = None
    embedding_model: str = "intfloat/multilingual-e5-small"
    device: Optional[str] = None

    multi_route: bool = False
    rewrite: Literal["none", "template", "llm"] = "template"
    rewrite_base_url: Optional[str] = None
    rewrite_api_key: Optional[str] = None
    rewrite_model: Optional[str] = None
    rewrite_max_out: int = 5
    rewrite_timeout_s: float = 60.0

    rerank: bool = False
    rerank_model: str = "BAAI/bge-reranker-large"
    rerank_candidates: int = 10
    rerank_batch_size: int = 32
    rerank_keep_topn: int = 0
    rerank_filter_overlap: float = 0.15

    expand_neighbors: int = 1

    include_hits_text: bool = True
    include_contexts_text: bool = True
    per_text_max_chars: int = 5000


class RagReindexRequest(BaseModel):
    raw_dir: Optional[str] = None
    index_dir: Optional[str] = None
    embedding_model: str = "intfloat/multilingual-e5-small"
    batch_size: int = 32
    limit_books: Optional[int] = None
    device: Optional[str] = None


class JobStatusResponse(BaseModel):
    ok: bool
    trace_id: str
    job_id: str
    status: Literal["queued", "running", "succeeded", "failed"]
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None


class ToolSpecOut(BaseModel):
    name: str
    description: str
    input_schema: Dict[str, Any]


class ToolListResponse(BaseModel):
    ok: bool
    trace_id: str
    tools: List[ToolSpecOut]


class ToolCallRequest(BaseModel):
    tool_name: str = Field(description='Tool name, e.g. "rag.search"')
    arguments: Dict[str, Any] = Field(default_factory=dict)


class AgentPlanAndSolveRequest(BaseModel):
    query: str
    llm_base_url: Optional[str] = None
    llm_api_key: Optional[str] = None
    llm_model: Optional[str] = None
    temperature: float = 0.2
    max_tokens: int = 768
    allowed_tools: Optional[List[str]] = None


class AgentPlanAndSolveResponse(BaseModel):
    ok: bool
    trace_id: str
    answer: Optional[str] = None
    plan: Optional[List[Dict[str, Any]]] = None
    tool_results: Optional[List[Dict[str, Any]]] = None
    error: Optional[ToolError] = None
