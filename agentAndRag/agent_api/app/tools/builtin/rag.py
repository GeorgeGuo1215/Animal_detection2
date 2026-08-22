"""本地兽医知识库工具契约。基础设施参数只由部署配置控制。"""

from __future__ import annotations

from ..rag_tools import rag_reindex_tool, rag_search_tool
from ..tool_registry import ToolRegistry, ToolSpec

_RAG_SEARCH_PUBLIC_ARGUMENTS = frozenset({
    "query", "top_k", "category", "multi_route", "rewrite", "rewrite_max_out",
    "rerank", "rerank_candidates", "rerank_keep_topn", "expand_neighbors",
    "per_text_max_chars", "include_hits_text", "include_contexts_text",
})


def _rag_search_handler(**kwargs):
    """只转发公开检索参数，忽略模型伪造的路径、密钥或设备配置。"""
    safe_arguments = {
        name: value for name, value in kwargs.items()
        if name in _RAG_SEARCH_PUBLIC_ARGUMENTS
    }
    return rag_search_tool(**safe_arguments)


def register_rag_tools(registry: ToolRegistry) -> None:
    """注册检索与运维重建工具，保持既有工具名称稳定。"""
    registry.register(ToolSpec(
        name="rag.search",
        description="Search the veterinary knowledge base and return evidence hits and contexts.",
        input_schema={
            "type": "object",
            "properties": {
                "query": {"type": "string"},
                "top_k": {"type": "integer", "default": 5, "minimum": 1, "maximum": 50},
                "category": {
                    "description": "Optional knowledge category id(s), including prefix wildcards such as clinical.*.",
                    "oneOf": [
                        {"type": "string"},
                        {"type": "array", "items": {"type": "string"}},
                        {"type": "null"},
                    ],
                    "default": None,
                },
                "multi_route": {"type": "boolean", "default": False},
                "rewrite": {
                    "type": "string", "enum": ["none", "template", "llm"], "default": "template",
                },
                "rewrite_max_out": {"type": "integer", "default": 5, "minimum": 1, "maximum": 16},
                "rerank": {"type": "boolean", "default": True},
                "rerank_candidates": {"type": "integer", "default": 10, "minimum": 1, "maximum": 200},
                "rerank_keep_topn": {"type": "integer", "default": 0, "minimum": 0, "maximum": 200},
                "expand_neighbors": {"type": "integer", "default": 1, "minimum": 0, "maximum": 5},
                "per_text_max_chars": {"type": "integer", "default": 5000, "minimum": 0, "maximum": 20000},
                "include_hits_text": {"type": "boolean", "default": True},
                "include_contexts_text": {"type": "boolean", "default": True},
            },
            "required": ["query"],
        },
        handler=_rag_search_handler,
    ))
    registry.register(ToolSpec(
        name="rag.reindex",
        description="Rebuild the configured veterinary vector index; internal operations only.",
        input_schema={
            "type": "object",
            "properties": {
                "batch_size": {"type": "integer", "default": 32, "minimum": 1, "maximum": 1024},
                "limit_books": {"type": ["integer", "null"], "default": None, "minimum": 1},
            },
            "required": [],
        },
        handler=lambda **kwargs: rag_reindex_tool(
            **{name: value for name, value in kwargs.items() if name in {"batch_size", "limit_books"}}
        ),
    ))
