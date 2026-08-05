from __future__ import annotations

from typing import Any, Dict

from .rag_tools import rag_reindex_tool, rag_search_tool
from .tool_registry import ToolSpec, ToolRegistry


def register_builtin_tools(registry: ToolRegistry) -> None:
    """
    Register built-in tools here.

    Keep the spec stable: n8n workflows will depend on tool name + input schema.
    """

    registry.register(
        ToolSpec(
            name="rag.search",
            description="Search the veterinary knowledge base (hybrid/multiroute optional) and return hits/contexts.",
            input_schema={
                "type": "object",
                "properties": {
                    "query": {"type": "string"},
                    "top_k": {"type": "integer", "default": 5, "minimum": 1, "maximum": 50},
                    "index_dir": {"type": ["string", "null"], "default": None},
                    "embedding_model": {"type": "string", "default": "intfloat/multilingual-e5-small"},
                    "device": {"type": ["string", "null"], "default": None},
                    "multi_route": {"type": "boolean", "default": False},
                    "rewrite": {"type": "string", "enum": ["none", "template", "llm"], "default": "template"},
                    "rewrite_base_url": {"type": ["string", "null"], "default": None},
                    "rewrite_api_key": {"type": ["string", "null"], "default": None},
                    "rewrite_model": {"type": ["string", "null"], "default": None},
                    "rewrite_max_out": {"type": "integer", "default": 5, "minimum": 1, "maximum": 16},
                    "rewrite_timeout_s": {"type": "number", "default": 60.0, "minimum": 1.0, "maximum": 300.0},
                    "rerank": {"type": "boolean", "default": False},
                    "rerank_model": {"type": "string", "default": "BAAI/bge-reranker-large"},
                    "rerank_candidates": {"type": "integer", "default": 10, "minimum": 1, "maximum": 200},
                    "rerank_batch_size": {"type": "integer", "default": 32, "minimum": 1, "maximum": 512},
                    "rerank_keep_topn": {"type": "integer", "default": 0, "minimum": 0, "maximum": 200},
                    "rerank_filter_overlap": {"type": "number", "default": 0.15, "minimum": 0.0, "maximum": 1.0},
                    "expand_neighbors": {"type": "integer", "default": 1, "minimum": 0, "maximum": 5},
                    "per_text_max_chars": {"type": "integer", "default": 5000, "minimum": 0, "maximum": 20000},
                    "include_hits_text": {"type": "boolean", "default": True},
                    "include_contexts_text": {"type": "boolean", "default": True},
                },
                "required": ["query"],
            },
            handler=lambda **kwargs: rag_search_tool(**kwargs),
        )
    )

    registry.register(
        ToolSpec(
            name="rag.reindex",
            description="(Re)build the vector index from RAG/data/raw into RAG/index (may take long).",
            input_schema={
                "type": "object",
                "properties": {
                    "raw_dir": {"type": ["string", "null"], "default": None},
                    "index_dir": {"type": ["string", "null"], "default": None},
                    "embedding_model": {"type": "string", "default": "intfloat/multilingual-e5-small"},
                    "batch_size": {"type": "integer", "default": 32, "minimum": 1, "maximum": 1024},
                    "limit_books": {"type": ["integer", "null"], "default": None, "minimum": 1},
                    "device": {"type": ["string", "null"], "default": None},
                },
                "required": [],
            },
            handler=lambda **kwargs: rag_reindex_tool(**kwargs),
        )
    )


def _debug_echo_tool(**kwargs: Any) -> Dict[str, Any]:
    return {"echo": kwargs}


def register_debug_tools(registry: ToolRegistry) -> None:
    registry.register(
        ToolSpec(
            name="debug.echo",
            description="Echo back the arguments (for workflow debugging).",
            input_schema={"type": "object", "properties": {}, "required": []},
            handler=_debug_echo_tool,
        )
    )

