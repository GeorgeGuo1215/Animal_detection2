from __future__ import annotations

import os
from typing import Any, Dict

from ..sql_search.tool import sql_search_tool
from ..sql_search.vitals_summary import vitals_summary_tool
from .rag_tools import rag_reindex_tool, rag_search_tool
from .tool_registry import ToolSpec, ToolRegistry

_DEFAULT_DEVICE = os.getenv("AGENT_WARMUP_DEVICE") or None


def register_builtin_tools(registry: ToolRegistry) -> None:
    """在此注册内置工具。保持工具名与 input schema 对 Agent 调用方稳定。"""

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
                    "category": {
                        "description": (
                            "Optional knowledge subcategory id(s) to restrict retrieval "
                            "(e.g. 'pharmacy.papich' or ['clinical.internal_medicine','diagnostics.differential']). "
                            "Supports prefix wildcards like 'clinical.*'. Omit to search the full index."
                        ),
                        "oneOf": [
                            {"type": "string"},
                            {"type": "array", "items": {"type": "string"}},
                            {"type": "null"},
                        ],
                        "default": None,
                    },
                    "embedding_model": {"type": "string", "default": "intfloat/multilingual-e5-small"},
                    "device": {"type": ["string", "null"], "default": _DEFAULT_DEVICE,
                               "description": "Inference device (auto-set from AGENT_WARMUP_DEVICE)"},
                    "multi_route": {"type": "boolean", "default": False},
                    "rewrite": {"type": "string", "enum": ["none", "template", "llm"], "default": "template"},
                    "rewrite_base_url": {"type": ["string", "null"], "default": None},
                    "rewrite_api_key": {"type": ["string", "null"], "default": None},
                    "rewrite_model": {"type": ["string", "null"], "default": None},
                    "rewrite_max_out": {"type": "integer", "default": 5, "minimum": 1, "maximum": 16},
                    "rewrite_timeout_s": {"type": "number", "default": 60.0, "minimum": 1.0, "maximum": 300.0},
                    "rerank": {"type": "boolean", "default": True},
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

    _sql_where_item = {
        "type": "object",
        "properties": {
            "column": {"type": "string"},
            "op": {
                "type": "string",
                "enum": ["eq", "ne", "gt", "gte", "lt", "lte", "in", "between", "like"],
            },
            "value": {},
        },
        "required": ["column", "op"],
    }
    _sql_order_item = {
        "type": "object",
        "properties": {
            "column": {"type": "string"},
            "direction": {"type": "string", "enum": ["asc", "desc"], "default": "asc"},
        },
        "required": ["column"],
    }

    registry.register(
        ToolSpec(
            name="sql.search",
            description=(
                "Read-only SQL on PetMind whitelist tables. Do NOT use for veterinary textbook / "
                "clinical knowledge — use rag.search. For raw per-second vitals (HR/RR/temperature) "
                "use vitals.summary instead; this tool returns table rows, not time-series aggregates."
                "\n\n**Whitelisted tables (each scoped to the request's animal_id automatically):**\n"
                "- `daily_reports`: per-day health summary (report_text/json, risk_level, confidence).\n"
                "- `animals`: pet profile (species[dog/cat/...], name, breed, sex, age_months, weight_kg).\n"
                "- `sensor_events`: collar upload windows (event_id, ts, device_id, window_start/end, notes, tags, location_*).\n"
                "\n\n**Availability**: Like optional MCP tools, this tool is exposed to the agent only when the client "
                "sends a non-empty `animal_id` on the same HTTP request (JSON field `animal_id` or header `X-Animal-Id`). "
                "If `animal_id` is missing, the tool is omitted from the tool list."
                "\n\n**Table `daily_reports` structure**:\n"
                "- `id` BIGINT UNSIGNED, PK, auto_increment\n"
                "- `report_date` DATE NOT NULL (calendar day)\n"
                "- `animal_id` VARCHAR(64) NOT NULL — **always filtered server-side to the request’s animal_id**; "
                "do not add conflicting filters on animal_id\n"
                "- `risk_level` TINYINT NULL\n"
                "- `confidence` ENUM('low','medium','high') NULL\n"
                "- `report_text` MEDIUMTEXT NOT NULL (main narrative)\n"
                "- `report_json` JSON NULL, `evidence_json` JSON NULL\n"
                "- `agent_trace_id` VARCHAR(64) NULL\n"
                "- `created_at`, `updated_at` TIMESTAMP\n"
                "Unique: (`report_date`, `animal_id`)."
                "\n\n**Call shape**: `table` must be `daily_reports`; optional `columns`, `where` (other columns), "
                "`order_by`, `limit`. Server injects `animal_id = <request>`."
                "\nSchema file: `agent_api/app/sql_search/petmind_schema.sql`."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "database": {"type": "string", "default": "petmind"},
                    "target": {"type": "string", "enum": ["single_table"], "default": "single_table"},
                    "table": {
                        "type": "string",
                        "enum": ["daily_reports", "animals", "sensor_events"],
                        "default": "daily_reports",
                    },
                    "columns": {"type": "array", "items": {"type": "string"}},
                    "where": {"type": "array", "items": _sql_where_item},
                    "order_by": {"type": "array", "items": _sql_order_item},
                    "limit": {"type": "integer", "default": 50, "minimum": 1, "maximum": 500},
                },
                "required": [],
            },
            handler=lambda **kwargs: sql_search_tool(**kwargs),
        )
    )

    registry.register(
        ToolSpec(
            name="vitals.summary",
            description=(
                "Aggregated physiological vital signs for the request-scoped pet, computed from the "
                "collar time-series (vitals_samples / temp_samples joined via sensor_events). "
                "Returns count/min/max/avg for heart rate (hr_bpm), respiratory rate (rr_bpm) and "
                "body temperature (temp_c), plus the sample time range. "
                "Use this (NOT sql.search) whenever you need the pet's actual HR/RR/temperature trends. "
                "\n\n**Availability**: exposed only when the request includes a non-empty animal_id "
                "(JSON field `animal_id` or header `X-Animal-Id`); server enforces the animal scope."
            ),
            input_schema={
                "type": "object",
                "properties": {
                    "days": {
                        "type": ["integer", "null"],
                        "default": None,
                        "minimum": 1,
                        "description": "If set, only include samples from the last N days; otherwise all history.",
                    },
                },
                "required": [],
            },
            handler=lambda **kwargs: vitals_summary_tool(**kwargs),
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
                    "device": {"type": ["string", "null"], "default": _DEFAULT_DEVICE,
                               "description": "Inference device (auto-set from AGENT_WARMUP_DEVICE)"},
                },
                "required": [],
            },
            handler=lambda **kwargs: rag_reindex_tool(**kwargs),
        )
    )


def _debug_echo_tool(**kwargs: Any) -> Dict[str, Any]:
    """回显参数，供工作流调试。"""
    return {"echo": kwargs}


def register_debug_tools(registry: ToolRegistry) -> None:
    """注册 debug.echo 调试工具。"""
    registry.register(
        ToolSpec(
            name="debug.echo",
            description="Echo back the arguments (for workflow debugging).",
            input_schema={"type": "object", "properties": {}, "required": []},
            handler=_debug_echo_tool,
        )
    )
