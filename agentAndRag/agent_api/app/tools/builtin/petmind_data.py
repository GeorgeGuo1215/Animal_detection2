"""PetMind 个体数据工具契约；数据库实现位于 integrations.petmind_mysql。"""

from __future__ import annotations

from ...integrations.petmind_mysql import sql_search_tool, vitals_summary_tool
from ..tool_registry import ToolRegistry, ToolSpec

_SQL_WHERE_ITEM = {
    "type": "object",
    "properties": {
        "column": {"type": "string"},
        "op": {"type": "string", "enum": ["eq", "ne", "gt", "gte", "lt", "lte", "in", "between", "like"]},
        "value": {},
    },
    "required": ["column", "op"],
}
_SQL_ORDER_ITEM = {
    "type": "object",
    "properties": {
        "column": {"type": "string"},
        "direction": {"type": "string", "enum": ["asc", "desc"], "default": "asc"},
    },
    "required": ["column"],
}


def register_petmind_data_tools(registry: ToolRegistry) -> None:
    """注册自动按请求 animal_id 隔离的只读数据工具。"""
    registry.register(ToolSpec(
        name="sql.search",
        description=(
            "Read-only access to PetMind whitelist tables (daily_reports, animals, sensor_events). "
            "The server always injects the request-scoped animal_id; use rag.search for textbook knowledge "
            "and vitals.summary for physiological time-series aggregates."
        ),
        input_schema={
            "type": "object",
            "properties": {
                "database": {"type": "string", "default": "petmind"},
                "target": {"type": "string", "enum": ["single_table"], "default": "single_table"},
                "table": {
                    "type": "string", "enum": ["daily_reports", "animals", "sensor_events"],
                    "default": "daily_reports",
                },
                "columns": {"type": "array", "items": {"type": "string"}},
                "where": {"type": "array", "items": _SQL_WHERE_ITEM},
                "order_by": {"type": "array", "items": _SQL_ORDER_ITEM},
                "limit": {"type": "integer", "default": 50, "minimum": 1, "maximum": 500},
            },
            "required": [],
        },
        handler=lambda **kwargs: sql_search_tool(**kwargs),
    ))
    registry.register(ToolSpec(
        name="vitals.summary",
        description=(
            "Return request-scoped HR, RR and temperature aggregates. The tool is available only when "
            "the request contains a non-empty animal_id."
        ),
        input_schema={
            "type": "object",
            "properties": {
                "days": {
                    "type": ["integer", "null"], "default": None, "minimum": 1,
                    "description": "Optional trailing-day window; omit for all available history.",
                },
            },
            "required": [],
        },
        handler=lambda **kwargs: vitals_summary_tool(**kwargs),
    ))
