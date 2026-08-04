"""Per-request context (e.g. animal_id for sql.search scoping)."""

from __future__ import annotations

from contextvars import ContextVar
from typing import List, Optional

_REQUEST_ANIMAL_ID: ContextVar[Optional[str]] = ContextVar("request_animal_id", default=None)

SQL_SEARCH_TOOL_NAME = "sql.search"

# Tools that must stay hidden unless the request carries an animal_id (MCP-like conditional tools).
ANIMAL_REQUIRED_TOOLS = frozenset({"sql.search", "vitals.summary"})


def _normalize_animal_id(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    s = str(value).strip()
    return s or None


def set_request_animal_id(
    *,
    explicit: Optional[str] = None,
    body_animal_id: Optional[str] = None,
    header_animal_id: Optional[str] = None,
) -> None:
    """Resolve animal_id: explicit > body > header. Empty means unset."""
    aid = _normalize_animal_id(explicit)
    if aid is None:
        aid = _normalize_animal_id(body_animal_id)
    if aid is None:
        aid = _normalize_animal_id(header_animal_id)
    _REQUEST_ANIMAL_ID.set(aid)


def get_request_animal_id() -> Optional[str]:
    return _REQUEST_ANIMAL_ID.get()


def filter_tools_without_animal(tool_names: List[str]) -> List[str]:
    """Hide animal-scoped tools (sql.search, vitals.summary) when no animal_id is set."""
    if get_request_animal_id():
        return list(tool_names)
    return [n for n in tool_names if n not in ANIMAL_REQUIRED_TOOLS]
