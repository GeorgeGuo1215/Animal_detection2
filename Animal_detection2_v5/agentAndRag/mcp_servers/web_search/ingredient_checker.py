"""
Ingredient safety checker against a contraindications database.

Flow:
1. Search for product ingredient list via Tavily web search (MCP) + LLM
2. Cross-reference against contraindications.json
3. Return conflicts or INSUFFICIENT_DATA if ingredients are unavailable
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import httpx

from .tavily_client import tavily_search

logger = logging.getLogger(__name__)

_DATA_DIR = Path(__file__).parent / "data"
_CONTRAINDICATIONS: Optional[Dict[str, Any]] = None

_LLM_BASE = (os.getenv("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/")
_LLM_KEY = os.getenv("OPENAI_API_KEY") or os.getenv("DEEPSEEK_API_KEY") or ""
_LLM_MODEL = os.getenv("OPENAI_MODEL") or os.getenv("DEEPSEEK_MODEL") or "deepseek-chat"


def _load_contraindications() -> Dict[str, Any]:
    global _CONTRAINDICATIONS
    if _CONTRAINDICATIONS is None:
        path = _DATA_DIR / "contraindications.json"
        if path.exists():
            _CONTRAINDICATIONS = json.loads(path.read_text(encoding="utf-8"))
        else:
            _CONTRAINDICATIONS = {}
    return _CONTRAINDICATIONS


async def _llm_call(prompt: str, max_tokens: int = 1024) -> str:
    if not _LLM_KEY:
        return "{}"
    url = f"{_LLM_BASE}/chat/completions"
    payload = {
        "model": _LLM_MODEL,
        "messages": [
            {"role": "system", "content": "You are a pet nutrition expert. Output strict JSON only."},
            {"role": "user", "content": prompt},
        ],
        "temperature": 0.1,
        "max_tokens": max_tokens,
        "response_format": {"type": "json_object"},
    }
    headers = {"Authorization": f"Bearer {_LLM_KEY}", "Content-Type": "application/json"}
    async with httpx.AsyncClient(timeout=60) as client:
        r = await client.post(url, headers=headers, json=payload)
        r.raise_for_status()
        data = r.json()
    try:
        return data["choices"][0]["message"]["content"].strip()
    except (KeyError, IndexError):
        return "{}"


async def _search_ingredients(product_name: str) -> Optional[tuple]:
    """Try to find ingredient list via Tavily web search (MCP) + LLM extraction."""
    results = await tavily_search(
        query=f"{product_name} ingredients list",
        max_results=8,
        search_depth="basic",
    )
    if not results:
        return None

    snippets = "\n".join(r.get("content", "") for r in results)[:3000]
    prompt = f"""From these search results, extract the ingredient list for "{product_name}".

Search results:
{snippets}

Output JSON:
{{
  "ingredients_found": true/false,
  "ingredients": ["ingredient1", "ingredient2", ...],
  "guaranteed_analysis": {{
    "protein_percent": null or number,
    "fat_percent": null or number,
    "fiber_percent": null or number,
    "moisture_percent": null or number,
    "phosphorus_percent": null or number
  }},
  "source_confidence": "high/medium/low"
}}

If you cannot find clear ingredient information, set ingredients_found to false.
"""
    try:
        raw = await _llm_call(prompt)
    except Exception as exc:
        logger.warning("LLM extraction failed: %s", exc)
        return None

    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None

    if not data.get("ingredients_found"):
        return None
    return data.get("ingredients"), data.get("guaranteed_analysis"), data.get("source_confidence", "low")


def _find_matching_conditions(health_context: str) -> List[Dict[str, Any]]:
    """Match the health context string against known conditions."""
    db = _load_contraindications()
    matches = []
    context_lower = health_context.lower()
    # Generic stopwords to exclude from keyword matching
    _stopwords = {"the", "and", "stage", "type", "grade", "disease", "chronic", "acute"}

    for condition, info in db.items():
        cond_lower = condition.lower()
        # Exact substring match (e.g. "Chronic Kidney Disease" in context)
        if cond_lower in context_lower:
            matches.append({"condition": condition, **info})
            continue
        # Meaningful keyword match: require at least 2 non-stopword tokens to match
        tokens = [w for w in cond_lower.split() if w not in _stopwords and len(w) > 2]
        if tokens and sum(1 for t in tokens if t in context_lower) >= max(1, len(tokens) - 1):
            matches.append({"condition": condition, **info})
    return matches


async def check_ingredients(
    product_name: str,
    current_health_context: str,
) -> Dict[str, Any]:
    """
    Check product ingredients against health-based contraindications.

    Returns INSUFFICIENT_DATA if ingredient info is unavailable.
    """
    matching_conditions = _find_matching_conditions(current_health_context)

    if not matching_conditions:
        return {
            "status": "NO_MATCHING_CONDITIONS",
            "product_name": product_name,
            "health_context": current_health_context,
            "message": "No known contraindications found for the given health context.",
            "conflicts": [],
            "confidence": "medium",
        }

    search_result = await _search_ingredients(product_name)

    if search_result is None:
        return {
            "status": "INSUFFICIENT_DATA",
            "product_name": product_name,
            "health_context": current_health_context,
            "matching_conditions": [c["condition"] for c in matching_conditions],
            "message": "Unable to find ingredient information for this product. Please provide the ingredient list from the product packaging.",
            "conflicts": [],
            "confidence": "none",
        }

    ingredients, analysis, source_confidence = search_result
    ingredients_lower = [i.lower() for i in (ingredients or [])]
    conflicts: List[Dict[str, Any]] = []

    for cond in matching_conditions:
        restricted = cond.get("restricted_ingredients", [])
        for restricted_item in restricted:
            for idx, ing in enumerate(ingredients_lower):
                if restricted_item.lower() in ing:
                    conflicts.append({
                        "condition": cond["condition"],
                        "restricted_ingredient": restricted_item,
                        "found_in": ingredients[idx],
                        "severity": "warning",
                        "notes": cond.get("notes", ""),
                    })

        ranges = cond.get("recommended_ranges", {})
        if analysis and ranges:
            for key, limit in ranges.items():
                if key.endswith("_max") and analysis.get(key.replace("_max", "").replace("_percent", "_percent")):
                    actual = analysis.get(key.replace("_max", "").replace("_percent", "_percent"))
                    if actual and isinstance(actual, (int, float)) and isinstance(limit, (int, float)):
                        if actual > limit:
                            conflicts.append({
                                "condition": cond["condition"],
                                "parameter": key,
                                "limit": limit,
                                "actual": actual,
                                "severity": "concern",
                                "notes": f"Exceeds recommended maximum of {limit}",
                            })

    overall_confidence = source_confidence if source_confidence else "medium"
    if not ingredients:
        overall_confidence = "low"

    return {
        "status": "OK",
        "product_name": product_name,
        "health_context": current_health_context,
        "ingredients_count": len(ingredients or []),
        "conflicts": conflicts,
        "conflict_count": len(conflicts),
        "safe": len(conflicts) == 0,
        "confidence": overall_confidence,
        "matching_conditions": [c["condition"] for c in matching_conditions],
        "guaranteed_analysis": analysis,
    }
