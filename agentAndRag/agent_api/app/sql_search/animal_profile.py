"""按 animal_id 读取宠物画像（固定只读 SQL）。

供 MoE 物种软过滤与个性化使用：路由/专家在请求带 animal_id 时，先取 species 等画像，
注入到提示词与 RAG 查询改写中，降低"犬猫方案混用"的风险。
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from .config import load_mysql_config
from .executor import execute_readonly


_PROFILE_SQL = (
    "SELECT animal_id, species, name, breed, sex, age_months, weight_kg "
    "FROM animals WHERE animal_id = %s LIMIT 1"
)


def fetch_animal_profile(animal_id: Optional[str]) -> Optional[Dict[str, Any]]:
    """Return the animal profile dict, or None if missing / on any error.

    This never raises: profile is an enhancement, not a hard dependency of the agent.
    """
    aid = (animal_id or "").strip()
    if not aid:
        return None
    try:
        cfg = load_mysql_config()
        rows, _ = execute_readonly(_PROFILE_SQL, [aid], cfg)
    except Exception:  # noqa: BLE001
        return None
    if not rows:
        return None
    return rows[0]


def species_label(profile: Optional[Dict[str, Any]]) -> Optional[str]:
    """Human-facing species word for prompts; None when unknown/other."""
    if not profile:
        return None
    sp = str(profile.get("species") or "").strip().lower()
    mapping = {
        "dog": "犬（狗）",
        "cat": "猫",
        "pig": "猪",
        "sheep": "羊",
        "cattle": "牛",
        "horse": "马",
    }
    return mapping.get(sp)
