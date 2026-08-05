"""
Core nutritional calculations: RER, MER, meal planning.

RER (Resting Energy Requirement) = 70 × (body_weight_kg) ^ 0.75
MER (Maintenance Energy Requirement) = RER × activity_factor
"""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

_DATA_DIR = Path(__file__).parent / "data"
_ACTIVITY_FACTORS: Optional[Dict[str, Any]] = None
_FOOD_DENSITY: Optional[Dict[str, Any]] = None


def _load_activity_factors() -> Dict[str, Any]:
    global _ACTIVITY_FACTORS
    if _ACTIVITY_FACTORS is None:
        path = _DATA_DIR / "activity_factors.json"
        _ACTIVITY_FACTORS = json.loads(path.read_text(encoding="utf-8"))
    return _ACTIVITY_FACTORS


def _load_food_density() -> Dict[str, Any]:
    global _FOOD_DENSITY
    if _FOOD_DENSITY is None:
        path = _DATA_DIR / "food_caloric_density.json"
        _FOOD_DENSITY = json.loads(path.read_text(encoding="utf-8"))
    return _FOOD_DENSITY


def compute_rer(weight_kg: float) -> float:
    """RER = 70 × weight^0.75"""
    if weight_kg <= 0:
        return 0.0
    return 70.0 * math.pow(weight_kg, 0.75)


def _resolve_activity_factor(
    life_stage: str = "adult",
    neutered: bool = True,
    medical_instructions: str = "",
) -> tuple[float, str]:
    """Determine the appropriate activity factor based on pet profile."""
    factors = _load_activity_factors()
    medical_lower = medical_instructions.lower()

    if any(kw in medical_lower for kw in ["surgery", "post-op", "recovery", "post-surgery"]):
        entry = factors.get("post_surgery_recovery", {})
        return entry.get("factor", 1.0), "post_surgery_recovery"

    if any(kw in medical_lower for kw in ["weight loss", "overweight", "obesity", "diet"]):
        entry = factors.get("weight_loss", {})
        return entry.get("factor", 1.0), "weight_loss"

    stage = life_stage.lower().strip()
    if stage in ("puppy", "puppy_under_4m", "kitten"):
        entry = factors.get("puppy_under_4m", {})
        return entry.get("factor", 3.0), "puppy_under_4m"
    if stage in ("puppy_4m_to_1y", "young"):
        entry = factors.get("puppy_4m_to_1y", {})
        return entry.get("factor", 2.0), "puppy_4m_to_1y"
    if stage in ("senior", "geriatric"):
        entry = factors.get("senior", {})
        return entry.get("factor", 1.2), "senior"

    if neutered:
        entry = factors.get("adult_neutered", {})
        return entry.get("factor", 1.6), "adult_neutered"
    else:
        entry = factors.get("adult_intact", {})
        return entry.get("factor", 1.8), "adult_intact"


def calculate_meal_plan(
    pet_id: str,
    weight_kg: float,
    activity_calories_burned: float = 0,
    calories_consumed_today: float = 0,
    medical_instructions: str = "",
    neutered: bool = True,
    life_stage: str = "adult",
    food_type: str = "dry_kibble_standard",
    current_hour: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Calculate a dynamic meal plan.

    Returns meal grams, calorie balance, and special flags like FEEDING_INQUIRY_NEEDED.
    """
    rer = compute_rer(weight_kg)
    factor, factor_label = _resolve_activity_factor(life_stage, neutered, medical_instructions)
    mer = rer * factor

    calorie_balance = mer - calories_consumed_today
    remaining_calories = max(0, calorie_balance)

    density_db = _load_food_density()
    food_info = density_db.get(food_type, density_db.get("default", {}))
    kcal_per_gram = food_info.get("kcal_per_gram", 3.5)

    next_meal_grams = round(remaining_calories / kcal_per_gram, 1) if kcal_per_gram > 0 else 0

    hour = current_hour if current_hour is not None else datetime.now().hour

    result: Dict[str, Any] = {
        "status": "OK",
        "pet_id": pet_id,
        "weight_kg": weight_kg,
        "rer_kcal": round(rer, 1),
        "activity_factor": factor,
        "activity_factor_label": factor_label,
        "mer_kcal": round(mer, 1),
        "calories_consumed_today": calories_consumed_today,
        "activity_calories_burned": activity_calories_burned,
        "calorie_balance": round(calorie_balance, 1),
        "remaining_calories_needed": round(remaining_calories, 1),
        "food_type": food_type,
        "kcal_per_gram": kcal_per_gram,
        "next_meal_grams": next_meal_grams,
        "flags": [],
    }

    if calories_consumed_today == 0 and hour >= 20:
        result["flags"].append("FEEDING_INQUIRY_NEEDED")
        result["inquiry_message"] = "No feeding recorded today and it's past 8 PM. Has the pet eaten today?"

    if calories_consumed_today > mer * 1.2:
        result["flags"].append("OVERFED_WARNING")
        result["next_meal_grams"] = 0
        result["overfed_message"] = (
            f"Calorie intake ({calories_consumed_today} kcal) exceeds MER ({round(mer, 1)} kcal) by "
            f"{round(calories_consumed_today - mer, 1)} kcal. Consider skipping the next meal or offering a small portion."
        )

    if medical_instructions:
        result["medical_note"] = medical_instructions

    return result
