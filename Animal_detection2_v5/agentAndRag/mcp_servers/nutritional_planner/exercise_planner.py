"""
Exercise planning based on calorie deficit and medical constraints.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List

# Approximate calorie burn rates per kg of body weight per minute
_EXERCISE_TYPES = {
    "slow_walk": {
        "kcal_per_kg_per_min": 0.05,
        "description": "Slow walk (leash walk, 2-3 km/h)",
        "intensity": "low",
        "contraindicated_for": [],
    },
    "moderate_walk": {
        "kcal_per_kg_per_min": 0.10,
        "description": "Moderate walk (brisk pace, 4-5 km/h)",
        "intensity": "moderate",
        "contraindicated_for": ["post-surgery", "severe arthritis"],
    },
    "jogging": {
        "kcal_per_kg_per_min": 0.20,
        "description": "Jogging (6-8 km/h)",
        "intensity": "high",
        "contraindicated_for": ["post-surgery", "heart disease", "arthritis", "obesity", "recovery"],
    },
    "swimming": {
        "kcal_per_kg_per_min": 0.15,
        "description": "Swimming (low impact, full body)",
        "intensity": "moderate",
        "contraindicated_for": ["ear infection", "open wounds"],
    },
    "fetch_play": {
        "kcal_per_kg_per_min": 0.18,
        "description": "Fetch / active play",
        "intensity": "high",
        "contraindicated_for": ["post-surgery", "limit jumping", "arthritis", "recovery"],
    },
    "treadmill": {
        "kcal_per_kg_per_min": 0.12,
        "description": "Treadmill (controlled pace)",
        "intensity": "moderate",
        "contraindicated_for": ["post-surgery", "recovery"],
    },
    "gentle_play": {
        "kcal_per_kg_per_min": 0.06,
        "description": "Gentle indoor play / puzzle toys",
        "intensity": "low",
        "contraindicated_for": [],
    },
}


def _is_contraindicated(exercise: Dict[str, Any], medical_instructions: str) -> bool:
    medical_lower = medical_instructions.lower()
    for contra in exercise.get("contraindicated_for", []):
        if contra.lower() in medical_lower:
            return True
    return False


def generate_exercise_plan(
    pet_id: str,
    weight_kg: float,
    activity_calories_burned: float = 0,
    medical_instructions: str = "",
    target_daily_calories: float = 0,
) -> Dict[str, Any]:
    """
    Generate exercise recommendations based on calorie needs and medical constraints.
    """
    remaining_deficit = max(0, target_daily_calories - activity_calories_burned)

    available_exercises: List[Dict[str, Any]] = []
    for name, info in _EXERCISE_TYPES.items():
        if _is_contraindicated(info, medical_instructions):
            continue
        kcal_per_min = info["kcal_per_kg_per_min"] * weight_kg
        available_exercises.append({
            "exercise": name,
            "description": info["description"],
            "intensity": info["intensity"],
            "kcal_per_minute": round(kcal_per_min, 2),
        })

    recommendations: List[Dict[str, Any]] = []
    if remaining_deficit > 0 and available_exercises:
        for ex in sorted(available_exercises, key=lambda e: e["kcal_per_minute"], reverse=True):
            minutes_needed = math.ceil(remaining_deficit / ex["kcal_per_minute"]) if ex["kcal_per_minute"] > 0 else 999
            minutes_capped = min(minutes_needed, 60)
            calories_achieved = round(minutes_capped * ex["kcal_per_minute"], 1)
            recommendations.append({
                **ex,
                "suggested_minutes": minutes_capped,
                "calories_burned": calories_achieved,
            })

    # Build a simple daily plan suggestion
    daily_plan: List[Dict[str, Any]] = []
    if recommendations:
        low_intensity = [r for r in recommendations if r["intensity"] == "low"]
        mod_intensity = [r for r in recommendations if r["intensity"] == "moderate"]

        if mod_intensity:
            pick = mod_intensity[0]
            daily_plan.append({
                "time": "morning",
                "exercise": pick["exercise"],
                "description": pick["description"],
                "duration_minutes": min(30, pick["suggested_minutes"]),
            })
        if low_intensity:
            pick = low_intensity[0]
            daily_plan.append({
                "time": "evening",
                "exercise": pick["exercise"],
                "description": pick["description"],
                "duration_minutes": min(20, pick["suggested_minutes"]),
            })
        if not daily_plan and recommendations:
            pick = recommendations[-1]
            daily_plan.append({
                "time": "anytime",
                "exercise": pick["exercise"],
                "description": pick["description"],
                "duration_minutes": min(20, pick["suggested_minutes"]),
            })

    result: Dict[str, Any] = {
        "status": "OK",
        "pet_id": pet_id,
        "weight_kg": weight_kg,
        "target_daily_calories": target_daily_calories,
        "activity_calories_burned_so_far": activity_calories_burned,
        "remaining_calorie_deficit": round(remaining_deficit, 1),
        "available_exercises": available_exercises,
        "recommendations": recommendations[:5],
        "daily_plan": daily_plan,
    }

    if medical_instructions:
        result["medical_constraints_applied"] = medical_instructions

    if not available_exercises:
        result["status"] = "NO_SAFE_EXERCISES"
        result["message"] = (
            "Given the current medical constraints, no safe exercises could be recommended. "
            "Please consult your veterinarian for guidance."
        )

    return result
