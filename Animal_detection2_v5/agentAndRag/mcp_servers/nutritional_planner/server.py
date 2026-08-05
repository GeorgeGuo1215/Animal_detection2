"""
Nutritional_Planner MCP Server.

Exposes two tools:
  - calculate_meal_plan: Compute RER/MER and next meal grams
  - generate_exercise_plan: Exercise recommendations with medical constraints
"""
from __future__ import annotations

import json
import logging

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

from .exercise_planner import generate_exercise_plan
from .nutrition_calc import calculate_meal_plan

logger = logging.getLogger(__name__)

server = Server("nutritional_planner")


@server.list_tools()
async def list_tools() -> list[Tool]:
    return [
        Tool(
            name="calculate_meal_plan",
            description=(
                "Calculate a dynamic meal plan based on the pet's weight, activity, and health status. "
                "Computes RER (Resting Energy Requirement) = 70 * weight^0.75, "
                "then MER based on activity factor. Returns next meal grams and calorie balance. "
                "Time-aware: if no food logged after 20:00, triggers a feeding inquiry flag."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "pet_id": {
                        "type": "string",
                        "description": "Unique pet identifier (optional, defaults to 'anonymous')",
                        "default": "anonymous",
                    },
                    "weight_kg": {
                        "type": "number",
                        "description": "Pet's weight in kilograms",
                    },
                    "activity_calories_burned": {
                        "type": "number",
                        "description": "Calories burned through activity today",
                        "default": 0,
                    },
                    "calories_consumed_today": {
                        "type": "number",
                        "description": "Total calories consumed today",
                        "default": 0,
                    },
                    "medical_instructions": {
                        "type": "string",
                        "description": "Medical constraints, e.g. 'Post-surgery recovery, limit jumping'",
                        "default": "",
                    },
                    "neutered": {
                        "type": "boolean",
                        "description": "Whether the pet is neutered/spayed",
                        "default": True,
                    },
                    "life_stage": {
                        "type": "string",
                        "enum": ["puppy", "puppy_4m_to_1y", "adult", "senior"],
                        "description": "Pet's life stage",
                        "default": "adult",
                    },
                    "food_type": {
                        "type": "string",
                        "description": "Type of food being fed (for caloric density lookup)",
                        "default": "dry_kibble_standard",
                    },
                },
                "required": ["weight_kg"],
            },
        ),
        Tool(
            name="generate_exercise_plan",
            description=(
                "Generate exercise recommendations based on calorie deficit and medical constraints. "
                "Filters out contraindicated exercises and provides a daily plan with time and duration."
            ),
            inputSchema={
                "type": "object",
                "properties": {
                    "pet_id": {
                        "type": "string",
                        "description": "Unique pet identifier (optional, defaults to 'anonymous')",
                        "default": "anonymous",
                    },
                    "weight_kg": {
                        "type": "number",
                        "description": "Pet's weight in kilograms",
                    },
                    "activity_calories_burned": {
                        "type": "number",
                        "description": "Calories already burned today",
                        "default": 0,
                    },
                    "medical_instructions": {
                        "type": "string",
                        "description": "Medical constraints for exercise",
                        "default": "",
                    },
                    "target_daily_calories": {
                        "type": "number",
                        "description": "Target daily calorie expenditure",
                        "default": 0,
                    },
                },
                "required": ["weight_kg"],
            },
        ),
    ]


@server.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent]:
    if name == "calculate_meal_plan":
        result = calculate_meal_plan(
            pet_id=arguments.get("pet_id", "anonymous"),
            weight_kg=arguments["weight_kg"],
            activity_calories_burned=arguments.get("activity_calories_burned", 0),
            calories_consumed_today=arguments.get("calories_consumed_today", 0),
            medical_instructions=arguments.get("medical_instructions", ""),
            neutered=arguments.get("neutered", True),
            life_stage=arguments.get("life_stage", "adult"),
            food_type=arguments.get("food_type", "dry_kibble_standard"),
        )
    elif name == "generate_exercise_plan":
        result = generate_exercise_plan(
            pet_id=arguments.get("pet_id", "anonymous"),
            weight_kg=arguments["weight_kg"],
            activity_calories_burned=arguments.get("activity_calories_burned", 0),
            medical_instructions=arguments.get("medical_instructions", ""),
            target_daily_calories=arguments.get("target_daily_calories", 0),
        )
    else:
        result = {"error": f"Unknown tool: {name}"}

    return [TextContent(type="text", text=json.dumps(result, ensure_ascii=False, indent=2))]


async def run_server():
    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())
