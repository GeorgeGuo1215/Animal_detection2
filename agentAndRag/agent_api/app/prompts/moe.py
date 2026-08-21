from __future__ import annotations

from typing import Optional


PETHEALTH_VITALS_TOOL = "mcp.vitals_alert.check_vitals"


def inject_prompt(base_prompt: str, injected_prompt: Optional[str]) -> str:
    """在有有效内容时，把请求级提示块追加到基础提示词后。"""
    base = (base_prompt or "").strip()
    injected = (injected_prompt or "").strip()
    if not injected:
        return base
    if not base:
        return injected
    return f"{base}\n\n{injected}"


def build_pethealth_vitals_injection(
    *,
    animal_id: Optional[str],
    heart_rate_abnormal: bool,
    vitals_window_hours: int = 24,
    stage: str,
) -> str:
    """为 MoE 各阶段构建 PetHealth_Server 体征提示注入。"""
    aid = str(animal_id or "").strip()
    if not heart_rate_abnormal or not aid:
        return ""
    try:
        hours = int(vitals_window_hours)
    except (TypeError, ValueError):
        hours = 24
    hours = max(1, min(720, hours))

    shared = (
        "**PetHealth_Server 外部体征提示注入**\n"
        f"- PetHealth_Server 报告 `heart_rate_abnormal=true`，对应 PetHealth `animal_id/pet_id` 为 `{aid}`。\n"
        "- 这只是外部监测系统发来的待核实信号，不是诊断、不是病史确认，也不能单独证明当前宠物处于急症。\n"
        f"- 若需要核实真实心率，请使用 `{PETHEALTH_VITALS_TOOL}`，参数为 "
        f'{{"pet_id":"{aid}","hours":{hours}}}。'
    )
    if stage == "router":
        return shared + (
            "\n- 路由时应把本轮视为宠物健康相关上下文，优先考虑临床专家参与。"
            "\n- `emergency=true` 仍必须依据用户已报告的当前红旗，或后续真实体征工具结果；"
            "不得只因外部 flag 就判定急症。"
        )
    if stage == "aggregator":
        return shared + (
            "\n- 终答必须结合本次 MoE payload 中的 `pethealth_vitals_result`；如果其中显示 "
            "`alert_level=warning/alert` 或心率越界，应明确告知该结果和时间窗口。"
            "\n- 若 `pethealth_vitals_result` 缺失、工具不可用，或返回 `DB_UNAVAILABLE`、`NO_DATA`、"
            "`PET_NOT_FOUND`、`TOOL_NOT_ALLOWED`，必须说明无法核实真实心率，禁止编造心率数值、"
            "异常次数或采样时间。"
            "\n- 外部异常 flag 只能作为触发核实和提醒的原因；最终风险判断以用户描述、工具返回和"
            "兽医安全边界综合决定。"
        )
    return shared
