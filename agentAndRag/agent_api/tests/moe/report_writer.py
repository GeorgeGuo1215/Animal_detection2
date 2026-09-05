"""把 MoETrace 渲染为可读的 markdown 评测报告。"""
from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List

from agent_api.app.services.moe.trace import MoETrace


def _esc_cell(s: Any) -> str:
    """转义 Markdown 表格单元格中的竖线和换行。"""
    return str(s).replace("|", "\\|").replace("\n", " ")


def _fmt_messages(messages: List[Dict[str, Any]]) -> str:
    """把对话消息列表格式化成分角色文本。"""
    lines: List[str] = []
    for m in messages or []:
        role = m.get("role", "?")
        content = m.get("content", "")
        lines.append(f"--- {role} ---\n{content}")
    return "\n\n".join(lines)


def _config_table(config: Dict[str, Any]) -> str:
    """把运行超参渲染成 Markdown 表。"""
    if not config:
        return "_（无）_"
    rows = ["| 超参 | 取值 |", "| --- | --- |"]
    for k in sorted(config.keys()):
        rows.append(f"| {_esc_cell(k)} | {_esc_cell(config[k])} |")
    return "\n".join(rows)


def _router_section(trace: MoETrace) -> str:
    """渲染路由决策：拒答、急症、专家分与权重。"""
    d = trace.router_decision
    if not d:
        return "_（无路由记录）_"
    out: List[str] = []
    out.append(f"- 是否拒答(out_of_scope): **{d.get('out_of_scope')}**")
    out.append(f"- 是否急症(emergency): **{d.get('emergency')}**（关键词命中: {d.get('emergency_rule_hit')}）")
    out.append(f"- 选中专家: **{', '.join(d.get('selected_experts') or []) or '无'}**")
    out.append(f"- 理由: {d.get('reason') or ''}")
    scores = d.get("scores") or {}
    weights = d.get("weights") or {}
    raw_w = d.get("raw_weights") or {}
    out.append("")
    out.append("| 专家 | 相关性分(0-10) | softmax权重 | 选中后权重 |")
    out.append("| --- | --- | --- | --- |")
    for k in scores:
        out.append(f"| {k} | {scores.get(k)} | {raw_w.get(k)} | {weights.get(k, '-')} |")
    return "\n".join(out)


def _intent_section(trace: MoETrace) -> str:
    """渲染医生端意图分类结果。"""
    decision = trace.intent_decision
    if not decision:
        return "_（非医生端请求或无意图分类记录）_"
    return "\n".join([
        f"- 主意图: **{decision.get('intent_id')} {decision.get('name')}**",
        f"- 置信度: **{decision.get('confidence')}**",
        f"- 输出变体: `{decision.get('output_variant')}`",
        f"- 回退: **{decision.get('fallback')}**",
        f"- 理由: {decision.get('reason') or ''}",
        f"- 错误: {decision.get('error') or '无'}",
    ])


def _llm_calls_section(trace: MoETrace) -> str:
    """渲染各次 LLM 调用的耗时、token 与输入输出明细。"""
    if not trace.llm_calls:
        return "_（无 LLM 调用）_"
    out: List[str] = []
    out.append("| # | 阶段 | 模型 | 耗时(ms) | prompt_tok | cache_hit | cache_miss | completion_tok | total_tok |")
    out.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for c in trace.llm_calls:
        out.append(
            f"| {c.seq} | {_esc_cell(c.stage)} | {_esc_cell(c.model)} | {c.latency_ms} | "
            f"{c.prompt_tokens} | {c.prompt_cache_hit_tokens} | {c.prompt_cache_miss_tokens} | "
            f"{c.completion_tokens} | {c.total_tokens} |"
        )
    out.append("")
    out.append("### 各次调用输入/输出明细")
    for c in trace.llm_calls:
        out.append("")
        out.append(f"<details><summary>#{c.seq} · {c.stage} · {c.latency_ms}ms</summary>")
        out.append("")
        out.append("**输入 messages：**")
        out.append("")
        out.append("```text")
        out.append(_fmt_messages(c.messages))
        out.append("```")
        out.append("")
        out.append("**输出：**")
        out.append("")
        out.append("```text")
        out.append(c.output or "")
        out.append("```")
        out.append("</details>")
    return "\n".join(out)


def _experts_section(trace: MoETrace) -> str:
    """渲染各专家意见、检索情况、结论与风险。"""
    if not trace.expert_opinions:
        return "_（未激活专家）_"
    out: List[str] = []
    for o in trace.expert_opinions:
        out.append(f"#### {o.get('name_zh')} （weight={o.get('weight')}, confidence={o.get('confidence')}）")
        out.append(f"- RAG 命中: {o.get('rag_hits')}，最高分: {o.get('rag_best_score')}")
        tu = o.get("tools_used") or []
        if tu:
            out.append(f"- 调用工具: {', '.join(tu)}")
        required = o.get("required_tools") or []
        recommended = o.get("recommended_tools") or []
        attempted = o.get("attempted_tools") or []
        pending = o.get("pending_tools") or []
        if o.get("retrieval_required"):
            out.append(f"- 必需检索: {', '.join(required) or '工具不可用'}")
            out.append(f"- 已尝试检索: {', '.join(attempted) or '无'}")
            out.append(f"- 尚待检索: {', '.join(pending) or '无'}")
        if recommended:
            out.append(f"- 建议检索（不阻止提交）: {', '.join(recommended)}")
        out.append(f"- **结论**: {o.get('conclusion')}")
        ev = o.get("evidence") or []
        if ev:
            out.append("- **依据**:")
            out.extend(f"  - {x}" for x in ev)
        rk = o.get("risks") or []
        if rk:
            out.append("- **风险提示**:")
            out.extend(f"  - {x}" for x in rk)
        out.append("")
    return "\n".join(out)


def _rag_section(trace: MoETrace) -> str:
    """渲染 RAG 检索次数、分数与查询文本。"""
    if not trace.rag_calls:
        return "_（无 RAG 检索）_"
    out = ["| # | 阶段 | 命中数 | 最高分 | 耗时(ms) | 查询 |", "| --- | --- | --- | --- | --- | --- |"]
    for r in trace.rag_calls:
        out.append(f"| {r.seq} | {_esc_cell(r.stage)} | {r.hits_count} | {r.best_score} | {r.latency_ms} | {_esc_cell(r.query)} |")
    return "\n".join(out)


def _tools_section(trace: MoETrace) -> str:
    """渲染工具调用时序及失败详情。"""
    if not getattr(trace, "tool_calls", None):
        return "_（无工具调用）_"
    out = [
        "| # | 阶段 | 工具 | 成功 | 耗时(ms) | 参数摘要 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for t in trace.tool_calls:
        ok = "✅" if t.ok else "❌"
        out.append(
            f"| {t.seq} | {_esc_cell(t.stage)} | {_esc_cell(t.tool_name)} | {ok} | "
            f"{t.latency_ms} | {_esc_cell(t.arguments)} |"
        )
    fails = [t for t in trace.tool_calls if not t.ok and t.error]
    if fails:
        out.append("")
        out.append("失败详情：")
        out.extend(f"- #{t.seq} {_esc_cell(t.tool_name)}: {_esc_cell(t.error)}" for t in fails)
    return "\n".join(out)


def _critic_section(trace: MoETrace) -> str:
    """渲染审核器裁决、问题与约束。"""
    c = trace.critic_result
    if not c:
        return "_（无审核记录）_"
    out = [f"- 裁决(verdict): **{c.get('verdict')}**", f"- 理由: {c.get('reason') or ''}"]
    iss = c.get("issues") or []
    if iss:
        out.append("- 问题:")
        out.extend(f"  - {x}" for x in iss)
    cons = c.get("constraints") or []
    if cons:
        out.append("- 下达约束:")
        out.extend(f"  - {x}" for x in cons)
    return "\n".join(out)


def _summary_section(trace: MoETrace) -> str:
    """汇总 LLM/RAG/工具次数、耗时与 token。"""
    stage_ms: Dict[str, float] = {}
    for c in trace.llm_calls:
        stage_ms[c.stage] = round(stage_ms.get(c.stage, 0.0) + c.latency_ms, 1)
    rows = [
        "| 指标 | 值 |", "| --- | --- |",
        f"| 总 LLM 调用次数 | {trace.total_llm_calls()} |",
        f"| 总 RAG 检索次数 | {len(trace.rag_calls)} |",
        f"| 总工具调用次数 | {len(getattr(trace, 'tool_calls', []))} |",
        f"| 总耗时(ms) | {trace.total_ms} |",
        f"| 总 token | {trace.total_tokens()} (prompt {trace.total_prompt_tokens()} / completion {trace.total_completion_tokens()}) |",
        f"| Prompt Cache | hit {trace.total_prompt_cache_hit_tokens()} / miss {trace.total_prompt_cache_miss_tokens()} / rate {trace.prompt_cache_hit_rate():.2%} |",
        f"| 触发拒答 | {trace.out_of_scope} |",
        f"| 触发安全兜底(block) | {trace.blocked} |",
    ]
    rows.append("")
    rows.append("各阶段累计 LLM 耗时：")
    rows.append("")
    rows.append("| 阶段 | 累计耗时(ms) |")
    rows.append("| --- | --- |")
    for k, v in stage_ms.items():
        rows.append(f"| {_esc_cell(k)} | {v} |")
    return "\n".join(rows)


def render(trace: MoETrace) -> str:
    """把 MoETrace 拼成完整的 Markdown 评测报告。"""
    parts: List[str] = []
    parts.append("# MoE 评测报告")
    parts.append("")
    parts.append(f"- 生成时间: {datetime.now().isoformat(timespec='seconds')}")
    parts.append(f"- user_role: {trace.user_role}")
    parts.append("")
    parts.append("## 运行配置（超参）")
    parts.append(_config_table(trace.config))
    parts.append("")
    parts.append("## 输入问题")
    parts.append(f"> {trace.question}")
    parts.append("")
    parts.append("## 医生端意图分类")
    parts.append(_intent_section(trace))
    parts.append("")
    parts.append("## 路由决策")
    parts.append(_router_section(trace))
    parts.append("")
    parts.append("## LLM 调用顺序")
    parts.append(_llm_calls_section(trace))
    parts.append("")
    parts.append("## 专家意见")
    parts.append(_experts_section(trace))
    parts.append("")
    parts.append("## RAG 检索指标")
    parts.append(_rag_section(trace))
    parts.append("")
    parts.append("## 工具调用时序")
    parts.append(_tools_section(trace))
    parts.append("")
    parts.append("## Critic 裁决")
    parts.append(_critic_section(trace))
    parts.append("")
    parts.append("## 最终答案")
    parts.append("")
    parts.append(trace.final_answer or "_（无）_")
    parts.append("")
    parts.append("## 汇总指标")
    parts.append(_summary_section(trace))
    parts.append("")
    return "\n".join(parts)
