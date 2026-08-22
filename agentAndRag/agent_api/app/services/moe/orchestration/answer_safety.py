"""MoE 终答预算、结束原因、证据引用与保守修复规则。"""

from __future__ import annotations

import os
import re
from typing import Any, Dict, List, Optional, Tuple

_BLOCK_FALLBACK_TEXT_OWNER = (
    "出于安全考虑，我不能直接给出该建议。你描述的情况可能涉及较高风险，"
    "强烈建议尽快联系或前往专业兽医进行线下评估与处理。\n\n"
    "**免责声明**：以上内容仅供健康管理参考，不能替代执业兽医的诊断与治疗。"
)

_BLOCK_FALLBACK_TEXT_VET = (
    "出于安全边界考虑，当前草案触及不可自动放行的用药/处置红线，"
    "不宜在此直接给出可执行方案。\n\n"
    "请结合体格检查、实验室与影像结果，按院内急症流程再评估；"
    "若涉及高风险药物，请核对物种特异性毒性、剂量与监测后再处方。\n\n"
    "**说明**：本系统以 AI 助手身份提供临床参考，不能替代现场诊疗决策。"
)

_DEFAULT_FINAL_ANSWER_MAX_TOKENS = 2500

_HIGH_RISK_SPECIFIC_RE = re.compile(
    r"(?ix)(?:"
    r"\b\d+(?:\.\d+)?\s*(?:mg|mcg|µg|μg|g|ml|mL|IU|U)\s*(?:/\s*(?:kg|day|d|h))?"
    r"|\bq\s*\d+(?:[-–]\d+)?\s*h\b"
    r"|(?:洗脱|减量|剂量|阈值|复查|监测|给药).{0,48}?\d+(?:\.\d+)?(?:\s*[-–]\s*\d+(?:\.\d+)?)?\s*(?:天|日|周|月|小时|h)"
    r")"
)

_EXPLICIT_NO_FIXED_RE = re.compile(
    r"(?:请勿|不要|不得|避免|不应|无需).{0,12}(?:猜|给出|提供|编造|使用)?.{0,12}(?:固定|具体|精确).{0,8}(?:剂量|数值|天数|时长|时间|阈值|频率)",
    flags=re.IGNORECASE,
)
_HIGH_RISK_VALUE_RE = re.compile(
    r"(?ix)(?:"
    r"\b\d+(?:\.\d+)?\s*(?:mg|mcg|µg|μg|g|ml|mL|IU|U)\s*(?:/\s*(?:kg|day|d|h))?"
    r"|\bq\s*\d+(?:[-–]\d+)?\s*h\b"
    r"|\d+(?:\.\d+)?(?:\s*[-–—]\s*\d+(?:\.\d+)?)?\s*(?:天|日|周|月|小时|h)"
    r")"
)
_HIGH_RISK_CONTEXT_RE = re.compile(
    r"洗脱|减量|剂量|阈值|复查|监测|给药|联用|合用|同用|切换|停药|停用|禁忌|间隔|频率",
    flags=re.IGNORECASE,
)


def final_answer_max_tokens() -> int:
    """MoE 终答 token 预算：既作为默认值，也作为显式请求的封顶值。

    经 `MOE_FINAL_ANSWER_MAX_TOKENS` 覆盖，是该预算的唯一来源；路由层与编排器都
    必须取自此处，避免同一数字在多个文件里各写一份。
    """
    raw = (os.getenv("MOE_FINAL_ANSWER_MAX_TOKENS") or "").strip()
    if not raw:
        return _DEFAULT_FINAL_ANSWER_MAX_TOKENS
    try:
        return max(1, int(raw))
    except ValueError:
        return _DEFAULT_FINAL_ANSWER_MAX_TOKENS


def normalize_finish_reason(reason: Optional[str]) -> str:
    """将上游 finish_reason 规范为 stop/truncated 等。"""
    normalized = str(reason or "stop").strip().lower()
    if normalized in {"length", "max_tokens", "token_limit", "truncated"}:
        return "truncated"
    if normalized in {"stop", "tool_calls", "content_filter"}:
        return normalized
    return "stop"


def _response_finish_reason(response: Dict[str, Any]) -> str:
    """从 LLM 响应提取 finish_reason。"""
    choices = response.get("choices") if isinstance(response, dict) else None
    choice = choices[0] if isinstance(choices, list) and choices else {}
    reason = choice.get("finish_reason") if isinstance(choice, dict) else None
    return normalize_finish_reason(reason)


def _block_fallback_text(user_role: str) -> str:
    """按用户角色返回安全兜底话术。"""
    if user_role == "veterinarian":
        return _BLOCK_FALLBACK_TEXT_VET
    return _BLOCK_FALLBACK_TEXT_OWNER


def _collect_retrieved_sources(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从成功的 RAG/Web 工具结果提取、去重并编号可引用来源。

    仅保留可公开的标题、路径、URL、页码和截断摘录；相同来源不会因被多个
    专家复用而重复进入最终引用列表。
    """
    sources: List[Dict[str, Any]] = []
    seen = set()
    counts = {"rag": 0, "web": 0}

    for opinion in opinions or []:
        for tool_result in opinion.get("tool_results") or []:
            if not isinstance(tool_result, dict) or not tool_result.get("ok"):
                continue
            tool_name = str(tool_result.get("tool_name") or "")
            result = tool_result.get("result")
            if not isinstance(result, dict):
                continue
            if tool_name == "rag.search":
                items = result.get("hits") or []
                source_type = "rag"
                prefix = "R"
                sufficiency = (
                    tool_result.get("sufficiency")
                    if isinstance(tool_result.get("sufficiency"), dict)
                    else None
                )
                evidence_status = str(
                    (sufficiency or {}).get("status") or "unassessed"
                ).strip().lower()
                # A successful tool call is not automatically usable evidence.
                # Explicitly weak/unknown results are hidden from the citation
                # catalogue so the aggregator cannot accidentally cite them.
                if evidence_status in {"unsupported", "unknown"}:
                    continue
                matched_indexes: Optional[set[int]] = None
                if evidence_status == "partial":
                    matched_indexes = set()
                    for hit_id in (sufficiency or {}).get("matched_hit_ids") or []:
                        value = str(hit_id).strip().lower()
                        if value.startswith("h") and value[1:].isdigit():
                            matched_indexes.add(int(value[1:]) - 1)
            elif tool_name.startswith("mcp.web_search"):
                items = result.get("results") or result.get("hits") or []
                source_type = "web"
                prefix = "W"
                sufficiency = (
                    tool_result.get("sufficiency")
                    if isinstance(tool_result.get("sufficiency"), dict)
                    else None
                )
                evidence_status = str(
                    (sufficiency or {}).get("status") or "unknown"
                ).strip().lower()
                if evidence_status in {"unsupported", "unknown"}:
                    continue
                matched_indexes = None
                if evidence_status == "partial":
                    matched_indexes = set()
                    for hit_id in (sufficiency or {}).get("matched_hit_ids") or []:
                        value = str(hit_id).strip().lower()
                        if value.startswith("h") and value[1:].isdigit():
                            matched_indexes.add(int(value[1:]) - 1)
            else:
                continue
            if not isinstance(items, list):
                continue

            for item_index, item in enumerate(items):
                if not isinstance(item, dict):
                    continue
                if matched_indexes is not None and item_index not in matched_indexes:
                    continue
                metadata = item.get("metadata") if isinstance(item.get("metadata"), dict) else {}
                source_path = str(item.get("source_path") or metadata.get("source_path") or "").strip()
                title = str(item.get("title") or metadata.get("title") or source_path).strip()
                url = str(item.get("url") or metadata.get("url") or "").strip()
                page = item.get("page") or metadata.get("page") or metadata.get("page_number")
                excerpt = str(item.get("text") or item.get("content") or item.get("snippet") or "").strip()
                key = (source_type, source_path, title, url, str(page or ""), excerpt[:1200])
                if key in seen or not any((source_path, title, url, excerpt)):
                    continue
                seen.add(key)
                counts[source_type] += 1
                source = {
                    "id": f"{prefix}{counts[source_type]}",
                    "type": source_type,
                    "title": title,
                    "source_path": source_path,
                    "url": url,
                    "excerpt": excerpt[:1800],
                    "evidence_status": evidence_status,
                }
                if page not in (None, ""):
                    source["page"] = page
                sources.append(source)
    return sources


def _synthesis_opinions(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """为融合阶段整理专家意见。"""
    fields = (
        "expert", "name_zh", "weight", "conclusion", "evidence", "risks", "confidence",
        "required_tools", "attempted_tools", "successful_tools", "pending_tools",
        "unavailable_required_tools", "evidence_sufficiency",
    )
    return [{field: opinion.get(field) for field in fields} for opinion in opinions or []]


def _evidence_audit_summary(opinions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """按证据目标合并多波审计，供终答作唯一的最终充分性判断。"""
    grouped: Dict[Tuple[str, str], Dict[str, Any]] = {}
    status_rank = {"unsupported": 0, "unknown": 1, "partial": 2, "supported": 3}
    for opinion in opinions or []:
        expert = str(opinion.get("expert") or "")
        for result in opinion.get("tool_results") or []:
            if not isinstance(result, dict):
                continue
            sufficiency = result.get("sufficiency")
            if not isinstance(sufficiency, dict):
                continue
            query = str((result.get("arguments") or {}).get("query") or "")[:500]
            goal = str(result.get("evidence_goal") or query)[:500]
            status = str(sufficiency.get("status") or "unknown").strip().lower()
            if status not in status_rank:
                status = "unknown"
            key = (expert, goal)
            item = grouped.setdefault(key, {
                "expert": expert,
                "goal": goal,
                "status": "unsupported",
                "reason": "",
                "matched_hit_ids": [],
                "attempts": [],
            })
            attempt = {
                "tool_name": str(result.get("tool_name") or ""),
                "query": query,
                "scope": str(result.get("scope") or "expert"),
                "status": status,
                "reason": str(sufficiency.get("reason") or "")[:500],
            }
            item["attempts"].append(attempt)
            if status_rank[status] >= status_rank.get(str(item["status"]), 0):
                item["status"] = status
                item["reason"] = attempt["reason"]
                item["matched_hit_ids"] = list(sufficiency.get("matched_hit_ids") or [])
    return list(grouped.values())


def _opinions_used_web(opinions: List[Dict[str, Any]]) -> bool:
    """判断意见是否使用了网络搜索。"""
    if any(source.get("type") == "web" for source in _collect_retrieved_sources(opinions)):
        return True
    return any(
        str(tool_name).startswith("mcp.web_search")
        for opinion in opinions or []
        for tool_name in opinion.get("tools_used") or []
    )


def _answer_requires_evidence_repair(
    answer: str,
    evidence_audit: List[Dict[str, Any]],
) -> bool:
    """仅当存在弱证据目标且草案含高风险具体数值时触发一次安全编辑。"""
    weak = any(
        str(item.get("status") or "unknown").lower() in {"partial", "unsupported", "unknown"}
        for item in evidence_audit
        if isinstance(item, dict)
    )
    return weak and bool(_HIGH_RISK_SPECIFIC_RE.search(str(answer or "")))


def _explicit_forbidden_claims(query: str, answer: str) -> List[str]:
    """提取用户明确禁止、且不是病例原始数据的新增高风险数值片段。"""
    if not _EXPLICIT_NO_FIXED_RE.search(str(query or "")):
        return []
    query_values = {
        re.sub(r"\s+", "", match.group(0)).lower()
        for match in _HIGH_RISK_VALUE_RE.finditer(str(query or ""))
    }
    claims: List[str] = []
    for line in re.split(r"(?<=[。！？；;])|\n", str(answer or "")):
        if not _HIGH_RISK_CONTEXT_RE.search(line):
            continue
        for match in _HIGH_RISK_VALUE_RE.finditer(line):
            normalized = re.sub(r"\s+", "", match.group(0)).lower()
            if normalized in query_values:
                continue
            snippet = line.strip()
            if snippet and snippet not in claims:
                claims.append(snippet[:500])
    return claims


def _remove_forbidden_claims(answer: str, forbidden_claims: List[str]) -> str:
    """在安全编辑仍保留明确禁用值时，删除对应句并给出保守占位说明。"""
    forbidden_values = {
        re.sub(r"\s+", "", match.group(0)).lower()
        for claim in forbidden_claims
        for match in _HIGH_RISK_VALUE_RE.finditer(claim)
    }
    if not forbidden_values:
        return answer
    replacement = "相关具体剂量、固定时长或数值阈值缺乏充分证据，需结合患者资料和可靠依据个体化核定。"
    # 仅在句末标点后切分；换行本身留在后续片段中，避免安全删句把
    # Markdown 标题、列表与段落压成一行。
    parts = re.split(r"(?<=[。！？；;])", str(answer or ""))
    revised: List[str] = []
    replaced = False
    for part in parts:
        values = {
            re.sub(r"\s+", "", match.group(0)).lower()
            for match in _HIGH_RISK_VALUE_RE.finditer(part)
        }
        if values & forbidden_values and _HIGH_RISK_CONTEXT_RE.search(part):
            last_newline = part.rfind("\n")
            if last_newline >= 0:
                revised.append(part[:last_newline + 1])
            if not replaced:
                revised.append(replacement)
                replaced = True
            continue
        revised.append(part)
    return "".join(revised).strip()


def _strip_markdown_envelope(text: str) -> str:
    """仅移除包住整篇回答的 Markdown 围栏。"""
    value = str(text or "").strip()
    match = re.fullmatch(r"```(?:markdown|md)?\s*\n([\s\S]*?)\n```", value, flags=re.IGNORECASE)
    return match.group(1).strip() if match else value


# 向后兼容别名（宠主文案）
_BLOCK_FALLBACK_TEXT = _BLOCK_FALLBACK_TEXT_OWNER
