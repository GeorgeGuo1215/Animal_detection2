from __future__ import annotations

import json
import os
import re
import time
from typing import Any, Dict, List, Optional, Tuple

from ..context.request_context import ANIMAL_REQUIRED_TOOLS, get_request_animal_id
from ..llm.llm_client import AsyncOpenAIClient, OpenAICompatClient, extract_text
from ..prompts.plan_and_solve import build_async_planner_prompt, build_sync_planner_prompt
from ..prompts.solve import answer_char_budget, build_solve_prompt_text
from ..tools.tool_registry import ToolRegistry
from ..concurrency import ResourceBusyError

# 具体病情信号：症状 / 时间线 / 既往与处置
_CONDITION_SIGNAL_RE = re.compile(
    r"("
    r"疾病|症状|诊断|治疗|手术|用药|药物|剂量|病例|感染|炎症|肿瘤|癌|骨折|"
    r"呕吐|干呕|腹泻|软便|便秘|发烧|发热|咳嗽|抽搐|中毒|过敏|寄生虫|"
    r"疫苗|免疫|麻醉|驱虫|尿血|血尿|尿频|尿少|排尿|猫砂|蹲很久|尿不出|"
    r"膀胱|结石|食欲|不爱吃|精神|趴着|舔下面|喘气|呼吸|跛行|伤口|"
    r"今天|昨天|早上|昨晚|十几分钟|持续|最近|"
    r"吃药|去医院|就诊|既往|病史|"
    r"disease|symptom|diagnos|treatment|surgery|medication|dose|infection|"
    r"tumor|cancer|fracture|vomit|diarrhea|fever|seizure|poison|parasite|vaccine|"
    r"hematuria|stranguria|dysuria|anorexi|letharg"
    r")",
    re.IGNORECASE,
)

def _is_medical_query(query: str) -> bool:
    """Simple heuristic to detect medical/clinical queries for pets."""
    return bool(_CONDITION_SIGNAL_RE.search(query or ""))


_WEB_SEARCH_TOOL = "mcp.web_search.web_search"
_VITALS_SQL_ONLY = frozenset({"vitals.summary", "sql.search"})


def ensure_rag_and_web_tool_steps(
    tool_steps: List[Dict[str, Any]],
    *,
    visible_tools: List[str],
    query: str,
    rag_query: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """For medical queries, ensure both rag.search and web_search are planned when available.

    Skips when either tool is unavailable, query is non-medical, or the plan is
    purely vitals/sql (personal-data path). Does not truncate; caller may cap steps.
    """
    visible = set(visible_tools or [])
    if "rag.search" not in visible or _WEB_SEARCH_TOOL not in visible:
        return list(tool_steps or [])
    if not _is_medical_query(query):
        return list(tool_steps or [])

    steps = [s for s in (tool_steps or []) if isinstance(s, dict)]
    names = {str(s.get("tool_name") or "") for s in steps}
    if names and names <= _VITALS_SQL_ONLY:
        return steps

    out = list(steps)
    if "rag.search" not in names:
        out.insert(
            0,
            {
                "type": "tool",
                "tool_name": "rag.search",
                "arguments": {"query": (rag_query or query)},
                "note": "paired with web_search for medical synthesis",
            },
        )
    if _WEB_SEARCH_TOOL not in names:
        out.append(
            {
                "type": "tool",
                "tool_name": _WEB_SEARCH_TOOL,
                "arguments": {"query": query, "max_results": 5},
                "note": "paired with rag.search for medical synthesis",
            }
        )
    return out


def build_solve_prompt(
    user_role: str = "pet_owner",
    has_web_search: bool = False,
    query: str = "",
    max_tokens: Optional[int] = None,
) -> str:
    """Build role-aware system prompt for the solve/generation stage.

    `max_tokens` 是该次生成的实际 token 预算；传入后会折算成汉字预算写进提示词，
    让模型主动分配篇幅而不是被硬截断。省略则不附加篇幅约束。
    """
    return build_solve_prompt_text(
        user_role=user_role,
        has_web_search=has_web_search,
        medical_query=_is_medical_query(query),
        max_tokens=max_tokens,
    )


def _safe_json_loads(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    """
    Best-effort JSON extraction for LLM outputs.
    Returns (obj, error_message).
    """
    text = (text or "").strip()
    if not text:
        return None, "empty response"
    try:
        return json.loads(text), ""
    except Exception:
        pass

    # try to extract the first JSON object
    l = text.find("{")
    r = text.rfind("}")
    if l >= 0 and r > l:
        snippet = text[l : r + 1]
        try:
            return json.loads(snippet), ""
        except Exception as e:  # noqa: BLE001
            return None, f"json parse failed: {e}"
    return None, "json object not found"


def _planner_tool_brief(
    registry: ToolRegistry,
    allowed_tools: Optional[List[str]],
) -> List[Dict[str, Any]]:
    allowed = set(allowed_tools) if allowed_tools is not None else None
    return [
        {"name": tool.name, "description": tool.description, "input_schema": tool.input_schema}
        for tool in registry.list_tools()
        if (tool.name != "sql.search" or get_request_animal_id())
        and (allowed is None or tool.name in allowed)
    ]


def _planner_user_payload(query: str, tool_brief: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "query": query,
        "available_tools": tool_brief,
        "output_format": {
            "steps": [
                {
                    "type": "tool",
                    "tool_name": "<tool_name>",
                    "arguments": {"<key>": "<value>"},
                    "note": "为什么调用这个工具",
                },
                {"type": "final", "note": "最后如何组织答案"},
            ]
        },
    }


def _parse_plan_output(text: str) -> List[Dict[str, Any]]:
    obj, err = _safe_json_loads(text)
    if not obj or "steps" not in obj or not isinstance(obj["steps"], list):
        raise RuntimeError(f"Planner output is not valid JSON plan: {err}. raw={text[:800]}")
    return obj["steps"]


def _solve_messages(
    *,
    query: str,
    plan_steps: List[Dict[str, Any]],
    tool_results: List[Dict[str, Any]],
    user_role: str,
    max_tokens: Optional[int] = None,
) -> List[Dict[str, str]]:
    has_web = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    payload = {"query": query, "plan": plan_steps, "tool_results": tool_results}
    return [
        {
            "role": "system",
            "content": build_solve_prompt(
                user_role=user_role, has_web_search=has_web, query=query,
                max_tokens=max_tokens,
            ),
        },
        {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
    ]


class PlanAndSolveAgent:
    def __init__(self, *, registry: ToolRegistry, llm: OpenAICompatClient) -> None:
        self.registry = registry
        self.llm = llm

    @staticmethod
    def _force_rag_search_defaults(arguments: Dict[str, Any]) -> Dict[str, Any]:
        """
        Force RAG quality defaults for agent runs.

        Rationale: in practice, planners may omit critical knobs. For this project we want
        multi-route retrieval + reranking + neighbor expansion enabled by default to reduce
        retrieval misses and improve evidence quality.

        This function enforces:
        - multi_route = True
        - rerank = True
        - expand_neighbors >= 1

        It also fills a few safe defaults when missing.
        """
        args = dict(arguments or {})

        # hard-enforce key behaviors
        args["multi_route"] = True
        args["rerank"] = True
        if int(args.get("expand_neighbors") or 0) < 1:
            args["expand_neighbors"] = 1

        # sensible defaults if missing
        args.setdefault("rewrite", "template")
        args.setdefault("top_k", 5)
        args.setdefault("device", os.getenv("AGENT_WARMUP_DEVICE") or None)

        # ensure we retrieve enough candidates before rerank
        top_k = int(args.get("top_k") or 5)
        args.setdefault("rerank_candidates", max(10, top_k * 2))
        args.setdefault("rerank_batch_size", 32)
        args.setdefault("rerank_filter_overlap", float(os.getenv("RAG_OVERLAP_THRESHOLD", "0.15")))

        return args

    def plan(self, *, query: str, allowed_tools: Optional[List[str]] = None) -> List[Dict[str, Any]]:
        tool_brief = _planner_tool_brief(self.registry, allowed_tools)

        sys = build_sync_planner_prompt()
        user = _planner_user_payload(query, tool_brief)

        resp = self.llm.chat(
            messages=[
                {"role": "system", "content": sys},
                {"role": "user", "content": json.dumps(user, ensure_ascii=False)},
            ],
            temperature=0.1,
            max_tokens=512,
            # if supported, nudges JSON output
            response_format={"type": "json_object"},
        )
        text = extract_text(resp)
        return _parse_plan_output(text)

    def solve(
        self,
        *,
        query: str,
        plan_steps: List[Dict[str, Any]],
        allowed_tools: Optional[List[str]] = None,
        temperature: float = 0.2,
        max_tokens: int = 768,
        user_role: str = "pet_owner",
    ) -> Tuple[str, List[Dict[str, Any]]]:
        allowed = set(allowed_tools) if allowed_tools is not None else None
        tool_results: List[Dict[str, Any]] = []

        for i, step in enumerate(plan_steps):
            stype = step.get("type")
            if stype == "tool":
                tool_name = str(step.get("tool_name") or "")
                if not tool_name:
                    continue
                if allowed is not None and tool_name not in allowed:
                    raise RuntimeError(f"Tool not allowed: {tool_name}")
                args = step.get("arguments") or {}
                if not isinstance(args, dict):
                    args = {}

                if tool_name == "rag.search":
                    args = self._force_rag_search_defaults(args)
                if tool_name == "sql.search" and not get_request_animal_id():
                    raise RuntimeError("sql.search requires request animal_id (JSON animal_id or X-Animal-Id header)")
                result = self.registry.call_sync(tool_name, args)
                tool_results.append({"step": i, "tool_name": tool_name, "arguments": args, "result": result})
            elif stype == "final":
                break
            else:
                continue

        resp = self.llm.chat(
            messages=_solve_messages(
                query=query, plan_steps=plan_steps, tool_results=tool_results, user_role=user_role,
                max_tokens=max_tokens,
            ),
            temperature=temperature,
            max_tokens=max_tokens,
        )
        answer = extract_text(resp)
        return answer, tool_results


async def execute_tool_steps(
    *,
    registry: ToolRegistry,
    plan_steps: List[Dict[str, Any]],
    allowed_tools: Optional[List[str]] = None,
) -> List[Dict[str, Any]]:
    """Execute the `tool` steps of a plan against the registry.

    Shared by AsyncPlanAndSolveAgent.solve and the MoE experts so tool dispatch
    semantics stay identical. Enforces rag.search quality defaults, the animal_id
    requirement for animal-scoped tools (sql.search / vitals.summary) and an optional
    allowed_tools whitelist. A failing tool is captured (ok=False) instead of aborting
    the whole run. Returns one entry per executed tool with result, ok and latency_ms.
    """
    allowed = set(allowed_tools) if allowed_tools is not None else None
    results: List[Dict[str, Any]] = []
    for i, step in enumerate(plan_steps):
        stype = step.get("type")
        if stype == "final":
            break
        if stype != "tool":
            continue
        tool_name = str(step.get("tool_name") or "")
        if not tool_name:
            continue
        if allowed is not None and tool_name not in allowed:
            raise RuntimeError(f"Tool not allowed: {tool_name}")
        args = step.get("arguments") or {}
        if not isinstance(args, dict):
            args = {}
        if tool_name == "rag.search":
            args = PlanAndSolveAgent._force_rag_search_defaults(args)
        if tool_name in ANIMAL_REQUIRED_TOOLS and not get_request_animal_id():
            raise RuntimeError(
                f"{tool_name} requires request animal_id (JSON animal_id or X-Animal-Id header)"
            )
        t0 = time.perf_counter()
        try:
            result = await registry.call(tool_name, args)
            ok, err = True, ""
        except ResourceBusyError as exc:
            result, ok, err = exc.as_dict(), False, str(exc)
        except Exception as exc:  # noqa: BLE001
            result, ok, err = {"error": str(exc)}, False, str(exc)
        results.append(
            {
                "step": i,
                "tool_name": tool_name,
                "arguments": args,
                "result": result,
                "ok": ok,
                "latency_ms": round((time.perf_counter() - t0) * 1000.0, 1),
                "error": err,
            }
        )
    return results


class AsyncPlanAndSolveAgent:
    """Fully async version that uses AsyncOpenAIClient and async registry.call()."""

    def __init__(self, *, registry: ToolRegistry, llm: AsyncOpenAIClient) -> None:
        self.registry = registry
        self.llm = llm

    _force_rag_search_defaults = staticmethod(PlanAndSolveAgent._force_rag_search_defaults)

    async def plan(
        self,
        *,
        query: str,
        allowed_tools: Optional[List[str]] = None,
        recorder: Optional[Any] = None,
        stage: str = "planner",
    ) -> List[Dict[str, Any]]:
        tool_brief = _planner_tool_brief(self.registry, allowed_tools)

        sys = build_async_planner_prompt()
        user = _planner_user_payload(query, tool_brief)

        messages = [
            {"role": "system", "content": sys},
            {"role": "user", "content": json.dumps(user, ensure_ascii=False)},
        ]
        t0 = time.perf_counter()
        resp = await self.llm.chat(
            messages=messages,
            temperature=0.1,
            max_tokens=512,
            response_format={"type": "json_object"},
        )
        text = extract_text(resp)
        if recorder is not None:
            try:
                recorder.record_llm(
                    stage=stage,
                    model=getattr(self.llm, "model", ""),
                    messages=messages,
                    output=text,
                    latency_ms=(time.perf_counter() - t0) * 1000.0,
                    usage=(resp or {}).get("usage"),
                )
            except Exception:  # noqa: BLE001
                pass
        return _parse_plan_output(text)

    async def solve(
        self,
        *,
        query: str,
        plan_steps: List[Dict[str, Any]],
        allowed_tools: Optional[List[str]] = None,
        temperature: float = 0.2,
        max_tokens: int = 768,
        user_role: str = "pet_owner",
    ) -> Tuple[str, List[Dict[str, Any]]]:
        tool_results = await execute_tool_steps(
            registry=self.registry,
            plan_steps=plan_steps,
            allowed_tools=allowed_tools,
        )

        resp = await self.llm.chat(
            messages=_solve_messages(
                query=query, plan_steps=plan_steps, tool_results=tool_results, user_role=user_role,
                max_tokens=max_tokens,
            ),
            temperature=temperature,
            max_tokens=max_tokens,
        )
        answer = extract_text(resp)
        return answer, tool_results
