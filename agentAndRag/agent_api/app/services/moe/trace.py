"""MoE 执行链路的可观测性记录器。

`MoETrace` 在一次问答中按时间顺序记录：
- 每次 LLM 调用（阶段、模型、输入消息、输出全文、耗时、token 用量）；
- 每次 RAG 检索（阶段、查询、命中数、最高分、耗时）；
- 路由决策、专家意见、Critic 裁决与最终答案。

生产链路（SSE）默认不传入 recorder（None），仅评测脚本传入以产出 markdown 报告。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


def extract_usage(resp: Dict[str, Any]) -> Dict[str, int]:
    """从 OpenAI 兼容响应中提取 token 用量（DeepSeek/OpenAI 均返回 `usage`）。"""
    usage = (resp or {}).get("usage") or {}
    return {
        "prompt_tokens": int(usage.get("prompt_tokens") or 0),
        "completion_tokens": int(usage.get("completion_tokens") or 0),
        "total_tokens": int(usage.get("total_tokens") or 0),
    }


@dataclass
class LLMCallRecord:
    seq: int
    stage: str
    model: str
    messages: List[Dict[str, Any]]
    output: str
    latency_ms: float
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0
    meta: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RagCallRecord:
    seq: int
    stage: str
    query: str
    hits_count: int
    best_score: float
    latency_ms: float


@dataclass
class ToolCallRecord:
    seq: int
    stage: str
    tool_name: str
    arguments: str
    ok: bool
    latency_ms: float
    error: str = ""


@dataclass
class MoETrace:
    """单次 MoE 运行的全量追踪。"""

    question: str = ""
    user_role: str = "pet_owner"
    config: Dict[str, Any] = field(default_factory=dict)

    llm_calls: List[LLMCallRecord] = field(default_factory=list)
    rag_calls: List[RagCallRecord] = field(default_factory=list)
    tool_calls: List[ToolCallRecord] = field(default_factory=list)

    router_decision: Optional[Dict[str, Any]] = None
    expert_opinions: List[Dict[str, Any]] = field(default_factory=list)
    critic_result: Optional[Dict[str, Any]] = None

    final_answer: str = ""
    out_of_scope: bool = False
    blocked: bool = False

    _started_at: float = field(default_factory=time.perf_counter)
    total_ms: float = 0.0
    _seq: int = 0

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def record_llm(
        self,
        *,
        stage: str,
        model: str,
        messages: List[Dict[str, Any]],
        output: str,
        latency_ms: float,
        usage: Optional[Dict[str, int]] = None,
        meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        usage = usage or {}
        self.llm_calls.append(
            LLMCallRecord(
                seq=self._next_seq(),
                stage=stage,
                model=model,
                messages=list(messages or []),
                output=output or "",
                latency_ms=round(float(latency_ms), 1),
                prompt_tokens=int(usage.get("prompt_tokens") or 0),
                completion_tokens=int(usage.get("completion_tokens") or 0),
                total_tokens=int(usage.get("total_tokens") or 0),
                meta=dict(meta or {}),
            )
        )

    def record_rag(
        self,
        *,
        stage: str,
        query: str,
        hits_count: int,
        best_score: float,
        latency_ms: float,
    ) -> None:
        self.rag_calls.append(
            RagCallRecord(
                seq=self._next_seq(),
                stage=stage,
                query=query or "",
                hits_count=int(hits_count),
                best_score=round(float(best_score), 4),
                latency_ms=round(float(latency_ms), 1),
            )
        )

    def record_tool(
        self,
        *,
        stage: str,
        tool_name: str,
        arguments: Any,
        ok: bool,
        latency_ms: float,
        error: Optional[str] = None,
    ) -> None:
        try:
            arg_s = json.dumps(arguments, ensure_ascii=False)[:300]
        except Exception:  # noqa: BLE001
            arg_s = str(arguments)[:300]
        self.tool_calls.append(
            ToolCallRecord(
                seq=self._next_seq(),
                stage=stage,
                tool_name=tool_name or "",
                arguments=arg_s,
                ok=bool(ok),
                latency_ms=round(float(latency_ms), 1),
                error=str(error or ""),
            )
        )

    def finalize(self) -> None:
        self.total_ms = round((time.perf_counter() - self._started_at) * 1000, 1)

    # --- aggregate metrics (for reports) ---
    def total_llm_calls(self) -> int:
        return len(self.llm_calls)

    def total_tokens(self) -> int:
        return sum(c.total_tokens for c in self.llm_calls)

    def total_prompt_tokens(self) -> int:
        return sum(c.prompt_tokens for c in self.llm_calls)

    def total_completion_tokens(self) -> int:
        return sum(c.completion_tokens for c in self.llm_calls)
