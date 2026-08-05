"""离线接线自测：用假 LLM + 假 rag.search 端到端验证 MoE 流程，不触网、不加载索引。

直接运行即可（无需 API key / RAG 索引）：
    python agentAndRag/agent_api/tests/moe/_selftest_offline.py
覆盖：on-topic 正常会诊+审核+融合，off-topic 命中拒答。
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

_THIS = Path(__file__).resolve()
for _p in (str(_THIS.parents[2]), str(_THIS.parents[3])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from app.services.moe import MoEOrchestrator, OrchestratorConfig, RouterConfig, MoETrace
from app.services.moe.orchestrator import _OUT_OF_SCOPE_TEXT
from app.tools.tool_registry import ToolRegistry, ToolSpec
from report_writer import render


def _resp(content: str):
    return {"choices": [{"message": {"content": content}}],
            "usage": {"prompt_tokens": 10, "completion_tokens": 20, "total_tokens": 30}}


class FakeLLM:
    model = "fake-model"

    def __init__(self, *, on_topic=True):
        self.on_topic = on_topic

    async def chat(self, *, messages, temperature=0.2, max_tokens=768, response_format=None):
        sys_text = messages[0]["content"]
        if "路由器" in sys_text:
            if self.on_topic:
                return _resp('{"scores": {"clinical": 8, "nutrition": 5, "pharmacy": 1, "behavior": 1}, "emergency": false, "reason": "软便偏临床+营养"}')
            return _resp('{"scores": {"clinical": 1, "nutrition": 0, "pharmacy": 0, "behavior": 1}, "emergency": false, "reason": "与宠物无关"}')
        if "规划器" in sys_text:
            # 专家工具规划：rag.search + 一个 MCP 工具，验证全工具覆盖管线
            return _resp(
                '{"steps": [{"type": "tool", "tool_name": "rag.search", "arguments": {"query": "soft stool"}},'
                ' {"type": "tool", "tool_name": "mcp.web_search.web_search", "arguments": {"query": "cat soft stool"}},'
                ' {"type": "final", "note": "综合作答"}]}'
            )
        if "输出严格 JSON" in sys_text and "conclusion" in sys_text:
            return _resp('{"conclusion": "建议观察并清淡饮食。", "evidence": ["《猫病学》p.10"], "risks": ["持续则就医"], "confidence": 0.7}')
        if "边界审核专家" in sys_text:
            return _resp('{"verdict": "revise", "issues": ["缺免责"], "constraints": ["补充免责并提示就医"], "reason": "需加免责"}')
        # aggregator (run() non-stream)
        return _resp("**结论** 软便多数良性。\n**何时必须就医** 持续超过48小时请就医。\n免责声明：仅供参考。")


def _fake_rag(**kwargs):
    return {"hits": [{"score": 0.82, "source_path": "books/cat.pdf", "text": "soft stool management ..."}]}


def _fake_web_search(**kwargs):
    return {"results": [{"title": "Cat soft stool causes", "url": "https://example.com/x"}]}


def _make_registry():
    reg = ToolRegistry()
    reg.register(ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_fake_rag))
    reg.register(ToolSpec(
        name="mcp.web_search.web_search", description="fake web search",
        input_schema={"type": "object"}, handler=_fake_web_search,
    ))
    return reg


async def _run_on_topic():
    orch = MoEOrchestrator(
        registry=_make_registry(),
        llm=FakeLLM(on_topic=True),
        config=OrchestratorConfig(router=RouterConfig(), user_role="pet_owner"),
    )
    trace = MoETrace(question="我家猫软便", user_role="pet_owner", config={"demo": True})
    answer, _ = await orch.run(query="我家猫软便", recorder=trace)
    assert not trace.out_of_scope, "on-topic 不应拒答"
    assert trace.expert_opinions, "应有专家意见"
    assert trace.critic_result and trace.critic_result["verdict"] == "revise"
    assert "结论" in answer
    # 全工具覆盖：FakeLLM 规划了 rag + web_search；有工具结果与意见即可（不再强制每位专家必有 rag）
    for o in trace.expert_opinions:
        assert o.get("tools_used"), f"{o['expert']} 应至少执行一次工具"
        assert o.get("conclusion"), f"{o['expert']} 应产出结论"
        assert "mcp.web_search.web_search" in o["tools_used"], f"{o['expert']} 应按规划调用 web_search"
    assert trace.tool_calls, "应记录非 rag 工具调用 (record_tool)"
    assert any(t.tool_name == "mcp.web_search.web_search" and t.ok for t in trace.tool_calls)
    md = render(trace)
    assert "MoE 评测报告" in md and "路由决策" in md and "LLM 调用顺序" in md
    assert "工具调用时序" in md
    print("[on_topic] experts:", [o["expert"] for o in trace.expert_opinions],
          "| llm_calls:", trace.total_llm_calls(),
          "| tool_calls:", len(trace.tool_calls),
          "| tools_used:", trace.expert_opinions[0]["tools_used"],
          "| md_len:", len(md))


async def _run_off_topic():
    orch = MoEOrchestrator(
        registry=_make_registry(),
        llm=FakeLLM(on_topic=False),
        config=OrchestratorConfig(router=RouterConfig(), user_role="pet_owner"),
    )
    trace = MoETrace(question="帮我写段Python", user_role="pet_owner", config={"demo": True})
    answer, _ = await orch.run(query="帮我写段Python", recorder=trace)
    assert trace.out_of_scope, "off-topic 应拒答"
    assert answer == _OUT_OF_SCOPE_TEXT
    assert not trace.expert_opinions, "拒答不应激活专家"
    print("[off_topic] out_of_scope OK | llm_calls:", trace.total_llm_calls())


async def main():
    await _run_on_topic()
    await _run_off_topic()
    print("ALL_OK")


if __name__ == "__main__":
    asyncio.run(main())
