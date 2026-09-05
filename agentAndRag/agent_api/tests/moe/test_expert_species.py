"""Verify run_expert injects species/breed into the RAG query and the expert payload.

Uses fake registry + fake LLM (no DB / no network). ASCII sentinels only.
Run: pytest tests/moe/test_expert_species.py
"""
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from agent_api.app.services.moe.expert_runtime.service import EXPERTS, run_expert, _SPECIES_BREED_GUARD
from agent_api.app.services.moe.retrieval_policy import RetrievalRequirement


class FakeRegistry:
    def __init__(self):
        """初始化该测试替身。"""
        self.calls = []

    def list_tools(self):
        """列出测试替身所暴露的工具。"""
        from agent_api.app.tools.tool_registry import ToolSpec

        async def _rag(**kwargs):
            """返回测试用的假 RAG 检索结果。"""
            return {"hits": []}

        return [ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_rag)]

    def get(self, name):
        """按键读取测试替身中的值。"""
        return next((tool for tool in self.list_tools() if tool.name == name), None)

    async def call(self, name, args):
        """触发一次测试替身调用。"""
        self.calls.append((name, args))
        return {"hits": []}


class FakeLLM:
    model = "fake"

    def __init__(self):
        """初始化该测试替身。"""
        self.last_messages = None

    async def chat(self, messages=None, **kwargs):
        """测试用假 LLM 聊天实现。"""
        self.last_messages = messages
        content = json.dumps(
            {
                "action": "final",
                "opinion": {"conclusion": "ok", "evidence": [], "risks": [], "confidence": 0.5},
            }
        )
        return {"choices": [{"message": {"content": content}}]}


def _user_content(messages):
    """取出用户消息文本。"""
    return [m for m in messages if m["role"] == "user"][0]["content"]


def _system_content(messages):
    """取出系统提示词文本。"""
    return [m for m in messages if m["role"] == "system"][0]["content"]


def _retrieval(tool="rag.search"):
    """构造或拦截一次检索调用。"""
    return RetrievalRequirement(
        required_tools=(), recommended_tools=(tool,),
        require_web_on_rag_failure=False, reason="test assignment",
    )


def test_run_expert_injects_species():
    """验证跑专家时会注入物种。"""
    reg, llm = FakeRegistry(), FakeLLM()
    res = asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="my pet keeps vomiting",
            weight=0.8,
            registry=reg,
            llm=llm,
            species_en="cat",
            species_zh="ZH_SENTINEL",
            recorder=None,
            retrieval_requirement=_retrieval(),
        )
    )
    assert reg.calls, "rag.search was not called"
    rag_args = reg.calls[0][1]
    assert "cat" in rag_args["query"]

    user_content = _user_content(llm.last_messages)
    assert "ZH_SENTINEL" in user_content
    assert "species" in user_content
    assert res["expert"] == "clinical"


def test_run_expert_injects_breed_and_guard():
    """验证跑专家时会注入品种和守卫约束。"""
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="bulldog exercise plan",
            weight=0.8,
            registry=reg,
            llm=llm,
            species_en="dog",
            species_zh="犬（狗）",
            breed="French Bulldog",
            recorder=None,
            retrieval_requirement=_retrieval(),
        )
    )
    assert "French Bulldog" in reg.calls[0][1]["query"]
    sys_content = _system_content(llm.last_messages)
    assert "物种/品种特异化" in sys_content or _SPECIES_BREED_GUARD[:12] in sys_content
    user_content = _user_content(llm.last_messages)
    assert "French Bulldog" in user_content
    assert "breed" in user_content


def test_run_expert_without_species():
    """验证没有物种时仍能跑专家。"""
    reg, llm = FakeRegistry(), FakeLLM()
    asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="my pet keeps vomiting",
            weight=0.8,
            registry=reg,
            llm=llm,
            recorder=None,
        )
    )
    user_content = _user_content(llm.last_messages)
    assert "species" not in user_content


def test_run_expert_only_executes_tool_assigned_by_task_policy():
    """验证专家只执行任务策略分配给它的工具。"""
    from agent_api.app.tools.tool_registry import ToolSpec

    class Reg(FakeRegistry):
        def list_tools(self):
            """列出测试替身所暴露的工具。"""
            async def _web(**kwargs):
                """拦截或构造网页检索。"""
                return {"results": []}

            async def _rag(**kwargs):
                """返回测试用的假 RAG 检索结果。"""
                return {"hits": []}

            return [
                ToolSpec(name="rag.search", description="fake", input_schema={"type": "object"}, handler=_rag),
                ToolSpec(
                    name="mcp.web_search.web_search",
                    description="fake web",
                    input_schema={"type": "object"},
                    handler=_web,
                ),
            ]

        def get(self, name):
            """按键读取测试替身中的值。"""
            return next((tool for tool in self.list_tools() if tool.name == name), None)

    reg, llm = Reg(), FakeLLM()
    res = asyncio.run(
        run_expert(
            expert=EXPERTS["clinical"],
            query="latest feline UTI guideline",
            weight=0.8,
            registry=reg,
            llm=llm,
            recorder=None,
            retrieval_requirement=_retrieval("mcp.web_search.web_search"),
        )
    )
    assert "mcp.web_search.web_search" in res["tools_used"]
    assert "rag.search" not in res["tools_used"]
