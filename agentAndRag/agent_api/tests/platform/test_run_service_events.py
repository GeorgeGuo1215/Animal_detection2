from agent_api.app.platform.runs.service import (
    _public_expert_trace,
    _trace_nodes_for_agent_event,
)


def test_public_expert_trace_is_useful_but_does_not_expose_prompts_or_raw_payloads():
    """验证公开专家追踪有用，但不会暴露提示词或原始载荷。"""
    raw = {
        "expert": "clinical",
        "name_zh": "兽医临床专家",
        "retrieval_reason": "核对泌尿急症资料",
        "required_tools": ["rag.search"],
        "recommended_tools": [],
        "conclusion": "优先排查尿道梗阻",
        "evidence": ["频繁蹲盆"],
        "risks": ["完全尿闭需要升级处置"],
        "confidence": 0.82,
        "tool_results": [{
            "tool_name": "rag.search",
            "ok": True,
            "latency_ms": 123.4,
            "arguments": {"query": "feline urethral obstruction triage"},
            "evidence_goal": "核对猫尿道梗阻急症分诊证据",
            "round": 2,
            "scope": "expanded",
            "sufficiency": {
                "status": "partial",
                "reason": "只支持急症升级，不支持具体处置剂量",
            },
            "result": {
                "hits": [{
                    "score": 0.9,
                    "source_path": "clinical/book.mmd",
                    "text": "raw retrieved passage must stay private",
                }],
            },
        }],
        "messages": [{"role": "system", "content": "secret system prompt"}],
    }

    public = _public_expert_trace(raw)

    assert public["execution"] == "single_pass"
    assert public["opinion"]["conclusion"] == "优先排查尿道梗阻"
    assert public["tools"][0]["result"] == {
        "hits": 1,
        "sources": ["clinical/book.mmd"],
    }
    assert public["tools"][0]["query"] == "feline urethral obstruction triage"
    assert public["tools"][0]["goal"] == "核对猫尿道梗阻急症分诊证据"
    assert public["tools"][0]["wave"] == 2
    assert public["tools"][0]["scope"] == "expanded"
    assert public["tools"][0]["sufficiency"]["status"] == "partial"
    serialized = str(public)
    assert "secret system prompt" not in serialized
    assert "raw retrieved passage" not in serialized


def test_trace_nodes_show_public_queries_without_prompts_or_raw_tool_text():
    """验证前端轨迹展示真实查询与状态，但不携带提示词和检索正文。"""
    public_expert = _public_expert_trace({
        "expert": "pharmacy",
        "name_zh": "兽医药理专家",
        "conclusion": "仅给出条件性建议",
        "evidence": [],
        "risks": [],
        "confidence": 0.6,
        "tool_results": [{
            "tool_name": "rag.search",
            "ok": True,
            "round": 2,
            "scope": "expanded",
            "arguments": {"query": "canine corticosteroid NSAID washout"},
            "result": {"hits": [{
                "source_path": "C:/private/index/books/083.mmd",
                "text": "private retrieved paragraph",
            }]},
            "sufficiency": {
                "status": "unsupported",
                "reason": "没有直接支持固定洗脱天数",
            },
        }],
        "messages": [{"role": "system", "content": "private system prompt"}],
    })

    nodes = _trace_nodes_for_agent_event(
        "expert_complete", {}, public_expert=public_expert,
    )
    query_node = next(node for node in nodes if node["node_type"] == "query")

    assert query_node["details"]["query"] == "canine corticosteroid NSAID washout"
    assert query_node["details"]["scope"] == "expanded"
    assert query_node["details"]["sufficiency"]["status"] == "unsupported"
    assert query_node["details"]["result"]["sources"] == ["books/083.mmd"]
    serialized = str(nodes)
    assert "private retrieved paragraph" not in serialized
    assert "private system prompt" not in serialized


def test_trace_keeps_rag_and_web_nodes_with_the_same_query_separate():
    """验证第二波相同查询文本的扩类 RAG 与 Web 补证不会互相覆盖。"""
    nodes = _trace_nodes_for_agent_event(
        "expert_complete",
        {},
        public_expert={
            "expert": "pharmacy",
            "name": "兽医药理专家",
            "opinion": {"confidence": 0.5},
            "tools": [
                {
                    "tool_name": "rag.search", "query": "canine NSAID washout",
                    "wave": 2, "scope": "expanded", "ok": True,
                },
                {
                    "tool_name": "mcp.web_search.web_search", "query": "canine NSAID washout",
                    "wave": 2, "scope": "expert", "ok": True,
                },
            ],
        },
    )
    query_nodes = [node for node in nodes if node["node_type"] == "query"]
    assert len(query_nodes) == 2
    assert len({node["node_id"] for node in query_nodes}) == 2

