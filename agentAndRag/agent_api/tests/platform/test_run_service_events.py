from agent_api.app.platform.run_service import _public_expert_trace


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
            "arguments": {"query": "secret query"},
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
    serialized = str(public)
    assert "secret system prompt" not in serialized
    assert "raw retrieved passage" not in serialized
    assert "secret query" not in serialized

