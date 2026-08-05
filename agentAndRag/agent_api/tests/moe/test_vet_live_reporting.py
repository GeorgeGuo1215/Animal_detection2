from __future__ import annotations

from agent_api.tests.moe.run_moe_vet_live import (
    _render_web_search_section,
    _web_search_records,
)


def test_web_search_results_are_extracted_and_rendered_with_urls():
    events = [{
        "agent_status": "expert_complete",
        "agent_detail": {
            "expert": "clinical",
            "opinion": {
                "expert": "clinical",
                "name_zh": "兽医临床专家",
                "tool_results": [{
                    "tool_name": "mcp.web_search.web_search",
                    "arguments": {"query": "current feline guideline"},
                    "ok": True,
                    "latency_ms": 123.4,
                    "error": "",
                    "result": {
                        "results": [{
                            "title": "Guideline",
                            "url": "https://example.com/guideline",
                            "content": "Current evidence summary.",
                        }]
                    },
                }],
            },
        },
    }]

    records = _web_search_records(events)
    rendered = "\n".join(_render_web_search_section(records))

    assert records[0]["arguments"]["query"] == "current feline guideline"
    assert "https://example.com/guideline" in rendered
    assert "Current evidence summary" in rendered
