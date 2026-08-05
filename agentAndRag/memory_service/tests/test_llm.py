"""LLM 输出解析的容错测试。

模型返回的东西千奇百怪：带代码围栏、前面加一句解释、干脆不给 JSON。这些都不该
让提升任务失败——失败会触发重试，而重试并不会让模型突然学会输出 JSON，只会把
重试次数白白耗光。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.llm import (  # noqa: E402
    LLMError,
    OpenAICompatClient,
    as_string_list,
    parse_json_object,
    strip_code_fence,
)


# ---------------------------------------------------------------- JSON 解析


def test_parses_plain_json():
    assert parse_json_object('{"a": 1}') == {"a": 1}


def test_strips_markdown_code_fence():
    assert parse_json_object('```json\n{"a": 1}\n```') == {"a": 1}


def test_strips_bare_code_fence():
    assert strip_code_fence("```\nhello\n```") == "hello"


def test_salvages_json_wrapped_in_prose():
    """模型爱在 JSON 前后加一句"好的，这是结果"。"""
    raw = '好的，这是分析结果：\n{"concerns": ["体重"]}\n希望对你有帮助。'
    assert parse_json_object(raw) == {"concerns": ["体重"]}


@pytest.mark.parametrize(
    "raw",
    ["", "   ", "抱歉，我无法完成这个请求。", "[1, 2, 3]", '"just a string"', "{不是合法json}"],
)
def test_unparseable_output_yields_empty_dict_not_exception(raw):
    assert parse_json_object(raw) == {}


def test_nested_structures_survive():
    raw = '{"petFacts": {"咪咪": {"品种": "布偶", "过敏": ["鸡肉"]}}}'
    parsed = parse_json_object(raw)
    assert parsed["petFacts"]["咪咪"]["过敏"] == ["鸡肉"]


# ---------------------------------------------------------------- 列表归一化


def test_list_input_is_cleaned():
    assert as_string_list(["- 事实一", "• 事实二"]) == ["事实一", "事实二"]


def test_newline_string_becomes_list():
    assert as_string_list("第一条\n第二条") == ["第一条", "第二条"]


def test_none_and_empty_yield_empty_list():
    assert as_string_list(None) == []
    assert as_string_list([]) == []
    assert as_string_list("") == []


@pytest.mark.parametrize("placeholder", ["无", "暂无", "none", "N/A", "没有"])
def test_placeholder_answers_are_dropped(placeholder):
    """模型表达"这次没什么可记的"的各种写法，都不该被写进知识库。"""
    assert as_string_list([placeholder]) == []


def test_mixed_content_keeps_only_real_items():
    assert as_string_list(["布偶猫对鸡肉过敏", "无", "  "]) == ["布偶猫对鸡肉过敏"]


def test_scalar_is_wrapped():
    assert as_string_list(42) == ["42"]


# ---------------------------------------------------------------- 客户端


def test_missing_api_key_fails_fast_without_network():
    """没配 key 就别发请求，直接报错比等一个 401 超时强。"""
    client = OpenAICompatClient(base_url="https://example.test/v1", api_key="", model="m")
    with pytest.raises(LLMError, match="api key"):
        client.complete([{"role": "user", "content": "hi"}])


def test_base_url_trailing_slash_is_normalised():
    client = OpenAICompatClient(base_url="https://example.test/v1/", api_key="k", model="m")
    assert client.base_url == "https://example.test/v1"
