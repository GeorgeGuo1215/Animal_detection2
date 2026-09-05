from __future__ import annotations

import pytest

from RAG.simple_rag.query_rewrite import NoRewrite, TemplateRewriter
from RAG.simple_rag.scoring import overlap_score, tokenize_for_overlap


pytestmark = pytest.mark.unit


def test_overlap_score_boundaries() -> None:
    assert overlap_score("", "anything") == 0.0
    assert overlap_score("feline urinary obstruction", "feline urinary obstruction emergency") == 1.0
    assert overlap_score("heart disease", "unrelated dermatology") == 0.0
    assert 0.0 < overlap_score("猫 排尿 困难", "猫频繁排尿并表现困难") <= 1.0


def test_overlap_tokenization_is_bilingual() -> None:
    tokens = tokenize_for_overlap("The feline 心脏 disease")
    assert "feline" in tokens
    assert "disease" in tokens
    assert "心脏" in tokens


def test_rewriters_handle_empty_long_and_bilingual_queries() -> None:
    assert NoRewrite().rewrite("  \n") == []
    assert NoRewrite().rewrite("first line\n\nignored") == ["first line"]
    rewritten = TemplateRewriter(max_out=4).rewrite("猫心脏疾病")
    assert rewritten[0] == "猫心脏疾病"
    assert any("cardiac" in item for item in rewritten)
    assert len(rewritten) <= 4
