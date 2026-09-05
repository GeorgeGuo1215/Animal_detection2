from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

from RAG.simple_rag.reranker import CrossEncoderReranker


pytestmark = pytest.mark.unit


class _Tokenizer:
    def __call__(self, text, pair=None, **kwargs):
        matches = list(re.finditer(r"\S+", text))
        ids = list(range(len(matches)))
        if pair is not None:
            ids += [0] * (len(pair.split()) + 3)
        return {"input_ids": ids, "offset_mapping": [(m.start(), m.end()) for m in matches]}

    def num_special_tokens_to_add(self, pair=False):
        return 3 if pair else 2


class _Predictor:
    tokenizer = _Tokenizer()
    max_length = 512
    def predict(self, pairs, *, batch_size: int, show_progress_bar: bool):
        assert batch_size == 2
        assert show_progress_bar is False
        return [0.1, 0.9]


def test_reranker_sort_empty_and_top_k() -> None:
    reranker = object.__new__(CrossEncoderReranker)
    reranker.model = _Predictor()
    assert reranker.rerank(query="q", passages=[], top_k=2) == []
    ranked = reranker.rerank(query="q", passages=["low", "high"], top_k=1, batch_size=2)
    assert [(item.index, item.score) for item in ranked] == [(1, 0.9)]


def test_pair_windows_preserve_tail_evidence_and_enforce_budget():
    class Predictor(_Predictor):
        max_length = 40

        def predict(self, pairs, **kwargs):
            assert all(len(self.tokenizer(q, p)["input_ids"]) <= self.max_length for q, p in pairs)
            return [1.0 if "tail-evidence" in passage else 0.0 for _, passage in pairs]
    reranker = object.__new__(CrossEncoderReranker)
    reranker.model = Predictor()
    assert reranker.score_pairs(query="case", passages=["word " * 100 + "tail-evidence"]) == [1.0]


def test_production_modules_do_not_import_maintenance_or_benchmarks() -> None:
    rag_root = Path(__file__).resolve().parents[2]
    production_roots = [rag_root / "simple_rag", rag_root.parent / "agent_api" / "app"]
    violations: list[str] = []
    forbidden = ("RAG.maintenance", "RAG.experiments", "RAG.tools")
    for root in production_roots:
        for path in root.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith(forbidden):
                    violations.append(f"{path}:{node.lineno}:{node.module}")
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name.startswith(forbidden):
                            violations.append(f"{path}:{node.lineno}:{alias.name}")
    assert violations == []
