from __future__ import annotations

import json
from pathlib import Path

import pytest

from RAG.simple_rag.category_index import (
    clear_taxonomy_cache,
    normalize_categories,
    resolve_category_index_dirs,
)
from RAG.simple_rag.config import default_config


pytestmark = pytest.mark.unit


def test_normalize_categories() -> None:
    assert normalize_categories(None) == []
    assert normalize_categories(" clinical.*, pharmacy.papich ") == ["clinical.*", "pharmacy.papich"]
    assert normalize_categories(["a", "", " b "]) == ["a", "b"]


def test_taxonomy_exact_wildcard_unknown_and_cache(tmp_path: Path) -> None:
    taxonomy = tmp_path / "taxonomy.json"
    root = tmp_path / "indexes"
    taxonomy.write_text(
        json.dumps(
            {
                "categories": [
                    {"id": "clinical.cardiology", "index_dir": "indexes/clinical.cardiology"},
                    {"id": "clinical.surgery", "index_dir": "indexes/clinical.surgery"},
                ]
            }
        ),
        encoding="utf-8",
    )
    clear_taxonomy_cache()
    dirs = resolve_category_index_dirs(
        repo_root=tmp_path,
        category="clinical.*",
        taxonomy_path=taxonomy,
        category_root=root,
    )
    assert [path.name for path in dirs] == ["clinical.cardiology", "clinical.surgery"]
    unknown = resolve_category_index_dirs(
        repo_root=tmp_path,
        category="custom.future",
        taxonomy_path=taxonomy,
        category_root=root,
    )
    assert unknown == [root / "custom.future"]


def test_default_config_points_to_current_full_index(tmp_path: Path) -> None:
    cfg = default_config(tmp_path)
    assert cfg.raw_dir == tmp_path / "RAG" / "data" / "raw"
    assert cfg.index_dir == tmp_path / "RAG" / "data" / "rag_index_e5"
