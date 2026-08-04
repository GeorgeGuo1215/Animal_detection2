"""二级分类索引解析：taxonomy + category → index_dir 列表。"""
from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Union

_LOCK = threading.RLock()
_TAXONOMY_CACHE: Dict[str, Dict[str, Any]] = {}


def default_taxonomy_path(repo_root: Path) -> Path:
    return repo_root / "RAG" / "data" / "category_taxonomy.json"


def default_category_root(repo_root: Path) -> Path:
    return repo_root / "RAG" / "data" / "rag_index_e5_by_cat"


def load_taxonomy(taxonomy_path: Path) -> Dict[str, Any]:
    key = str(taxonomy_path.resolve())
    with _LOCK:
        cached = _TAXONOMY_CACHE.get(key)
        if cached is not None:
            return cached
        data = json.loads(taxonomy_path.read_text(encoding="utf-8"))
        _TAXONOMY_CACHE[key] = data
        return data


def clear_taxonomy_cache() -> None:
    with _LOCK:
        _TAXONOMY_CACHE.clear()


def _expand_patterns(patterns: Sequence[str], all_ids: Sequence[str]) -> List[str]:
    """Support exact id or prefix* / prefix.* wildcards."""
    out: List[str] = []
    seen = set()
    id_set = list(all_ids)
    for pat in patterns:
        p = (pat or "").strip()
        if not p:
            continue
        if p.endswith(".*"):
            prefix = p[:-2]
            matched = [cid for cid in id_set if cid == prefix or cid.startswith(prefix + ".")]
        elif p.endswith("*"):
            prefix = p[:-1]
            matched = [cid for cid in id_set if cid.startswith(prefix)]
        else:
            matched = [p] if p in id_set else []
            if not matched:
                # allow unknown exact ids later (dir may still exist)
                matched = [p]
        for cid in matched:
            if cid not in seen:
                seen.add(cid)
                out.append(cid)
    return out


def normalize_categories(category: Optional[Union[str, Sequence[str]]]) -> List[str]:
    if category is None:
        return []
    if isinstance(category, str):
        s = category.strip()
        if not s:
            return []
        # allow comma-separated
        if "," in s:
            return [x.strip() for x in s.split(",") if x.strip()]
        return [s]
    return [str(x).strip() for x in category if str(x).strip()]


def resolve_category_index_dirs(
    *,
    repo_root: Path,
    category: Optional[Union[str, Sequence[str]]],
    taxonomy_path: Optional[Path] = None,
    category_root: Optional[Path] = None,
) -> List[Path]:
    """Resolve category patterns to existing/placeholder index directories.

    Empty category list means caller should use the full default index.
    """
    cats = normalize_categories(category)
    if not cats:
        return []

    tax_path = taxonomy_path or default_taxonomy_path(repo_root)
    root = category_root or default_category_root(repo_root)
    all_ids: List[str] = []
    id_to_dir: Dict[str, Path] = {}
    if tax_path.exists():
        tax = load_taxonomy(tax_path)
        for c in tax.get("categories") or []:
            cid = str(c.get("id") or "")
            if not cid:
                continue
            all_ids.append(cid)
            idx = c.get("index_dir")
            id_to_dir[cid] = Path(idx) if idx else (root / cid)
    else:
        # fallback: glob directories under category_root
        if root.exists():
            all_ids = sorted(p.name for p in root.iterdir() if p.is_dir())
            id_to_dir = {cid: root / cid for cid in all_ids}

    resolved_ids = _expand_patterns(cats, all_ids)
    dirs: List[Path] = []
    for cid in resolved_ids:
        d = id_to_dir.get(cid) or (root / cid)
        dirs.append(d)
    return dirs


def list_nonempty_category_ids(repo_root: Path, taxonomy_path: Optional[Path] = None) -> List[str]:
    tax_path = taxonomy_path or default_taxonomy_path(repo_root)
    if not tax_path.exists():
        return []
    tax = load_taxonomy(tax_path)
    return [str(c["id"]) for c in tax.get("categories") or [] if int(c.get("chunk_count") or 0) > 0]


def expert_category_warmup_ids() -> List[str]:
    """Categories used by the four MoE experts (may include empty placeholders)."""
    return [
        # clinical
        "basic.anatomy",
        "basic.terminology",
        "clinical.internal_medicine",
        "clinical.surgery",
        "clinical.emergency_critical",
        "clinical.cardiology",
        "clinical.dermatology",
        "clinical.ophthalmology",
        "clinical.neurology",
        "clinical.oncology",
        "diagnostics.clinical_pathology",
        "diagnostics.imaging",
        "diagnostics.differential",
        "diagnostics.laboratory",
        "clinical_skills.techniques",
        "clinical_skills.nursing",
        "integrative.general",
        "anesthesia.default",
        "immunology.default",
        "reproduction.default",
        "infectious.placeholder",
        "exotic.default",
        "zoonosis.toxoplasmosis",
        # nutrition
        "nutrition.placeholder",
        "equine.nutrition",
        # pharmacy
        "pharmacy.papich",
        "pharmacy.applied_pharmacology",
        "basic.pharmacology_fundamentals",
        # behavior
        "behavior.dog_cat_problems",
        "behavior.feline_welfare",
    ]
