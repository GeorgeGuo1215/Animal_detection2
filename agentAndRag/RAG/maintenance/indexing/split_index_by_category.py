"""从全量 rag_index_e5 按 Excel 二级分类切出子索引。

用法（conda RAG）:
  python -m RAG.maintenance.indexing.split_index_by_category
  python -m RAG.maintenance.indexing.split_index_by_category --xlsx PATH --src-index PATH --out-root PATH
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import unicodedata
import zipfile
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

_NS = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"

# (zh_l1, zh_l2) -> stable category id. L2="" means synthetic "默认".
_CATEGORY_IDS: Dict[Tuple[str, str], str] = {
    ("基础医学", "解剖学"): "basic.anatomy",
    ("基础医学", "兽医术语"): "basic.terminology",
    ("基础医学", "生理学"): "basic.physiology",
    ("基础医学", "病理学"): "basic.pathology",
    ("基础医学", "组织学"): "basic.histology",
    ("基础医学", "微生物"): "basic.microbiology",
    ("基础医学", "药理基础"): "basic.pharmacology_fundamentals",
    ("临床医学", "综合内科学"): "clinical.internal_medicine",
    ("临床医学", "外科学"): "clinical.surgery",
    ("临床医学", "急诊与重症"): "clinical.emergency_critical",
    ("临床医学", "心脏病学"): "clinical.cardiology",
    ("临床医学", "皮肤病学"): "clinical.dermatology",
    ("临床医学", "眼科学"): "clinical.ophthalmology",
    ("临床医学", "神经病学"): "clinical.neurology",
    ("临床医学", "肿瘤学"): "clinical.oncology",
    ("临床医学", "综合临床参考"): "clinical.reference",
    ("诊断医学", "临床病理与细胞学"): "diagnostics.clinical_pathology",
    ("诊断医学", "影像诊断"): "diagnostics.imaging",
    ("诊断医学", "鉴别诊断"): "diagnostics.differential",
    ("诊断医学", "实验室诊断"): "diagnostics.laboratory",
    ("药学", "Papich兽医药物手册"): "pharmacy.papich",
    ("药学", "兽医药理学（技师）"): "pharmacy.applied_pharmacology",
    ("药学", "药理与临床治疗"): "pharmacy.clinical_therapeutics",
    ("麻醉与镇痛", "默认"): "anesthesia.default",
    ("免疫与疫苗", "默认"): "immunology.default",
    ("兽医繁殖与产科学", "默认"): "reproduction.default",
    ("行为学", "犬猫行为问题"): "behavior.dog_cat_problems",
    ("行为学", "猫行为健康与福利"): "behavior.feline_welfare",
    ("综合医学", "综合兽医学"): "integrative.general",
    ("临床技能与护理", "临床技能与操作"): "clinical_skills.techniques",
    ("临床技能与护理", "护理学"): "clinical_skills.nursing",
    ("临床技能与护理", "麻醉与镇痛"): "anesthesia.default",
    ("感染病、寄生虫与公共卫生", "无"): "infectious.placeholder",
    ("感染病、寄生虫与公共卫生", "默认"): "infectious.general",
    ("营养学", "无"): "nutrition.placeholder",
    ("营养学", "默认"): "nutrition.general",
    ("特殊动物医学", "默认"): "exotic.default",
    ("大型动物与马医学", "大动物内科"): "equine.large_animal_internal",
    ("大型动物与马医学", "运动医学"): "equine.sports_medicine",
    ("大型动物与马医学", "马腹痛"): "equine.colic",
    ("大型动物与马医学", "神经病学"): "equine.neurology",
    ("大型动物与马医学", "心脏学"): "equine.cardiology",
    ("大型动物与马医学", "繁殖学"): "equine.reproduction",
    ("大型动物与马医学", "马营养"): "equine.nutrition",
    ("大型动物与马医学", "传染病"): "equine.infectious",
    ("大型动物与马医学", "马肿瘤"): "equine.oncology",
    ("大型动物与马医学", "马临床并发症"): "equine.complications",
    ("人兽共患病", "动物和人类的弓形虫病"): "zoonosis.toxoplasmosis",
    ("综合医学（中兽医辅助）", "综合兽医学"): "integrative.tcm_support",
    ("个体差异医学", "遗传与品种易感性"): "individual.genetics_breed",
    ("指南、共识与循证医学", "默认"): "guidelines.general",
    ("个体差异医学", "这里可以放品种、年龄、性别和特发病倾向相关的资料，如果有"): "individual.placeholder",
    ("中兽医学", "这个一般没有专门写小动物的书"): "tcm.placeholder",
    ("指南、共识与循证医学", "感觉是一些医生共识、规范、系统的东西"): "guidelines.placeholder",
    ("兽医职业", "涉及职业道德、沟通技巧之类的，不知道要不要放进去，可能没有专门的书籍？但会有类似一百个沟通小技巧"): "profession.placeholder",
}

# L1 categories that put books directly under synthetic L2 "默认"
_L1_DEFAULT: Set[str] = {
    "麻醉与镇痛",
    "免疫与疫苗",
    "兽医繁殖与产科学",
    "特殊动物医学",
}


def _norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", (s or "").lower())
    return re.sub(r"[\s_\-–—:：,，.。()（）\[\]{}'\"“”‘’/=＋+]+", "", s)


@dataclass
class BookEntry:
    book_id: str
    mmd: str
    title: str
    source_name: str = ""
    source_path: Optional[str] = None


@dataclass
class CategorySpec:
    category_id: str
    zh_l1: str
    zh_l2: str
    books: List[BookEntry] = field(default_factory=list)
    source_paths: List[str] = field(default_factory=list)
    chunk_count: int = 0
    row_indices: List[int] = field(default_factory=list)


def _repo_rag_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_xlsx() -> Path:
    # 优先仓库内受版本控制的分类标准；否则回退本机历史路径。
    candidates = [
        _repo_rag_root() / "data" / "veterinary_materials_classification_2.0.xlsx",
        Path(r"C:\Users\ROG\Downloads\小动物agent知识库新增资料\兽医医学资料分类2.0.xlsx"),
        Path(
            r"C:\Users\ROG\xwechat_files\wxid_69gt4wibd3wv22_ddff\msg\file\2026-07\兽医医学资料分类(1).xlsx"
        ),
    ]
    for p in candidates:
        if p.exists():
            return p
    return candidates[0]


def read_xlsx_rows(xlsx: Path) -> List[List[str]]:
    with zipfile.ZipFile(xlsx) as z:
        ss: List[str] = []
        root = ET.fromstring(z.read("xl/sharedStrings.xml"))
        for si in root.findall(f"{_NS}si"):
            ss.append("".join((t.text or "") for t in si.iter(f"{_NS}t")))
        sheet = ET.fromstring(z.read("xl/worksheets/sheet1.xml"))
        rows: List[List[str]] = []
        for row in sheet.findall(f"{_NS}sheetData/{_NS}row"):
            cells: Dict[str, str] = {}
            for c in row.findall(f"{_NS}c"):
                ref = c.attrib.get("r", "")
                col = "".join(ch for ch in ref if ch.isalpha())
                t = c.attrib.get("t")
                v = c.find(f"{_NS}v")
                if v is None:
                    val = ""
                elif t == "s":
                    val = ss[int(v.text)]
                else:
                    val = v.text or ""
                cells[col] = val
            rows.append([cells.get(x, "") for x in "ABCDEFG"])
    return rows


def parse_taxonomy_from_rows(rows: Sequence[Sequence[str]]) -> OrderedDict[str, CategorySpec]:
    """Return OrderedDict category_id -> CategorySpec (books not yet path-resolved)."""
    cats: OrderedDict[str, CategorySpec] = OrderedDict()
    cur_l1 = ""
    cur_l2 = ""

    def _ensure(l1: str, l2: str) -> CategorySpec:
        key = (l1, l2)
        if key not in _CATEGORY_IDS:
            # fallback slug
            slug = f"misc.{_norm(l1)[:20]}.{_norm(l2)[:20] or 'default'}"
            cid = slug
        else:
            cid = _CATEGORY_IDS[key]
        if cid not in cats:
            cats[cid] = CategorySpec(category_id=cid, zh_l1=l1, zh_l2=l2)
        return cats[cid]

    # Pre-create all known categories (including empty placeholders)
    for (l1, l2), cid in _CATEGORY_IDS.items():
        cats.setdefault(cid, CategorySpec(category_id=cid, zh_l1=l1, zh_l2=l2))

    for r in rows:
        a, b, c, d, e, f = (list(r) + [""] * 6)[:6]
        if str(a).strip():
            cur_l1 = str(a).strip()
            cur_l2 = "默认" if cur_l1 in _L1_DEFAULT else ""
        if str(b).strip():
            cur_l2 = str(b).strip()
        book_id = str(c or "").strip()
        mmd = str(d or "").strip()
        title = str(e or f or "").strip()
        source_name = str(f or "").strip()
        if book_id == "无" and not mmd:
            continue
        if not (book_id or mmd):
            continue
        if not cur_l1:
            continue
        l2 = cur_l2 or ("默认" if cur_l1 in _L1_DEFAULT else "")
        if not l2:
            l2 = "默认"
        spec = _ensure(cur_l1, l2)
        spec.books.append(
            BookEntry(book_id=book_id, mmd=mmd, title=title, source_name=source_name)
        )
    return cats


def build_path_index(source_paths: Sequence[str]) -> Dict[str, str]:
    path_by_norm: Dict[str, str] = {}
    for sp in source_paths:
        p = Path(sp)
        bn, parent = p.name, p.parent.name
        for key in (_norm(bn), _norm(parent), _norm(bn.replace(".mmd", ""))):
            if key:
                path_by_norm.setdefault(key, sp)
        for text in (parent, bn):
            m = re.match(r"^(\d{1,3})\b", text)
            if m:
                path_by_norm.setdefault(m.group(1).zfill(3), sp)
                path_by_norm.setdefault(m.group(1), sp)
    return path_by_norm


def match_book_to_path(book: BookEntry, path_by_norm: Dict[str, str]) -> Optional[str]:
    candidates = [
        _norm(book.mmd),
        _norm(book.title),
        _norm(book.title[:60]),
        book.book_id.zfill(3) if book.book_id.isdigit() else "",
        book.book_id if book.book_id.isdigit() else "",
    ]
    m = re.match(r"^(\d{1,3})\b", book.title or "")
    if m:
        candidates.append(m.group(1).zfill(3))
        candidates.append(m.group(1))
    for key in candidates:
        if key and key in path_by_norm:
            return path_by_norm[key]
    nt = _norm(book.title)
    if nt and len(nt) > 20:
        for k, sp in path_by_norm.items():
            if nt in k or k in nt:
                return sp
    return None


def load_meta_paths(meta_jsonl: Path) -> Tuple[List[dict], List[str]]:
    metas: List[dict] = []
    paths: List[str] = []
    with meta_jsonl.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = json.loads(line)
            metas.append(m)
            paths.append(str(m.get("source_path") or ""))
    return metas, paths


def write_empty_store(out_dir: Path, dim: int = 384) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "store_config.json").write_text(
        json.dumps({"dim": dim, "metric": "cosine_dot"}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (out_dir / "meta.jsonl").write_text("", encoding="utf-8")
    np.save(out_dir / "embeddings.npy", np.zeros((0, dim), dtype=np.float32))


def write_store_slice(
    out_dir: Path,
    *,
    embeddings: np.ndarray,
    metas: List[dict],
    row_indices: Sequence[int],
    dim: int,
) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "store_config.json").write_text(
        json.dumps({"dim": dim, "metric": "cosine_dot"}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    if not row_indices:
        write_empty_store(out_dir, dim=dim)
        return 0
    idx = np.asarray(sorted(set(int(i) for i in row_indices)), dtype=np.int64)
    emb = embeddings[idx].astype(np.float32, copy=False)
    np.save(out_dir / "embeddings.npy", emb)
    with (out_dir / "meta.jsonl").open("w", encoding="utf-8") as f:
        for i in idx:
            f.write(json.dumps(metas[int(i)], ensure_ascii=False) + "\n")
    return int(len(idx))


def split_index(
    *,
    xlsx: Path,
    src_index: Path,
    out_root: Path,
    taxonomy_out: Path,
) -> Dict:
    rows = read_xlsx_rows(xlsx)
    cats = parse_taxonomy_from_rows(rows)
    metas, all_paths = load_meta_paths(src_index / "meta.jsonl")
    path_by_norm = build_path_index(set(all_paths))

    # path -> list of row indices
    path_to_rows: Dict[str, List[int]] = {}
    for i, sp in enumerate(all_paths):
        path_to_rows.setdefault(sp, []).append(i)

    unmatched: List[dict] = []
    matched_books = 0
    for spec in cats.values():
        resolved: Set[str] = set()
        for book in spec.books:
            sp = match_book_to_path(book, path_by_norm)
            book.source_path = sp
            if sp is None:
                unmatched.append(
                    {
                        "category_id": spec.category_id,
                        "book_id": book.book_id,
                        "mmd": book.mmd,
                        "title": book.title[:120],
                    }
                )
            else:
                matched_books += 1
                resolved.add(sp)
        spec.source_paths = sorted(resolved)
        row_idx: List[int] = []
        for sp in resolved:
            row_idx.extend(path_to_rows.get(sp, []))
        spec.row_indices = row_idx

    cfg = json.loads((src_index / "store_config.json").read_text(encoding="utf-8"))
    dim = int(cfg.get("dim", 384))
    embeddings = np.load(src_index / "embeddings.npy").astype(np.float32, copy=False)
    if embeddings.ndim != 2 or embeddings.shape[0] != len(metas):
        raise ValueError(f"embeddings/meta mismatch: {embeddings.shape} vs {len(metas)}")

    out_root.mkdir(parents=True, exist_ok=True)
    for spec in cats.values():
        cat_dir = out_root / spec.category_id
        n = write_store_slice(
            cat_dir,
            embeddings=embeddings,
            metas=metas,
            row_indices=spec.row_indices,
            dim=dim,
        )
        spec.chunk_count = n

    taxonomy = {
        "version": 1,
        "source_index": str(src_index.resolve()),
        "out_root": str(out_root.resolve()),
        "xlsx": str(xlsx.resolve()),
        "dim": dim,
        "matched_books": matched_books,
        "unmatched_books": unmatched,
        "categories": [
            {
                "id": s.category_id,
                "zh_l1": s.zh_l1,
                "zh_l2": s.zh_l2,
                "book_ids": [b.book_id for b in s.books],
                "books": [
                    {
                        "book_id": b.book_id,
                        "mmd": b.mmd,
                        "title": b.title,
                        "source_path": b.source_path,
                    }
                    for b in s.books
                ],
                "source_paths": s.source_paths,
                "chunk_count": s.chunk_count,
                "index_dir": str((out_root / s.category_id).resolve()),
            }
            for s in cats.values()
        ],
    }
    taxonomy_out.parent.mkdir(parents=True, exist_ok=True)
    taxonomy_out.write_text(json.dumps(taxonomy, ensure_ascii=False, indent=2), encoding="utf-8")
    return taxonomy


def main(argv: Optional[Sequence[str]] = None) -> int:
    rag_root = _repo_rag_root()
    parser = argparse.ArgumentParser(description="按二级分类切分 RAG 索引")
    parser.add_argument("--xlsx", type=str, default=str(_default_xlsx()))
    parser.add_argument("--src-index", type=str, default=str(rag_root / "data" / "rag_index_e5"))
    parser.add_argument("--out-root", type=str, default=str(rag_root / "data" / "rag_index_e5_by_cat"))
    parser.add_argument(
        "--taxonomy-out",
        type=str,
        default=str(rag_root / "data" / "category_taxonomy.json"),
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    xlsx = Path(args.xlsx)
    if not xlsx.exists():
        print(f"[error] xlsx not found: {xlsx}", file=sys.stderr)
        return 2

    report = split_index(
        xlsx=xlsx,
        src_index=Path(args.src_index),
        out_root=Path(args.out_root),
        taxonomy_out=Path(args.taxonomy_out),
    )
    n_cat = len(report["categories"])
    n_nonempty = sum(1 for c in report["categories"] if c["chunk_count"] > 0)
    print(f"[ok] categories={n_cat} nonempty={n_nonempty}")
    print(f"[ok] matched_books={report['matched_books']} unmatched={len(report['unmatched_books'])}")
    print(f"[ok] taxonomy={args.taxonomy_out}")
    print(f"[ok] out_root={args.out_root}")
    if report["unmatched_books"]:
        for u in report["unmatched_books"][:20]:
            print(f"  unmatched: {u}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
