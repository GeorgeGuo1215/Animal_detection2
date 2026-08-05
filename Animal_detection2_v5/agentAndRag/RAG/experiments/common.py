from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Tuple


def read_jsonl(path: Path) -> Iterator[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            yield json.loads(line)


def write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


def normalize_ws(s: str) -> str:
    return re.sub(r"\s+", " ", s or "").strip()


def book_id_from_source_path(source_path: str) -> str:
    """
    从 meta.jsonl 里的 source_path 推一个稳定的 book_id：
    - 你当前 raw 数据结构通常是 .../raw/<书名目录>/<书名>.mmd
    - book_id 取 <书名目录> 这一层（更稳定，也更像“书”）
    """
    p = Path(source_path)
    # .../raw/<book_dir>/<file>.mmd
    if p.parent.name:
        return p.parent.name
    return p.stem


def safe_int(x, default: int = 0) -> int:
    try:
        return int(x)
    except Exception:
        return default


EN_STOP = {
    "the",
    "a",
    "an",
    "and",
    "or",
    "to",
    "of",
    "in",
    "on",
    "for",
    "with",
    "as",
    "is",
    "are",
    "was",
    "were",
    "be",
    "by",
    "that",
    "this",
    "it",
    "from",
    "at",
    "into",
    "during",
    "within",
    "without",
    "over",
    "under",
    "between",
}


def simple_keywords(text: str, top_n: int = 6) -> List[str]:
    """
    不引入 sklearn 的简易关键词抽取：
    - 对英文：按 token 频次，去 stopwords、去太短 token
    - 中文：由于无分词，会退化；但你语料主要是英文医学书，够用。
    """
    text = normalize_ws(text).lower()
    tokens = re.findall(r"[a-z][a-z0-9\-]{2,}", text)
    freq: Dict[str, int] = {}
    for t in tokens:
        if t in EN_STOP:
            continue
        freq[t] = freq.get(t, 0) + 1
    # 频次优先，长度次之
    ranked = sorted(freq.items(), key=lambda kv: (kv[1], len(kv[0])), reverse=True)
    return [w for w, _ in ranked[:top_n]]


def head_like_query(text: str) -> Optional[str]:
    """
    从 chunk 内部抓一个最像标题的行作为 query（markdown # / ## / ###）。
    """
    for ln in (text or "").splitlines():
        ln = ln.strip()
        if ln.startswith("#"):
            q = ln.lstrip("#").strip()
            if len(q) >= 6:
                return q
    return None



