from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator


_RE_MD_IMAGE = re.compile(r"!\[[^\]]*\]\([^)]+\)")
_RE_HTML_TAG = re.compile(r"<[^>]+>")
_RE_PAGE_SPLIT = re.compile(r"<---\s*Page Split\s*--->", re.IGNORECASE)
_RE_WHITESPACE = re.compile(r"[ \t]+")
_RE_SENT_SPLIT = re.compile(r"(?<=[。！？.!?])\s+")


def sha1_bytes(data: bytes) -> str:
    return hashlib.sha1(data).hexdigest()


def read_text_lossy(path: Path) -> tuple[str, str]:
    """
    返回 (text, sha1)。
    - 尽量用 utf-8 读取，失败则忽略非法字符，保证不会因为单个文件编码导致全流程中断。
    """
    raw = path.read_bytes()
    return raw.decode("utf-8", errors="ignore"), sha1_bytes(raw)


def cleanup_mmd_text(text: str) -> str:
    """
    针对 .mmd（通常是 OCR/转换产物的 Markdown）做轻量清洗：
    - 去掉图片引用
    - 去掉 HTML tag
    - 统一分页分隔符为换行
    - 合并多余空白
    """
    text = _RE_PAGE_SPLIT.sub("\n\n", text)
    text = _RE_MD_IMAGE.sub("", text)
    text = _RE_HTML_TAG.sub("", text)
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    text = _RE_WHITESPACE.sub(" ", text)

    # 压缩空行（最多保留两行）
    lines = [ln.strip() for ln in text.split("\n")]
    out: list[str] = []
    blank = 0
    for ln in lines:
        if not ln:
            blank += 1
            if blank <= 2:
                out.append("")
            continue
        blank = 0
        out.append(ln)
    return "\n".join(out).strip()


def iter_pages(clean_text: str) -> Iterator[str]:
    """
    以空行分段，近似“页/段落”粒度；后续 chunker 会再合并到目标大小。
    """
    buf: list[str] = []
    for ln in clean_text.split("\n"):
        if ln.strip() == "":
            if buf:
                yield "\n".join(buf).strip()
                buf = []
            continue
        buf.append(ln)
    if buf:
        yield "\n".join(buf).strip()


def word_count(text: str) -> int:
    # 简单按空白切分；对英文书够用。中文也能工作（会偏小）。
    return len([w for w in re.split(r"\s+", text.strip()) if w])


def split_sentences(text: str) -> list[str]:
    """
    以句号/问号/感叹号为主的“近似句子切分”。
    - 英文：. ! ?
    - 中文：。！？ 
    """
    s = re.sub(r"\s+", " ", (text or "").strip())
    if not s:
        return []
    parts = _RE_SENT_SPLIT.split(s)
    out = [p.strip() for p in parts if p and p.strip()]
    return out


def recursive_sentence_chunks(
    *,
    text: str,
    chunk_words: int,
    chunk_overlap_words: int,
    min_chunk_words: int,
) -> list[str]:
    """
    递归分块（句子边界优先）：
    - 先切句子
    - 句子过长则降级按逗号/分号切，再不行按词切
    - 再按 chunk_words 合并，并做 overlap
    """
    assert chunk_words > 0
    assert 0 <= chunk_overlap_words < chunk_words

    def split_fallback(s: str) -> list[str]:
        s = s.strip()
        if not s:
            return []
        # 二级：逗号/分号
        parts = re.split(r"(?<=[,;，；])\s+", s)
        parts = [p.strip() for p in parts if p and p.strip()]
        if len(parts) >= 2:
            return parts
        # 三级：按词硬切
        ws = [w for w in re.split(r"\s+", s) if w]
        if not ws:
            return []
        step = max(20, chunk_words // 3)
        out = []
        for i in range(0, len(ws), step):
            out.append(" ".join(ws[i : i + step]))
        return out

    # 1) 句子切分
    sentences: list[str] = []
    for sent in split_sentences(text):
        if word_count(sent) > chunk_words * 1.2:
            sentences.extend(split_fallback(sent))
        else:
            sentences.append(sent)

    # 2) 合并到目标大小
    merged: list[str] = []
    cur: list[str] = []
    cur_words = 0
    for s in sentences:
        w = word_count(s)
        if cur and (cur_words + w) > chunk_words:
            merged.append(" ".join(cur).strip())
            cur = [s]
            cur_words = w
        else:
            cur.append(s)
            cur_words += w
    if cur:
        merged.append(" ".join(cur).strip())

    # 3) overlap（按词近似）
    out: list[str] = []
    for i, block in enumerate(merged):
        words = [w for w in re.split(r"\s+", block.strip()) if w]
        if len(words) < min_chunk_words:
            continue
        if chunk_overlap_words <= 0 or i == 0:
            out.append(" ".join(words))
        else:
            prev_words = [w for w in re.split(r"\s+", merged[i - 1].strip()) if w]
            tail = prev_words[-chunk_overlap_words:] if prev_words else []
            out.append(" ".join(tail + words))
    return out


@dataclass(frozen=True)
class TextChunk:
    chunk_id: str
    source_path: str
    source_sha1: str
    chunk_index: int
    text: str
    n_words: int


def chunk_text(
    *,
    source_path: Path,
    source_sha1: str,
    clean_text: str,
    chunk_words: int,
    chunk_overlap_words: int,
    min_chunk_words: int,
) -> list[TextChunk]:
    """
    递归分块（句子边界优先）：
    - 先按空行分段（页/段落）
    - 段内用 recursive_sentence_chunks 做句子递归分块
    """
    assert chunk_words > 0
    assert 0 <= chunk_overlap_words < chunk_words

    paragraphs = [p for p in iter_pages(clean_text) if p]

    merged: list[str] = []
    for p in paragraphs:
        merged.extend(
            recursive_sentence_chunks(
                text=p,
                chunk_words=chunk_words,
                chunk_overlap_words=chunk_overlap_words,
                min_chunk_words=min_chunk_words,
            )
        )

    chunks: list[TextChunk] = []
    for i, block in enumerate(merged):
        text = block.strip()
        n_words = len([w for w in re.split(r"\s+", text.strip()) if w])
        if n_words < min_chunk_words:
            continue

        chunk_id = hashlib.sha1(
            (str(source_path).lower() + ":" + source_sha1 + ":" + str(i)).encode("utf-8")
        ).hexdigest()
        chunks.append(
            TextChunk(
                chunk_id=chunk_id,
                source_path=str(source_path),
                source_sha1=source_sha1,
                chunk_index=i,
                text=text,
                n_words=n_words,
            )
        )
    return chunks


def iter_mmd_files(raw_dir: Path) -> Iterable[Path]:
    return sorted(raw_dir.rglob("*.mmd"))


