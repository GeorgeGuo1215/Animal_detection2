"""Auditable OCR block cleaning and tokenizer-bounded semantic-v3 chunks.

Raw sources are immutable. Rejected blocks remain available in the quarantine
manifest; numeric values, units and negations are never rewritten by this module.
"""
from __future__ import annotations

import hashlib
import re
import unicodedata
from collections import Counter
from dataclasses import asdict, dataclass
from typing import Any

from bs4 import BeautifulSoup

VERSION = "semantic-v3"
_PAGE = re.compile(r"<---\s*Page Split\s*--->", re.I)
_IMAGE = re.compile(r"!\[[^\]]*\]\([^)]*\)")
_TABLE = re.compile(r"<table\b[^>]*>.*?</table>", re.I | re.S)
_HEADING = re.compile(r"^(#{1,6})\s+(.+)$")
_REFERENCES = re.compile(r"^(references(?: and further reading)?|bibliography|further reading|selected references)\s*[:.]?$", re.I)
_CONTENTS = re.compile(r"^(table of contents|contents|index|subject index|author index)\s*[:.]?$", re.I)
_COPYRIGHT = re.compile(r"^(copyright\b|all rights reserved\b|ISBN\b|printed (?:in|on)\b|published by\b|library of congress\b|cataloguing.in.publication\b|cataloging.in.publication\b)", re.I)
_CONTROL = re.compile(r"<\|[^>]*\|>")


def content_hash(text: str) -> str:
    # Case, punctuation and numbers are significant (e.g. mM vs mm).
    normalized = re.sub(r"\s+", " ", unicodedata.normalize("NFC", text)).strip()
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


@dataclass
class Block:
    text: str
    page: int
    section: str
    kind: str
    start: int
    end: int
    reason: str = ""


def _table_rows(raw: str) -> tuple[list[str], bool]:
    """Repeat headers for each row, preserving rowspan/colspan associations."""
    soup = BeautifulSoup(raw, "html.parser")
    spans: dict[int, tuple[str, int]] = {}
    matrix: list[list[str]] = []
    malformed = False
    for tr in soup.find_all("tr"):
        row: dict[int, str] = {i: value for i, (value, remaining) in spans.items()}
        spans = {i: (value, remaining - 1) for i, (value, remaining) in spans.items() if remaining > 1}
        col = 0
        for cell in tr.find_all(["th", "td"], recursive=False):
            while col in row:
                col += 1
            value = cell.get_text(" ", strip=True)
            try:
                colspan = max(1, min(30, int(cell.get("colspan", 1))))
                rowspan = max(1, min(100, int(cell.get("rowspan", 1))))
            except (ValueError, TypeError):
                malformed = True
                colspan = rowspan = 1
            for index in range(col, col + colspan):
                row[index] = value
                if rowspan > 1:
                    spans[index] = (value, rowspan - 1)
            col += colspan
        if row:
            matrix.append([row.get(i, "") for i in range(max(row) + 1)])
    if not matrix:
        return [], True
    width = max(map(len, matrix))
    # One-cell title/footer rows are allowed; intermediate missing columns are not guessed.
    malformed |= any(len(row) not in {1, width} for row in matrix)
    header = next((row for row in matrix if len(row) == width), matrix[0])
    if soup.find("th") is None and any(re.search(r"\d", cell) for cell in header):
        # The first row looks like data, so treating it as a header could silently
        # assign one animal's values to another. Keep the original in quarantine.
        malformed = True
    title = soup.find("caption")
    prefix = title.get_text(" ", strip=True) + "\n" if title else ""
    rows = []
    header_seen = False
    for row in matrix:
        if row is header and not header_seen:
            header_seen = True
            continue
        if len(row) == width:
            text = " | ".join(f"{label}: {value}" for label, value in zip(header, row))
        else:
            text = " ".join(row)
        rows.append(prefix + text)
    return rows or [prefix + " | ".join(header)], malformed


def _reason(text: str, kind: str, mode: str) -> str:
    if not text.strip():
        return "empty"
    if mode:
        return mode
    if kind == "table":
        return ""  # Numeric/low-lexical-density tables are valid evidence.
    if _COPYRIGHT.match(text) or (len(text) < 1600 and re.search(r"all rights reserved", text, re.I)):
        return "publication_metadata"
    if re.search(r"\.{5,}\s*\d+", text) and len(re.findall(r"\.{5,}\s*\d+", text)) >= 2:
        return "contents_listing"
    if len(re.findall(r"\b(?:19|20)\d{2}[a-z]?\b", text)) >= 3 and len(re.findall(r"\b(?:journal|vol\.|pp\.|et al|press|edition)\b", text, re.I)) >= 2:
        return "bibliographic_listing"
    if _CONTROL.search(text):
        return "ocr_control_tokens"
    # OCR sometimes removes spaces between an entire table of contents' entries.
    # Require many word joins and almost no sentence punctuation, preserving prose.
    joins = len(re.findall(r"[a-z]{3,}[A-Z][a-z]{2,}", text))
    sentences = len(re.findall(r"[a-z]{3,}[.!?](?:\s|$)", text))
    navigation_cues = len(re.findall(r"(?:Section|Chapter|Part)\s+(?:[IVX]+|\d+)\b", text))
    if len(text) > 200 and sentences < 2 and navigation_cues >= 2:
        return "concatenated_navigation"
    if text.count("\ufffd") / max(len(text), 1) > 0.015:
        return "unreadable_encoding"
    words = re.findall(r"\b[\w'-]+\b", text)
    if len(words) >= 30 and len(set(words)) / len(words) < 0.12:
        return "repeated_ocr_text"
    if re.search(r"(.)\1{15,}", text) and not re.search(r"[-=]{15,}", text):
        return "repeated_ocr_characters"
    if len(text) > 60 and sum(c.isalnum() for c in text) / len(text) < 0.2 and not re.search(r"\\[a-zA-Z]+|[=∑∫]", text):
        return "low_information_residue"
    if len(words) < 4 and re.fullmatch(r"[\d\s\W]+", text):
        return "page_number_or_separator"
    return ""


def parse_and_clean(raw: str) -> tuple[list[Block], list[dict]]:
    pages = list(_PAGE.finditer(raw))
    boundaries = [(0, pages[0].start() if pages else len(raw))]
    boundaries.extend((m.end(), pages[i+1].start() if i+1 < len(pages) else len(raw)) for i, m in enumerate(pages))
    # Only recurring page-edge lines qualify as running headers/footers.
    edges: Counter[str] = Counter()
    for start, end in boundaries:
        lines = [line.strip() for line in raw[start:end].splitlines() if line.strip()]
        edges.update(set(lines[:2] + lines[-2:]))
    repeated = {line for line, n in edges.items() if len(boundaries) >= 8 and n >= max(5, len(boundaries) * 0.25)
                and len(line) < 140 and not re.search(r"\b(?:mg|kg|dose|not|avoid|contraindicat)\b|[<>=]", line, re.I)
                and not line.startswith(("#", "<", "!"))}
    kept: list[Block] = []
    quarantine: list[dict] = []
    headings: list[tuple[int, str]] = []
    mode = ""
    mode_level = 0

    def record(text: str, page: int, kind: str, start: int, end: int, forced: str = ""):
        nonlocal mode
        text = text.strip()
        # OCR heading levels are inconsistent. A reference section must not swallow
        # an entire subsequent chapter when its next heading is unmarked.
        if mode == "reference_section" and kind != "heading" and len(text) > 200:
            citation = re.search(r"\b(?:19|20)\d{2}[a-z]?\b|\bet al\b|\b(?:Journal|Press|ISBN|doi)\b", text)
            # Narrative can cite a year without becoming a bibliography entry.
            sentences = len(re.findall(r"[a-z]{3,}[.!?](?:\s|$)", text))
            prose_words = len(re.findall(r"\b(?:the|is|are|should|may|can|must|was|were|has|have)\b", text, re.I))
            years = len(re.findall(r"\b(?:19|20)\d{2}[a-z]?\b", text))
            if not citation or (sentences >= 2 and prose_words >= 8 and years < 3):
                mode = ""
                headings[:] = [(level, title) for level, title in headings if not _REFERENCES.fullmatch(title)]
        if mode == "navigation_section" and kind != "heading" and len(text) > 200:
            # Some exports concatenate books after an index without a marked heading.
            # Narrative sentences cannot inherit an unbounded navigation region.
            prose = len(re.findall(r"[a-z]{3,}[.!?](?:\s|$)", text)) >= 2
            page_entries = len(re.findall(r"(?:\.{3,}\s*\d+|\b\d+(?:[–-]\d+)?\s*$)", text, re.M))
            numbered_tokens = len(re.findall(r"\b\d+(?:[–-]\d+)?\b", text))
            if prose and page_entries < 2 and numbered_tokens < max(4, len(text.split()) // 12):
                mode = ""
                headings[:] = [(level, title) for level, title in headings if not _CONTENTS.fullmatch(title)]
        section = " / ".join(title for _, title in headings)
        reason = forced or _reason(text, kind, mode)
        block = Block(text, page, section, kind, start, end, reason)
        if reason:
            quarantine.append({**asdict(block), "content_hash": content_hash(text)})
        elif text:
            kept.append(block)

    for page, (start, end) in enumerate(boundaries, 1):
        text = raw[start:end]
        segments = []
        cursor = 0
        for table in _TABLE.finditer(text):
            segments.extend((m.start()+cursor, m.end()+cursor, "text", m.group()) for m in re.finditer(r"\S[^\n]*(?:\n(?!\s*\n)[^\n]+)*", text[cursor:table.start()]))
            segments.append((table.start(), table.end(), "table", table.group()))
            cursor = table.end()
        segments.extend((m.start()+cursor, m.end()+cursor, "text", m.group()) for m in re.finditer(r"\S[^\n]*(?:\n(?!\s*\n)[^\n]+)*", text[cursor:]))
        for local_start, local_end, kind, content in segments:
            a, b = start + local_start, start + local_end
            if kind == "table":
                rows, malformed = _table_rows(content)
                if malformed:
                    record(content, page, kind, a, b, "ambiguous_table_structure")
                else:
                    for row in rows:
                        record(row, page, kind, a, b)
                continue
            content = _IMAGE.sub("", content).strip()
            # Native Markdown tables from newer OCR exports retain their row labels too.
            table_lines = [line.strip() for line in content.splitlines() if line.strip()]
            if len(table_lines) >= 2 and table_lines[0].startswith("|") and re.fullmatch(r"[|:\s-]+", table_lines[1]):
                headers = [v.strip() for v in table_lines[0].strip("|").split("|")]
                rows = [[v.strip() for v in line.strip("|").split("|")] for line in table_lines[2:]]
                is_contents = len(headers) == 2 and re.fullmatch(r"\d+|pages?|p\.?", headers[1], re.I)
                if is_contents and rows and all(len(row) == 2 and re.fullmatch(r"\d+(?:[–-]\d+)?", row[1]) for row in rows):
                    record(content, page, "table", a, b, "contents_listing")
                elif any(len(row) != len(headers) for row in rows):
                    record(content, page, "table", a, b, "ambiguous_table_structure")
                else:
                    for row in rows:
                        record(" | ".join(f"{label}: {value}" for label, value in zip(headers, row)), page, "table", a, b)
                continue
            # Convert harmless inline HTML without collapsing cell boundaries (tables already parsed).
            if re.search(r"</?(?:b|i|em|strong|br|p|sub|sup|span|center|div)\b", content, re.I):
                content = BeautifulSoup(content, "html.parser").get_text(" ", strip=True)
            lines = content.splitlines()
            body = []
            for line in lines:
                line = line.strip()
                heading = _HEADING.match(line)
                bare = re.sub(r"^\*\*|\*\*$", "", line).strip()
                special = _REFERENCES.fullmatch(bare) or _CONTENTS.fullmatch(bare)
                if heading or special:
                    if body:
                        record("\n".join(body), page, "text", a, b)
                        body = []
                    level = len(heading.group(1)) if heading else 1
                    title = heading.group(2).strip() if heading else bare
                    # Relative # depth is not reliable in OCR; every explicit new
                    # heading ends the previous navigation/reference region.
                    nested_navigation = mode == "navigation_section" and (
                        level > mode_level or bool(re.fullmatch(r"[A-Z]", title))
                        or bool(re.match(r"(?:Part|Section)\s+(?:[IVX]+|\d+|One|Two|Three|Four)\b", title))
                    )
                    if not nested_navigation:
                        mode = ""
                    if _REFERENCES.fullmatch(title):
                        mode, mode_level = "reference_section", level
                    elif _CONTENTS.fullmatch(title):
                        mode, mode_level = "navigation_section", level
                    headings = [(lv, name) for lv, name in headings if lv < level
                                and not _REFERENCES.fullmatch(name) and not _CONTENTS.fullmatch(name)]
                    headings.append((level, title))
                    record(line, page, "heading", a, b)
                elif line in repeated:
                    record(line, page, "text", a, b, "running_header_footer")
                else:
                    body.append(line)
            if body:
                record("\n".join(body), page, "text", a, b)
    return kept, quarantine


def tokenize_chunks(blocks: list[Block], tokenizer: Any, *, target: int = 320, maximum: int = 384, overlap: int = 48) -> list[dict]:
    """Split by real token offsets; attach source spans without inventing page numbers."""
    chunks: list[dict] = []
    pending: list[Block] = []

    def length(text: str) -> int:
        return len(tokenizer.encode("passage: " + text, add_special_tokens=True, truncation=False, verbose=False))

    def emit(group: list[Block]):
        if not group:
            return
        section = group[0].section
        # Bound pathological OCR headings independently, with original spans still in provenance.
        section_ids = tokenizer.encode(section, add_special_tokens=False)[:48]
        prefix = tokenizer.decode(section_ids, skip_special_tokens=True).strip()
        prefix = prefix + "\n\n" if prefix else ""
        text = "\n\n".join(block.text for block in group)
        encoded = tokenizer(text, add_special_tokens=False, return_offsets_mapping=True, truncation=False, verbose=False)
        offsets = encoded["offset_mapping"]
        budget = max(32, target - length(prefix))
        start = 0
        while start < len(offsets):
            stop = min(start + budget, len(offsets))
            a = offsets[start][0]
            b = offsets[stop-1][1]
            # Prefer a complete sentence near the end; never alter values or punctuation.
            if stop < len(offsets):
                candidates = [i for i in range(start + budget//2, stop) if text[offsets[i][1]-1:offsets[i][1]] in ".!?。！？"]
                if candidates:
                    stop = candidates[-1]+1
                    b = offsets[stop-1][1]
            value = prefix + text[a:b]
            while length(value) > maximum and stop > start + 1:
                stop -= 1
                b = offsets[stop-1][1]
                value = prefix + text[a:b]
            if length(value) > maximum:
                raise ValueError("single token window exceeds model budget")
            chunks.append({
                "text": value, "content_hash": content_hash(value), "section_path": section,
                "content_type": "table" if all(g.kind == "table" for g in group) else "text",
                "page_sequence_start": min(g.page for g in group), "page_sequence_end": max(g.page for g in group),
                "source_spans": [[g.start, g.end] for g in group],
                "window_start": a, "window_end": b, "token_count": length(value), "chunking_version": VERSION,
            })
            if stop == len(offsets):
                break
            start = max(start+1, stop - min(overlap, (stop-start)//4))

    for block in blocks:
        if block.kind == "heading":
            continue
        if pending and (pending[0].section != block.section or pending[0].kind != block.kind
                        or length("\n\n".join([b.text for b in pending] + [block.text])) > target):
            emit(pending)
            pending = []
        pending.append(block)
    emit(pending)
    return chunks
