from __future__ import annotations

import re


_RE_WORD = re.compile(r"[A-Za-z][A-Za-z0-9\-]{2,}")
_RE_CJK_CHAR = re.compile(r"[\u4e00-\u9fff\u3400-\u4dbf\uf900-\ufaff]")
_EN_STOP = {
    "the", "a", "an", "and", "or", "to", "of", "in", "on", "for", "with", "as",
    "is", "are", "was", "were", "be", "by", "that", "this", "it", "from", "at",
}


def tokenize_for_overlap(text: str) -> list[str]:
    """为中英混排检索结果生成轻量词面重叠 token。"""
    normalized = (text or "").lower()
    tokens = [token for token in _RE_WORD.findall(normalized) if token not in _EN_STOP]
    cjk_chars = _RE_CJK_CHAR.findall(normalized)
    tokens.extend(cjk_chars[index] + cjk_chars[index + 1] for index in range(len(cjk_chars) - 1))
    if len(cjk_chars) == 1:
        tokens.append(cjk_chars[0])
    return tokens


def overlap_score(query: str, context: str) -> float:
    """返回 context 覆盖 query 词面 token 的比例，范围为 0 到 1。"""
    query_tokens = set(tokenize_for_overlap(query))
    if not query_tokens:
        return 0.0
    context_tokens = set(tokenize_for_overlap(context))
    if not context_tokens:
        return 0.0
    return len(query_tokens & context_tokens) / float(len(query_tokens))
