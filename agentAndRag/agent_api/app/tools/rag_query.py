from __future__ import annotations

import re
from typing import Any


_ASCII_LETTER_RE = re.compile(r"[A-Za-z]")
_NON_ENGLISH_SCRIPT_RE = re.compile(
    "["
    "\u3040-\u30ff"
    "\u3400-\u4dbf"
    "\u4e00-\u9fff"
    "\uf900-\ufaff"
    "\uac00-\ud7af"
    "]"
)


def is_english_rag_query(query: Any) -> bool:
    """判断 RAG 查询是否为纯英文（含拉丁字母且无 CJK 等脚本）。"""
    text = str(query or "").strip()
    return bool(_ASCII_LETTER_RE.search(text)) and not _NON_ENGLISH_SCRIPT_RE.search(text)


def require_english_rag_query(query: Any) -> str:
    """校验 RAG 查询为英文；否则抛出 ValueError。"""
    text = str(query or "").strip()
    if not is_english_rag_query(text):
        raise ValueError("rag.search query must be written in English")
    return text


__all__ = ["is_english_rag_query", "require_english_rag_query"]
