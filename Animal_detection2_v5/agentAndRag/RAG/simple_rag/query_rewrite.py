from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Protocol

import httpx


class QueryRewriter(Protocol):
    """
    Query 重写器接口：输入原始 query，输出多个候选 query（用于多路召回/扩展检索）。
    """

    def rewrite(self, query: str) -> List[str]: ...


def _env(name: str, default: Optional[str] = None) -> Optional[str]:
    v = os.getenv(name)
    return v if v not in (None, "") else default


def _normalize_query(query: str) -> str:
    q0 = (query or "").strip()
    if not q0:
        return ""
    # 去掉选择题选项行（如果用户把完整题干扔进来）
    q = q0.split("\n\n", 1)[0].strip()
    return re.sub(r"\s+", " ", q)


@dataclass(frozen=True)
class NoRewrite(QueryRewriter):
    def rewrite(self, query: str) -> List[str]:
        q = _normalize_query(query)
        return [q] if q else []


@dataclass(frozen=True)
class TemplateRewriter(QueryRewriter):
    """
    简单的“模板化”重写：把一个 query 扩展成多种问法/关键词组合。
    - 不调用 LLM，完全可复现
    - 适合毕业设计里做“query rewrite / query expansion”的对比维度
    """

    templates: tuple[str, ...] = (
        "{q}",
        "definition of {q}",
        "what is {q}",
        "what is the function of {q}",
        "indications of {q}",
        "contraindications of {q}",
        "treatment for {q}",
        "diagnosis of {q}",
        "symptoms of {q}",
        "dose of {q}",
    )
    max_out: int = 8

    def rewrite(self, query: str) -> List[str]:
        q = _normalize_query(query)
        if not q:
            return []

        out: List[str] = []
        seen = set()
        for t in self.templates:
            cand = t.format(q=q).strip()
            cand = re.sub(r"\s+", " ", cand)
            if cand and cand not in seen:
                out.append(cand)
                seen.add(cand)
            if len(out) >= int(self.max_out):
                break
        return out


@dataclass(frozen=True)
class LLMRewriter(QueryRewriter):
    """
    基于 OpenAI-compatible 接口的 query 重写。
    - 保留原始 query
    - 让 LLM 生成少量更适合检索的英文改写
    - 失败时优雅回退到原 query
    """

    base_url: Optional[str] = None
    api_key: Optional[str] = None
    model: Optional[str] = None
    timeout_s: float = 60.0
    max_out: int = 5
    temperature: float = 0.0

    def _chat_json(self, messages: List[dict]) -> dict:
        base_url = (self.base_url or _env("OPENAI_BASE_URL") or "https://api.deepseek.com").rstrip("/")
        api_key = self.api_key or _env("OPENAI_API_KEY") or _env("DEEPSEEK_API_KEY") or ""
        model = self.model or _env("OPENAI_MODEL") or _env("DEEPSEEK_MODEL") or "deepseek-chat"
        if not api_key:
            raise RuntimeError("Missing API key for LLMRewriter.")

        url = f"{base_url}/chat/completions"
        headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
        payload = {
            "model": model,
            "messages": messages,
            "temperature": float(self.temperature),
            "max_tokens": 256,
            "response_format": {"type": "json_object"},
        }
        with httpx.Client(timeout=float(self.timeout_s)) as client:
            resp = client.post(url, headers=headers, json=payload)
            resp.raise_for_status()
            return resp.json()

    @staticmethod
    def _extract_queries(text: str, max_out: int) -> List[str]:
        text = (text or "").strip()
        if not text:
            return []
        try:
            obj = json.loads(text)
        except Exception:
            l = text.find("{")
            r = text.rfind("}")
            if l >= 0 and r > l:
                try:
                    obj = json.loads(text[l : r + 1])
                except Exception:
                    return []
            else:
                return []

        qs = obj.get("queries")
        if not isinstance(qs, list):
            return []
        out: List[str] = []
        seen = set()
        for item in qs:
            cand = re.sub(r"\s+", " ", str(item or "").strip())
            if cand and cand not in seen:
                out.append(cand)
                seen.add(cand)
            if len(out) >= int(max_out):
                break
        return out

    def rewrite(self, query: str) -> List[str]:
        q = _normalize_query(query)
        if not q:
            return []

        system = (
            "You are a retrieval query rewriting assistant for a veterinary RAG system. "
            "Generate a few concise English search queries that preserve the original meaning, "
            "expand key medical terms when useful, and improve recall. "
            "Return strict JSON only."
        )
        user = {
            "task": "rewrite_query_for_retrieval",
            "query": q,
            "requirements": [
                "Keep the original meaning unchanged.",
                "Use English only.",
                "Prefer short retrieval-friendly phrasing.",
                "Include the original query as one candidate.",
                f"Return at most {int(self.max_out)} queries.",
            ],
            "output_format": {"queries": ["original query", "rewritten query 1", "rewritten query 2"]},
        }

        try:
            resp = self._chat_json(
                [
                    {"role": "system", "content": system},
                    {"role": "user", "content": json.dumps(user, ensure_ascii=False)},
                ]
            )
            content = (resp.get("choices", [{}])[0].get("message", {}).get("content") or "").strip()
            llm_queries = self._extract_queries(content, max_out=int(self.max_out))
        except Exception:
            llm_queries = []

        out: List[str] = []
        seen = set()
        for cand in [q] + llm_queries:
            norm = re.sub(r"\s+", " ", (cand or "").strip())
            if norm and norm not in seen:
                out.append(norm)
                seen.add(norm)
            if len(out) >= int(self.max_out):
                break
        return out




