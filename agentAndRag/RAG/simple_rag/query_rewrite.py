from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Protocol

import httpx


class QueryRewriter(Protocol):
    def rewrite(self, query: str) -> List[str]: ...


def _env(name: str, default: Optional[str] = None) -> Optional[str]:
    v = os.getenv(name)
    return v if v not in (None, "") else default


def _normalize_query(query: str) -> str:
    q0 = (query or "").strip()
    if not q0:
        return ""
    q = q0.split("\n\n", 1)[0].strip()
    return re.sub(r"\s+", " ", q)


@dataclass(frozen=True)
class NoRewrite(QueryRewriter):
    def rewrite(self, query: str) -> List[str]:
        q = _normalize_query(query)
        return [q] if q else []


_ZH_EN_TOPIC_MAP: dict[str, str] = {
    "食性": "diet feeding",
    "营养": "nutrition nutritional",
    "饲养": "husbandry feeding management",
    "疫苗": "vaccine vaccination",
    "免疫": "immune immunology",
    "寄生虫": "parasite parasites",
    "肠道": "intestinal gut",
    "消化": "digestion digestive",
    "繁殖": "breeding reproduction",
    "遗传": "genetics genomics",
    "基因": "gene genetic",
    "行为": "behavior ethology",
    "训练": "training obedience",
    "手术": "surgery surgical",
    "麻醉": "anesthesia sedation",
    "皮肤": "dermatology skin",
    "骨骼": "orthopedic bone",
    "肾脏": "renal kidney",
    "肝脏": "hepatic liver",
    "心脏": "cardiac heart",
    "呼吸": "respiratory pulmonary",
    "泌尿": "urinary urological",
    "内分泌": "endocrine hormone",
    "肿瘤": "tumor oncology cancer",
    "传染病": "infectious disease",
    "中毒": "poisoning toxicology",
    "急救": "emergency first aid",
    "老年": "geriatric senior aging",
    "幼犬": "puppy neonatal",
    "幼猫": "kitten neonatal",
}


def _generate_en_variant(query: str) -> Optional[str]:
    """Generate an English keyword expansion for a Chinese query by mapping
    recognized Chinese terms to their English equivalents."""
    q = query.lower()
    en_parts: List[str] = []
    for zh, en in _ZH_EN_TOPIC_MAP.items():
        if zh in q:
            en_parts.append(en)
    if not en_parts:
        return None
    return "pet veterinary " + " ".join(en_parts[:4])


_RE_CJK_CHAR_QR = re.compile(r"[\u4e00-\u9fff\u3400-\u4dbf\uf900-\ufaff]")

_ZH_TEMPLATES: tuple[str, ...] = (
    "{q}",
    "{q} 是什么",
    "{q} 的定义",
    "{q} 的症状",
    "{q} 的治疗方法",
    "{q} 的诊断",
    "{q} 的禁忌症",
    "{q} 的用药剂量",
)

_EN_TEMPLATES: tuple[str, ...] = (
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


def _is_mainly_chinese(query: str) -> bool:
    chars = query.replace(" ", "")
    if not chars:
        return False
    return len(_RE_CJK_CHAR_QR.findall(query)) / len(chars) > 0.3


@dataclass(frozen=True)
class TemplateRewriter(QueryRewriter):
    """Template-based query expansion with bilingual support."""

    max_out: int = 10

    def rewrite(self, query: str) -> List[str]:
        q = _normalize_query(query)
        if not q:
            return []

        templates = _ZH_TEMPLATES if _is_mainly_chinese(q) else _EN_TEMPLATES

        out: List[str] = []
        seen: set[str] = set()

        def _add(cand: str) -> None:
            cand = re.sub(r"\s+", " ", (cand or "").strip())
            if cand and cand not in seen:
                out.append(cand)
                seen.add(cand)

        _add(q)

        en_variant = _generate_en_variant(q)
        if en_variant:
            _add(en_variant)

        for t in templates:
            _add(t.format(q=q))
            if len(out) >= int(self.max_out):
                break
        return out


@dataclass(frozen=True)
class LLMRewriter(QueryRewriter):
    """LLM-based query rewriting using OpenAI-compatible API."""

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
