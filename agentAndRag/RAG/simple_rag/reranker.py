from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class RerankResult:
    index: int
    score: float


class CrossEncoderReranker:
    """
    轻量 reranker：基于 sentence-transformers 的 CrossEncoder。

    用法：
    1) 先召回一批小块 candidates（例如 10/20）
    2) 用 reranker 对 (query, chunk_text) 打分
    3) 取 top_k rerank 后的块，再做邻居拼接/上下文打包喂给 LLM
    """

    def __init__(self, model_name_or_path: str, *, device: Optional[str] = None) -> None:
        from sentence_transformers import CrossEncoder

        self.model_name_or_path = model_name_or_path
        self.model = CrossEncoder(model_name_or_path, device=device)

    def score_pairs(
        self,
        *,
        query: str,
        passages: Sequence[str],
        batch_size: int = 32,
    ) -> List[float]:
        q = (query or "").strip()
        if not passages:
            return []
        tokenizer = self.model.tokenizer
        query_ids = tokenizer(q, add_special_tokens=False)["input_ids"]
        if len(query_ids) > 256:
            raise ValueError("reranker query exceeds 256 tokens")
        limit = min(int(self.model.max_length or 512), 512)
        budget = limit - len(query_ids) - tokenizer.num_special_tokens_to_add(pair=True)
        if budget < 32:
            raise ValueError("reranker pair budget too small")
        pairs: List[Tuple[str, str]] = []
        owners: list[int] = []
        for index, passage in enumerate(passages):
            text = (passage or "").strip()
            encoded = tokenizer(text, add_special_tokens=False, return_offsets_mapping=True, verbose=False)
            offsets = encoded["offset_mapping"]
            if not offsets:
                pairs.append((q, text)); owners.append(index)
                continue
            start = 0
            while start < len(offsets):
                end = min(start + budget, len(offsets))
                window = text[offsets[start][0]:offsets[end-1][1]]
                # Re-tokenizing a subword boundary can add a token; shrink to the actual pair budget.
                while len(tokenizer(q, window)["input_ids"]) > limit and end > start + 1:
                    end -= 1
                    window = text[offsets[start][0]:offsets[end-1][1]]
                pairs.append((q, window)); owners.append(index)
                if end == len(offsets):
                    break
                start = max(start + 1, end - 32)
        # CrossEncoder.predict 返回 np.ndarray 或 list[float]
        scores = self.model.predict(pairs, batch_size=int(batch_size), show_progress_bar=False)
        result = [float("-inf")] * len(passages)
        for owner, score in zip(owners, scores):
            result[owner] = max(result[owner], float(score))
        return result

    def rerank(
        self,
        *,
        query: str,
        passages: Sequence[str],
        top_k: int,
        batch_size: int = 32,
    ) -> List[RerankResult]:
        if top_k <= 0 or not passages:
            return []
        scores = self.score_pairs(query=query, passages=passages, batch_size=batch_size)
        results = [RerankResult(index=i, score=float(s)) for i, s in enumerate(scores)]
        results.sort(key=lambda x: x.score, reverse=True)
        return results[: int(top_k)]


