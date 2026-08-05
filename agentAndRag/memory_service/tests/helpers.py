"""测试与长跑模拟共用的确定性替身。

真实 embedding 模型和 LLM 都是不确定的，也都很慢。这里的假实现让整条链路可以
被逐位断言，同时完全不加载 torch、不碰显卡、不发网络请求。
"""

from __future__ import annotations

import hashlib
import math
import re
from datetime import datetime
from typing import Dict, List, Sequence

BASE_TIME = datetime(2026, 8, 1, 9, 0, 0)

# 假向量的主题维度。同一主题的文本落在同一维度上，因此语义相近的文本余弦相似度高，
# 话题合并、检索排序这些依赖相似度的逻辑才能被确定性验证。
TOPICS = [
    "喂养",
    "疫苗",
    "皮肤",
    "体重",
    "行为",
    "驱虫",
    "牙齿",
    "过敏",
    "绝育",
    "运动",
]


def fake_vector(text: str, dim: int = 384) -> List[float]:
    """确定性的假 embedding。

    命中主题词就在对应维度置位；一个都不命中时按文本哈希散到尾部维度，
    保证不同文本不会全都退化成同一个向量。
    """
    vec = [0.0] * dim
    matched = False
    for index, topic in enumerate(TOPICS):
        if topic in text:
            vec[index] = 1.0
            matched = True
    if not matched:
        digest = hashlib.md5(text.encode("utf-8")).hexdigest()
        vec[len(TOPICS) + int(digest, 16) % (dim - len(TOPICS))] = 1.0
    norm = math.sqrt(sum(x * x for x in vec)) or 1.0
    return [x / norm for x in vec]


class FakeEmbedder:
    """满足 Embedder 协议的确定性实现。"""

    def __init__(self, dim: int = 384) -> None:
        self.dim = dim
        self.calls = 0

    def embed_documents(self, texts: Sequence[str]) -> List[List[float]]:
        self.calls += 1
        return [fake_vector(t, self.dim) for t in texts]

    def embed_query(self, text: str) -> List[float]:
        self.calls += 1
        return fake_vector(text, self.dim)


_TURN_RE = re.compile(r"^\[(\d+)\]\s*(.*)$")


def _parse_numbered_turns(prompt: str) -> Dict[int, str]:
    """还原提示词里 [n] 标注的对话轮次，供假 LLM 做主题归类。"""
    turns: Dict[int, str] = {}
    for line in prompt.splitlines():
        match = _TURN_RE.match(line.strip())
        if match:
            index = int(match.group(1))
            turns[index] = turns.get(index, "") + match.group(2)
    return turns


class RecordingLLM:
    """按提示词特征产出确定性结果的假 LLM，并记录每次调用。

    调用次数本身就是被断言的对象：把每批提升的 LLM 调用从上游的 2N+3 压到固定 3 次
    是本方案的核心改动之一，没有计数就守不住这个性质。

    主题归类的行为也尽量贴近真实模型：按对话里出现的主题词分组，并如实填写
    每个主题覆盖了哪几轮，这样 consolidator 的分派逻辑才真的被走到。
    """

    def __init__(self) -> None:
        self.calls: List[Dict[str, str]] = []

    @property
    def call_count(self) -> int:
        return len(self.calls)

    def complete(
        self, messages, *, temperature: float = 0.3, max_tokens: int = 1024
    ) -> str:
        prompt = "\n".join(m.get("content", "") for m in messages)
        self.calls.append({"prompt": prompt})

        if "主题摘要" in prompt:
            return self._summaries(prompt)

        topics = [t for t in TOPICS if t in prompt]
        if "用户画像" in prompt:
            concerns = ",".join(f'"{t}"' for t in topics[:2])
            return (
                '{"communication": {"style": "偏好通俗解释"},'
                f' "concerns": [{concerns}]}}'
            )

        if "知识条目" in prompt:
            facts = ",".join(f'"主人关注{t}相关问题"' for t in topics[:3])
            return f'{{"facts": [{facts}]}}'

        return "{}"

    def _summaries(self, prompt: str) -> str:
        turns = _parse_numbered_turns(prompt)
        grouped: Dict[str, List[int]] = {}
        for index in sorted(turns):
            text = turns[index]
            theme = next((t for t in TOPICS if t in text), "日常")
            grouped.setdefault(theme, []).append(index)

        if not grouped:
            grouped = {"日常": []}

        items = ",".join(
            '{{"theme":"{theme}","content":"关于{theme}的对话","keywords":["{theme}"],'
            '"turns":[{turns}]}}'.format(theme=theme, turns=",".join(str(i) for i in idxs))
            for theme, idxs in grouped.items()
        )
        return f'{{"summaries": [{items}]}}'


class FailingLLM:
    """总是抛异常的 LLM，用于验证任务重试与错误记录。"""

    def __init__(self, message: str = "boom") -> None:
        self.message = message
        self.call_count = 0

    def complete(self, messages, *, temperature: float = 0.3, max_tokens: int = 1024) -> str:
        self.call_count += 1
        raise RuntimeError(self.message)
