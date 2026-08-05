from __future__ import annotations

"""
批量生成 Alpaca 格式问答对（带 animal_type 字段），并做去重。

特点：
- 按动物种类配额生成（默认每种 200 条，尽量 1:1）
- 一次性要求模型输出多条，降低重复概率；落盘前做去重（精确 + simhash 近似）
- 支持断点续跑：会读取已有输出，补齐缺口
- 提示词可替换：--prompt-file 指定模板文件即可切换主题/风格

输出（JSONL，每行一条）：
{
  "instruction": "...question...",
  "input": "",
  "output": "...answer...",
  "animal_type": "cat|dog|pig|sheep|cattle|horse",
  "meta": {...}
}
"""

import argparse
import json
import os
import random
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from tqdm import tqdm


# ----------------------------
# 文本规范化 / 去重
# ----------------------------

_RE_WORD = re.compile(r"[A-Za-z][A-Za-z0-9\-]{1,}")


def _norm_question(s: str) -> str:
    s = (s or "").strip().lower()
    s = re.sub(r"\s+", " ", s)
    # 保留字母数字空格，避免标点差异导致重复漏检
    s = re.sub(r"[^a-z0-9\s]", "", s)
    s = re.sub(r"\s+", " ", s).strip()
    return s


def _tokens(s: str) -> List[str]:
    return [t.lower() for t in _RE_WORD.findall(s or "")]


def _simhash64(tokens: Iterable[str]) -> int:
    """
    64-bit SimHash（无第三方依赖）。
    """
    v = [0] * 64
    for t in tokens:
        # 用稳定 hash：sha1 前 8 字节
        h = int.from_bytes(__import__("hashlib").sha1(t.encode("utf-8")).digest()[:8], "big", signed=False)
        for i in range(64):
            bit = (h >> i) & 1
            v[i] += 1 if bit else -1
    out = 0
    for i in range(64):
        if v[i] >= 0:
            out |= 1 << i
    return out


def _hamming64(a: int, b: int) -> int:
    return (a ^ b).bit_count()


@dataclass
class Deduper:
    """
    两级去重：
    - 精确：normalized question string
    - 近似：simhash 64-bit + 分桶 + hamming 阈值
    """

    simhash_max_hamming: int = 3
    bucket_bits: int = 16  # 64 / 16 = 4 段

    def __post_init__(self) -> None:
        self.seen_exact: set[str] = set()
        # bucket_key -> list[(simhash, exact_norm)]
        self.buckets: Dict[int, List[Tuple[int, str]]] = {}

    def _bucket_keys(self, sh: int) -> List[int]:
        keys: List[int] = []
        step = int(self.bucket_bits)
        if 64 % step != 0:
            # 兜底：不分段，单桶
            return [0]
        n = 64 // step
        mask = (1 << step) - 1
        for i in range(n):
            keys.append((sh >> (i * step)) & mask)
        return keys

    def add_if_new(self, question: str) -> bool:
        qn = _norm_question(question)
        if not qn:
            return False
        if qn in self.seen_exact:
            return False

        sh = _simhash64(_tokens(qn))
        # 近似查重：只在同桶里比对
        for bk in self._bucket_keys(sh):
            for old_sh, old_qn in self.buckets.get(bk, []):
                if _hamming64(sh, old_sh) <= int(self.simhash_max_hamming):
                    # 认为重复
                    return False

        # 接受
        self.seen_exact.add(qn)
        for bk in self._bucket_keys(sh):
            self.buckets.setdefault(bk, []).append((sh, qn))
        return True


# ----------------------------
# I/O
# ----------------------------


def read_jsonl(path: Path) -> List[dict]:
    if not path.exists():
        return []
    rows: List[dict] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:
                continue
    return rows


def append_jsonl(path: Path, rows: List[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


# ----------------------------
# Prompt 模板
# ----------------------------


DEFAULT_PROMPT_TEMPLATE = r"""
You are a veterinary-domain dataset generator.

Generate {n} HIGH-QUALITY, NON-OVERLAPPING professional Q&A pairs about: {theme}.
Animal type: {animal}
Language: {language}

Requirements:
- Each item must be a professional knowledge question (not casual pet tips).
- Cover a wide range: anatomy, physiology, pathology, diagnostics, treatment, pharmacology, nursing, husbandry, biosecurity, epidemiology, welfare, ethics, regulations (when relevant).
- Avoid multiple-choice. Generate open-ended questions.
- Avoid extremely long answers; keep answers concise but correct (3-10 sentences).
- Avoid references like "as shown above" / "in the figure".
- Avoid requiring images.
- Avoid copyright excerpts or verbatim textbook passages.
- Avoid asking for personal medical advice.
- Ensure questions are diverse, not just rephrasings.

Output format:
Return ONLY a JSON array. Each element is an object with exactly these keys:
  - "instruction": string (the question)
  - "input": string (leave empty "")
  - "output": string (the answer)

Do NOT wrap in markdown. Do NOT add extra keys. Do NOT add comments.
""".strip()


def load_prompt_template(path: str) -> str:
    p = Path(path)
    if not p.exists():
        raise SystemExit(f"找不到 prompt 模板文件：{p}")
    return p.read_text(encoding="utf-8")


# ----------------------------
# LLM 调用与解析
# ----------------------------


def build_openai_client(*, api_key: str, base_url: str, timeout: int):
    from openai import OpenAI

    return OpenAI(api_key=api_key, base_url=base_url, timeout=int(timeout))


def call_llm_json_array(
    *,
    client,
    model: str,
    system: str,
    user: str,
    temperature: float,
) -> str:
    resp = client.chat.completions.create(
        model=model,
        messages=[{"role": "system", "content": system}, {"role": "user", "content": user}],
        temperature=float(temperature),
        stream=False,
    )
    msg = resp.choices[0].message
    return (getattr(msg, "content", None) or "").strip()


def _extract_json_array(text: str) -> Optional[list]:
    """
    尝试从模型输出中解析 JSON array：
    - 优先 json.loads
    - 失败则截取第一个 '[' 到最后一个 ']' 的子串再 loads
    - 再失败尝试按行 JSONL
    """
    txt = (text or "").strip()
    if not txt:
        return None
    try:
        obj = json.loads(txt)
        if isinstance(obj, list):
            return obj
    except Exception:
        pass

    # 截取 JSON array
    try:
        i = txt.find("[")
        j = txt.rfind("]")
        if i >= 0 and j > i:
            obj2 = json.loads(txt[i : j + 1])
            if isinstance(obj2, list):
                return obj2
    except Exception:
        pass

    # JSONL fallback
    items: List[Any] = []
    ok = False
    for ln in txt.splitlines():
        ln = ln.strip().rstrip(",")
        if not ln or ln.startswith("```"):
            continue
        try:
            o = json.loads(ln)
        except Exception:
            continue
        if isinstance(o, dict):
            items.append(o)
            ok = True
    return items if ok else None


def coerce_alpaca_item(obj: dict) -> Optional[dict]:
    if not isinstance(obj, dict):
        return None
    ins = (obj.get("instruction") or "").strip()
    inp = (obj.get("input") or "").strip()
    out = (obj.get("output") or "").strip()
    if not ins or not out:
        return None
    return {"instruction": ins, "input": inp, "output": out}


# ----------------------------
# 主流程
# ----------------------------


def default_theme_for(animal: str) -> str:
    a = animal.lower().strip()
    if a in {"cat", "dog"}:
        return "companion animal professional knowledge (small animal medicine)"
    if a in {"pig", "sheep", "cattle", "cow", "horse"}:
        return "livestock / production animal professional knowledge (herd health & husbandry)"
    return "veterinary professional knowledge"


def main() -> None:
    ap = argparse.ArgumentParser(description="批量生成 Alpaca 问答对（带 animal_type），并去重/断点续跑。")
    ap.add_argument("--out", type=str, default="RAG/data/qa_gen/animals_alpaca.jsonl", help="输出 JSONL 路径")
    ap.add_argument(
        "--animals",
        type=str,
        default="cat,dog,pig,sheep,cattle,horse",
        help="动物列表（逗号分隔），将尽量按 1:1 补齐",
    )
    ap.add_argument("--per-animal", type=int, default=200, help="每种动物目标样本数")
    ap.add_argument("--batch-n", type=int, default=30, help="每次让模型生成多少条（建议 20-60）")
    ap.add_argument("--max-rounds", type=int, default=2000, help="最多生成轮数，避免死循环")
    ap.add_argument("--language", type=str, default="English", help="生成语言（例如 English / 中文）")
    ap.add_argument("--theme", type=str, default="", help="全局主题（留空则按动物自动选择默认主题）")
    ap.add_argument("--prompt-file", type=str, default="", help="自定义 prompt 模板文件（可切换主题/风格）")

    # LLM 配置
    ap.add_argument("--base-url", type=str, default=os.getenv("OPENAI_BASE_URL", "https://api.deepseek.com"))
    ap.add_argument("--model", type=str, default=os.getenv("OPENAI_MODEL", "deepseek-reasoner"))
    ap.add_argument(
        "--api-key",
        type=str,
        default=os.getenv("OPENAI_API_KEY", "") or os.getenv("API_KEY", "") or os.getenv("DEEPSEEK_API_KEY", ""),
    )
    ap.add_argument("--timeout", type=int, default=12000)
    ap.add_argument("--temperature", type=float, default=0.7)

    # 去重
    ap.add_argument("--simhash-max-hamming", type=int, default=3, help="近似去重阈值（越大越严格）")

    # 运行控制
    ap.add_argument("--sleep", type=float, default=0.0, help="每次请求后 sleep 秒数（防限流）")
    ap.add_argument("--seed", type=int, default=42)

    args = ap.parse_args()

    random.seed(int(args.seed))

    out_path = Path(args.out)
    animals = [a.strip() for a in str(args.animals).split(",") if a.strip()]
    if not animals:
        raise SystemExit("animals 为空")

    if not str(args.api_key or "").strip():
        raise SystemExit("未找到可用 API KEY：请设置环境变量 OPENAI_API_KEY（或 API_KEY / DEEPSEEK_API_KEY），或传入 --api-key")

    # prompt
    prompt_template = DEFAULT_PROMPT_TEMPLATE if not str(args.prompt_file).strip() else load_prompt_template(args.prompt_file)

    # 载入已有数据（断点续跑）并初始化去重器
    existing = read_jsonl(out_path)
    deduper = Deduper(simhash_max_hamming=int(args.simhash_max_hamming))

    per_animal_have: Dict[str, int] = {a: 0 for a in animals}
    total_loaded = 0
    for r in existing:
        a = str(r.get("animal_type") or "").strip()
        if a in per_animal_have:
            per_animal_have[a] += 1
        q = str(r.get("instruction") or "")
        if q:
            deduper.add_if_new(q)
        total_loaded += 1

    target = int(args.per_animal)
    print("[RESUME] loaded:", total_loaded, "existing rows")
    print("[RESUME] per_animal:", per_animal_have)
    need_total = sum(max(0, target - per_animal_have[a]) for a in animals)
    if need_total <= 0:
        print("[DONE] 已满足每种动物目标数，无需生成。")
        return

    # LLM
    client = build_openai_client(api_key=str(args.api_key), base_url=str(args.base_url), timeout=int(args.timeout))
    system = "You are a helpful assistant. Follow the user's format requirements strictly."

    # 轮询补齐：每轮挑“当前最缺的动物”去生成，尽量保持 1:1
    rounds = 0
    newly_written = 0
    pbar = tqdm(total=need_total, desc="generate_unique", unit="qa")

    try:
        while rounds < int(args.max_rounds):
            # 选择最缺的动物
            animal = min(animals, key=lambda a: per_animal_have.get(a, 0))
            if per_animal_have[animal] >= target:
                # 全部达标
                break

            theme = str(args.theme).strip() or default_theme_for(animal)
            n = min(int(args.batch_n), max(1, target - per_animal_have[animal]) + 10)  # 稍微超发，后续去重
            user = prompt_template.format(n=n, theme=theme, animal=animal, language=str(args.language))

            text = call_llm_json_array(
                client=client,
                model=str(args.model),
                system=system,
                user=user,
                temperature=float(args.temperature),
            )
            arr = _extract_json_array(text)
            if not isinstance(arr, list):
                rounds += 1
                if float(args.sleep) > 0:
                    time.sleep(float(args.sleep))
                continue

            accepted_rows: List[dict] = []
            for it in arr:
                base = coerce_alpaca_item(it if isinstance(it, dict) else {})
                if base is None:
                    continue
                q = base["instruction"]
                if not deduper.add_if_new(q):
                    continue
                if per_animal_have[animal] >= target:
                    # 本轮已补齐该动物，剩余丢弃（保证 1:1）
                    continue
                row = {
                    **base,
                    "animal_type": animal,
                    "meta": {
                        "theme": theme,
                        "model": str(args.model),
                        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    },
                }
                accepted_rows.append(row)
                per_animal_have[animal] += 1
                newly_written += 1
                pbar.update(1)

                if per_animal_have[animal] >= target:
                    break

            if accepted_rows:
                append_jsonl(out_path, accepted_rows)

            rounds += 1
            if float(args.sleep) > 0:
                time.sleep(float(args.sleep))

            # 进度打印（低频）
            if rounds % 10 == 0:
                print("\n[PROGRESS] rounds:", rounds, "new:", newly_written, "per_animal:", per_animal_have)

            if sum(max(0, target - per_animal_have[a]) for a in animals) <= 0:
                break
    finally:
        pbar.close()

    print("\n[DONE] out:", out_path)
    print("[DONE] rounds:", rounds, "new_rows:", newly_written)
    print("[DONE] per_animal:", per_animal_have)
    miss = {a: max(0, target - per_animal_have[a]) for a in animals}
    if any(v > 0 for v in miss.values()):
        print("[WARN] 未完全补齐（可重跑继续补齐）:", miss)


if __name__ == "__main__":
    main()


