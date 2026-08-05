from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Iterable, Iterator, List, Optional

HERE = Path(__file__).resolve()
# 允许直接运行：python RAG/experiments/build_qa_evalset_from_alpaca.py
sys.path.insert(0, str(HERE.parent))      # experiments/
sys.path.insert(0, str(HERE.parents[1]))  # RAG/

from common import write_jsonl  # noqa: E402


_RE_CHOICE = re.compile(r"\b([ABCD])\b", re.IGNORECASE)


def load_alpaca_json(path: Path) -> list[dict]:
    return json.loads(path.read_text(encoding="utf-8"))


def extract_query_from_input(inp: str) -> str:
    """
    按你说的规则：
    - 以 '\\n\\n' 作为分界线
    - 只取前半段（“之前的问题”）作为 query
    """
    s = (inp or "").strip()
    if "\n\n" in s:
        s = s.split("\n\n", 1)[0].strip()
    # 去掉 Markdown 强调符号
    s = s.strip().strip("*").strip()
    return s


def extract_gold_choice(output: str) -> Optional[str]:
    """
    从类似 '正确答案是 A) ...' 里提取选项字母。
    """
    if not output:
        return None
    m = _RE_CHOICE.search(output)
    if not m:
        return None
    return m.group(1).upper()


def iter_rows(paths: List[Path]) -> Iterator[dict]:
    for p in paths:
        data = load_alpaca_json(p)
        dataset = p.stem
        for i, item in enumerate(data):
            inp = item.get("input", "")
            out = item.get("output", "")
            query = extract_query_from_input(inp)
            gold = extract_gold_choice(out)
            if not query:
                continue
            yield {
                "dataset": dataset,
                "qid": f"{dataset}:{i}",
                "query": query,
                # 保留完整题干（含选项），便于后续做 RAG+LLM 作答
                "question_full": inp,
                "gold_output": out,
                "gold_choice": gold,
            }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="从 alpaca 选择题 JSON 构建 QA 评测集 jsonl（query 来自 input 的前段）"
    )
    parser.add_argument(
        "--inputs",
        nargs="*",
        default=[
            "record-thinkking/300QuestionsInAds_alpaca.json",
            "record-thinkking/300QuestionsInAnatomy_alpaca.json",
        ],
        help="输入 alpaca.json 文件路径（可多个）",
    )
    parser.add_argument("--out", type=str, default="RAG/experiments/out/qa_eval.jsonl")
    args = parser.parse_args()

    paths = [Path(x) for x in args.inputs]
    rows = list(iter_rows(paths))
    write_jsonl(Path(args.out), rows)
    print(f"写入 {len(rows)} 条 QA eval → {args.out}")


if __name__ == "__main__":
    main()



