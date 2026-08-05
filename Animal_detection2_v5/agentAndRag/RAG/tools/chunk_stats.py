from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="统计索引 meta.jsonl 的 chunk 长度分布（n_words）")
    parser.add_argument("--index-dir", type=str, default="RAG/data/rag_index_e5", help="索引目录（含 meta.jsonl）")
    args = parser.parse_args()

    index_dir = Path(args.index_dir)
    meta = index_dir / "meta.jsonl"
    if not meta.exists():
        raise SystemExit(f"找不到：{meta}")

    ns: list[int] = []
    bad = 0
    with meta.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                bad += 1
                continue
            n = obj.get("n_words")
            try:
                ns.append(int(n))
            except Exception:
                bad += 1

    ns.sort()
    N = len(ns)
    print("index_dir:", index_dir)
    print("chunks:", N)
    if bad:
        print("bad_rows:", bad)
    if N == 0:
        return

    avg = sum(ns) / float(N)
    p10 = ns[int(0.10 * (N - 1))]
    p50 = ns[int(0.50 * (N - 1))]
    p90 = ns[int(0.90 * (N - 1))]
    print("avg_n_words:", avg)
    print("min:", ns[0], "p10:", p10, "p50:", p50, "p90:", p90, "max:", ns[-1])


if __name__ == "__main__":
    main()


