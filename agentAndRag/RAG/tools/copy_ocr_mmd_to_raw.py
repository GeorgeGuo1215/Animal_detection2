"""将 OCR 产出的 <id>.mmd 拷到 RAG/data/raw（增量入库输入）。

用法:
  python -m RAG.tools.copy_ocr_mmd_to_raw
  python -m RAG.tools.copy_ocr_mmd_to_raw --ocr-root PATH --raw-dir PATH --only 089,080
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def _repo_rag() -> Path:
    return Path(__file__).resolve().parents[1]


def default_ocr_root() -> Path:
    return Path(
        r"C:\Users\ROG\Desktop\大四\上服务器文件及依赖\output\root\autodl-tmp"
        r"\.autodl\deepseek-ocr-offline\deepseek-ocr\DeepSeek-OCR"
        r"\DeepSeek-OCR-master\DeepSeek-OCR-vllm\output"
    )


def copy_books(*, ocr_root: Path, raw_dir: Path, only: set[str] | None = None) -> list[str]:
    raw_dir.mkdir(parents=True, exist_ok=True)
    copied: list[str] = []
    for d in sorted(ocr_root.iterdir()):
        if not d.is_dir() or not d.name.isdigit():
            continue
        if only and d.name not in only:
            continue
        src = d / f"{d.name}.mmd"
        if not src.exists():
            continue
        dst = raw_dir / f"{d.name}.mmd"
        shutil.copy2(src, dst)
        copied.append(d.name)
    return copied


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--ocr-root", type=str, default=str(default_ocr_root()))
    p.add_argument("--raw-dir", type=str, default=str(_repo_rag() / "data" / "raw"))
    p.add_argument("--only", type=str, default="")
    args = p.parse_args()
    only = {x.strip() for x in args.only.split(",") if x.strip()} or None
    done = copy_books(ocr_root=Path(args.ocr_root), raw_dir=Path(args.raw_dir), only=only)
    print(f"copied {len(done)} books → {args.raw_dir}: {done}")


if __name__ == "__main__":
    main()
