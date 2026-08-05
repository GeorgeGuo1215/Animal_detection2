from __future__ import annotations

"""
从 OCR/VLLM 输出目录递归提取 .mmd 文件，并导入到 RAG/data/raw。

需求点（按用户约定）：
- 递归扫描 src_root 下的 .mmd
- 排除 *_det.mmd（不需要）
- 以“顶层书文件夹名（src_root 下的一级目录名）”为去重 key
  - 若 dst_root/<book_dir> 已存在且包含至少一个非 *_det.mmd 的 .mmd，则跳过（断点续传）
- 拷贝时保留原目录名：dst_root/<book_dir>/<原文件名>.mmd
- 支持 dry-run、日志、统计汇总

典型输入结构示例：
src_root/
  64/
    64.mmd
    64_det.mmd
  Some Book Name/
    Some Book Name.mmd
    Some Book Name_det.mmd
"""

import argparse
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


def _is_mmd(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() == ".mmd"


def _is_det_mmd(path: Path) -> bool:
    # 只排除 *_det.mmd（大小写不敏感）
    return path.name.lower().endswith("_det.mmd")


def iter_mmd_files(src_root: Path) -> Iterable[Path]:
    # 递归遍历（不依赖 glob，兼容长路径）
    for p in src_root.rglob("*.mmd"):
        if _is_mmd(p) and (not _is_det_mmd(p)):
            yield p


def book_dir_from_src(src_root: Path, mmd_path: Path) -> Optional[str]:
    """
    根据“src_root 下的一级目录名”作为书的唯一 key。
    e.g. src_root/64/64.mmd -> "64"
    """
    try:
        rel = mmd_path.relative_to(src_root)
    except Exception:
        return None
    if not rel.parts:
        return None
    return str(rel.parts[0])


def dst_has_book(dst_root: Path, book_dir: str) -> bool:
    """
    断点续传判断：只要 dst_root/book_dir 下已经有至少一个非 *_det.mmd 的 .mmd，就认为已导入。
    """
    d = dst_root / book_dir
    if not d.exists() or not d.is_dir():
        return False
    for p in d.glob("*.mmd"):
        if p.is_file() and (not _is_det_mmd(p)):
            return True
    return False


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def write_jsonl(path: Path, rows: Iterable[dict]) -> None:
    ensure_dir(path.parent)
    with path.open("a", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


@dataclass
class Stats:
    scanned: int = 0
    unique_books_found: int = 0
    copied_books: int = 0
    skipped_books: int = 0
    copied_files: int = 0
    skipped_files: int = 0


def main() -> None:
    ap = argparse.ArgumentParser(description="递归导入 .mmd 到 RAG/data/raw（排除 *_det.mmd，按书目录名去重/断点续传）")
    ap.add_argument(
        "--src-root",
        type=str,
        required=True,
        help="源目录（DeepSeek-OCR-vllm 输出目录），例如：C:\\...\\DeepSeek-OCR-vllm\\output",
    )
    ap.add_argument(
        "--dst-root",
        type=str,
        default="RAG/data/raw",
        help="目标 raw 目录（默认 RAG/data/raw）",
    )
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不实际拷贝")
    ap.add_argument(
        "--log",
        type=str,
        default="RAG/tools/out/import_mmd_to_raw_log.jsonl",
        help="导入日志 JSONL（默认写到 RAG/tools/out/）",
    )
    ap.add_argument("--overwrite", action="store_true", help="若目标文件已存在则覆盖（默认不覆盖）")
    args = ap.parse_args()

    src_root = Path(args.src_root)
    dst_root = Path(args.dst_root)
    log_path = Path(args.log)

    if not src_root.exists():
        raise SystemExit(f"src_root 不存在：{src_root}")
    ensure_dir(dst_root)
    ensure_dir(log_path.parent)

    # 先收集：book_dir -> list[mmd_path]
    by_book: Dict[str, List[Path]] = {}
    scanned = 0
    for p in iter_mmd_files(src_root):
        scanned += 1
        bd = book_dir_from_src(src_root, p)
        if not bd:
            continue
        by_book.setdefault(bd, []).append(p)

    st = Stats(scanned=scanned, unique_books_found=len(by_book))
    print("[FOUND] mmd_files:", scanned)
    print("[FOUND] unique_book_dirs:", st.unique_books_found)
    print("[DST] raw_dir:", dst_root)

    # 稳定遍历顺序（便于复现）
    ts = time.strftime("%Y-%m-%dT%H:%M:%S")
    rows: List[dict] = []

    for book_dir in sorted(by_book.keys()):
        src_files = sorted(by_book[book_dir], key=lambda x: str(x))

        if dst_has_book(dst_root, book_dir):
            st.skipped_books += 1
            st.skipped_files += len(src_files)
            rows.append(
                {
                    "ts": ts,
                    "book_dir": book_dir,
                    "action": "skip_book",
                    "reason": "dst_has_book_dir_with_mmd",
                    "src_files": [str(p) for p in src_files],
                }
            )
            continue

        # 拷贝本书
        book_dst = dst_root / book_dir
        if not args.dry_run:
            ensure_dir(book_dst)

        copied_any = False
        for src in src_files:
            dst = book_dst / src.name
            if dst.exists() and (not bool(args.overwrite)):
                st.skipped_files += 1
                rows.append(
                    {
                        "ts": ts,
                        "book_dir": book_dir,
                        "action": "skip_file",
                        "reason": "dst_exists_no_overwrite",
                        "src": str(src),
                        "dst": str(dst),
                    }
                )
                continue

            if args.dry_run:
                st.copied_files += 1
                copied_any = True
                rows.append(
                    {
                        "ts": ts,
                        "book_dir": book_dir,
                        "action": "plan_copy",
                        "src": str(src),
                        "dst": str(dst),
                    }
                )
                continue

            shutil.copy2(src, dst)
            st.copied_files += 1
            copied_any = True
            rows.append(
                {
                    "ts": ts,
                    "book_dir": book_dir,
                    "action": "copy",
                    "src": str(src),
                    "dst": str(dst),
                }
            )

        if copied_any:
            st.copied_books += 1
        else:
            st.skipped_books += 1

        # 批量写日志（避免一次性太大）
        if len(rows) >= 2000:
            write_jsonl(log_path, rows)
            rows = []

    if rows:
        write_jsonl(log_path, rows)

    print("\n[SUMMARY]")
    print("- scanned_mmd_files:", st.scanned)
    print("- unique_books_found:", st.unique_books_found)
    print("- copied_books:", st.copied_books)
    print("- skipped_books:", st.skipped_books)
    print("- copied_files:", st.copied_files)
    print("- skipped_files:", st.skipped_files)
    print("- log:", log_path)
    if args.dry_run:
        print("[NOTE] dry-run：未实际拷贝")


if __name__ == "__main__":
    main()


