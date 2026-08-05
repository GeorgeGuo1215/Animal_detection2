from __future__ import annotations

"""
把“答案在书末尾”的 MCQ 书籍，自动回填答案到对应题目中（用于 RAG 检索/分块时能取到答案）。

目标书籍（默认）：
- RAG/data/raw/64/64.mmd
- RAG/data/raw/64  另外一个300   65到67/64  另外一个300   65到67.mmd
- RAG/data/raw/64 第3个300  65到67/64 第3个300  65到67.mmd
- RAG/data/raw/66  300 questions and answers in diagnostic aids for veterinary nurses/66  300 questions and answers in diagnostic aids for veterinary nurses.mmd

特性：
- 排除 *_det.mmd
- 自动识别答案区（优先找 "Answers" 标记；找不到则用启发式）
- 支持多种答案格式：
  - "1) b 23) a ..."
  - HTML table: <td>233)</td><td>c</td> ...
  - 表格风格：| 89 | b |
- 安全：默认 dry-run，不改文件；--inplace 才写回；--backup 默认开启
- 幂等：若题目块已包含 "Answer:" 行则跳过插入
"""

import argparse
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


def _is_det_mmd(p: Path) -> bool:
    return p.name.lower().endswith("_det.mmd")


def _read_text(p: Path) -> str:
    return p.read_text(encoding="utf-8", errors="ignore")


def _write_text(p: Path, text: str) -> None:
    p.write_text(text, encoding="utf-8")


# ----------------------------
# 解析：答案区定位
# ----------------------------


_RE_ANS_HEADER = re.compile(r"(?im)^\s*answers\b.*$", re.MULTILINE)


def _answer_region_score(snippet: str) -> int:
    """
    给一个片段打“像答案区”的分：
    - pairs 越多越像
    - 选项行越多越不像（题干区会有大量 a)/b)/c)/d)）
    """
    s = snippet or ""
    pairs = len(re.findall(r"(\d{1,4})\s*[)\.]\s*[a-dA-D](?=\s|$|<|\.|,)", s))
    opt_lines = len(re.findall(r"(?im)^\s*[a-dA-D]\s*[)\.]\s+.+", s))
    # 大量答案配对 + 很少选项行
    return int(pairs * 10 - opt_lines * 50)


def find_answer_section_start(text: str) -> Optional[int]:
    """
    返回“最可能的答案区”在 text 中的起始字符索引；找不到返回 None。

    注意：有些书名/前言会出现 "Questions & Answers ..."，会干扰简单的首次匹配。
    因此这里会：
    - 找到所有行首 "Answers" 的位置
    - 对每个候选点取后续一段文本打分，选分最高的那个
    """
    s = text or ""
    candidates = [m.start() for m in _RE_ANS_HEADER.finditer(s)]
    if not candidates:
        return None

    best_pos = None
    best_score = None
    for pos in candidates:
        # 只看后面一段（避免整本太大）；答案表通常紧随其后
        snippet = s[pos : pos + 20000]
        sc = _answer_region_score(snippet)
        if best_score is None or sc > best_score:
            best_score = sc
            best_pos = pos

    # 分数太低说明这些 "Answers" 可能都只是书名/目录
    if best_score is not None and best_score < 200:  # 经验阈值：至少要有几十个配对才像答案表
        return None
    return int(best_pos) if best_pos is not None else None


def looks_like_answer_region(text: str) -> bool:
    """
    启发式：答案区通常满足：
    - 很多 "123) a" 这种短配对
    - 很少出现 "a) xxx" 这种选项文本行
    """
    s = text or ""
    pairs = len(re.findall(r"(\d{1,4})\s*[)\.]\s*[a-dA-D](?=\s|$|<|\.|,)", s))
    opt_lines = len(re.findall(r"(?im)^\s*[a-dA-D]\s*[)\.]\s+.+", s))
    # pairs 多且 opt_lines 少：更像答案表
    return pairs >= 30 and opt_lines <= 3


def find_answer_section_start_heuristic(text: str) -> Optional[int]:
    """
    如果没找到 "Answers"，从后往前找最像答案页的区域。
    """
    sep = "<--- Page Split --->"
    if sep not in (text or ""):
        return None
    parts = (text or "").split(sep)
    # 从后往前找第一个“像答案页”的 page
    for i in range(len(parts) - 1, -1, -1):
        if looks_like_answer_region(parts[i]):
            # 计算字符 offset
            offset = sum(len(p) + len(sep) for p in parts[:i])
            return int(offset)
    return None


# ----------------------------
# 解析：答案提取
# ----------------------------


def extract_answers(text: str) -> Dict[int, str]:
    """
    从答案区文本中提取 {题号 -> 答案字母(A/B/C/D)}。
    """
    out: Dict[int, str] = {}
    s = text or ""

    # 1) 标准格式：1) a / 1. a / 1)a / 1) d.
    for num, ch in re.findall(r"(\d{1,4})\s*[)\.]\s*([a-dA-D])(?=\s|$|<|\.|,)", s):
        try:
            out[int(num)] = str(ch).upper()
        except Exception:
            continue

    # 2) HTML table：<td>233)</td><td>c</td>
    for num, ch in re.findall(r"<td>\s*(\d{1,4})\s*\)\s*</td>\s*<td>\s*([a-dA-D])\s*</td>", s):
        try:
            out[int(num)] = str(ch).upper()
        except Exception:
            continue

    # 3) pipe table：| 89 | b |
    for num, ch in re.findall(r"\|\s*(\d{1,4})\s*\|\s*([a-dA-D])\s*\|", s):
        try:
            out[int(num)] = str(ch).upper()
        except Exception:
            continue

    return out


# ----------------------------
# 解析：题目块提取 & 回填
# ----------------------------


@dataclass(frozen=True)
class QBlock:
    number: int
    start: int
    end: int
    text: str


def _line_no(text: str, char_pos: int) -> int:
    """
    将字符偏移转为 1-based 行号（用于终端定位）。
    """
    if char_pos <= 0:
        return 1
    return int((text or "").count("\n", 0, int(char_pos)) + 1)


def _preview_first_line(block_text: str, max_len: int = 140) -> str:
    s = (block_text or "").strip().splitlines()[0] if (block_text or "").strip() else ""
    s = re.sub(r"\s+", " ", s).strip()
    if len(s) > int(max_len):
        return s[: int(max_len)] + "..."
    return s


def iter_question_blocks(text: str, *, stop_at: Optional[int] = None, allow_dot_numbering: bool = False) -> List[QBlock]:
    """
    从全文中提取题目块：
    - 以 "数字)" 或 "数字." 开头
    - 块内必须包含 a)/b)/c)/d) 选项行（避免把答案表当题目）
    """
    s = text if stop_at is None else (text[: int(stop_at)])
    blocks: List[QBlock] = []

    # block：从题号开始，到下一个题号行/到文件末尾（或 stop_at）为止
    # 额外捕获分隔符（')' 或 '.'），但默认只接受 "数字)"（这些 MCQ 书是这种格式）
    pat = re.compile(r"(?ms)(^|\n)(\d{1,4})\s*([)\.])\s*(.*?)(?=(\n\d{1,4}\s*[)\.]\s)|\Z)")

    for m in pat.finditer(s):
        try:
            num = int(m.group(2))
        except Exception:
            continue
        delim = str(m.group(3) or ")")
        body = m.group(4) or ""
        # 用原始分隔符重建，避免把 "2." 变成 "2)" 造成误导
        full = f"{num}{delim} {body}".strip()

        # 默认不把 "数字." 当题目起始（避免把编目/目录条目误判成题目）
        if delim == "." and (not bool(allow_dot_numbering)):
            continue

        # 过滤一种常见噪声：
        # 出现 "2." 这种编目/目录条目，并且该块内还包含真正的 "2)" 题目时，
        # 说明这个 "2." 只是目录条目把后面题目吞进来了，应跳过此块，避免重复题号。
        if delim == ".":
            if re.search(rf"(?im)^\s*{num}\s*\)\s+", full):
                continue

        # 至少包含部分选项行（允许 OCR 丢失个别选项，避免大量漏检）
        has_a = re.search(r"(?im)^\s*a\s*[)\.]\s*", full) is not None
        has_b = re.search(r"(?im)^\s*b\s*[)\.]\s*", full) is not None
        has_c = re.search(r"(?im)^\s*c\s*[)\.]\s*", full) is not None
        has_d = re.search(r"(?im)^\s*d\s*[)\.]\s*", full) is not None
        n_opts = sum([has_a, has_b, has_c, has_d])
        if n_opts < 2:
            continue

        start = int(m.start(2))  # 题号数字起点
        # 让 start 回到行首（更稳）
        line_start = s.rfind("\n", 0, start)
        start2 = 0 if line_start < 0 else (line_start + 1)
        end = int(m.end())
        blocks.append(QBlock(number=num, start=start2, end=end, text=s[start2:end]))

    return blocks


def already_has_answer(block_text: str) -> bool:
    return re.search(r"(?im)^\s*answer\s*:\s*[a-dA-D]\b", block_text or "") is not None


def _remove_answer_lines(block_text: str) -> Tuple[str, Optional[str]]:
    """
    移除块内的 Answer: X 行，并返回 (clean_block, removed_answer_letter)。
    主要用于把旧格式（单独 Answer 行）迁移为“题干行内”格式。
    """
    removed: Optional[str] = None
    out_lines: List[str] = []
    for ln in (block_text or "").splitlines():
        m = re.match(r"(?im)^\s*answer\s*:\s*([a-dA-D])\b", ln.strip())
        if m:
            removed = str(m.group(1)).upper()
            continue
        out_lines.append(ln)
    return "\n".join(out_lines), removed


def _inject_answer_inline(block_text: str, *, number: int, answer: str) -> str:
    """
    把答案插入到题干行末尾：
    例：291) The epiglottis is composed of:  -> 291) The epiglottis is composed of:  [Answer: A]
    若题干行末尾已经包含 [Answer: X] 则不重复插入。
    """
    ans = (answer or "").strip().upper()
    if ans not in {"A", "B", "C", "D"}:
        return block_text

    lines = (block_text or "").splitlines()
    if not lines:
        return block_text

    # 找到题干行（通常是第一行，以 "<num>)" 开头）
    qline_idx = None
    q_pat = re.compile(rf"^\s*{int(number)}\)\s*")
    for i, ln in enumerate(lines[:8]):  # 题干行通常很靠前
        if q_pat.search(ln):
            qline_idx = i
            break
    if qline_idx is None:
        return block_text

    qline = lines[qline_idx]
    if re.search(r"(?i)\[\s*answer\s*:\s*[a-d]\s*\]\s*$", qline.strip()):
        return block_text

    # 保留 markdown 双空格换行语义：先去尾空格，再补回原有尾空格
    tail_ws = ""
    mws = re.search(r"(\s*)$", qline)
    if mws:
        tail_ws = mws.group(1)
    qline0 = qline.rstrip()
    # 若原本有 markdown line break（两个空格）也保留
    lines[qline_idx] = f"{qline0}  [Answer: {ans}]{tail_ws}"
    return "\n".join(lines)


def attach_answers_to_text(
    text: str,
    *,
    answers: Dict[int, str],
    answer_section_start: Optional[int],
    allow_dot_numbering: bool,
) -> Tuple[str, dict]:
    """
    返回 (new_text, stats)
    """
    stop_at = answer_section_start
    qblocks = iter_question_blocks(text, stop_at=stop_at, allow_dot_numbering=allow_dot_numbering)

    matched = 0
    missing = 0
    skipped_already = 0
    skipped_duplicate_number = 0
    seen_numbers: set[int] = set()
    duplicate_numbers: set[int] = set()
    # num -> list[occurrence], each occurrence: {line, start, end, preview}
    occurrences: Dict[int, List[dict]] = {}

    # 重建文本：按块拼接并在块内部“题干行内”注入答案
    out_parts: List[str] = []
    cur = 0
    for qb in qblocks:
        # 同题号重复（OCR 重复页/重复段落）：只对第一次出现插入答案，避免污染文本
        if qb.number in seen_numbers:
            skipped_duplicate_number += 1
            duplicate_numbers.add(int(qb.number))
            occurrences.setdefault(int(qb.number), []).append(
                {
                    "line": _line_no(text, qb.start),
                    "start": int(qb.start),
                    "end": int(qb.end),
                    "preview": _preview_first_line(qb.text),
                }
            )
            continue
        seen_numbers.add(qb.number)
        occurrences.setdefault(int(qb.number), []).append(
            {
                "line": _line_no(text, qb.start),
                "start": int(qb.start),
                "end": int(qb.end),
                "preview": _preview_first_line(qb.text),
            }
        )

        # 先输出块前文本
        out_parts.append(text[cur:qb.start])
        cur = qb.end

        # 迁移旧格式：如果块内已经有 Answer: X 行，把它移除，并最终统一用行内格式
        clean_block, removed_ans = _remove_answer_lines(qb.text)
        if removed_ans:
            skipped_already += 1  # 统计上算“已有答案信息”

        ans = answers.get(qb.number) or removed_ans
        if not ans:
            # 没找到答案：保持原块不变（但已移除 Answer 行的话也不要动，避免丢信息）
            missing += 1
            out_parts.append(qb.text)
            continue

        new_block = _inject_answer_inline(clean_block, number=qb.number, answer=ans)
        if new_block != qb.text:
            matched += 1
        out_parts.append(new_block)

    out_parts.append(text[cur:])
    new_text = "".join(out_parts)

    ans_numbers = set(int(k) for k in answers.keys())
    missing_q_numbers = sorted(ans_numbers - seen_numbers)
    extra_q_numbers = sorted(seen_numbers - ans_numbers)
    dup_list = sorted(duplicate_numbers)
    dup_locs: Dict[int, List[dict]] = {}
    for n in dup_list:
        occs = occurrences.get(int(n), [])
        if len(occs) >= 2:
            # 只输出前两处，终端更清爽
            dup_locs[int(n)] = occs[:2]

    stats = {
        "q_blocks": len(qblocks),
        "q_unique_numbers": len(seen_numbers),
        "answers_found": len(answers),
        "matched": matched,
        "missing": missing,
        "skipped_already": skipped_already,
        "skipped_duplicate_number": skipped_duplicate_number,
        "missing_question_numbers": missing_q_numbers,  # 答案表里有，但题目区没识别到
        "extra_question_numbers": extra_q_numbers,      # 题目区有，但答案表里没有（一般不该发生）
        "duplicate_question_numbers": dup_list,         # 题目区重复出现的题号
        "duplicate_question_locations": dup_locs,       # 题号 -> 两处位置（行号+预览）
        "answer_section_start": answer_section_start,
    }
    return new_text, stats


# ----------------------------
# CLI
# ----------------------------


DEFAULT_INPUTS = [
    "RAG/data/raw/64/64.mmd",
    "RAG/data/raw/64  另外一个300   65到67/64  另外一个300   65到67.mmd",
    "RAG/data/raw/64 第3个300  65到67/64 第3个300  65到67.mmd",
    "RAG/data/raw/66  300 questions and answers in diagnostic aids for veterinary nurses/66  300 questions and answers in diagnostic aids for veterinary nurses.mmd",
]


def iter_input_mmds(paths: List[str]) -> List[Path]:
    out: List[Path] = []
    for x in paths:
        p = Path(x)
        if p.is_dir():
            for m in p.rglob("*.mmd"):
                if not _is_det_mmd(m):
                    out.append(m)
        else:
            out.append(p)
    # 去重 + 稳定排序
    uniq = sorted({str(p): p for p in out}.values(), key=lambda z: str(z))
    return uniq


def main() -> None:
    ap = argparse.ArgumentParser(description="为 MCQ .mmd 回填答案到题目中（从末尾答案区解析）。")
    ap.add_argument(
        "--inputs",
        nargs="*",
        default=DEFAULT_INPUTS,
        help="输入 .mmd 文件或目录（默认处理 4 本指定书）",
    )
    ap.add_argument("--dry-run", action="store_true", help="仅统计/打印，不写文件（默认）")
    ap.add_argument("--inplace", action="store_true", help="原地写回（会覆盖原文件）")
    ap.add_argument("--backup", action="store_true", help="写回前生成 .bak 备份（建议开启）")
    ap.add_argument(
        "--allow-dot-numbering",
        action="store_true",
        help="允许把 '数字.' 当题目起始（默认关闭；关闭可避免目录/编目条目误判为题目）",
    )
    ap.add_argument(
        "--prefer-answers-header",
        action="store_true",
        help="强制优先使用 'Answers' 标记定位答案区（默认：找不到则启发式）",
    )
    args = ap.parse_args()

    dry_run = bool(args.dry_run) or (not bool(args.inplace))
    if bool(args.inplace) and (not bool(args.backup)):
        # 默认更安全：inplace 时建议备份
        pass

    mmds = iter_input_mmds(list(args.inputs or []))
    if not mmds:
        raise SystemExit("未找到输入文件")

    print("[INFO] files:", len(mmds))
    for p in mmds:
        if not p.exists():
            print("[MISS]", p)
    mmds = [p for p in mmds if p.exists() and p.is_file() and (not _is_det_mmd(p))]
    if not mmds:
        raise SystemExit("没有可处理的 .mmd 文件（可能都不存在或都是 *_det.mmd）")

    for p in mmds:
        text = _read_text(p)
        start = find_answer_section_start(text)
        if start is None and (not bool(args.prefer_answers_header)):
            start = find_answer_section_start_heuristic(text)

        if start is None:
            print(f"\n[WARN] {p} 未找到答案区（Answers/启发式都失败），跳过")
            continue

        ans_text = text[int(start) :]
        answers = extract_answers(ans_text)
        new_text, st = attach_answers_to_text(
            text,
            answers=answers,
            answer_section_start=start,
            allow_dot_numbering=bool(args.allow_dot_numbering),
        )

        changed = (new_text != text)
        print(f"\n[FILE] {p}")
        print("  - answer_section_start:", st["answer_section_start"])
        print("  - q_blocks:", st["q_blocks"])
        print("  - q_unique_numbers:", st.get("q_unique_numbers"))
        print("  - answers_found:", st["answers_found"])
        print("  - matched:", st["matched"])
        print("  - missing:", st["missing"])
        print("  - skipped_already:", st["skipped_already"])
        print("  - skipped_duplicate_number:", st.get("skipped_duplicate_number"))
        # 关键：定位缺失题号（用于回到 mmd 查格式/漏页）
        if st.get("missing_question_numbers"):
            print("  - missing_question_numbers:", st.get("missing_question_numbers"))
        if st.get("extra_question_numbers"):
            print("  - extra_question_numbers:", st.get("extra_question_numbers"))
        if st.get("duplicate_question_numbers"):
            print("  - duplicate_question_numbers:", st.get("duplicate_question_numbers"))
        if st.get("duplicate_question_locations"):
            print("  - duplicate_question_locations:")
            for qn, locs in (st.get("duplicate_question_locations") or {}).items():
                print(f"    - {qn}:")
                for j, loc in enumerate(locs, start=1):
                    print(f"      [{j}] line={loc.get('line')}  preview={loc.get('preview')}")
        print("  - changed:", changed)

        if dry_run:
            continue

        if not changed:
            continue

        if bool(args.backup):
            bak = p.with_suffix(p.suffix + ".bak")
            if not bak.exists():
                shutil.copy2(p, bak)
        _write_text(p, new_text)
        print("  - write_back: OK")


if __name__ == "__main__":
    main()


