"""
将「小动物agent知识库新增资料」批量 OCR 为 DeepSeek-OCR-vllm 规范输出。

输出（每书一目录，与参考 output/64/ 对齐）:
  <OUTPUT>/<id>/
    <id>.mmd          # 全书合并，页间 <--- Page Split --->
    <id>_det.mmd      # 可选：每页检测原文拼接
    images/           # 页面图（及可选插图）images/{page}_0.jpg
    images_ocr/       # 同页渲染备份 page_XXXX.jpg

特性:
  - 三位书号目录下递归发现 PDF；一书多 PDF 按相对路径排序合并
  - 裸 PDF = 一书
  - 按书 / 按页断点续传（processing_log.json）
  - 适合本地隔夜跑，无需 IDE 挂着监测

用法（conda RAG，需 CUDA + DeepSeek-OCR 权重）:
  cd agentAndRag/DeepSeek-OCR-master/DeepSeek-OCR-hf
  python batch_ocr_to_vllm_layout.py
  python batch_ocr_to_vllm_layout.py --only 089,080
  python batch_ocr_to_vllm_layout.py --max-pages 3   # 冒烟
"""
from __future__ import annotations

import argparse
import gc
import hashlib
import io
import json
import os
import re
import sys
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

# --------------- defaults（可用 CLI 覆盖）---------------
DEFAULT_SOURCE = Path(r"C:\Users\ROG\Downloads\小动物agent知识库新增资料")
DEFAULT_OUTPUT = Path(
    r"C:\Users\ROG\Desktop\大四\上服务器文件及依赖\output\root\autodl-tmp"
    r"\.autodl\deepseek-ocr-offline\deepseek-ocr\DeepSeek-OCR"
    r"\DeepSeek-OCR-master\DeepSeek-OCR-vllm\output"
)
MODEL_NAME = os.environ.get("DEEPSEEK_OCR_MODEL", "deepseek-ai/DeepSeek-OCR")
PAGE_SPLIT = "\n\n<--- Page Split --->\n\n"
DOC_PROMPT = "<image>\nConvert the document to markdown. "
DET_PROMPT = "<image>\nDetect and list all figures, tables, charts, and photos on this page."
DPI = int(os.environ.get("OCR_DPI", "200"))
BASE_SIZE = 1024
IMAGE_SIZE = 640
CROP_MODE = True


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="新增书籍 OCR → vllm 布局输出")
    p.add_argument("--source", type=str, default=str(DEFAULT_SOURCE))
    p.add_argument("--output", type=str, default=str(DEFAULT_OUTPUT))
    p.add_argument("--model", type=str, default=MODEL_NAME)
    p.add_argument("--dpi", type=int, default=DPI)
    p.add_argument("--device", type=str, default=None, help="cuda / cpu（默认自动）")
    p.add_argument("--only", type=str, default="", help="仅处理这些书号，逗号分隔，如 089,080")
    p.add_argument("--max-pages", type=int, default=0, help="每 PDF 最多页（0=不限；冒烟用 3）")
    p.add_argument("--skip-det", action="store_true", help="不写 _det.mmd（更快）")
    p.add_argument("--force", action="store_true", help="忽略完成标记，整书重跑")
    return p.parse_args()


def _file_sha1(path: Path) -> str:
    digest = hashlib.sha1()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def discover_books(source: Path) -> List[Dict[str, Any]]:
    """递归发现三位书号对应的 PDF，返回按书号排序的任务。

    支持 ``084/a.pdf``、``084/volume-1/a.pdf`` 以及根目录 ``084.pdf``。
    书号取 PDF 相对路径中最靠近文件的三位数字目录；没有数字目录时，
    才使用 PDF 文件名。这样分类/批次目录可以任意嵌套而不会漏书。
    """
    books: Dict[str, List[Path]] = {}
    if not source.is_dir():
        raise FileNotFoundError(f"源目录不存在: {source}")

    for pdf in source.rglob("*"):
        if not pdf.is_file() or pdf.suffix.lower() != ".pdf":
            continue
        rel = pdf.relative_to(source)
        book_id: Optional[str] = None
        for part in reversed(rel.parts[:-1]):
            if re.fullmatch(r"\d{3}", part):
                book_id = part
                break
        if book_id is None and re.fullmatch(r"\d{3}", pdf.stem):
            book_id = pdf.stem
        if book_id is not None:
            books.setdefault(book_id, []).append(pdf)

    for book_id, pdfs in books.items():
        by_path = {str(path.resolve()).casefold(): path for path in pdfs}
        ordered = sorted(
            by_path.values(), key=lambda path: str(path.relative_to(source)).casefold()
        )
        # 下载目录中常同时保留命名版 PDF 和多个完全相同的 download.pdf。
        # 按内容去重，避免同一指南被重复 OCR/拼入全书 MMD。
        by_sha1: Dict[str, Path] = {}
        for path in ordered:
            digest = _file_sha1(path)
            by_sha1.setdefault(digest, path)
        books[book_id] = list(by_sha1.values())

    return [{"id": bid, "pdfs": books[bid]} for bid in sorted(books.keys())]


def load_log(path: Path) -> Dict[str, Any]:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def save_log(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def clean_ocr_text(text: str) -> str:
    if not text:
        return ""
    text = re.sub(r"<\|ref\|>.*?</\|ref\|>", "", text, flags=re.DOTALL)
    text = re.sub(r"<\|det\|>\[\[.*?\]\]</\|det\|>", "", text, flags=re.DOTALL)
    text = re.sub(r"<\|.*?\|>", "", text)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip()


def pdf_pages(
    pdf_path: Path, dpi: int, max_pages: int = 0
) -> Iterator[Tuple[int, "Image.Image"]]:
    import fitz
    from PIL import Image

    doc = fitz.open(str(pdf_path))
    try:
        n = doc.page_count
        if max_pages and max_pages > 0:
            n = min(n, max_pages)
        zoom = dpi / 72.0
        matrix = fitz.Matrix(zoom, zoom)
        for i in range(n):
            page = doc[i]
            pix = page.get_pixmap(matrix=matrix, alpha=False)
            img = Image.open(io.BytesIO(pix.tobytes("png")))
            if img.mode != "RGB":
                img = img.convert("RGB")
            yield i + 1, img
            del pix
            if (i + 1) % 5 == 0:
                gc.collect()
    finally:
        doc.close()


def build_model(model_name: str, device: str):
    import torch
    from transformers import AutoModel, AutoTokenizer

    print(f"加载模型: {model_name} @ {device}")
    tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
    model = AutoModel.from_pretrained(model_name, trust_remote_code=True, use_safetensors=True)
    model = model.eval().to(device)
    if device == "cuda":
        model = model.to(torch.bfloat16)
    print("模型就绪")
    return tokenizer, model


def infer_page(model, tokenizer, image, work_dir: Path, prompt: str, device: str) -> str:
    import torch

    work_dir.mkdir(parents=True, exist_ok=True)
    tmp = work_dir / "temp_image.jpg"
    image.save(tmp, "JPEG", quality=95)
    try:
        with torch.no_grad():
            _ = model.infer(
                tokenizer,
                prompt=prompt,
                image_file=str(tmp),
                output_path=str(work_dir),
                base_size=BASE_SIZE,
                image_size=IMAGE_SIZE,
                crop_mode=CROP_MODE,
                save_results=True,
                test_compress=False,
            )
        result_file = work_dir / "result.mmd"
        text = result_file.read_text(encoding="utf-8") if result_file.exists() else ""
        if result_file.exists():
            result_file.unlink()
        return clean_ocr_text(text)
    finally:
        if tmp.exists():
            tmp.unlink()
        if device == "cuda":
            import torch

            torch.cuda.empty_cache()


def process_book(
    *,
    book_id: str,
    pdfs: List[Path],
    out_root: Path,
    tokenizer,
    model,
    device: str,
    dpi: int,
    max_pages: int,
    skip_det: bool,
    force: bool,
    log: Dict[str, Any],
    log_path: Path,
) -> None:
    book_dir = out_root / book_id
    images_dir = book_dir / "images"
    images_ocr_dir = book_dir / "images_ocr"
    work_dir = book_dir / "_work"
    mmd_path = book_dir / f"{book_id}.mmd"
    det_path = book_dir / f"{book_id}_det.mmd"
    pages_state_path = book_dir / "pages_state.json"

    entry = log.setdefault(book_id, {})
    if not force and entry.get("status") == "completed" and mmd_path.exists():
        print(f"  ⊙ 跳过已完成: {book_id}")
        return

    book_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(exist_ok=True)
    images_ocr_dir.mkdir(exist_ok=True)

    pages_state: Dict[str, Any] = {}
    if pages_state_path.exists() and not force:
        pages_state = json.loads(pages_state_path.read_text(encoding="utf-8"))

    entry.update(
        {
            "status": "processing",
            "pdfs": [str(p) for p in pdfs],
            "start_time": entry.get("start_time") or datetime.now().isoformat(),
            "output_dir": str(book_dir),
        }
    )
    save_log(log_path, log)

    page_texts: Dict[int, str] = {
        int(k): v for k, v in (pages_state.get("page_texts") or {}).items()
    }
    det_texts: Dict[int, str] = {
        int(k): v for k, v in (pages_state.get("det_texts") or {}).items()
    }
    global_page = int(pages_state.get("next_global_page") or 1)
    start_pdf_idx = int(pages_state.get("pdf_idx") or 0)
    start_page_in_pdf = int(pages_state.get("page_in_pdf") or 1)

    try:
        for pdf_idx, pdf in enumerate(pdfs):
            if pdf_idx < start_pdf_idx:
                continue
            print(f"  PDF [{pdf_idx+1}/{len(pdfs)}]: {pdf.name}")
            for local_page, image in pdf_pages(pdf, dpi=dpi, max_pages=max_pages):
                if pdf_idx == start_pdf_idx and local_page < start_page_in_pdf:
                    continue
                g = global_page
                key_done = str(g) in {str(k) for k in page_texts.keys()}
                if key_done and not force:
                    print(f"    · page {g} 已有，跳过")
                    global_page = g + 1
                    continue

                print(f"    · OCR 全局页 {g} (pdf页 {local_page}) ...", flush=True)
                # 存图
                page_jpg = images_ocr_dir / f"page_{g:04d}.jpg"
                image.save(page_jpg, "JPEG", quality=92)
                img_ref = images_dir / f"{g}_0.jpg"
                image.save(img_ref, "JPEG", quality=92)

                text = infer_page(model, tokenizer, image, work_dir, DOC_PROMPT, device)
                if not text:
                    text = f"![page](images/{g}_0.jpg)\n"
                elif f"images/{g}_0" not in text and "![" not in text[:200]:
                    # 保证至少有一页图引用，贴近 vllm 习惯
                    text = text + f"\n\n![](images/{g}_0.jpg)\n"
                page_texts[g] = text

                if not skip_det:
                    det = infer_page(model, tokenizer, image, work_dir, DET_PROMPT, device)
                    det_texts[g] = det or ""

                global_page = g + 1
                pages_state = {
                    "page_texts": {str(k): page_texts[k] for k in sorted(page_texts)},
                    "det_texts": {str(k): det_texts[k] for k in sorted(det_texts)},
                    "next_global_page": global_page,
                    "pdf_idx": pdf_idx,
                    "page_in_pdf": local_page + 1,
                    "updated": datetime.now().isoformat(),
                }
                pages_state_path.write_text(
                    json.dumps(pages_state, ensure_ascii=False, indent=2), encoding="utf-8"
                )
                # 每页落盘 mmd，中断也可读
                ordered = [page_texts[k] for k in sorted(page_texts)]
                mmd_path.write_text(PAGE_SPLIT.join(ordered) + "\n", encoding="utf-8")
                if not skip_det and det_texts:
                    det_ordered = [
                        f"## Page {k}\n\n{det_texts[k]}" for k in sorted(det_texts)
                    ]
                    det_path.write_text("\n\n".join(det_ordered) + "\n", encoding="utf-8")

                del image
                gc.collect()

            # 下一 PDF 从第 1 页开始
            start_pdf_idx = pdf_idx + 1
            start_page_in_pdf = 1
            pages_state["pdf_idx"] = start_pdf_idx
            pages_state["page_in_pdf"] = 1
            pages_state_path.write_text(
                json.dumps(pages_state, ensure_ascii=False, indent=2), encoding="utf-8"
            )

        entry["status"] = "completed"
        entry["end_time"] = datetime.now().isoformat()
        entry["pages"] = len(page_texts)
        save_log(log_path, log)
        print(f"  ✓ 完成 {book_id}，共 {len(page_texts)} 页 → {mmd_path}")
    except Exception as exc:  # noqa: BLE001
        entry["status"] = "failed"
        entry["error"] = str(exc)
        entry["traceback"] = traceback.format_exc()
        save_log(log_path, log)
        print(f"  ❌ {book_id} 失败: {exc}")
        raise


def main() -> int:
    args = _parse_args()
    source = Path(args.source)
    out_root = Path(args.output)
    out_root.mkdir(parents=True, exist_ok=True)
    log_path = out_root / "processing_log_new_books.json"
    log = load_log(log_path)

    only = {x.strip() for x in args.only.split(",") if x.strip()}
    books = discover_books(source)
    if only:
        books = [b for b in books if b["id"] in only]
    if not books:
        print("未发现待处理书籍")
        return 2

    print("=" * 72)
    print("DeepSeek-OCR → vllm layout")
    print(f"源目录: {source}")
    print(f"输出:   {out_root}")
    print(f"书籍数: {len(books)} → {[b['id'] for b in books]}")
    print("=" * 72)

    import torch

    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
    tokenizer, model = build_model(args.model, device)

    for i, book in enumerate(books, 1):
        print(f"\n[{i}/{len(books)}] 书号 {book['id']} ({len(book['pdfs'])} 个 PDF)")
        try:
            process_book(
                book_id=book["id"],
                pdfs=book["pdfs"],
                out_root=out_root,
                tokenizer=tokenizer,
                model=model,
                device=device,
                dpi=args.dpi,
                max_pages=args.max_pages,
                skip_det=args.skip_det,
                force=args.force,
                log=log,
                log_path=log_path,
            )
        except KeyboardInterrupt:
            print("\n用户中断；进度已写入 pages_state.json / processing_log_new_books.json")
            return 130
        except Exception:
            print("继续下一本…")
            continue

    print("\n全部任务结束。日志:", log_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
