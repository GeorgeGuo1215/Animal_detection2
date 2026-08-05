"""
批量处理PDF文件的OCR脚本
适用于大型PDF文件，带进度跟踪和错误处理
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os
import glob
import json
import ast
from datetime import datetime
import traceback
import re
import gc
import numpy as np

try:
    import cv2
except ImportError:
    print("❌ 需要安装 opencv-python: pip install opencv-python")
    exit(1)

# PDF处理库
try:
    import fitz  # PyMuPDF
    print("✓ PyMuPDF 已加载")
except ImportError:
    print("❌ 需要安装 PyMuPDF: pip install PyMuPDF")
    exit(1)

from PIL import Image
import io

# ============ 配置 ============
PDF_SOURCE_DIR = r"C:\Users\ROG\Desktop\上服务器文件及依赖\input"
OUTPUT_DIR = r"F:\BaiduNetdiskDownload\港城大\answer_Results"
LOG_FILE = os.path.join(OUTPUT_DIR, "processing_log.json")

# 模型配置
MODEL_NAME = 'deepseek-ai/DeepSeek-OCR'
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# OCR配置
PROMPT = "<image>\ndescribe this image in detail"
FIGURE_PROMPT = PROMPT
PAGE_DETECTION_PROMPT = "<image>\nDetect and list all figures, tables, charts, and photos on this page."
BASE_SIZE = 1024
IMAGE_SIZE = 640
CROP_MODE = True

# 插图检测配置
MIN_FIGURE_AREA_RATIO = 0.01  # 插图占页面面积最小比例
MIN_FIGURE_EDGE = 80  # 插图最短边像素要求
MIN_COLOR_STD = 18.0  # 颜色标准差阈值，过滤纯文本块
MIN_FILL_RATIO = 0.1  # 非白色像素比例阈值
FIGURE_IOU_THRESHOLD = 0.3  # 合并重复框的IoU阈值
MODEL_FIGURE_LABELS = {
    "image", "figure", "photo", "picture",
    "chart", "table", "graph", "diagram", "illustration"
}
REF_DET_PATTERN = re.compile(r'(<\|ref\|>(.*?)<\|/ref\|><\|det\|>(.*?)<\|/det\|>)', re.DOTALL)
NORMALIZED_MAX = 999.0

# PDF处理配置
DPI = 200  # PDF转图像的DPI（降低以节省内存）
MAX_PAGES_PER_BATCH = 5  # 每批处理的页数（避免内存溢出）
SKIP_EXISTING = True  # 跳过已处理的文件

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

# ============ 初始化 ============
print("\n" + "="*80)
print("DeepSeek-OCR 批量PDF处理工具")
print("="*80)

# 创建输出目录
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 加载处理日志
processing_log = {}
if os.path.exists(LOG_FILE):
    with open(LOG_FILE, 'r', encoding='utf-8') as f:
        processing_log = json.load(f)
    print(f"✓ 加载处理日志: {len(processing_log)} 个已处理文件")

# 扫描PDF文件
print(f"\n扫描PDF文件: {PDF_SOURCE_DIR}")
pdf_files = glob.glob(os.path.join(PDF_SOURCE_DIR, "**/*.pdf"), recursive=True)
pdf_files = [f for f in pdf_files if os.path.isfile(f)]

print(f"✓ 找到 {len(pdf_files)} 个PDF文件")

# 过滤已处理的文件
if SKIP_EXISTING:
    pending_files = []
    for pdf_file in pdf_files:
        rel_path = os.path.relpath(pdf_file, PDF_SOURCE_DIR)
        if rel_path not in processing_log or processing_log[rel_path].get('status') != 'completed':
            pending_files.append(pdf_file)
        else:
            print(f"  ⊙ 跳过已处理: {os.path.basename(pdf_file)}")
    pdf_files = pending_files

print(f"\n待处理: {len(pdf_files)} 个PDF文件")

if len(pdf_files) == 0:
    print("\n✓ 所有文件已处理完成！")
    exit(0)

# 加载模型
print("\n" + "="*80)
print("加载DeepSeek-OCR模型...")
print("="*80)

try:
    tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME, trust_remote_code=True)
    model = AutoModel.from_pretrained(MODEL_NAME, trust_remote_code=True, use_safetensors=True)
    model = model.eval().to(DEVICE)
    if DEVICE == 'cuda':
        model = model.to(torch.bfloat16)
    print(f"✓ 模型加载成功 (设备: {DEVICE})")
except Exception as e:
    print(f"❌ 模型加载失败: {e}")
    exit(1)

# ============ 工具函数 ============

def clean_filename(filename):
    """生成安全的文件名"""
    # 移除或替换非法字符
    filename = re.sub(r'[<>:"/\\|?*]', '_', filename)
    # 限制长度
    if len(filename) > 200:
        name, ext = os.path.splitext(filename)
        filename = name[:200] + ext
    return filename

def clip_bbox(bbox, width, height):
    """将bbox裁剪在图像范围内"""
    x0, y0, x1, y1 = bbox
    x0 = max(0, min(int(round(x0)), width))
    y0 = max(0, min(int(round(y0)), height))
    x1 = max(0, min(int(round(x1)), width))
    y1 = max(0, min(int(round(y1)), height))
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1, y1)

def calculate_iou(box_a, box_b):
    """计算两个bbox的IoU"""
    ax0, ay0, ax1, ay1 = box_a
    bx0, by0, bx1, by1 = box_b
    inter_x0 = max(ax0, bx0)
    inter_y0 = max(ay0, by0)
    inter_x1 = min(ax1, bx1)
    inter_y1 = min(ay1, by1)
    if inter_x1 <= inter_x0 or inter_y1 <= inter_y0:
        return 0.0
    inter_area = (inter_x1 - inter_x0) * (inter_y1 - inter_y0)
    area_a = (ax1 - ax0) * (ay1 - ay0)
    area_b = (bx1 - bx0) * (by1 - by0)
    union_area = area_a + area_b - inter_area
    if union_area == 0:
        return 0.0
    return inter_area / union_area

def normalize_points_to_bbox(points, width, height):
    """将0-999归一化坐标转换成像素坐标"""
    if len(points) != 4:
        return None
    x0, y0, x1, y1 = points
    x0 = int(round(x0 / NORMALIZED_MAX * width))
    y0 = int(round(y0 / NORMALIZED_MAX * height))
    x1 = int(round(x1 / NORMALIZED_MAX * width))
    y1 = int(round(y1 / NORMALIZED_MAX * height))
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1, y1)

def parse_model_detections(raw_text, width, height):
    """解析DeepSeek输出中的<|ref|><|det|>结构，提取插图bbox"""
    if not raw_text:
        return []
    boxes = []
    matches = REF_DET_PATTERN.findall(raw_text)
    for _, label, coord_text in matches:
        if not label:
            continue
        label_clean = label.strip().lower()
        if label_clean not in MODEL_FIGURE_LABELS:
            continue
        try:
            coords = ast.literal_eval(coord_text)
        except Exception:
            continue
        if isinstance(coords, (list, tuple)):
            for entry in coords:
                bbox = normalize_points_to_bbox(entry, width, height)
                if bbox:
                    boxes.append(bbox)
    return boxes

def extract_pdf_image_boxes(page, zoom_x, zoom_y):
    """基于PDF结构提取图像块的像素级bbox"""
    boxes = []
    page_width = int(page.rect.width * zoom_x)
    page_height = int(page.rect.height * zoom_y)
    try:
        page_dict = page.get_text("dict")
        for block in page_dict.get("blocks", []):
            if block.get("type") != 1:
                continue
            bbox = block.get("bbox", [])
            if len(bbox) != 4:
                continue
            x0, y0, x1, y1 = bbox
            scaled_bbox = (
                x0 * zoom_x,
                y0 * zoom_y,
                x1 * zoom_x,
                y1 * zoom_y
            )
            clipped = clip_bbox(scaled_bbox, page_width, page_height)
            if clipped:
                boxes.append(clipped)
    except Exception as e:
        print(f"  ⚠ 无法读取PDF图像块: {e}")
    return boxes

def detect_candidate_boxes_via_cv(image):
    """使用OpenCV分析页面图像，寻找可能的插图区域"""
    np_img = np.array(image)
    gray = cv2.cvtColor(np_img, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(
        blur,
        0,
        255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    page_area = image.width * image.height
    boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w < MIN_FIGURE_EDGE or h < MIN_FIGURE_EDGE:
            continue
        area_ratio = (w * h) / page_area
        if area_ratio < MIN_FIGURE_AREA_RATIO or area_ratio > 0.8:
            continue

        roi_color = np_img[y:y+h, x:x+w]
        if roi_color.size == 0:
            continue
        color_std = float(np.std(roi_color))

        roi_gray = gray[y:y+h, x:x+w]
        _, roi_bin = cv2.threshold(
            roi_gray,
            240,
            255,
            cv2.THRESH_BINARY_INV
        )
        fill_ratio = cv2.countNonZero(roi_bin) / (w * h)

        if color_std < MIN_COLOR_STD and fill_ratio < MIN_FILL_RATIO:
            continue

        boxes.append((x, y, x + w, y + h))

    return boxes

def extract_figures_from_page(image, pdf_boxes=None, model_boxes=None):
    """组合PDF结构/模型检测/传统视觉结果提取插图"""
    width, height = image.size
    figures = []
    used_boxes = []

    def _append_figure(bbox, source):
        clipped = clip_bbox(bbox, width, height)
        if not clipped:
            return
        w = clipped[2] - clipped[0]
        h = clipped[3] - clipped[1]
        if w < MIN_FIGURE_EDGE or h < MIN_FIGURE_EDGE:
            return
        crop = image.crop(clipped)
        figures.append({
            'bbox': clipped,
            'image': crop,
            'source': source
        })
        used_boxes.append(clipped)

    pdf_boxes = pdf_boxes or []
    for bbox in pdf_boxes:
        _append_figure(bbox, 'pdf')

    model_boxes = model_boxes or []
    for bbox in model_boxes:
        _append_figure(bbox, 'model')

    cv_boxes = detect_candidate_boxes_via_cv(image)
    for bbox in cv_boxes:
        if any(calculate_iou(bbox, existing) > FIGURE_IOU_THRESHOLD for existing in used_boxes):
            continue
        _append_figure(bbox, 'cv')

    figures.sort(key=lambda item: (item['bbox'][1], item['bbox'][0]))
    return figures

def pdf_to_images_batched(pdf_path, dpi=200, max_pages=None):
    """
    将PDF转换为图像（使用PyMuPDF，逐页处理）
    
    Args:
        pdf_path: PDF文件路径
        dpi: 图像DPI
        max_pages: 最大页数（用于测试）
    
    Yields:
        (page_num, image, pdf_boxes) 元组
    """
    try:
        # 打开PDF
        pdf_document = fitz.open(pdf_path)
        total_pages = pdf_document.page_count
        
        if max_pages:
            total_pages = min(total_pages, max_pages)
        
        print(f"  总页数: {total_pages}")
        
        # 计算缩放比例
        zoom = dpi / 72.0
        matrix = fitz.Matrix(zoom, zoom)
        zoom_x = matrix.a
        zoom_y = matrix.d
        
        # 逐页转换
        for page_num in range(total_pages):
            if page_num % MAX_PAGES_PER_BATCH == 0:
                print(f"  处理页面 {page_num + 1}-{min(page_num + MAX_PAGES_PER_BATCH, total_pages)}...", end='', flush=True)
            
            page = pdf_document[page_num]
            pixmap = page.get_pixmap(matrix=matrix, alpha=False)
            pdf_boxes = extract_pdf_image_boxes(page, zoom_x, zoom_y)
            
            # 转换为PIL Image
            img_data = pixmap.tobytes("png")
            img = Image.open(io.BytesIO(img_data))
            
            # 如果是RGBA，转换为RGB
            if img.mode in ('RGBA', 'LA'):
                background = Image.new('RGB', img.size, (255, 255, 255))
                background.paste(img, mask=img.split()[-1] if img.mode == 'RGBA' else None)
                img = background
            
            yield page_num + 1, img, pdf_boxes
            
            # 清理
            del pixmap
            if (page_num + 1) % MAX_PAGES_PER_BATCH == 0:
                print(" ✓")
                gc.collect()
        
        # 关闭PDF
        pdf_document.close()
        print(" ✓")
        
    except Exception as e:
        print(f"\n  ❌ PDF转图像失败: {e}")
        raise

def ocr_image(image, working_dir, prompt=None):
    """
    对单张图像进行OCR/描述
    
    Args:
        image: PIL Image
        working_dir: 输出临时文件的目录
        prompt: 模型提示词
    
    Returns:
        str: OCR结果文本
    """
    try:
        os.makedirs(working_dir, exist_ok=True)
        target_prompt = prompt or PROMPT
        with torch.no_grad():
            # 临时保存图像
            temp_image_path = os.path.join(working_dir, "temp_image.jpg")
            image.save(temp_image_path, 'JPEG', quality=95)
            
            # 调用模型推理 - save_results=True 会生成文件但返回None
            _ = model.infer(
                tokenizer,
                prompt=target_prompt,
                image_file=temp_image_path,
                output_path=working_dir,
                base_size=BASE_SIZE,
                image_size=IMAGE_SIZE,
                crop_mode=CROP_MODE,
                save_results=True,  # 必须为True才能生成输出
                test_compress=False
            )
            
            # 从生成的文件中读取结果
            result_file = os.path.join(working_dir, "result.mmd")
            if os.path.exists(result_file):
                with open(result_file, 'r', encoding='utf-8') as f:
                    result = f.read()
                os.remove(result_file)
            else:
                result = ""
            
            # 清理临时文件
            if os.path.exists(temp_image_path):
                os.remove(temp_image_path)
            
            # 清理GPU缓存
            if DEVICE == 'cuda':
                torch.cuda.empty_cache()
            
            return result if result else ""
            
    except Exception as e:
        print(f"\n    ❌ OCR失败: {e}")
        return f"[OCR Error: {str(e)}]\n"

def clean_ocr_text(text):
    """清理OCR文本，移除定位标签"""
    if not text:
        return ""
    
    # 移除 <|ref|>, <|det|> 等标签
    text = re.sub(r'<\|ref\|>.*?<\/ref\|>', '', text)
    text = re.sub(r'<\|det\|>\[\[.*?\]\]<\/det\|>', '', text)
    text = re.sub(r'<\|.*?\|>', '', text)
    
    # 清理多余的空行
    text = re.sub(r'\n\n\n+', '\n\n', text)
    
    return text.strip()

def save_progress():
    """保存处理进度"""
    with open(LOG_FILE, 'w', encoding='utf-8') as f:
        json.dump(processing_log, f, indent=2, ensure_ascii=False)

# ============ 主处理循环 ============

print("\n" + "="*80)
print("开始批量处理")
print("="*80)

total_files = len(pdf_files)
completed_files = 0
failed_files = 0

for file_idx, pdf_path in enumerate(pdf_files, 1):
    pdf_name = os.path.basename(pdf_path)
    rel_path = os.path.relpath(pdf_path, PDF_SOURCE_DIR)
    
    print(f"\n[{file_idx}/{total_files}] 处理: {pdf_name}")
    print("-" * 80)
    
    # 生成输出目录
    book_folder_name = clean_filename(os.path.splitext(pdf_name)[0])
    book_output_dir = os.path.join(OUTPUT_DIR, book_folder_name)
    os.makedirs(book_output_dir, exist_ok=True)
    
    # 初始化日志条目
    if rel_path not in processing_log:
        processing_log[rel_path] = {}
    
    processing_log[rel_path].update({
        'pdf_name': pdf_name,
        'output_folder': book_folder_name,
        'start_time': datetime.now().isoformat(),
        'status': 'processing'
    })
    save_progress()
    
    try:
        # 获取PDF信息
        file_size_mb = os.path.getsize(pdf_path) / (1024 * 1024)
        print(f"  文件大小: {file_size_mb:.2f} MB")
        
        # 转换PDF为图像并逐页提取插图
        page_count = 0
        total_figures = 0
        error_pages = []
        page_summaries = []
        
        # 使用生成器逐页处理
        for page_num, image, pdf_boxes in pdf_to_images_batched(pdf_path, dpi=DPI):
            print(f"  处理第 {page_num} 页...")
            figures = []
            detection_file = None
            detection_relpath = None
            
            try:
                page_count += 1
                page_dir = os.path.join(book_output_dir, f"page_{page_num:04d}")
                os.makedirs(page_dir, exist_ok=True)
                
                # 保存整页图像，方便人工核查
                page_image_path = os.path.join(page_dir, f"page_{page_num:04d}.jpg")
                image.save(page_image_path, 'JPEG', quality=95)
                
                # 调用模型进行布局检测
                model_boxes = []
                try:
                    detection_text = ocr_image(
                        image,
                        working_dir=page_dir,
                        prompt=PAGE_DETECTION_PROMPT
                    )
                    if detection_text:
                        detection_file = os.path.join(page_dir, f"page_{page_num:04d}_det.mmd")
                        with open(detection_file, 'w', encoding='utf-8') as det_f:
                            det_f.write(detection_text)
                        detection_relpath = os.path.relpath(detection_file, OUTPUT_DIR)
                        model_boxes = parse_model_detections(detection_text, image.width, image.height)
                        if model_boxes:
                            print(f"    → 模型检测到 {len(model_boxes)} 个候选区域")
                except Exception as det_err:
                    print(f"    ⚠ 模型检测失败: {det_err}")
                
                figures = extract_figures_from_page(image, pdf_boxes, model_boxes)
                if figures:
                    print(f"    → 综合检测后保留 {len(figures)} 个插图")
                else:
                    print("    → 未检测到插图（跳过）")
                
                figure_records = []
                for fig_idx, figure in enumerate(figures, 1):
                    figure_image = figure['image']
                    figure_filename = f"figure_{fig_idx:02d}.jpg"
                    figure_path = os.path.join(page_dir, figure_filename)
                    figure_image.save(figure_path, 'JPEG', quality=95)
                    
                    description_text = ocr_image(
                        figure_image,
                        working_dir=page_dir,
                        prompt=FIGURE_PROMPT
                    )
                    cleaned_text = clean_ocr_text(description_text)
                    
                    description_filename = f"figure_{fig_idx:02d}.md"
                    description_path = os.path.join(page_dir, description_filename)
                    with open(description_path, 'w', encoding='utf-8') as desc_f:
                        desc_f.write(f"# Page {page_num} - Figure {fig_idx}\n\n")
                        desc_f.write(cleaned_text or "*模型未返回描述*\n")
                    
                    figure_records.append({
                        'id': fig_idx,
                        'bbox': list(figure['bbox']),
                        'source': figure['source'],
                        'image_file': os.path.relpath(figure_path, OUTPUT_DIR),
                        'description_file': os.path.relpath(description_path, OUTPUT_DIR)
                    })
                    total_figures += 1
                    
                    figure_image.close()
                    figure.pop('image', None)
                
                page_summaries.append({
                    'page_num': page_num,
                    'page_dir': os.path.relpath(page_dir, OUTPUT_DIR),
                    'page_image': os.path.relpath(page_image_path, OUTPUT_DIR),
                    'detection_file': detection_relpath,
                    'figure_count': len(figure_records),
                    'figures': figure_records
                })
                
            except Exception as e:
                print(f"    ✗ 错误: {e}")
                error_pages.append(page_num)
            
            finally:
                for figure in figures:
                    figure_img = figure.get('image')
                    if figure_img is not None:
                        figure_img.close()
                image.close()
                del image
                gc.collect()
        
        # 更新日志
        processing_log[rel_path].update({
            'status': 'completed',
            'end_time': datetime.now().isoformat(),
            'page_count': page_count,
            'error_pages': error_pages,
            'figure_total': total_figures,
            'page_summaries': page_summaries
        })
        
        completed_files += 1
        print(f"  ✓ 完成! 处理了 {page_count} 页，提取 {total_figures} 个插图")
        
        if error_pages:
            print(f"  ⚠ 有 {len(error_pages)} 页处理失败: {error_pages}")
        
    except Exception as e:
        print(f"\n  ❌ 处理失败: {e}")
        print(traceback.format_exc())
        
        processing_log[rel_path].update({
            'status': 'failed',
            'end_time': datetime.now().isoformat(),
            'error': str(e)
        })
        
        failed_files += 1
    
    finally:
        # 保存进度
        save_progress()
        
        # 清理内存
        gc.collect()
        if DEVICE == 'cuda':
            torch.cuda.empty_cache()

# ============ 完成总结 ============

print("\n" + "="*80)
print("批量处理完成！")
print("="*80)
print(f"总文件数: {total_files}")
print(f"成功: {completed_files}")
print(f"失败: {failed_files}")
print(f"\n结果保存在: {OUTPUT_DIR}")
print(f"日志文件: {LOG_FILE}")
print("="*80)

# 生成处理报告
report_path = os.path.join(OUTPUT_DIR, "processing_report.txt")
with open(report_path, 'w', encoding='utf-8') as f:
    f.write("DeepSeek-OCR 批量处理报告\n")
    f.write("="*80 + "\n\n")
    f.write(f"处理时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"总文件数: {total_files}\n")
    f.write(f"成功: {completed_files}\n")
    f.write(f"失败: {failed_files}\n\n")
    
    f.write("="*80 + "\n")
    f.write("文件列表\n")
    f.write("="*80 + "\n\n")
    
    for rel_path, info in processing_log.items():
        status = info.get('status', 'unknown')
        status_symbol = "✓" if status == 'completed' else "✗"
        f.write(f"{status_symbol} {info.get('pdf_name', rel_path)}\n")
        f.write(f"  状态: {status}\n")
        if status == 'completed':
            f.write(f"  页数: {info.get('page_count', 'N/A')}\n")
            f.write(f"  插图总数: {info.get('figure_total', 0)}\n")
            f.write(f"  输出目录: {info.get('output_folder', 'N/A')}\n")
            page_samples = info.get('page_summaries', []) or []
            for page in page_samples[:5]:
                f.write(f"    · 第 {page.get('page_num')} 页: 插图 {page.get('figure_count', 0)} 个\n")
        elif status == 'failed':
            f.write(f"  错误: {info.get('error', 'N/A')}\n")
        f.write("\n")

print(f"✓ 处理报告已保存: {report_path}")

