"""
测试脚本：处理单个PDF文件
用于验证环境配置和估算处理时间
"""

from transformers import AutoModel, AutoTokenizer
import torch
import os
from datetime import datetime
import time

# 检查依赖
try:
    import fitz  # PyMuPDF
    print("✓ PyMuPDF 已安装")
except ImportError:
    print("❌ 需要安装 PyMuPDF")
    print("   运行: pip install PyMuPDF")
    exit(1)

from PIL import Image
import io

# ============ 配置 ============
# 请修改这里指定要测试的PDF文件
TEST_PDF = r"F:\BaiduNetdiskDownload\港城大\deepseek-ocr\DeepSeek-OCR\DeepSeek_OCR_paper.pdf"
OUTPUT_DIR = r"F:\BaiduNetdiskDownload\港城大\test_output"

# 测试参数
TEST_PAGES = 3  # 只处理前3页（快速测试）
DPI = 150  # 使用较低DPI快速测试

MODEL_NAME = 'deepseek-ai/DeepSeek-OCR'
PROMPT = "<image>\nConvert the document to markdown. "

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

# ============ 测试 ============
print("\n" + "="*80)
print("DeepSeek-OCR 单文件测试")
print("="*80)

# 创建输出目录
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 检查测试文件
if not os.path.exists(TEST_PDF):
    print(f"❌ 测试文件不存在: {TEST_PDF}")
    print("\n请修改脚本中的 TEST_PDF 变量，指向一个实际存在的PDF文件")
    exit(1)

print(f"\n测试文件: {os.path.basename(TEST_PDF)}")
print(f"文件大小: {os.path.getsize(TEST_PDF) / (1024*1024):.2f} MB")
print(f"测试页数: 前 {TEST_PAGES} 页")
print(f"输出目录: {OUTPUT_DIR}")

# 测试1: PDF转图像
print("\n" + "-"*80)
print("测试1: PDF转图像 (使用PyMuPDF)")
print("-"*80)

try:
    print("转换PDF为图像...", end='', flush=True)
    start_time = time.time()
    
    # 使用PyMuPDF转换
    pdf_document = fitz.open(TEST_PDF)
    total_pages = min(TEST_PAGES, pdf_document.page_count)
    
    images = []
    zoom = DPI / 72.0
    matrix = fitz.Matrix(zoom, zoom)
    
    for page_num in range(total_pages):
        page = pdf_document[page_num]
        pixmap = page.get_pixmap(matrix=matrix, alpha=False)
        
        # 转换为PIL Image
        img_data = pixmap.tobytes("png")
        img = Image.open(io.BytesIO(img_data))
        
        # 转换为RGB
        if img.mode in ('RGBA', 'LA'):
            background = Image.new('RGB', img.size, (255, 255, 255))
            background.paste(img, mask=img.split()[-1] if img.mode == 'RGBA' else None)
            img = background
        
        images.append(img)
    
    pdf_document.close()
    
    convert_time = time.time() - start_time
    print(f" ✓ 完成")
    print(f"  转换时间: {convert_time:.2f}秒")
    print(f"  图像数量: {len(images)}")
    print(f"  图像大小: {images[0].size}")
    
    # 保存测试图像
    test_img_path = os.path.join(OUTPUT_DIR, "test_page1.jpg")
    images[0].save(test_img_path)
    print(f"  保存测试图像: {test_img_path}")
    
except Exception as e:
    print(f" ✗ 失败")
    print(f"❌ 错误: {e}")
    print("\n可能的原因:")
    print("  1. PyMuPDF未正确安装")
    print("  2. PDF文件损坏")
    print("  3. 权限问题")
    print("\n安装PyMuPDF: pip install PyMuPDF")
    exit(1)

# 测试2: 加载模型
print("\n" + "-"*80)
print("测试2: 加载OCR模型")
print("-"*80)

try:
    print("加载模型...", end='', flush=True)
    start_time = time.time()
    
    tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME, trust_remote_code=True)
    model = AutoModel.from_pretrained(MODEL_NAME, trust_remote_code=True, use_safetensors=True)
    
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = model.eval().to(device)
    if device == 'cuda':
        model = model.to(torch.bfloat16)
    
    load_time = time.time() - start_time
    print(f" ✓ 完成")
    print(f"  加载时间: {load_time:.2f}秒")
    print(f"  设备: {device}")
    
    if device == 'cuda':
        print(f"  GPU: {torch.cuda.get_device_name(0)}")
        print(f"  显存: {torch.cuda.get_device_properties(0).total_memory / (1024**3):.1f} GB")
    
except Exception as e:
    print(f" ✗ 失败")
    print(f"❌ 错误: {e}")
    exit(1)

# 测试3: OCR处理
print("\n" + "-"*80)
print("测试3: OCR处理")
print("-"*80)

markdown_content = []
total_ocr_time = 0

for idx, image in enumerate(images, 1):
    print(f"处理第 {idx} 页...", end='', flush=True)
    
    try:
        start_time = time.time()
        
        # 保存临时图像
        temp_img = os.path.join(OUTPUT_DIR, "temp.jpg")
        image.save(temp_img)
        
        # OCR - save_results=True 会生成 result.mmd 文件但返回 None
        with torch.no_grad():
            _ = model.infer(
                tokenizer,
                prompt=PROMPT,
                image_file=temp_img,
                output_path=OUTPUT_DIR,
                base_size=1024,
                image_size=640,
                crop_mode=True,
                save_results=True,  # 必须为True才能生成输出
                test_compress=False
            )
        
        # 从生成的文件中读取结果
        result_file = os.path.join(OUTPUT_DIR, "result.mmd")
        if os.path.exists(result_file):
            with open(result_file, 'r', encoding='utf-8') as f:
                result = f.read()
        else:
            result = ""
        
        ocr_time = time.time() - start_time
        total_ocr_time += ocr_time
        
        print(f" ✓ ({ocr_time:.1f}秒)")
        
        # 添加到markdown
        markdown_content.append(f"## Page {idx}\n\n")
        if result:
            markdown_content.append(result)
        else:
            markdown_content.append("*[无OCR结果]*")
        markdown_content.append("\n\n---\n\n")
        
        # 清理临时文件
        if os.path.exists(temp_img):
            os.remove(temp_img)
        
        if device == 'cuda':
            torch.cuda.empty_cache()
        
    except Exception as e:
        print(f" ✗ 错误: {e}")
        markdown_content.append(f"## Page {idx}\n\n*[处理失败: {e}]*\n\n---\n\n")

# 保存结果
print("\n" + "-"*80)
print("保存结果")
print("-"*80)

output_md = os.path.join(OUTPUT_DIR, "test_result.md")
with open(output_md, 'w', encoding='utf-8') as f:
    f.write(f"# 测试结果\n\n")
    f.write(f"*测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*\n\n")
    f.write(''.join(markdown_content))

print(f"✓ 结果已保存: {output_md}")
print(f"  文件大小: {os.path.getsize(output_md) / 1024:.1f} KB")

# 性能总结
print("\n" + "="*80)
print("性能总结")
print("="*80)

avg_page_time = total_ocr_time / len(images)

print(f"\nPDF转图像: {convert_time:.2f}秒")
print(f"OCR处理:")
print(f"  总时间: {total_ocr_time:.2f}秒")
print(f"  平均每页: {avg_page_time:.2f}秒")
print(f"  预计速度: {3600/avg_page_time:.1f} 页/小时")

# 估算完整处理时间
print(f"\n估算 (基于当前配置):")

scenarios = [
    ("小型PDF (50页)", 50),
    ("中型PDF (200页)", 200),
    ("大型PDF (500页)", 500),
    ("特大PDF (1000页)", 1000),
]

for name, pages in scenarios:
    est_minutes = (pages * avg_page_time) / 60
    if est_minutes < 60:
        print(f"  {name}: ~{est_minutes:.0f}分钟")
    else:
        print(f"  {name}: ~{est_minutes/60:.1f}小时")

# 批量处理估算
print(f"\n批量处理44个PDF (假设平均200页/个):")
total_pages = 44 * 200
total_hours = (total_pages * avg_page_time) / 3600
print(f"  总页数: {total_pages}")
print(f"  预计时间: ~{total_hours:.1f}小时")

print("\n" + "="*80)
print("测试完成！")
print("="*80)
print("\n下一步:")
print("  1. 检查输出文件质量")
print("  2. 如果满意，可以运行批量处理脚本")
print("  3. 如果速度太慢，可以降低DPI或使用更快的GPU")
print("\n运行批量处理:")
print("  python batch_process_pdfs.py")
print("="*80)

