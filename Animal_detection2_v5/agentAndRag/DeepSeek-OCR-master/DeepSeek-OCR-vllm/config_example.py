# DeepSeek-OCR 配置文件示例
# 复制此文件为 config.py 并修改相应路径

# ============= 模型模式配置 =============
# 可选模式：
# Tiny: base_size = 512, image_size = 512, crop_mode = False   (64 tokens, 快速)
# Small: base_size = 640, image_size = 640, crop_mode = False  (100 tokens, 一般)
# Base: base_size = 1024, image_size = 1024, crop_mode = False (256 tokens, 高质量)
# Large: base_size = 1280, image_size = 1280, crop_mode = False (400 tokens, 复杂文档)
# Gundam: base_size = 1024, image_size = 640, crop_mode = True (动态 tokens, 大型文档)

BASE_SIZE = 1024        # 基础尺寸
IMAGE_SIZE = 640        # 图像分块尺寸
CROP_MODE = True        # 是否启用裁剪模式（Gundam 模式）
MIN_CROPS = 2           # 最小裁剪数量
MAX_CROPS = 6           # 最大裁剪数量 (显存小建议设为 4-6)
MAX_CONCURRENCY = 100   # 最大并发数（显存不足时降低）
NUM_WORKERS = 64        # 图像预处理工作线程数
PRINT_NUM_VIS_TOKENS = False  # 是否打印视觉 token 数量
SKIP_REPEAT = True      # 跳过重复内容

# ============= 模型路径配置 =============
# 选项 1: 使用 HuggingFace 模型名称（首次运行会自动下载）
MODEL_PATH = 'deepseek-ai/DeepSeek-OCR'

# 选项 2: 使用本地模型路径
# MODEL_PATH = 'F:/models/DeepSeek-OCR'  # Windows 路径示例
# MODEL_PATH = '/home/user/models/DeepSeek-OCR'  # Linux 路径示例

# ============= 输入输出路径配置 =============
# 重要: 必须设置这些路径才能运行！

# 输入路径（根据运行的脚本选择）:
# - run_dpsk_ocr_image.py: 单张图片路径 (.jpg, .png, .jpeg)
# - run_dpsk_ocr_pdf.py: PDF 文件路径 (.pdf)
# - run_dpsk_ocr_eval_batch.py: 图片目录路径

# 示例配置（请修改为你的实际路径）:
INPUT_PATH = 'test_images/sample.jpg'  # 单图测试
# INPUT_PATH = 'test_pdfs/document.pdf'  # PDF 测试
# INPUT_PATH = 'test_images/'  # 批量测试

# 输出路径（结果保存目录）
OUTPUT_PATH = 'output'

# ============= 提示词配置 =============
# 根据不同任务选择合适的提示词

# 文档转 Markdown（推荐，保留布局）
PROMPT = '<image>\n<|grounding|>Convert the document to markdown.'

# 其他常用提示词（取消注释使用）:
# PROMPT = '<image>\n<|grounding|>OCR this image.'  # 通用 OCR
# PROMPT = '<image>\nFree OCR.'  # 纯文本 OCR（无布局）
# PROMPT = '<image>\nParse the figure.'  # 解析图表
# PROMPT = '<image>\nDescribe this image in detail.'  # 详细描述
# PROMPT = '<image>\nLocate <|ref|>关键词<|/ref|> in the image.'  # 文本定位

# ============= Tokenizer 初始化 =============
from transformers import AutoTokenizer

TOKENIZER = AutoTokenizer.from_pretrained(MODEL_PATH, trust_remote_code=True)





















