"""
批量OCR识别图片脚本（流式生成版本）
扫描已生成的书籍文件夹中的images子文件夹，对每张图片进行OCR识别
结果保存在与images同级的images_ocr文件夹中
"""

import asyncio
import os
import glob
import gc
import re
from tqdm import tqdm

import torch
if torch.version.cuda == '11.8':
    os.environ["TRITON_PTXAS_PATH"] = "/usr/local/cuda-11.8/bin/ptxas"

os.environ['VLLM_USE_V1'] = '0'
os.environ["CUDA_VISIBLE_DEVICES"] = '0'

from vllm import AsyncLLMEngine, SamplingParams
from vllm.engine.arg_utils import AsyncEngineArgs
from vllm.model_executor.models.registry import ModelRegistry
from deepseek_ocr import DeepseekOCRForCausalLM
from PIL import Image, ImageOps
import time
from process.ngram_norepeat import NoRepeatNGramLogitsProcessor
from process.image_process import DeepseekOCRProcessor
from config import MODEL_PATH, CROP_MODE

# ==================== 配置 ====================
# OCR结果的根目录（包含多个书籍文件夹的目录）
BOOKS_ROOT_DIR = 'F:/BaiduNetdiskDownload/港城大/deepseek-ocr/DeepSeek-OCR/output_batch'

# 图片文件夹名称
IMAGES_FOLDER_NAME = 'images'

# OCR结果文件夹名称
OCR_OUTPUT_FOLDER_NAME = 'images_ocr'

# 图片OCR的prompt（针对图片/图表内容识别）
# 选项1: 解析图表/图片中的内容
IMAGE_OCR_PROMPT = '<image>\nParse the figure.'
# 选项2: 详细描述图片
# IMAGE_OCR_PROMPT = '<image>\nDescribe this image in detail.'
# 选项3: 自由OCR（不带grounding标记）
# IMAGE_OCR_PROMPT = '<image>\nFree OCR.'

# 是否跳过已处理的图片
SKIP_EXISTING = True

# 是否显示流式输出
SHOW_STREAM_OUTPUT = False

# 支持的图片格式
SUPPORTED_FORMATS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tiff', '.tif'}
# =====================================================


class Colors:
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    CYAN = '\033[36m'
    MAGENTA = '\033[35m'
    RESET = '\033[0m'


# 全局引擎（避免重复初始化）
_engine = None


def clear_memory():
    """清理内存"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


def load_image(image_path):
    """加载图片并处理EXIF旋转"""
    try:
        image = Image.open(image_path)
        corrected_image = ImageOps.exif_transpose(image)
        return corrected_image.convert('RGB')
    except Exception as e:
        print(f"加载图片失败 {image_path}: {e}")
        return None


def get_all_book_folders(root_dir):
    """获取所有书籍文件夹（包含images子文件夹的）"""
    book_folders = []
    
    if not os.path.isdir(root_dir):
        return book_folders
    
    for item in os.listdir(root_dir):
        item_path = os.path.join(root_dir, item)
        if os.path.isdir(item_path):
            images_path = os.path.join(item_path, IMAGES_FOLDER_NAME)
            if os.path.isdir(images_path):
                book_folders.append(item_path)
    
    return sorted(book_folders)


def get_images_in_folder(images_folder):
    """获取文件夹中的所有图片文件"""
    image_files = []
    
    if not os.path.isdir(images_folder):
        return image_files
    
    for filename in os.listdir(images_folder):
        ext = os.path.splitext(filename)[1].lower()
        if ext in SUPPORTED_FORMATS:
            image_files.append(os.path.join(images_folder, filename))
    
    return sorted(image_files)


def get_ocr_output_path(image_path, ocr_folder):
    """获取OCR结果的输出路径"""
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    return os.path.join(ocr_folder, f'{image_name}_OCR.txt')


def is_image_processed(image_path, ocr_folder):
    """检查图片是否已处理"""
    output_path = get_ocr_output_path(image_path, ocr_folder)
    return os.path.exists(output_path) and os.path.getsize(output_path) > 0


def clean_ocr_output(text):
    """
    清理OCR输出
    保留实际内容，只移除特殊标记符号
    """
    # 移除结束标记
    if '<｜end▁of▁sentence｜>' in text:
        text = text.replace('<｜end▁of▁sentence｜>', '')
    
    # 移除grounding标记但保留文本内容
    # <|ref|>text<|/ref|> -> text
    text = re.sub(r'<\|ref\|>', '', text)
    text = re.sub(r'<\|/ref\|>', '', text)
    
    # 移除det标记和坐标（这部分是坐标信息，通常不需要）
    # <|det|>[[x1,y1,x2,y2]]<|/det|> -> 删除
    text = re.sub(r'<\|det\|>.*?<\|/det\|>', '', text, flags=re.DOTALL)
    
    # 移除grounding标记
    text = text.replace('<|grounding|>', '')
    
    # 清理LaTeX特殊字符
    text = text.replace('\\coloneqq', ':=').replace('\\eqqcolon', '=:')
    
    # 清理多余空行
    text = re.sub(r'\n{3,}', '\n\n', text)
    
    return text.strip()


async def get_async_engine():
    """获取或创建异步引擎"""
    global _engine
    
    if _engine is None:
        print(f'{Colors.YELLOW}正在初始化异步引擎...{Colors.RESET}')
        
        ModelRegistry.register_model("DeepseekOCRForCausalLM", DeepseekOCRForCausalLM)
        
        engine_args = AsyncEngineArgs(
            model=MODEL_PATH,
            hf_overrides={"architectures": ["DeepseekOCRForCausalLM"]},
            block_size=256,
            max_model_len=8192,
            enforce_eager=False,
            trust_remote_code=True,
            tensor_parallel_size=1,
            gpu_memory_utilization=0.85,
            swap_space=4,
        )
        _engine = AsyncLLMEngine.from_engine_args(engine_args)
        print(f'{Colors.GREEN}引擎初始化完成!{Colors.RESET}')
    
    return _engine


async def stream_generate_single(image_features, prompt, show_output=False):
    """流式生成单张图片的OCR结果"""
    engine = await get_async_engine()
    
    logits_processors = [NoRepeatNGramLogitsProcessor(
        ngram_size=30, window_size=90, whitelist_token_ids={128821, 128822}
    )]
    
    sampling_params = SamplingParams(
        temperature=0.0,
        max_tokens=8192,
        logits_processors=logits_processors,
        skip_special_tokens=False,
    )
    
    request_id = f"request-{int(time.time() * 1000)}"
    
    if image_features and '<image>' in prompt:
        request = {
            "prompt": prompt,
            "multi_modal_data": {"image": image_features}
        }
    else:
        request = {"prompt": prompt}
    
    printed_length = 0
    final_output = ""
    
    async for request_output in engine.generate(request, sampling_params, request_id):
        if request_output.outputs:
            full_text = request_output.outputs[0].text
            if show_output:
                new_text = full_text[printed_length:]
                print(new_text, end='', flush=True)
                printed_length = len(full_text)
            final_output = full_text
    
    if show_output:
        print()
    
    return final_output


async def process_single_image_async(image_path, ocr_folder, prompt, show_output=False):
    """异步处理单张图片"""
    try:
        # 加载图片
        image = load_image(image_path)
        if image is None:
            return False, "加载图片失败"
        
        # 准备图片特征
        image_features = DeepseekOCRProcessor().tokenize_with_images(
            images=[image], bos=True, eos=True, cropping=CROP_MODE
        )
        
        # 流式生成OCR结果
        result = await stream_generate_single(image_features, prompt, show_output)
        
        # 清理输出
        cleaned_result = clean_ocr_output(result)
        
        # 同时保存原始输出和清理后的输出
        output_path = get_ocr_output_path(image_path, ocr_folder)
        
        # 保存清理后的结果
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(cleaned_result)
        
        # 同时保存原始结果（用于调试）
        raw_output_path = output_path.replace('_OCR.txt', '_OCR_raw.txt')
        with open(raw_output_path, 'w', encoding='utf-8') as f:
            f.write(result)
        
        return True, cleaned_result
        
    except Exception as e:
        return False, str(e)


async def process_book_images(book_folder, prompt):
    """处理单本书的所有图片"""
    book_name = os.path.basename(book_folder)
    images_folder = os.path.join(book_folder, IMAGES_FOLDER_NAME)
    ocr_folder = os.path.join(book_folder, OCR_OUTPUT_FOLDER_NAME)
    
    # 获取所有图片
    image_files = get_images_in_folder(images_folder)
    
    if not image_files:
        print(f'{Colors.YELLOW}  该书籍没有图片，跳过{Colors.RESET}')
        return 0, 0, 0
    
    # 创建OCR输出文件夹
    os.makedirs(ocr_folder, exist_ok=True)
    
    processed = 0
    skipped = 0
    failed = 0
    
    # 过滤已处理的图片
    pending_images = []
    for img_path in image_files:
        if SKIP_EXISTING and is_image_processed(img_path, ocr_folder):
            skipped += 1
        else:
            pending_images.append(img_path)
    
    if skipped > 0:
        print(f'{Colors.YELLOW}  跳过已处理: {skipped} 张{Colors.RESET}')
    
    if not pending_images:
        print(f'{Colors.GREEN}  所有图片已处理完成{Colors.RESET}')
        return processed, skipped, failed
    
    print(f'{Colors.CYAN}  待处理: {len(pending_images)} 张图片{Colors.RESET}')
    
    # 逐张处理图片（流式生成）
    for i, img_path in enumerate(pending_images, 1):
        img_name = os.path.basename(img_path)
        print(f'{Colors.MAGENTA}  [{i}/{len(pending_images)}] {img_name}{Colors.RESET}', end=' ')
        
        success, result = await process_single_image_async(
            img_path, ocr_folder, prompt, show_output=SHOW_STREAM_OUTPUT
        )
        
        if success:
            # 显示结果摘要
            preview = result[:100].replace('\n', ' ')
            if len(result) > 100:
                preview += '...'
            print(f'{Colors.GREEN}✓{Colors.RESET} ({len(result)} chars)')
            if SHOW_STREAM_OUTPUT:
                print(f'    预览: {preview}')
            processed += 1
        else:
            print(f'{Colors.RED}✗ {result}{Colors.RESET}')
            failed += 1
        
        # 定期清理内存
        if i % 10 == 0:
            clear_memory()
    
    return processed, skipped, failed


async def main_async():
    """异步主函数"""
    
    print(f'{Colors.CYAN}{"="*60}{Colors.RESET}')
    print(f'{Colors.CYAN}批量图片OCR识别工具（流式生成版）{Colors.RESET}')
    print(f'{Colors.CYAN}{"="*60}{Colors.RESET}')
    
    # 检查输入目录
    if not os.path.isdir(BOOKS_ROOT_DIR):
        print(f'{Colors.RED}错误: 目录不存在: {BOOKS_ROOT_DIR}{Colors.RESET}')
        return
    
    # 获取所有书籍文件夹
    book_folders = get_all_book_folders(BOOKS_ROOT_DIR)
    
    if not book_folders:
        print(f'{Colors.YELLOW}警告: 未找到包含 {IMAGES_FOLDER_NAME} 文件夹的书籍目录{Colors.RESET}')
        return
    
    print(f'{Colors.GREEN}找到 {len(book_folders)} 个书籍文件夹{Colors.RESET}')
    print(f'{Colors.CYAN}使用Prompt: {IMAGE_OCR_PROMPT}{Colors.RESET}')
    print()
    
    # 统计总图片数
    total_images = 0
    for book_folder in book_folders:
        images_folder = os.path.join(book_folder, IMAGES_FOLDER_NAME)
        images = get_images_in_folder(images_folder)
        total_images += len(images)
        book_name = os.path.basename(book_folder)
        print(f'  - {book_name}: {len(images)} 张图片')
    
    print(f'\n{Colors.GREEN}共计 {total_images} 张图片{Colors.RESET}')
    print()
    
    # 初始化引擎
    await get_async_engine()
    
    # 统计
    total_processed = 0
    total_skipped = 0
    total_failed = 0
    
    # 遍历每本书
    for book_idx, book_folder in enumerate(book_folders, 1):
        book_name = os.path.basename(book_folder)
        
        print(f'{Colors.BLUE}{"="*60}{Colors.RESET}')
        print(f'{Colors.BLUE}[{book_idx}/{len(book_folders)}] 处理书籍: {book_name}{Colors.RESET}')
        print(f'{Colors.BLUE}{"="*60}{Colors.RESET}')
        
        processed, skipped, failed = await process_book_images(book_folder, IMAGE_OCR_PROMPT)
        
        total_processed += processed
        total_skipped += skipped
        total_failed += failed
        
        print(f'{Colors.GREEN}  书籍完成: 成功{processed}, 跳过{skipped}, 失败{failed}{Colors.RESET}')
        
        # 每本书处理完后清理内存
        clear_memory()
    
    # 最终统计
    print()
    print(f'{Colors.GREEN}{"="*60}{Colors.RESET}')
    print(f'{Colors.GREEN}批量OCR处理完成!{Colors.RESET}')
    print(f'{Colors.GREEN}{"="*60}{Colors.RESET}')
    print(f'成功处理: {total_processed} 张图片')
    print(f'跳过(已存在): {total_skipped} 张图片')
    print(f'失败: {total_failed} 张图片')
    print(f'\n输出位置: {BOOKS_ROOT_DIR}/<书名>/{OCR_OUTPUT_FOLDER_NAME}/')
    print(f'  - *_OCR.txt: 清理后的结果')
    print(f'  - *_OCR_raw.txt: 原始输出（含标记）')


def main():
    """主函数入口"""
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
