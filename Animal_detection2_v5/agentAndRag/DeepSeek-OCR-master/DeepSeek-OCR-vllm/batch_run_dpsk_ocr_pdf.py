"""
批量处理PDF文件转Markdown的脚本
基于 run_dpsk_ocr_pdf.py 修改，支持批量处理整个文件夹中的PDF文件
增加分批处理功能，解决大文件内存不足问题
"""

import os
import fitz
import img2pdf
import io
import re
import gc
import glob
from tqdm import tqdm
import torch
from concurrent.futures import ThreadPoolExecutor


if torch.version.cuda == '11.8':
    os.environ["TRITON_PTXAS_PATH"] = "/usr/local/cuda-11.8/bin/ptxas"
os.environ['VLLM_USE_V1'] = '0'
os.environ["CUDA_VISIBLE_DEVICES"] = '0'


from config import MODEL_PATH, PROMPT, SKIP_REPEAT, MAX_CONCURRENCY, NUM_WORKERS, CROP_MODE

from PIL import Image, ImageDraw, ImageFont
import numpy as np
from deepseek_ocr import DeepseekOCRForCausalLM

from vllm.model_executor.models.registry import ModelRegistry

from vllm import LLM, SamplingParams
from process.ngram_norepeat import NoRepeatNGramLogitsProcessor
from process.image_process import DeepseekOCRProcessor

# ==================== 批量处理配置 ====================
# 设置PDF输入目录（包含多个PDF文件的文件夹）
PDF_INPUT_DIR = 'F:/BaiduNetdiskDownload/港城大/deepseek-ocr/DeepSeek-OCR/input_pdfs'

# 设置输出根目录（每个PDF会在此目录下创建独立的子目录）
OUTPUT_ROOT_DIR = 'F:/BaiduNetdiskDownload/港城大/deepseek-ocr/DeepSeek-OCR/output_batch'

# 是否递归搜索子目录中的PDF
RECURSIVE_SEARCH = False

# ==================== 内存优化配置 ====================
# 每批处理的页数（根据你的内存大小调整，建议8-32）
BATCH_SIZE_PAGES = 16

# 是否启用断点续传（跳过已处理的PDF）
RESUME_MODE = True
# =====================================================

ModelRegistry.register_model("DeepseekOCRForCausalLM", DeepseekOCRForCausalLM)

# 初始化模型（只需初始化一次）
print("正在加载模型...")
llm = LLM(
    model=MODEL_PATH,
    hf_overrides={"architectures": ["DeepseekOCRForCausalLM"]},
    block_size=256,
    enforce_eager=False,
    trust_remote_code=True, 
    max_model_len=8192,
    swap_space=4,  # 增加swap空间，使用CPU内存作为缓冲
    max_num_seqs=MAX_CONCURRENCY,
    tensor_parallel_size=1,
    gpu_memory_utilization=0.9,
    disable_mm_preprocessor_cache=True
)

logits_processors = [NoRepeatNGramLogitsProcessor(ngram_size=20, window_size=50, whitelist_token_ids={128821, 128822})]

sampling_params = SamplingParams(
    temperature=0.0,
    max_tokens=8192,
    logits_processors=logits_processors,
    skip_special_tokens=False,
    include_stop_str_in_output=True,
)


class Colors:
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    CYAN = '\033[36m'
    MAGENTA = '\033[35m'
    RESET = '\033[0m' 


def clear_memory():
    """清理内存"""
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


def pdf_to_images_generator(pdf_path, dpi=144, image_format="PNG"):
    """
    PDF转图片生成器 - 逐页生成，节省内存
    """
    pdf_document = fitz.open(pdf_path)
    
    zoom = dpi / 72.0
    matrix = fitz.Matrix(zoom, zoom)
    total_pages = pdf_document.page_count
    
    for page_num in range(total_pages):
        page = pdf_document[page_num]
        pixmap = page.get_pixmap(matrix=matrix, alpha=False)
        Image.MAX_IMAGE_PIXELS = None

        if image_format.upper() == "PNG":
            img_data = pixmap.tobytes("png")
            img = Image.open(io.BytesIO(img_data))
        else:
            img_data = pixmap.tobytes("png")
            img = Image.open(io.BytesIO(img_data))
            if img.mode in ('RGBA', 'LA'):
                background = Image.new('RGB', img.size, (255, 255, 255))
                background.paste(img, mask=img.split()[-1] if img.mode == 'RGBA' else None)
                img = background
        
        yield page_num, img
    
    pdf_document.close()


def get_pdf_page_count(pdf_path):
    """获取PDF页数"""
    pdf_document = fitz.open(pdf_path)
    count = pdf_document.page_count
    pdf_document.close()
    return count


def pdf_to_images_batch(pdf_path, start_page, end_page, dpi=144, image_format="PNG"):
    """
    PDF转图片 - 只加载指定范围的页面
    """
    images = []
    pdf_document = fitz.open(pdf_path)
    
    zoom = dpi / 72.0
    matrix = fitz.Matrix(zoom, zoom)
    
    for page_num in range(start_page, min(end_page, pdf_document.page_count)):
        page = pdf_document[page_num]
        pixmap = page.get_pixmap(matrix=matrix, alpha=False)
        Image.MAX_IMAGE_PIXELS = None

        if image_format.upper() == "PNG":
            img_data = pixmap.tobytes("png")
            img = Image.open(io.BytesIO(img_data))
        else:
            img_data = pixmap.tobytes("png")
            img = Image.open(io.BytesIO(img_data))
            if img.mode in ('RGBA', 'LA'):
                background = Image.new('RGB', img.size, (255, 255, 255))
                background.paste(img, mask=img.split()[-1] if img.mode == 'RGBA' else None)
                img = background
        
        images.append(img)
    
    pdf_document.close()
    return images


def pil_to_pdf_img2pdf(pil_images, output_path):
    if not pil_images:
        return
    
    image_bytes_list = []
    
    for img in pil_images:
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        img_buffer = io.BytesIO()
        img.save(img_buffer, format='JPEG', quality=95)
        img_bytes = img_buffer.getvalue()
        image_bytes_list.append(img_bytes)
    
    try:
        pdf_bytes = img2pdf.convert(image_bytes_list)
        with open(output_path, "wb") as f:
            f.write(pdf_bytes)

    except Exception as e:
        print(f"error: {e}")


def re_match(text):
    pattern = r'(<\|ref\|>(.*?)<\|/ref\|><\|det\|>(.*?)<\|/det\|>)'
    matches = re.findall(pattern, text, re.DOTALL)

    mathes_image = []
    mathes_other = []
    for a_match in matches:
        if '<|ref|>image<|/ref|>' in a_match[0]:
            mathes_image.append(a_match[0])
        else:
            mathes_other.append(a_match[0])
    return matches, mathes_image, mathes_other


def extract_coordinates_and_label(ref_text, image_width, image_height):
    try:
        label_type = ref_text[1]
        cor_list = eval(ref_text[2])
    except Exception as e:
        print(e)
        return None

    return (label_type, cor_list)


def draw_bounding_boxes(image, refs, jdx, output_images_dir):
    image_width, image_height = image.size
    img_draw = image.copy()
    draw = ImageDraw.Draw(img_draw)

    overlay = Image.new('RGBA', img_draw.size, (0, 0, 0, 0))
    draw2 = ImageDraw.Draw(overlay)
    
    font = ImageFont.load_default()

    img_idx = 0
    
    for i, ref in enumerate(refs):
        try:
            result = extract_coordinates_and_label(ref, image_width, image_height)
            if result:
                label_type, points_list = result
                
                color = (np.random.randint(0, 200), np.random.randint(0, 200), np.random.randint(0, 255))

                color_a = color + (20, )
                for points in points_list:
                    x1, y1, x2, y2 = points

                    x1 = int(x1 / 999 * image_width)
                    y1 = int(y1 / 999 * image_height)

                    x2 = int(x2 / 999 * image_width)
                    y2 = int(y2 / 999 * image_height)

                    if label_type == 'image':
                        try:
                            cropped = image.crop((x1, y1, x2, y2))
                            cropped.save(f"{output_images_dir}/{jdx}_{img_idx}.jpg")
                        except Exception as e:
                            print(e)
                            pass
                        img_idx += 1
                        
                    try:
                        if label_type == 'title':
                            draw.rectangle([x1, y1, x2, y2], outline=color, width=4)
                            draw2.rectangle([x1, y1, x2, y2], fill=color_a, outline=(0, 0, 0, 0), width=1)
                        else:
                            draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
                            draw2.rectangle([x1, y1, x2, y2], fill=color_a, outline=(0, 0, 0, 0), width=1)

                        text_x = x1
                        text_y = max(0, y1 - 15)
                            
                        text_bbox = draw.textbbox((0, 0), label_type, font=font)
                        text_width = text_bbox[2] - text_bbox[0]
                        text_height = text_bbox[3] - text_bbox[1]
                        draw.rectangle([text_x, text_y, text_x + text_width, text_y + text_height], 
                                    fill=(255, 255, 255, 30))
                        
                        draw.text((text_x, text_y), label_type, font=font, fill=color)
                    except:
                        pass
        except:
            continue
    img_draw.paste(overlay, (0, 0), overlay)
    return img_draw


def process_image_with_refs(image, ref_texts, jdx, output_images_dir):
    result_image = draw_bounding_boxes(image, ref_texts, jdx, output_images_dir)
    return result_image


def process_single_image(args):
    """处理单张图片"""
    image, prompt = args
    cache_item = {
        "prompt": prompt,
        "multi_modal_data": {"image": DeepseekOCRProcessor().tokenize_with_images(images=[image], bos=True, eos=True, cropping=CROP_MODE)},
    }
    return cache_item


def get_pdf_files(input_dir, recursive=False):
    """获取目录中的所有PDF文件"""
    if recursive:
        pattern = os.path.join(input_dir, '**', '*.pdf')
        pdf_files = glob.glob(pattern, recursive=True)
    else:
        pattern = os.path.join(input_dir, '*.pdf')
        pdf_files = glob.glob(pattern)
    
    # 也支持大写扩展名
    if recursive:
        pattern_upper = os.path.join(input_dir, '**', '*.PDF')
        pdf_files.extend(glob.glob(pattern_upper, recursive=True))
    else:
        pattern_upper = os.path.join(input_dir, '*.PDF')
        pdf_files.extend(glob.glob(pattern_upper))
    
    return sorted(set(pdf_files))


def is_pdf_completed(pdf_output_dir, pdf_name):
    """检查PDF是否已经处理完成"""
    mmd_path = os.path.join(pdf_output_dir, f'{pdf_name}.mmd')
    return os.path.exists(mmd_path) and os.path.getsize(mmd_path) > 0


def process_single_pdf(pdf_path, output_dir, prompt):
    """处理单个PDF文件 - 分批处理版本"""
    
    # 获取PDF文件名（不含扩展名）
    pdf_name = os.path.splitext(os.path.basename(pdf_path))[0]
    
    # 创建该PDF的输出目录
    pdf_output_dir = os.path.join(output_dir, pdf_name)
    
    # 断点续传检查
    if RESUME_MODE and is_pdf_completed(pdf_output_dir, pdf_name):
        print(f'{Colors.YELLOW}跳过已完成: {pdf_name}{Colors.RESET}')
        return True
    
    os.makedirs(pdf_output_dir, exist_ok=True)
    os.makedirs(f'{pdf_output_dir}/images', exist_ok=True)
    
    print(f'{Colors.CYAN}正在处理: {pdf_path}{Colors.RESET}')
    
    # 获取PDF总页数
    try:
        total_pages = get_pdf_page_count(pdf_path)
    except Exception as e:
        print(f'{Colors.RED}加载PDF失败: {pdf_path}, 错误: {e}{Colors.RESET}')
        return False
    
    print(f'{Colors.GREEN}PDF共 {total_pages} 页，分批处理中（每批 {BATCH_SIZE_PAGES} 页）...{Colors.RESET}')
    
    # 生成输出文件路径
    mmd_det_path = os.path.join(pdf_output_dir, f'{pdf_name}_det.mmd')
    mmd_path = os.path.join(pdf_output_dir, f'{pdf_name}.mmd')
    pdf_out_path = os.path.join(pdf_output_dir, f'{pdf_name}_layouts.pdf')
    
    contents_det = ''
    contents = ''
    all_draw_images = []
    global_page_idx = 0
    
    # 计算批次数
    num_batches = (total_pages + BATCH_SIZE_PAGES - 1) // BATCH_SIZE_PAGES
    
    # 分批处理
    for batch_idx in range(num_batches):
        start_page = batch_idx * BATCH_SIZE_PAGES
        end_page = min(start_page + BATCH_SIZE_PAGES, total_pages)
        
        print(f'{Colors.MAGENTA}  批次 [{batch_idx + 1}/{num_batches}]: 页面 {start_page + 1}-{end_page}{Colors.RESET}')
        
        # 加载当前批次的页面
        try:
            images = pdf_to_images_batch(pdf_path, start_page, end_page)
        except Exception as e:
            print(f'{Colors.RED}  加载页面失败: {e}{Colors.RESET}')
            continue
        
        if not images:
            continue
        
        # 预处理当前批次的图片
        with ThreadPoolExecutor(max_workers=min(NUM_WORKERS, len(images))) as executor:
            batch_inputs = list(tqdm(
                executor.map(process_single_image, [(img, prompt) for img in images]),
                total=len(images),
                desc=f"  预处理批次 {batch_idx + 1}",
                leave=False
            ))
        
        # 使用模型进行OCR
        try:
            outputs_list = llm.generate(
                batch_inputs,
                sampling_params=sampling_params
            )
        except Exception as e:
            print(f'{Colors.RED}  OCR失败: {e}{Colors.RESET}')
            # 清理内存后继续
            del batch_inputs
            del images
            clear_memory()
            continue
        
        # 处理当前批次的输出
        for output, img in zip(outputs_list, images):
            content = output.outputs[0].text

            if '<｜end▁of▁sentence｜>' in content:
                content = content.replace('<｜end▁of▁sentence｜>', '')
            else:
                if SKIP_REPEAT:
                    global_page_idx += 1
                    continue

            page_num = f'\n<--- Page Split --->'
            contents_det += content + f'\n{page_num}\n'

            image_draw = img.copy()
            matches_ref, matches_images, mathes_other = re_match(content)
            result_image = process_image_with_refs(image_draw, matches_ref, global_page_idx, f'{pdf_output_dir}/images')

            all_draw_images.append(result_image)

            for idx, a_match_image in enumerate(matches_images):
                content = content.replace(a_match_image, f'![](images/' + str(global_page_idx) + '_' + str(idx) + '.jpg)\n')

            for idx, a_match_other in enumerate(mathes_other):
                content = content.replace(a_match_other, '').replace('\\coloneqq', ':=').replace('\\eqqcolon', '=:').replace('\n\n\n\n', '\n\n').replace('\n\n\n', '\n\n')

            contents += content + f'\n{page_num}\n'
            global_page_idx += 1
        
        # 清理当前批次的内存
        del batch_inputs
        del outputs_list
        del images
        clear_memory()
        
        print(f'{Colors.GREEN}  批次 {batch_idx + 1} 完成{Colors.RESET}')

    # 保存输出文件
    print(f'{Colors.YELLOW}正在保存输出文件...{Colors.RESET}')
    
    with open(mmd_det_path, 'w', encoding='utf-8') as afile:
        afile.write(contents_det)

    with open(mmd_path, 'w', encoding='utf-8') as afile:
        afile.write(contents)

    if all_draw_images:
        pil_to_pdf_img2pdf(all_draw_images, pdf_out_path)
    
    # 清理
    del all_draw_images
    clear_memory()
    
    print(f'{Colors.GREEN}完成: {pdf_name}{Colors.RESET}')
    print(f'  - Markdown: {mmd_path}')
    print(f'  - 详细输出: {mmd_det_path}')
    print(f'  - 布局PDF: {pdf_out_path}')
    
    return True


def main():
    """主函数 - 批量处理PDF"""
    
    # 检查输入目录
    if not os.path.isdir(PDF_INPUT_DIR):
        print(f'{Colors.RED}错误: 输入目录不存在: {PDF_INPUT_DIR}{Colors.RESET}')
        return
    
    # 创建输出根目录
    os.makedirs(OUTPUT_ROOT_DIR, exist_ok=True)
    
    # 获取所有PDF文件
    pdf_files = get_pdf_files(PDF_INPUT_DIR, recursive=RECURSIVE_SEARCH)
    
    if not pdf_files:
        print(f'{Colors.YELLOW}警告: 在 {PDF_INPUT_DIR} 中未找到PDF文件{Colors.RESET}')
        return
    
    print(f'{Colors.GREEN}找到 {len(pdf_files)} 个PDF文件待处理{Colors.RESET}')
    print(f'{Colors.CYAN}内存优化配置: 每批处理 {BATCH_SIZE_PAGES} 页{Colors.RESET}')
    if RESUME_MODE:
        print(f'{Colors.CYAN}断点续传: 已启用（跳过已完成的PDF）{Colors.RESET}')
    
    for i, pdf_file in enumerate(pdf_files, 1):
        print(f'  {i}. {os.path.basename(pdf_file)}')
    print()
    
    prompt = PROMPT
    
    # 统计处理结果
    success_count = 0
    fail_count = 0
    skip_count = 0
    failed_files = []
    
    # 逐个处理PDF文件
    for i, pdf_path in enumerate(pdf_files, 1):
        print(f'\n{Colors.BLUE}{"="*60}{Colors.RESET}')
        print(f'{Colors.BLUE}[{i}/{len(pdf_files)}] 处理中...{Colors.RESET}')
        print(f'{Colors.BLUE}{"="*60}{Colors.RESET}')
        
        try:
            # 检查是否已完成（断点续传）
            pdf_name = os.path.splitext(os.path.basename(pdf_path))[0]
            pdf_output_dir = os.path.join(OUTPUT_ROOT_DIR, pdf_name)
            
            if RESUME_MODE and is_pdf_completed(pdf_output_dir, pdf_name):
                print(f'{Colors.YELLOW}跳过已完成: {pdf_name}{Colors.RESET}')
                skip_count += 1
                continue
            
            success = process_single_pdf(pdf_path, OUTPUT_ROOT_DIR, prompt)
            if success:
                success_count += 1
            else:
                fail_count += 1
                failed_files.append(pdf_path)
                
        except Exception as e:
            print(f'{Colors.RED}处理失败: {pdf_path}{Colors.RESET}')
            print(f'{Colors.RED}错误信息: {e}{Colors.RESET}')
            import traceback
            traceback.print_exc()
            fail_count += 1
            failed_files.append(pdf_path)
        
        # 每个PDF处理完后清理内存
        clear_memory()
    
    # 打印最终统计
    print(f'\n{Colors.GREEN}{"="*60}{Colors.RESET}')
    print(f'{Colors.GREEN}批量处理完成!{Colors.RESET}')
    print(f'{Colors.GREEN}{"="*60}{Colors.RESET}')
    print(f'成功: {success_count} 个文件')
    print(f'跳过: {skip_count} 个文件（已完成）')
    print(f'失败: {fail_count} 个文件')
    print(f'输出目录: {OUTPUT_ROOT_DIR}')
    
    if failed_files:
        print(f'\n{Colors.RED}失败的文件:{Colors.RESET}')
        for f in failed_files:
            print(f'  - {f}')


if __name__ == "__main__":
    main()
