"""
从OCR处理后的Markdown文件中提取问答对，生成LoRA微调用的JSON格式数据

处理流程：
1. 读取OCR处理后的markdown文件
2. 识别问题区域（以"序号)"格式开头，如 "291) 问题内容"）
3. 识别选项（a), b), c), d) 格式）
4. 识别答案区域（以"Answers"后跟"序号) 字母"格式）
5. 匹配问题和答案
6. 输出为LoRA微调格式的JSON
"""

import re
import json
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict

# ============ 配置 ============
# 输入文件路径（OCR处理后的markdown文件）
INPUT_MD_FILE = r"/root/project/deepseek-ocr-offline/output/300QuestionsInAnatomy.md"

# 输出JSON文件路径
OUTPUT_JSON_FILE = r"/root/project/deepseek-ocr-offline/output/300QuestionsInAnatomy.json"

# 是否输出详细日志
VERBOSE = True

# 调试模式：显示每页的内容摘要和检测结果
DEBUG_MODE = True

# ============ 数据结构 ============
@dataclass
class Question:
    """问题数据结构"""
    number: int              # 题号
    question_text: str       # 题干
    option_a: str           # 选项A
    option_b: str           # 选项B
    option_c: str           # 选项C
    option_d: str           # 选项D
    page_number: int = 0    # 所在页码（可选）

@dataclass
class Answer:
    """答案数据结构"""
    number: int             # 题号
    answer: str             # 答案（a/b/c/d）

# ============ 核心解析函数 ============

def read_markdown_file(file_path: str) -> str:
    """读取markdown文件内容"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

def split_into_pages(content: str) -> List[Tuple[int, str]]:
    """
    将markdown内容按页分割
    
    Returns:
        List of (page_number, page_content)
    """
    pages = []
    
    # 匹配 "## Page X" 格式
    page_pattern = r'## Page (\d+)\s*\n(.*?)(?=## Page \d+|---\s*$|$)'
    matches = re.findall(page_pattern, content, re.DOTALL)
    
    for match in matches:
        page_num = int(match[0])
        page_content = match[1].strip()
        pages.append((page_num, page_content))
    
    if VERBOSE:
        print(f"✓ 识别到 {len(pages)} 页内容")
    
    return pages

def extract_questions_from_text(text: str, page_num: int = 0) -> List[Question]:
    """
    从文本中提取问题
    
    问题格式识别：
    - 以"数字)"开头，如 "291) 问题内容"
    - 选项格式：a) / b) / c) / d)
    """
    questions = []
    
    # 匹配问题模式：数字) 开头，直到下一个数字) 或 Answers
    # 问题序号模式：支持 "291)" 或 "291 )" 或 "291."
    question_pattern = r'(\d+)\s*[)\.]\s*([^a-d].*?)(?=\n\s*a\s*[)\.]\s*)'
    
    # 先找出所有可能的问题块（从序号到下一个序号之间的内容）
    # 使用更宽松的模式来匹配整个问题块
    block_pattern = r'(\d+)\s*[)\.]\s*(.*?)(?=\n\s*\d+\s*[)\.]\s*[^a-d]|Answers\s*\n|\Z)'
    blocks = re.findall(block_pattern, text, re.DOTALL | re.IGNORECASE)
    
    for block in blocks:
        try:
            q_num = int(block[0])
            full_content = block[1].strip()
            
            # 检查是否是问题（应该包含选项a, b, c, d）
            if not re.search(r'\ba\s*[)\.]\s*', full_content, re.IGNORECASE):
                continue
            
            # 提取题干（在选项a之前的内容）
            question_match = re.match(r'(.*?)(?=\n?\s*a\s*[)\.]\s*)', full_content, re.DOTALL | re.IGNORECASE)
            if not question_match:
                continue
            
            question_text = question_match.group(1).strip()
            
            # 提取四个选项
            options = extract_options(full_content)
            
            if options and len(options) >= 4:
                question = Question(
                    number=q_num,
                    question_text=question_text,
                    option_a=options.get('a', ''),
                    option_b=options.get('b', ''),
                    option_c=options.get('c', ''),
                    option_d=options.get('d', ''),
                    page_number=page_num
                )
                questions.append(question)
                
        except Exception as e:
            if VERBOSE:
                print(f"  ⚠ 解析问题 {block[0]} 时出错: {e}")
            continue
    
    return questions

def extract_options(text: str) -> Dict[str, str]:
    """
    从文本中提取选项 a), b), c), d)
    
    Returns:
        Dict with keys 'a', 'b', 'c', 'd' and their content
    """
    options = {}
    
    # 匹配选项模式：字母) 内容
    # 支持 "a)" "a )" "a." "A)" 等格式
    option_pattern = r'([a-dA-D])\s*[)\.]\s*(.*?)(?=\n\s*[b-dB-D]\s*[)\.]\s*|\n\s*\d+\s*[)\.]\s*|\Z)'
    
    matches = re.findall(option_pattern, text, re.DOTALL | re.IGNORECASE)
    
    for match in matches:
        letter = match[0].lower()
        content = match[1].strip()
        # 清理内容，移除多余的换行和空格
        content = re.sub(r'\s+', ' ', content).strip()
        options[letter] = content
    
    return options

def extract_answers_from_text(text: str) -> List[Answer]:
    """
    从文本中提取答案
    
    支持两种答案格式：
    1. 标准格式：数字) 字母  如 "1) a" 或 "1. a"
    2. 表格格式：| 数字 | 字母 | 如 "| 89 | b | 125 | a |"
    """
    answers = []
    
    # 模式1：标准格式 "1) a" "1. a" "1)a" 等
    standard_pattern = r'(\d+)\s*[)\.]\s*([a-dA-D])\b'
    standard_matches = re.findall(standard_pattern, text)
    
    for match in standard_matches:
        try:
            a_num = int(match[0])
            a_letter = match[1].lower()
            answers.append(Answer(number=a_num, answer=a_letter))
        except Exception as e:
            if VERBOSE:
                print(f"  ⚠ 解析标准格式答案时出错: {e}")
            continue
    
    # 模式2：表格格式 "| 数字 | 字母 |"
    # 匹配表格行，提取所有 "数字 | 字母" 对
    table_row_pattern = r'\|[^\|]*\|'
    if '|' in text and re.search(r'\|\s*\d+\s*\|\s*[a-dA-D]\s*\|', text):
        # 检测到表格格式
        # 匹配每个 "| 数字 | 字母 |" 对
        table_pair_pattern = r'\|\s*(\d+)\s*\|\s*([a-dA-D])\s*'
        table_matches = re.findall(table_pair_pattern, text, re.IGNORECASE)
        
        if DEBUG_MODE and table_matches:
            print(f"      [DEBUG] 检测到表格格式答案: {len(table_matches)} 个")
        
        for match in table_matches:
            try:
                a_num = int(match[0])
                a_letter = match[1].lower()
                # 避免重复添加
                if not any(a.number == a_num for a in answers):
                    answers.append(Answer(number=a_num, answer=a_letter))
            except Exception as e:
                if VERBOSE:
                    print(f"  ⚠ 解析表格格式答案时出错: {e}")
                continue
    
    # 按题号排序
    answers.sort(key=lambda x: x.number)
    
    if DEBUG_MODE:
        print(f"      [DEBUG] 共提取 {len(answers)} 个答案")
    
    return answers

def is_answer_section_start(text: str) -> bool:
    """
    判断页面是否为答案区域的起始页
    
    判据：包含"Answers"关键字（支持多种变体）
    """
    
    pattern = r'Answers\s*\n(.*?)(?=\n\s*##|\Z)'
    
    if re.search(pattern, text, re.DOTALL | re.IGNORECASE):
        if DEBUG_MODE:
            match = re.search(pattern, text,re.DOTALL | re.IGNORECASE)
            if match:
                # 显示匹配位置前后的内容
                start = max(0, match.start() - 20)
                end = min(len(text), match.end() + 50)
                context = text[start:end].replace('\n', '\\n')
                print(f"      [DEBUG] 找到答案关键字: '{match.group()}' 上下文: ...{context}...")
        return True
    return False

def has_answer_format(text: str) -> bool:
    """
    判断页面是否包含答案格式
    
    支持：
    1. 标准格式：数字) 字母
    2. 表格格式：| 数字 | 字母 |
    """
    # 模式列表
    patterns = [
        r'\d+\s*[)\.]\s*[a-dA-D]\b',           # 标准格式：1) a 或 1. a
        r'\d+\s*[a-dA-D]\s*[)\.)]',             # 1 a) 格式
        r'\d+\s*:\s*[a-dA-D]\b',                # 1: a 格式
        r'\|\s*\d+\s*\|\s*[a-dA-D]\s*\|',      # 表格格式：| 89 | b |
    ]
    
    for pattern in patterns:
        matches = re.findall(pattern, text, re.IGNORECASE)
        if matches:
            if DEBUG_MODE:
                print(f"      [DEBUG] 找到答案格式: {matches[:5]}...")
            return True
    return False

def debug_page_content(page_num: int, content: str):
    """调试：显示页面内容摘要"""
    if not DEBUG_MODE:
        return
    
    # 显示前200个字符
    preview = content[:200].replace('\n', '\\n')
    print(f"    [DEBUG] 页面内容预览: {preview}...")
    
    # 检查是否包含可能的答案关键字
    answer_keywords = ['answer', 'Answer', 'ANSWER', 'Answers', '答案']
    found_keywords = [kw for kw in answer_keywords if kw in content]
    if found_keywords:
        print(f"    [DEBUG] 发现关键字: {found_keywords}")
    
    # 检查是否有 "数字)" 格式
    number_matches = re.findall(r'\d+\s*[)\.]', content)
    if number_matches:
        print(f"    [DEBUG] 发现序号格式: {number_matches[:10]}...")

def parse_markdown_content(content: str) -> Tuple[List[Question], List[Answer]]:
    """
    解析完整的markdown内容，提取问题和答案
    
    逻辑：
    1. 遍历所有页面
    2. 当遇到包含"Answers"的页面时，标记答案区域开始
    3. 从答案区域开始后，持续提取答案，直到遇到不包含"数字) 字母"格式的页面
    4. 非答案区域的页面提取问题
    
    Returns:
        (questions_list, answers_list)
    """
    all_questions = []
    all_answers = []
    
    # 分页处理
    pages = split_into_pages(content)
    
    if not pages:
        # 如果没有识别到页码格式，直接处理整个内容
        if VERBOSE:
            print("  未识别到页码格式，直接处理全文...")
        
        # 分离问题区域和答案区域
        if is_answer_section_start(content):
            # 找到Answers的位置，分割内容
            answers_start = re.search(r'\bAnswers\b', content, re.IGNORECASE)
            if answers_start:
                questions_text = content[:answers_start.start()]
                answers_text = content[answers_start.start():]
                
                all_questions.extend(extract_questions_from_text(questions_text))
                all_answers.extend(extract_answers_from_text(answers_text))
        else:
            all_questions.extend(extract_questions_from_text(content))
        
        return all_questions, all_answers
    
    # 按页处理，需要追踪是否在答案区域中
    in_answer_section = False
    answer_section_start_page = None
    answer_section_end_page = None
    
    # 第一遍：找出答案区域的范围
    print("\n  === 第一遍扫描：查找答案区域 ===")
    for i, (page_num, page_content) in enumerate(pages):
        if DEBUG_MODE:
            print(f"\n  扫描第 {page_num} 页...")
            debug_page_content(page_num, page_content)
        
        if not in_answer_section:
            # 检查是否是答案区域的起始页（包含"Answers"关键字）
            if is_answer_section_start(page_content):
                in_answer_section = True
                answer_section_start_page = i
                if VERBOSE:
                    print(f"  ★ 第 {page_num} 页: 发现答案区域起始!")
        else:
            # 已在答案区域中，检查是否仍有答案格式
            if has_answer_format(page_content):
                # 继续在答案区域中
                if VERBOSE:
                    print(f"  第 {page_num} 页: 答案区域继续")
            else:
                # 答案区域结束
                answer_section_end_page = i
                in_answer_section = False
                if VERBOSE:
                    print(f"  第 {page_num} 页: 答案区域结束")
                break
    
    # 如果答案区域延续到最后一页
    if in_answer_section and answer_section_end_page is None:
        answer_section_end_page = len(pages)
    
    if VERBOSE:
        if answer_section_start_page is not None:
            start_page_num = pages[answer_section_start_page][0]
            end_page_num = pages[answer_section_end_page - 1][0] if answer_section_end_page else "最后一页"
            print(f"\n  ✓ 答案区域: 第 {start_page_num} 页 到 第 {end_page_num} 页")
        else:
            print(f"\n  ⚠ 未找到答案区域!")
            print(f"    提示: 请检查您的文件中是否有 'Answers' 关键字")
            print(f"    提示: 如果关键字不同，请修改 is_answer_section_start 函数")
    
    print("\n  === 第二遍扫描：提取内容 ===")
    
    # 第二遍：按区域提取内容
    for i, (page_num, page_content) in enumerate(pages):
        if VERBOSE:
            print(f"  处理第 {page_num} 页...")
        
        # 判断当前页是否在答案区域内
        is_in_answer_region = (
            answer_section_start_page is not None and 
            i >= answer_section_start_page and 
            (answer_section_end_page is None or i < answer_section_end_page)
        )
        
        if is_in_answer_region:
            # 答案区域
            if i == answer_section_start_page:
                # 答案区域起始页，可能有"Answers"之前的题目
                answers_marker = re.search(r'\bAnswers\b', page_content, re.IGNORECASE)
                if answers_marker:
                    # 提取Answers之前的题目
                    questions_part = page_content[:answers_marker.start()]
                    questions = extract_questions_from_text(questions_part, page_num)
                    all_questions.extend(questions)
                    if VERBOSE and questions:
                        print(f"    ↳ 提取到 {len(questions)} 个问题（Answers之前）")
                    
                    # 提取Answers之后的答案
                    answers_part = page_content[answers_marker.start():]
                    answers = extract_answers_from_text(answers_part)
                    all_answers.extend(answers)
                    if VERBOSE:
                        print(f"    ↳ 提取到 {len(answers)} 个答案")
            else:
                # 答案区域的后续页面，整页都是答案
                answers = extract_answers_from_text(page_content)
                all_answers.extend(answers)
                if VERBOSE:
                    print(f"    ↳ 提取到 {len(answers)} 个答案（答案续页）")
        else:
            # 问题区域
            questions = extract_questions_from_text(page_content, page_num)
            all_questions.extend(questions)
            if VERBOSE and questions:
                print(f"    ↳ 提取到 {len(questions)} 个问题")
    
    return all_questions, all_answers

def match_questions_and_answers(
    questions: List[Question], 
    answers: List[Answer]
) -> List[Dict]:
    """
    匹配问题和答案
    
    Returns:
        List of matched Q&A dicts
    """
    # 构建答案查找表
    answer_map = {a.number: a.answer for a in answers}
    
    matched_qa = []
    unmatched_questions = []
    
    for q in questions:
        if q.number in answer_map:
            matched_qa.append({
                'number': q.number,
                'question': q,
                'answer': answer_map[q.number]
            })
        else:
            unmatched_questions.append(q.number)
    
    if VERBOSE:
        print(f"\n匹配结果:")
        print(f"  ✓ 成功匹配: {len(matched_qa)} 对")
        if unmatched_questions:
            print(f"  ⚠ 未找到答案的问题: {unmatched_questions[:10]}...")
    
    return matched_qa

def convert_to_lora_format(matched_qa: List[Dict], format_type: str = "alpaca") -> List[Dict]:
    """
    转换为LoRA微调格式
    
    支持的格式类型：
    - "alpaca": Alpaca格式 (instruction, input, output)
    - "sharegpt": ShareGPT格式 (conversations)
    - "simple": 简单格式 (question, answer)
    """
    lora_data = []
    
    for qa in matched_qa:
        q: Question = qa['question']
        answer_letter = qa['answer']
        
        # 构建完整的问题文本
        full_question = f"{q.question_text}\n\na) {q.option_a}\nb) {q.option_b}\nc) {q.option_c}\nd) {q.option_d}"
        
        # 获取正确答案的完整内容
        answer_content = {
            'a': q.option_a,
            'b': q.option_b,
            'c': q.option_c,
            'd': q.option_d
        }.get(answer_letter, '')
        
        if format_type == "alpaca":
            # Alpaca格式
            lora_data.append({
                "instruction": "请回答以下选择题，选择正确的答案。",
                "input": full_question,
                "output": f"正确答案是 {answer_letter.upper()}) {answer_content}"
            })
            
        elif format_type == "sharegpt":
            # ShareGPT格式
            lora_data.append({
                "conversations": [
                    {
                        "from": "human",
                        "value": f"请回答以下选择题：\n\n{full_question}"
                    },
                    {
                        "from": "gpt",
                        "value": f"正确答案是 {answer_letter.upper()}) {answer_content}"
                    }
                ]
            })
            
        elif format_type == "simple":
            # 简单格式
            lora_data.append({
                "question_number": q.number,
                "question": q.question_text,
                "options": {
                    "a": q.option_a,
                    "b": q.option_b,
                    "c": q.option_c,
                    "d": q.option_d
                },
                "correct_answer": answer_letter,
                "correct_option_text": answer_content,
                "page": q.page_number
            })
    
    return lora_data

def save_json(data: List[Dict], output_path: str):
    """保存JSON文件"""
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"✓ 已保存到: {output_path}")

# ============ 主程序 ============

def main():
    print("\n" + "="*80)
    print("OCR问答提取工具 - 生成LoRA微调数据")
    print("="*80)
    
    # 检查输入文件
    if not os.path.exists(INPUT_MD_FILE):
        print(f"❌ 输入文件不存在: {INPUT_MD_FILE}")
        print("\n请修改 INPUT_MD_FILE 变量为正确的文件路径")
        return
    
    print(f"\n输入文件: {INPUT_MD_FILE}")
    print(f"输出文件: {OUTPUT_JSON_FILE}")
    
    # 读取markdown内容
    print("\n" + "-"*40)
    print("步骤1: 读取markdown文件")
    print("-"*40)
    content = read_markdown_file(INPUT_MD_FILE)
    print(f"✓ 读取完成，共 {len(content)} 字符")
    
    # 解析问题和答案
    print("\n" + "-"*40)
    print("步骤2: 解析问题和答案")
    print("-"*40)
    questions, answers = parse_markdown_content(content)
    print(f"\n提取结果:")
    print(f"  问题数量: {len(questions)}")
    print(f"  答案数量: {len(answers)}")
    
    # 显示部分问题示例
    if VERBOSE and questions:
        print(f"\n问题示例（前3个）:")
        for q in questions[:3]:
            print(f"  #{q.number}: {q.question_text[:50]}...")
            print(f"    a) {q.option_a[:30]}...")
    
    # 显示部分答案示例
    if VERBOSE and answers:
        print(f"\n答案示例（前10个）:")
        for a in answers[:10]:
            print(f"  #{a.number}: {a.answer}")
    
    # 匹配问题和答案
    print("\n" + "-"*40)
    print("步骤3: 匹配问题和答案")
    print("-"*40)
    matched_qa = match_questions_and_answers(questions, answers)
    
    if not matched_qa:
        print("❌ 没有匹配到任何问答对！")
        print("\n可能的原因:")
        print("  1. 问题格式不符合预期（应为 '序号) 问题内容'）")
        print("  2. 答案格式不符合预期（应为 'Answers' 后跟 '序号) 字母'）")
        print("  3. 问题序号和答案序号不匹配")
        return
    
    # 转换为LoRA格式
    print("\n" + "-"*40)
    print("步骤4: 转换为LoRA微调格式")
    print("-"*40)
    
    # 生成三种格式
    alpaca_data = convert_to_lora_format(matched_qa, "alpaca")
    sharegpt_data = convert_to_lora_format(matched_qa, "sharegpt")
    simple_data = convert_to_lora_format(matched_qa, "simple")
    
    # 保存文件
    base_path = os.path.splitext(OUTPUT_JSON_FILE)[0]
    
    save_json(alpaca_data, f"{base_path}_alpaca.json")
    save_json(sharegpt_data, f"{base_path}_sharegpt.json")
    save_json(simple_data, f"{base_path}_simple.json")
    
    # 显示示例输出
    print("\n" + "-"*40)
    print("输出示例 (Alpaca格式):")
    print("-"*40)
    if alpaca_data:
        print(json.dumps(alpaca_data[0], ensure_ascii=False, indent=2))
    
    print("\n" + "="*80)
    print("处理完成！")
    print("="*80)
    print(f"共生成 {len(matched_qa)} 对问答数据")
    print(f"\n输出文件:")
    print(f"  Alpaca格式: {base_path}_alpaca.json")
    print(f"  ShareGPT格式: {base_path}_sharegpt.json")
    print(f"  简单格式: {base_path}_simple.json")

if __name__ == "__main__":
    main()

