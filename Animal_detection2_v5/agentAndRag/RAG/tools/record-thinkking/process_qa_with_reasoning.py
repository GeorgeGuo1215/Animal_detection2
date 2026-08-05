#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
问答对深度思考处理脚本

功能：
1. 读取alpaca格式的问答对JSON文件
2. 使用闭源模型API（支持DeepSeek、OpenAI等兼容API）对问题进行处理
3. 记录模型的"深度思考"过程（reasoning_content）
4. 将思考过程加入到问答对JSON中

使用方法：
    1. 修改下方 CONFIG 配置
    2. 运行: python process_qa_with_reasoning.py
"""

import json
import os
import time
from typing import Optional, Dict, Any, List
from tqdm import tqdm
import logging

# ============================================================
# 配置区域 - 请在此处修改配置
# ============================================================

CONFIG = {
    # API配置
    "api_key": "",  # API密钥，留空则从环境变量读取
    "base_url": "https://api.deepseek.com/v3.2_speciale_expires_on_20251215",  # API地址
    "model": "deepseek-reasoner",  # 模型名称 (deepseek-reasoner = DeepSeek-R1)
    "max_tokens": 8192,  # 最大生成token数
    "temperature": 0.7,  # 温度参数
    "timeout": 120,  # 请求超时时间(秒)
    
    
    # 文件配置
    "input_file": "300QuestionsInAnatomy_alpaca.json",  # 输入文件
    "output_file": "",  # 输出文件，留空自动生成
    
    # 处理配置
    "start_index": 0,  # 起始索引
    "end_index": None,  # 结束索引，None表示全部
    "save_interval": 10,  # 每处理N条保存一次
    "delay": 1.0,  # 请求间隔(秒)
    "skip_processed": True,  # 跳过已处理的条目
    
    # 系统提示词
    "system_prompt": """ 你是一个专业的问答助手。请仔细分析问题，进行深度思考，然后给出准确的答案。
在回答选择题时，请先分析每个选项，然后给出正确答案并解释原因。给出答案时请直接给出答案，不要给出任何解释。"""
}

# ============================================================
# 以下代码无需修改
# ============================================================

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ReasoningModelClient:
    """支持深度思考的模型客户端"""
    
    def __init__(self, api_key: str, base_url: str, model: str, 
                 max_tokens: int, temperature: float, timeout: int):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.timeout = timeout
        self._init_client()
    
    def _init_client(self):
        """初始化API客户端"""
        try:
            from openai import OpenAI
            self.client = OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
                timeout=self.timeout
            )
            logger.info(f"API客户端初始化成功: {self.base_url}")
        except ImportError:
            raise ImportError("请先安装openai库: pip install openai")
    
    def get_response_with_reasoning(
        self, 
        instruction: str, 
        input_text: str,
        system_prompt: str
    ) -> Dict[str, Any]:
        """获取模型响应，包含深度思考过程"""
        
        user_message = f"{instruction}\n\n{input_text}" if instruction else input_text
        
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
        
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                stream=False,
                extra_body={"thinking": {"type": "enabled"}}
            )
            
            result = {
                "reasoning_content": "",
                "content": "",
                "usage": None,
                "model": self.model,
                "success": True,
                "error": None
            }
            
            # 提取响应内容
            if response.choices and len(response.choices) > 0:
                message = response.choices[0].message
                
                # 获取reasoning_content（DeepSeek-R1支持）
                if hasattr(message, 'reasoning_content') and message.reasoning_content:
                    result["reasoning_content"] = message.reasoning_content
                
                # 获取主要内容
                if hasattr(message, 'content') and message.content:
                    result["content"] = message.content
            
            # 提取token使用情况
            if response.usage:
                result["usage"] = {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens
                }
                if hasattr(response.usage, 'completion_tokens_details'):
                    details = response.usage.completion_tokens_details
                    if hasattr(details, 'reasoning_tokens'):
                        result["usage"]["reasoning_tokens"] = details.reasoning_tokens
            
            return result
            
        except Exception as e:
            logger.error(f"API调用失败: {str(e)}")
            return {
                "reasoning_content": "",
                "content": "",
                "usage": None,
                "model": self.model,
                "success": False,
                "error": str(e)
            }


def load_json(file_path: str) -> List[Dict]:
    """加载JSON文件"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_json(data: List[Dict], file_path: str):
    """保存JSON文件"""
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main():
    # 获取API密钥
    api_key = CONFIG["api_key"] or os.getenv("API_KEY") or os.getenv("DEEPSEEK_API_KEY")
    if not api_key:
        logger.error("请设置API密钥！修改CONFIG中的api_key，或设置环境变量 API_KEY / DEEPSEEK_API_KEY")
        return
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 解析文件路径
    input_file = CONFIG["input_file"]
    if not os.path.isabs(input_file):
        input_file = os.path.join(script_dir, input_file)
    
    output_file = CONFIG["output_file"]
    if not output_file:
        base_name = os.path.splitext(input_file)[0]
        output_file = f"{base_name}_with_reasoning.json"
    elif not os.path.isabs(output_file):
        output_file = os.path.join(script_dir, output_file)
    
    # 打印配置
    logger.info("=" * 50)
    logger.info("问答对深度思考处理脚本")
    logger.info("=" * 50)
    logger.info(f"输入文件: {input_file}")
    logger.info(f"输出文件: {output_file}")
    logger.info(f"API地址: {CONFIG['base_url']}")
    logger.info(f"模型: {CONFIG['model']}")
    logger.info(f"请求间隔: {CONFIG['delay']}秒")
    logger.info("=" * 50)
    
    # 加载数据
    logger.info(f"正在加载文件...")
    data = load_json(input_file)
    logger.info(f"共加载 {len(data)} 条数据")
    
    # 断点续传：如果输出文件已存在，加载它
    if os.path.exists(output_file) and CONFIG["skip_processed"]:
        logger.info(f"发现已有输出文件，尝试断点续传")
        try:
            existing_data = load_json(output_file)
            if len(existing_data) == len(data):
                data = existing_data
                logger.info("成功加载已有数据，将跳过已处理的条目")
        except Exception as e:
            logger.warning(f"无法加载已有输出文件: {e}")
    
    # 初始化客户端
    client = ReasoningModelClient(
        api_key=api_key,
        base_url=CONFIG["base_url"],
        model=CONFIG["model"],
        max_tokens=CONFIG["max_tokens"],
        temperature=CONFIG["temperature"],
        timeout=CONFIG["timeout"]
    )
    
    # 确定处理范围
    start_idx = CONFIG["start_index"]
    end_idx = CONFIG["end_index"] if CONFIG["end_index"] is not None else len(data)
    end_idx = min(end_idx, len(data))
    
    logger.info(f"将处理索引 {start_idx} 到 {end_idx - 1} 的数据")
    
    # 统计
    processed_count = 0
    skipped_count = 0
    error_count = 0
    
    try:
        for idx in tqdm(range(start_idx, end_idx), desc="处理进度"):
            item = data[idx]
            
            # 检查是否已处理
            if CONFIG["skip_processed"] and item.get("reasoning_content"):
                skipped_count += 1
                continue
            
            # 获取问题内容
            instruction = item.get("instruction", "")
            input_text = item.get("input", "")
            
            if not input_text:
                logger.warning(f"索引 {idx} 的input为空，跳过")
                skipped_count += 1
                continue
            
            # 调用API
            result = client.get_response_with_reasoning(
                instruction=instruction,
                input_text=input_text,
                system_prompt=CONFIG["system_prompt"]
            )
            
            if result["success"]:
                item["reasoning_content"] = result["reasoning_content"]
                item["model_response"] = result["content"]
                item["model_used"] = result["model"]
                if result["usage"]:
                    item["token_usage"] = result["usage"]
                processed_count += 1
            else:
                item["processing_error"] = result["error"]
                error_count += 1
                logger.warning(f"索引 {idx} 处理失败: {result['error']}")
            
            # 定期保存
            if processed_count > 0 and processed_count % CONFIG["save_interval"] == 0:
                save_json(data, output_file)
                logger.info(f"已保存进度，处理了 {processed_count} 条")
            
            # 请求间隔
            if CONFIG["delay"] > 0:
                time.sleep(CONFIG["delay"])
                
    except KeyboardInterrupt:
        logger.info("检测到中断信号，正在保存当前进度...")
    finally:
        save_json(data, output_file)
        logger.info(f"\n处理完成！")
        logger.info(f"- 成功处理: {processed_count} 条")
        logger.info(f"- 跳过: {skipped_count} 条")
        logger.info(f"- 错误: {error_count} 条")
        logger.info(f"- 输出文件: {output_file}")


if __name__ == "__main__":
    main()
