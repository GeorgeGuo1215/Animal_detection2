from __future__ import annotations

import argparse
import os

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer


def main():
    parser = argparse.ArgumentParser(description="合并 LoRA 适配器到基座模型并导出（merge_and_unload）")
    parser.add_argument("--model_id", type=str, required=True, help="HF 模型名或本地路径")
    parser.add_argument("--adapter_dir", type=str, required=True, help="LoRA 训练输出目录（adapter）")
    parser.add_argument("--merged_out", type=str, required=True, help="合并后的模型输出目录")
    parser.add_argument("--trust_remote_code", action="store_true", help="需要时再开，默认关闭更安全")
    parser.add_argument(
        "--dtype",
        type=str,
        default="auto",
        choices=["auto", "fp16", "bf16", "fp32"],
        help="导出权重 dtype：auto/fp16/bf16/fp32（auto 会在有 CUDA 时优先 fp16）",
    )
    args = parser.parse_args()

    os.makedirs(args.merged_out, exist_ok=True)

    tokenizer = AutoTokenizer.from_pretrained(args.model_id, trust_remote_code=args.trust_remote_code, use_fast=True)
    if tokenizer.pad_token is None and tokenizer.eos_token is not None:
        tokenizer.pad_token = tokenizer.eos_token

    if args.dtype == "fp32":
        torch_dtype = torch.float32
    elif args.dtype == "bf16":
        torch_dtype = torch.bfloat16
    elif args.dtype == "fp16":
        torch_dtype = torch.float16
    else:
        torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

    model = AutoModelForCausalLM.from_pretrained(
        args.model_id,
        trust_remote_code=args.trust_remote_code,
        torch_dtype=torch_dtype,
        device_map="auto",
    )

    try:
        from peft import PeftModel
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"缺少 peft，请先安装：{e}") from e

    model = PeftModel.from_pretrained(model, args.adapter_dir)
    model.eval()

    # 关键：合并 LoRA 权重并卸载 PEFT 包装
    try:
        merged = model.merge_and_unload()
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(
            "合并失败：如果你训练时使用了 4bit 量化加载，有时需要在合并阶段用非量化方式重新加载基座模型。\n"
            f"原始错误: {e}"
        ) from e

    # 保存合并后的模型与 tokenizer
    merged.save_pretrained(args.merged_out, safe_serialization=True)
    tokenizer.save_pretrained(args.merged_out)

    with open(os.path.join(args.merged_out, "MERGED_FROM.txt"), "w", encoding="utf-8") as f:
        f.write(f"base_model={args.model_id}\n")
        f.write(f"adapter_dir={args.adapter_dir}\n")
        f.write(f"dtype={args.dtype}\n")


if __name__ == "__main__":
    main()


