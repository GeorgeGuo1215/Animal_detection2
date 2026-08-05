from __future__ import annotations

import argparse

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from prompting import build_inference_prompt


def main():
    parser = argparse.ArgumentParser(description="加载基座模型 + LoRA 适配器进行推理")
    parser.add_argument("--model_id", type=str, required=True, help="HF 模型名或本地路径")
    parser.add_argument("--adapter_dir", type=str, required=True, help="训练输出目录（LoRA adapter）")
    parser.add_argument("--question", type=str, required=True)
    parser.add_argument("--system_prompt", type=str, default=None)
    parser.add_argument("--max_new_tokens", type=int, default=256)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--top_p", type=float, default=0.9)
    parser.add_argument("--trust_remote_code", action="store_true")
    parser.add_argument(
        "--device_map",
        type=str,
        default="auto",
        choices=["auto", "cuda", "cpu"],
        help=(
            "模型加载设备策略：auto 可能会把权重 offload 到 CPU（推理能跑但更慢）；"
            "cuda 强制整模上 GPU；cpu 仅调试。"
        ),
    )
    args = parser.parse_args()

    tokenizer = AutoTokenizer.from_pretrained(args.model_id, trust_remote_code=args.trust_remote_code, use_fast=True)
    if tokenizer.pad_token is None and tokenizer.eos_token is not None:
        tokenizer.pad_token = tokenizer.eos_token

    dtype = torch.float16 if torch.cuda.is_available() else torch.float32
    device_map = "auto" if args.device_map == "auto" else None
    model = AutoModelForCausalLM.from_pretrained(
        args.model_id,
        trust_remote_code=args.trust_remote_code,
        torch_dtype=dtype,
        device_map=device_map,
    )

    if args.device_map == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("你指定了 --device_map cuda，但当前环境没有可用 CUDA。")
        model = model.to("cuda")
    elif args.device_map == "cpu":
        model = model.to("cpu")

    try:
        from peft import PeftModel
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"缺少 peft，请先安装：{e}") from e

    model = PeftModel.from_pretrained(model, args.adapter_dir)
    model.eval()

    prompt = build_inference_prompt(
        tokenizer,
        args.question,
        system_prompt=args.system_prompt,
        prefer_chat_template=True,
    )

    inputs = tokenizer(prompt, return_tensors="pt")
    inputs = {k: v.to(model.device) for k, v in inputs.items()}

    with torch.no_grad():
        out = model.generate(
            **inputs,
            max_new_tokens=args.max_new_tokens,
            do_sample=args.temperature > 0,
            temperature=args.temperature,
            top_p=args.top_p,
            eos_token_id=tokenizer.eos_token_id,
            pad_token_id=tokenizer.pad_token_id,
        )

    text = tokenizer.decode(out[0], skip_special_tokens=True)
    # 为了更直观，尽量只打印“回答”部分（不同模板可能不完全一致，这里做个弱匹配）
    if args.question in text:
        print(text.split(args.question, 1)[-1].strip())
    else:
        print(text.strip())


if __name__ == "__main__":
    main()


