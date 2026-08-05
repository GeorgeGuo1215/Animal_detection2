from __future__ import annotations

import argparse
import inspect
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional

import torch
from datasets import load_dataset
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    TrainingArguments,
)

from peft import LoraConfig, TaskType

from prompting import format_alpaca_to_text


def _default_target_modules(model_type: str) -> List[str]:
    # 兼容大多数 decoder-only 架构（llama/mistral/qwen/gemma 等）
    common = ["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"]
    if model_type in {"llama", "mistral", "qwen2", "qwen2_moe", "gemma", "gemma2"}:
        return common
    return common


def _maybe_dtype(prefer_bf16: bool) -> torch.dtype:
    if prefer_bf16 and torch.cuda.is_available():
        # 有些平台不支持 bf16，这里做个保守判断
        if hasattr(torch.cuda, "is_bf16_supported") and torch.cuda.is_bf16_supported():
            return torch.bfloat16
    return torch.float16


def _try_build_4bit_config(enable_4bit: bool):
    if not enable_4bit:
        return None
    try:
        from transformers import BitsAndBytesConfig
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(
            "你启用了 --load_in_4bit，但当前环境缺少 BitsAndBytesConfig / bitsandbytes。\n"
            "Windows 环境多数情况下不建议 4bit；你可以先去掉 --load_in_4bit 直接训练。\n"
            f"原始错误: {e}"
        ) from e

    return BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.float16,
        bnb_4bit_use_double_quant=True,
    )


@dataclass
class DataConfig:
    data_path: str
    val_split: float = 0.0
    max_samples: int = 0
    system_prompt: Optional[str] = None
    prefer_chat_template: bool = True


def main():
    parser = argparse.ArgumentParser(description="LoRA SFT 微调（支持 Alpaca JSONL）")

    # 模型
    parser.add_argument("--model_id", type=str, required=True, help="HF 模型名或本地路径")
    parser.add_argument("--trust_remote_code", action="store_true", help="需要时再开，默认关闭更安全")
    parser.add_argument("--load_in_4bit", action="store_true", help="可选：4bit 量化加载（更省显存）")
    parser.add_argument(
        "--allow_cpu",
        action="store_true",
        help="允许在 CPU 上训练（强烈不建议用于 3B/7B；仅用于小模型/冒烟测试）",
    )

    # 数据
    parser.add_argument("--data_path", type=str, required=True, help="jsonl 路径（Alpaca 格式）")
    parser.add_argument("--val_split", type=float, default=0.0, help="划分验证集比例，0 表示不划分")
    parser.add_argument("--max_samples", type=int, default=0, help="仅用于快速调试，0 表示全量")
    parser.add_argument("--system_prompt", type=str, default=None, help="可选系统提示词（用于 chat_template）")

    # 训练超参
    parser.add_argument("--output_dir", type=str, required=True)
    parser.add_argument("--num_train_epochs", type=float, default=3.0)
    parser.add_argument("--max_seq_len", type=int, default=1024)
    parser.add_argument("--learning_rate", type=float, default=2e-4)
    parser.add_argument("--per_device_train_batch_size", type=int, default=1)
    parser.add_argument("--gradient_accumulation_steps", type=int, default=8)
    parser.add_argument(
        "--gradient_checkpointing",
        action="store_true",
        help="开启梯度检查点以节省显存（更适合 16GB 跑 7B；会变慢一些）",
    )
    parser.add_argument("--warmup_ratio", type=float, default=0.03)
    parser.add_argument("--logging_steps", type=int, default=10)
    parser.add_argument("--save_steps", type=int, default=100)
    parser.add_argument("--save_total_limit", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume_from_checkpoint", type=str, default=None)

    # LoRA
    parser.add_argument("--lora_r", type=int, default=16)
    parser.add_argument("--lora_alpha", type=int, default=32)
    parser.add_argument("--lora_dropout", type=float, default=0.05)
    parser.add_argument("--target_modules", type=str, default="", help="逗号分隔；留空则自动给常用值")

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    if not torch.cuda.is_available() and not args.allow_cpu:
        raise RuntimeError(
            "检测到当前环境没有可用 CUDA GPU（torch.cuda.is_available() = False）。\n"
            "3B/7B 级别模型在 CPU 上训练通常不可行（极慢/易内存不足）。\n"
            "请改用：\n"
            "1) 安装 CUDA 版 PyTorch 并在有 NVIDIA GPU 的环境运行；或\n"
            "2) 使用 WSL2 / Linux + CUDA；或\n"
            "3) 先用更小模型（如 0.5B）做流程验证。\n"
            "如果你确实要在 CPU 上尝试，请显式加 --allow_cpu。"
        )

    dtype = _maybe_dtype(prefer_bf16=True)
    quant_config = _try_build_4bit_config(args.load_in_4bit)

    tokenizer = AutoTokenizer.from_pretrained(args.model_id, trust_remote_code=args.trust_remote_code, use_fast=True)
    if tokenizer.pad_token is None and tokenizer.eos_token is not None:
        tokenizer.pad_token = tokenizer.eos_token

    model = AutoModelForCausalLM.from_pretrained(
        args.model_id,
        trust_remote_code=args.trust_remote_code,
        torch_dtype=dtype,
        device_map="auto",
        quantization_config=quant_config,
    )

    if args.gradient_checkpointing:
        # checkpointing 需要关闭 cache，否则可能报错/显存不降反升
        if hasattr(model, "config") and hasattr(model.config, "use_cache"):
            model.config.use_cache = False
        if hasattr(model, "gradient_checkpointing_enable"):
            model.gradient_checkpointing_enable()

    if args.load_in_4bit:
        try:
            from peft import prepare_model_for_kbit_training
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"启用 4bit 需要 peft.prepare_model_for_kbit_training，当前环境不支持：{e}") from e
        model = prepare_model_for_kbit_training(model)

    model_type = getattr(getattr(model, "config", None), "model_type", "") or ""
    if args.target_modules.strip():
        target_modules = [x.strip() for x in args.target_modules.split(",") if x.strip()]
    else:
        target_modules = _default_target_modules(model_type)

    peft_config = LoraConfig(
        r=args.lora_r,
        lora_alpha=args.lora_alpha,
        lora_dropout=args.lora_dropout,
        bias="none",
        task_type=TaskType.CAUSAL_LM,
        target_modules=target_modules,
    )

    # 数据加载与格式化
    data_cfg = DataConfig(
        data_path=args.data_path,
        val_split=args.val_split,
        max_samples=args.max_samples,
        system_prompt=args.system_prompt,
        prefer_chat_template=True,
    )

    # 兼容新手常见情况：从 lora/ 目录运行导致相对路径找不到
    # - 用户通常会写 data_path=RAG/data/...
    # - 但如果 cwd 在 lora/ 下，会变成 lora/RAG/... 从而报错
    data_path = Path(str(data_cfg.data_path)).expanduser()
    if not data_path.is_absolute():
        # 先按当前工作目录解析
        candidate = (Path.cwd() / data_path).resolve()
        if candidate.exists():
            data_path = candidate
        else:
            # 再按仓库根目录解析：lora/ 的上一级
            repo_root = (Path(__file__).resolve().parent / "..").resolve()
            candidate2 = (repo_root / data_path).resolve()
            if candidate2.exists():
                data_path = candidate2

    if not data_path.exists():
        raise FileNotFoundError(
            f"找不到数据集文件：{data_cfg.data_path}\n"
            f"- 当前工作目录: {Path.cwd()}\n"
            f"- 已尝试解析路径: {data_path}\n"
            "建议：在仓库根目录运行，或直接传入数据集的绝对路径。"
        )

    data_cfg.data_path = str(data_path)
    ds = load_dataset("json", data_files=data_cfg.data_path, split="train")
    if data_cfg.max_samples and data_cfg.max_samples > 0:
        ds = ds.select(range(min(len(ds), data_cfg.max_samples)))

    def _map_to_text(ex):
        return {
            "text": format_alpaca_to_text(
                ex,
                tokenizer,
                system_prompt=data_cfg.system_prompt,
                prefer_chat_template=data_cfg.prefer_chat_template,
                add_eos=True,
            )
        }

    ds = ds.map(_map_to_text, remove_columns=ds.column_names)

    eval_dataset = None
    train_dataset = ds
    if data_cfg.val_split and 0.0 < data_cfg.val_split < 1.0 and len(ds) >= 10:
        split = ds.train_test_split(test_size=data_cfg.val_split, seed=args.seed, shuffle=True)
        train_dataset = split["train"]
        eval_dataset = split["test"]

    # 训练参数
    use_bf16 = dtype == torch.bfloat16
    use_fp16 = dtype == torch.float16

    # transformers 不同版本的 TrainingArguments 字段可能有改名/增删（例如 evaluation_strategy）
    # 这里用 signature 过滤，只传当前版本支持的参数，避免 TypeError
    allowed = set(inspect.signature(TrainingArguments.__init__).parameters.keys())
    training_args_kwargs = {
        "output_dir": args.output_dir,
        "num_train_epochs": args.num_train_epochs,
        "per_device_train_batch_size": args.per_device_train_batch_size,
        "gradient_accumulation_steps": args.gradient_accumulation_steps,
        "gradient_checkpointing": bool(args.gradient_checkpointing),
        "learning_rate": args.learning_rate,
        "warmup_ratio": args.warmup_ratio,
        "logging_steps": args.logging_steps,
        "save_steps": args.save_steps,
        "save_total_limit": args.save_total_limit,
        "bf16": use_bf16,
        "fp16": use_fp16 and not use_bf16,
        "optim": "paged_adamw_8bit" if args.load_in_4bit else "adamw_torch",
        "lr_scheduler_type": "cosine",
        "report_to": "none",
        "seed": args.seed,
        "eval_steps": max(args.save_steps, 50) if eval_dataset is not None else None,
    }

    # evaluation_strategy 在某些版本可能改名为 eval_strategy
    if eval_dataset is not None:
        if "evaluation_strategy" in allowed:
            training_args_kwargs["evaluation_strategy"] = "steps"
        elif "eval_strategy" in allowed:
            training_args_kwargs["eval_strategy"] = "steps"
    else:
        if "evaluation_strategy" in allowed:
            training_args_kwargs["evaluation_strategy"] = "no"
        elif "eval_strategy" in allowed:
            training_args_kwargs["eval_strategy"] = "no"

    # 过滤掉当前版本不支持的键，以及 None 值（例如无 eval 时 eval_steps）
    filtered_kwargs = {k: v for k, v in training_args_kwargs.items() if k in allowed and v is not None}
    training_args = TrainingArguments(**filtered_kwargs)

    # TRL SFTTrainer（更适合指令微调）
    try:
        from trl import SFTTrainer
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"缺少 trl（SFTTrainer），请先 pip install -r lora/requirements.txt：{e}") from e

    # TRL 的 SFTTrainer 在不同版本里参数名/结构差异很大：
    # - 有的版本用 tokenizer=... + dataset_text_field=... + max_seq_length=...
    # - 有的版本改成 processing_class=...，甚至把 dataset_text_field 下沉到 data collator / config
    # 这里同样按 signature 做兼容，避免 TypeError
    sft_allowed = set(inspect.signature(SFTTrainer.__init__).parameters.keys())
    sft_kwargs = {
        "model": model,
        "args": training_args,
        "train_dataset": train_dataset,
        "eval_dataset": eval_dataset,
        "peft_config": peft_config,
        "packing": False,
    }

    # tokenizer / processing_class 二选一
    if "tokenizer" in sft_allowed:
        sft_kwargs["tokenizer"] = tokenizer
    elif "processing_class" in sft_allowed:
        # 新版 TRL 使用 processing_class（通常也能接收 tokenizer）
        sft_kwargs["processing_class"] = tokenizer

    # 文本字段名：我们上面 map 后只保留了 "text" 一列
    if "dataset_text_field" in sft_allowed:
        sft_kwargs["dataset_text_field"] = "text"

    # 序列长度字段名：max_seq_length / max_length
    if "max_seq_length" in sft_allowed:
        sft_kwargs["max_seq_length"] = args.max_seq_len
    elif "max_length" in sft_allowed:
        sft_kwargs["max_length"] = args.max_seq_len

    trainer = SFTTrainer(**sft_kwargs)

    trainer.train(resume_from_checkpoint=args.resume_from_checkpoint)

    # 保存 LoRA 适配器与 tokenizer
    trainer.model.save_pretrained(args.output_dir)
    tokenizer.save_pretrained(args.output_dir)

    # 记录一份训练配置，方便复现
    with open(os.path.join(args.output_dir, "run_args.txt"), "w", encoding="utf-8") as f:
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}={v}\n")
        f.write("\n[data_config]\n")
        for k, v in asdict(data_cfg).items():
            f.write(f"{k}={v}\n")
        f.write(f"\nmodel_type={model_type}\n")
        f.write(f"target_modules={target_modules}\n")


if __name__ == "__main__":
    main()


