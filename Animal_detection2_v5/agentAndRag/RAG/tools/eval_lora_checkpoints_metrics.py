from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
from datasets import load_dataset
from transformers import AutoModelForCausalLM, AutoTokenizer

try:
    # 评估脚本默认输出“表格”，不希望夹杂一堆 transformers warning
    from transformers.utils import logging as hf_logging

    hf_logging.set_verbosity_error()
except Exception:
    pass


def _repo_root() -> Path:
    # .../RAG/tools/xxx.py -> repo root is parents[2]
    return Path(__file__).resolve().parents[2]


def _add_lora_to_syspath():
    # lora/ 不是 python 包（没 __init__.py），这里用 sys.path 注入以复用 prompting.py
    lora_dir = _repo_root() / "lora"
    sys.path.insert(0, str(lora_dir))


def _join_instruction_input(instruction: str, input_text: str) -> str:
    instruction = (instruction or "").strip()
    input_text = (input_text or "").strip()
    if instruction and input_text:
        return f"{instruction}\n\n{input_text}"
    return instruction or input_text


def _parse_run_args_txt(path: Path) -> Dict[str, str]:
    """
    解析 train_sft_lora.py 写入的 run_args.txt（key=value 形式）。
    注意：它中间会插入 [data_config] 段，这里也一并收集（同样是 key=value）。
    """
    out: Dict[str, str] = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("[") or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _natural_key_checkpoint(name: str) -> Tuple[int, str]:
    m = re.search(r"checkpoint-(\d+)$", name)
    if m:
        return (int(m.group(1)), name)
    return (10**18, name)


def _decode_generated_only(tokenizer, input_ids, output_ids) -> str:
    # 只解码新生成部分，避免 prompt 混入
    input_len = int(input_ids.shape[-1])
    gen_ids = output_ids[0][input_len:]
    text = tokenizer.decode(gen_ids, skip_special_tokens=True)
    t = text.strip()
    if t.lower().startswith("assistant"):
        t = t[len("assistant") :].strip()
    return t


@dataclass(frozen=True)
class MetricsRow:
    checkpoint: str
    rouge1: float
    rouge2: float
    rougeL: float
    bleu: float
    bertscore_f1: float


def main():
    parser = argparse.ArgumentParser(description="评估多个 LoRA checkpoint：ROUGE / BLEU / BERTScore（基于验证集）")
    parser.add_argument("--model_id", type=str, required=True, help="HF 模型名或本地路径")
    parser.add_argument(
        "--output_dir",
        type=str,
        required=True,
        help="训练输出根目录（包含 checkpoint-* 子目录与 run_args.txt）。例如 out/qwen2.5-7b-animals",
    )
    parser.add_argument(
        "--checkpoints",
        type=str,
        default="",
        help="要评估的 checkpoint 名称列表（逗号分隔，如 checkpoint-138,checkpoint-204）。留空则自动扫描 output_dir 下全部。",
    )
    parser.add_argument(
        "--data_path",
        type=str,
        default="",
        help="Alpaca JSONL 数据路径。留空则尝试从 output_dir/run_args.txt 读取；否则默认 RAG/data/qa_gen/animals_alpaca.jsonl",
    )
    parser.add_argument("--val_split", type=float, default=-1.0, help="验证集比例。-1 表示尝试从 run_args.txt 读取")
    parser.add_argument("--seed", type=int, default=-1, help="划分验证集的随机种子。-1 表示尝试从 run_args.txt 读取")
    parser.add_argument("--max_new_tokens", type=int, default=256)
    parser.add_argument("--trust_remote_code", action="store_true")
    parser.add_argument("--device_map", type=str, default="cuda", choices=["auto", "cuda", "cpu"])
    parser.add_argument("--temperature", type=float, default=0.0, help="默认 0（greedy）更利于可复现评估")
    parser.add_argument("--top_p", type=float, default=0.9)
    parser.add_argument("--system_prompt", type=str, default=None)
    parser.add_argument("--bertscore_lang", type=str, default="en", help="BERTScore 语言：en/zh 等")
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="打开 transformers 的日志（默认关闭，避免输出里夹杂各种 warning）。",
    )
    parser.add_argument(
        "--save_csv",
        type=str,
        default="",
        help="可选：保存 CSV 路径（相对 output_dir 或绝对路径）。默认保存到 output_dir/metrics.csv",
    )
    args = parser.parse_args()

    # 如需更详细的加载/生成日志，可打开 --verbose
    try:
        from transformers.utils import logging as hf_logging2

        hf_logging2.set_verbosity_info() if args.verbose else hf_logging2.set_verbosity_error()
    except Exception:
        pass

    # 依赖：evaluate/bertscore/rouge/sacrebleu
    try:
        import evaluate  # type: ignore
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(
            "缺少 evaluate 相关依赖。请在你的 (lora) 环境执行：pip install -r lora/requirements.txt"
        ) from e

    root = _repo_root()
    out_dir = Path(args.output_dir).expanduser()
    if not out_dir.is_absolute():
        out_dir = (root / out_dir).resolve()
    if not out_dir.exists():
        raise FileNotFoundError(f"output_dir 不存在：{out_dir}")

    run_args = _parse_run_args_txt(out_dir / "run_args.txt")

    data_path = (args.data_path or run_args.get("data_path") or "RAG/data/qa_gen/animals_alpaca.jsonl").strip()
    data_path_p = Path(data_path).expanduser()
    if not data_path_p.is_absolute():
        # 优先相对 repo 根
        data_path_p = (root / data_path_p).resolve()
    if not data_path_p.exists():
        raise FileNotFoundError(f"找不到数据集：{data_path_p}")

    if args.val_split >= 0:
        val_split = float(args.val_split)
    else:
        val_split = float(run_args.get("val_split", "0.1"))
    if args.seed >= 0:
        seed = int(args.seed)
    else:
        seed = int(run_args.get("seed", "42"))
    if not (0.0 < val_split < 1.0):
        raise ValueError(f"val_split 必须在 (0,1) 内，当前={val_split}")

    # 找 checkpoint 列表
    if args.checkpoints.strip():
        ckpts = [x.strip() for x in args.checkpoints.split(",") if x.strip()]
    else:
        ckpts = [p.name for p in out_dir.iterdir() if p.is_dir() and p.name.startswith("checkpoint-")]
        ckpts.sort(key=_natural_key_checkpoint)
    if not ckpts:
        raise RuntimeError(f"在 {out_dir} 下没找到任何 checkpoint-* 目录")

    # 复现验证集：与 train_sft_lora.py 一致的方式（datasets 的 train_test_split）
    ds_all = load_dataset("json", data_files=str(data_path_p), split="train")
    if len(ds_all) < 10:
        raise RuntimeError("数据量太小，无法可靠划分验证集")
    split = ds_all.train_test_split(test_size=val_split, seed=seed, shuffle=True)
    eval_ds = split["test"]

    # 复用 prompting.py 的 build_inference_prompt（更贴近实际 Instruct 模板）
    _add_lora_to_syspath()
    try:
        from prompting import build_inference_prompt  # type: ignore
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"无法导入 lora/prompting.py：{e}") from e

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
        from peft import PeftModel  # type: ignore
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"缺少 peft：{e}") from e

    # 指标加载
    rouge = evaluate.load("rouge")
    sacrebleu = evaluate.load("sacrebleu")
    bertscore = evaluate.load("bertscore")

    # 先加载第一个 adapter，再把其它 adapter 作为多适配器加载进来，避免重复加载基座
    first_ckpt = ckpts[0]
    first_dir = out_dir / first_ckpt
    if not first_dir.exists():
        raise FileNotFoundError(f"checkpoint 不存在：{first_dir}")

    peft_model = PeftModel.from_pretrained(model, str(first_dir), adapter_name=first_ckpt)
    for ckpt in ckpts[1:]:
        p = out_dir / ckpt
        if not p.exists():
            raise FileNotFoundError(f"checkpoint 不存在：{p}")
        peft_model.load_adapter(str(p), adapter_name=ckpt)
    peft_model.eval()

    # 准备 eval prompts 与 references
    prompts: List[str] = []
    refs: List[str] = []
    for ex in eval_ds:
        q = _join_instruction_input(ex.get("instruction", ""), ex.get("input", ""))
        prompts.append(q)
        refs.append((ex.get("output") or "").strip())

    rows: List[MetricsRow] = []
    for ckpt in ckpts:
        peft_model.set_adapter(ckpt)
        preds: List[str] = []

        for q in prompts:
            prompt = build_inference_prompt(
                tokenizer,
                q,
                system_prompt=args.system_prompt,
                prefer_chat_template=True,
            )
            inputs = tokenizer(prompt, return_tensors="pt")
            inputs = {k: v.to(peft_model.device) for k, v in inputs.items()}

            with torch.no_grad():
                gen_kwargs = dict(
                    max_new_tokens=args.max_new_tokens,
                    eos_token_id=tokenizer.eos_token_id,
                    pad_token_id=tokenizer.pad_token_id,
                )
                if args.temperature and args.temperature > 0:
                    gen_kwargs.update(
                        dict(
                            do_sample=True,
                            temperature=args.temperature,
                            top_p=args.top_p,
                        )
                    )
                else:
                    gen_kwargs.update(dict(do_sample=False))

                out_ids = peft_model.generate(**inputs, **gen_kwargs)
            preds.append(_decode_generated_only(tokenizer, inputs["input_ids"], out_ids))

        # 计算指标（全部 corpus 级）
        rouge_res = rouge.compute(predictions=preds, references=refs)
        # evaluate/sacrebleu 期望 references 与 predictions 等长，且每条 prediction 对应一个 reference 列表
        # 也就是：references = [[ref1], [ref2], ...]
        bleu_res = sacrebleu.compute(predictions=preds, references=[[r] for r in refs])
        bs_res = bertscore.compute(predictions=preds, references=refs, lang=args.bertscore_lang)
        bs_f1 = float(sum(bs_res["f1"]) / len(bs_res["f1"])) if bs_res.get("f1") else 0.0

        rows.append(
            MetricsRow(
                checkpoint=ckpt,
                rouge1=float(rouge_res.get("rouge1", 0.0)),
                rouge2=float(rouge_res.get("rouge2", 0.0)),
                rougeL=float(rouge_res.get("rougeL", 0.0)),
                bleu=float(bleu_res.get("score", 0.0)),
                bertscore_f1=bs_f1,
            )
        )

    # 输出表格到终端
    headers = ["checkpoint", "rouge1", "rouge2", "rougeL", "bleu", "bertscore_f1"]
    print("\n=== Checkpoint Metrics (eval set) ===")
    print(f"data_path: {data_path_p}")
    print(f"val_split: {val_split} | seed: {seed} | eval_size: {len(eval_ds)}")
    print(" | ".join(headers))
    for r in rows:
        print(
            f"{r.checkpoint} | {r.rouge1:.4f} | {r.rouge2:.4f} | {r.rougeL:.4f} | {r.bleu:.2f} | {r.bertscore_f1:.4f}"
        )

    # 保存 CSV
    csv_path = Path(args.save_csv.strip()) if args.save_csv.strip() else (out_dir / "metrics.csv")
    if not csv_path.is_absolute():
        csv_path = (out_dir / csv_path).resolve()
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        for r in rows:
            w.writerow(
                dict(
                    checkpoint=r.checkpoint,
                    rouge1=f"{r.rouge1:.6f}",
                    rouge2=f"{r.rouge2:.6f}",
                    rougeL=f"{r.rougeL:.6f}",
                    bleu=f"{r.bleu:.6f}",
                    bertscore_f1=f"{r.bertscore_f1:.6f}",
                )
            )
    print(f"\nSaved: {csv_path}")


if __name__ == "__main__":
    main()


