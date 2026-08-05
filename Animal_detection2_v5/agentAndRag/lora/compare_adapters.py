from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from difflib import SequenceMatcher
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from prompting import build_inference_prompt

try:
    # 让“汇报输出”更干净：默认把 transformers 的日志降到 error
    from transformers.utils import logging as hf_logging

    hf_logging.set_verbosity_error()
except Exception:
    pass


@dataclass(frozen=True)
class EvalItem:
    title: str
    question: str
    # 如果能在 jsonl 里找到同 instruction 的 output，就作为参考答案展示
    match_instruction: Optional[str] = None


def _ratio(a: str, b: str) -> float:
    a = (a or "").strip()
    b = (b or "").strip()
    if not a or not b:
        return 0.0
    return SequenceMatcher(None, a, b).ratio()


def _load_gold_from_alpaca_jsonl(jsonl_path: Path) -> Dict[str, str]:
    """构建 instruction -> output 的查表，用于展示“参考答案”（不是严格测评）。"""
    gold: Dict[str, str] = {}
    if not jsonl_path.exists():
        return gold
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue
            ins = (obj.get("instruction") or "").strip()
            out = (obj.get("output") or "").strip()
            if ins and out and ins not in gold:
                gold[ins] = out
    return gold


def _print_block(title: str, text: str):
    print(f"\n[{title}]")
    print((text or "").strip())


def _clean_answer(text: str) -> str:
    """清理常见前缀，保证汇报输出更干净。"""
    t = (text or "").strip()
    # 一些模板会把 'assistant' 也当作普通文本出现在生成里
    if t.lower().startswith("assistant"):
        t = t[len("assistant") :].strip()
    return t


def _default_items() -> List[EvalItem]:
    # 问题/问法写死：适合汇报对比
    return [
        EvalItem(
            title="BRDC 基础问答（训练集中有同题，适合看“贴合度 vs 泛化”）",
            question="What are the common clinical signs and pathogenesis of bovine respiratory disease complex (BRDC)?",
            match_instruction="What are the common clinical signs and pathogenesis of bovine respiratory disease complex (BRDC)?",
        ),
        EvalItem(
            title="奶牛发情周期（训练集中有同题，结构化回答）",
            question="Explain the estrous cycle in dairy cattle and its importance in herd management.",
            match_instruction="Explain the estrous cycle in dairy cattle and its importance in herd management.",
        ),
        EvalItem(
            title="乳房炎诊断（训练集中有同题，适合看工具清单是否齐）",
            question="How is mastitis diagnosed in a dairy herd, and what diagnostic tools are used?",
            match_instruction="How is mastitis diagnosed in a dairy herd, and what diagnostic tools are used?",
        ),
        EvalItem(
            title="中文动物知识（训练集风格迁移到中文）",
            question="羊驼属于哪一类动物？简单介绍一下它的特征。",
            match_instruction=None,
        ),
        EvalItem(
            title="BRDC 场景题（训练集未必有同问法，适合看泛化能力）",
            question=(
                "A group of 5-month-old feedlot calves develops coughing, fever, and reduced appetite after transport. "
                "How would you triage, what diagnostics would you prioritize, and what immediate management steps would you take?"
            ),
            match_instruction=None,
        ),
    ]


def main():
    parser = argparse.ArgumentParser(description="汇报用：对比多个 LoRA checkpoint 的回答差异（问题写死）")
    parser.add_argument("--model_id", type=str, required=True, help="HF 模型名或本地路径")
    parser.add_argument(
        "--adapter_dirs",
        type=str,
        required=True,
        help="多个 adapter/checkpoint 目录，用逗号分隔。例如: path\\checkpoint-138,path\\checkpoint-204",
    )
    parser.add_argument(
        "--adapter_names",
        type=str,
        default="",
        help="可选：给每个 adapter 起个短名字（逗号分隔，数量需与 adapter_dirs 相同）",
    )
    parser.add_argument(
        "--alpaca_jsonl",
        type=str,
        default="RAG/data/qa_gen/animals_alpaca.jsonl",
        help="可选：用于展示参考答案的 alpaca jsonl 路径（相对仓库根或绝对路径）",
    )
    parser.add_argument("--max_new_tokens", type=int, default=256)
    parser.add_argument("--temperature", type=float, default=0.0, help="默认 0：更利于对比（可复现）")
    parser.add_argument("--top_p", type=float, default=0.9)
    parser.add_argument("--system_prompt", type=str, default=None)
    parser.add_argument("--trust_remote_code", action="store_true")
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="打开 transformers 的日志（默认关闭，避免输出里夹杂各种 warning）。",
    )
    parser.add_argument(
        "--device_map",
        type=str,
        default="cuda",
        choices=["auto", "cuda", "cpu"],
        help="建议汇报对比用 cuda，避免 offload 造成速度/输出波动。",
    )
    args = parser.parse_args()

    # 如需更详细的加载/生成日志，可打开 --verbose
    try:
        from transformers.utils import logging as hf_logging2

        hf_logging2.set_verbosity_info() if args.verbose else hf_logging2.set_verbosity_error()
    except Exception:
        pass

    # 解析路径（相对仓库根）
    repo_root = (Path(__file__).resolve().parent / "..").resolve()
    adapter_dirs = [Path(p.strip().strip('"')).expanduser() for p in args.adapter_dirs.split(",") if p.strip()]
    adapter_dirs = [(p if p.is_absolute() else (repo_root / p).resolve()) for p in adapter_dirs]
    for p in adapter_dirs:
        if not p.exists():
            raise FileNotFoundError(f"adapter_dir 不存在：{p}")

    if args.adapter_names.strip():
        adapter_names = [x.strip() for x in args.adapter_names.split(",")]
        if len(adapter_names) != len(adapter_dirs):
            raise ValueError("--adapter_names 数量必须与 --adapter_dirs 相同")
    else:
        adapter_names = [p.name for p in adapter_dirs]

    alpaca_jsonl = Path(args.alpaca_jsonl).expanduser()
    if not alpaca_jsonl.is_absolute():
        alpaca_jsonl = (repo_root / alpaca_jsonl).resolve()
    gold_map = _load_gold_from_alpaca_jsonl(alpaca_jsonl)

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

    # 用第一个 adapter 初始化，然后把其它 adapter 作为“多适配器”加载进来，便于切换对比
    peft_model = PeftModel.from_pretrained(model, str(adapter_dirs[0]), adapter_name=adapter_names[0])
    for name, p in zip(adapter_names[1:], adapter_dirs[1:]):
        peft_model.load_adapter(str(p), adapter_name=name)
    peft_model.eval()

    items = _default_items()
    print("=== LoRA 对比报告（自动生成）===")
    print(f"base_model: {args.model_id}")
    print(f"device_map: {args.device_map}")
    print(f"adapters: {', '.join(adapter_names)}")
    print(f"gold_jsonl: {alpaca_jsonl if alpaca_jsonl.exists() else '(not found)'}")

    # 统计每个 adapter 与 gold 的平均相似度（只对能匹配到 gold 的题）
    scored: Dict[str, List[float]] = {name: [] for name in adapter_names}

    for idx, it in enumerate(items, start=1):
        print("\n" + "=" * 88)
        print(f"Q{idx}. {it.title}")
        _print_block("Question", it.question)

        gold = None
        if it.match_instruction and it.match_instruction in gold_map:
            gold = gold_map[it.match_instruction]
            _print_block("Gold (from jsonl)", gold)

        for name in adapter_names:
            peft_model.set_adapter(name)
            prompt = build_inference_prompt(
                tokenizer,
                it.question,
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
                # 默认用 greedy（temperature=0），更利于可复现对比
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

                out = peft_model.generate(**inputs, **gen_kwargs)

            # 只解码“新生成部分”，避免 prompt/Question 混入回答
            input_len = int(inputs["input_ids"].shape[-1])
            gen_ids = out[0][input_len:]
            answer = tokenizer.decode(gen_ids, skip_special_tokens=True)
            answer = _clean_answer(answer)

            _print_block(f"Adapter: {name}", answer)

            if gold:
                scored[name].append(_ratio(answer, gold))

    # 总结：只对有 gold 的题做一个“参考性”分数（不是严格评测）
    if any(scored[name] for name in adapter_names):
        print("\n" + "=" * 88)
        
        pairs: List[Tuple[str, float, int]] = []
        for name in adapter_names:
            vals = scored[name]
            if vals:
                pairs.append((name, sum(vals) / len(vals), len(vals)))
        pairs.sort(key=lambda x: x[1], reverse=True)
        for name, avg, n in pairs:
            print(f"- {name}: avg_similarity={avg:.4f} (n={n})")
       


if __name__ == "__main__":
    main()


