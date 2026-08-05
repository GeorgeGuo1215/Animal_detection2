from __future__ import annotations

from typing import Any, Dict, List, Optional


def _join_instruction_input(instruction: str, input_text: str) -> str:
    instruction = (instruction or "").strip()
    input_text = (input_text or "").strip()
    if instruction and input_text:
        return f"{instruction}\n\n{input_text}"
    return instruction or input_text


def format_alpaca_to_text(
    example: Dict[str, Any],
    tokenizer: Any,
    *,
    system_prompt: Optional[str] = None,
    prefer_chat_template: bool = True,
    add_eos: bool = True,
) -> str:
    """
    将一条 Alpaca 样本（instruction/input/output）格式化成可训练的纯文本。
    - 若 tokenizer 存在 chat_template，则优先用 apply_chat_template（更贴合 Instruct 模型）。
    - 否则退回到 Alpaca 经典 prompt 模板。
    """
    instruction = (example.get("instruction") or "").strip()
    input_text = (example.get("input") or "").strip()
    output_text = (example.get("output") or "").strip()

    user_text = _join_instruction_input(instruction, input_text)

    use_chat = (
        bool(prefer_chat_template)
        and hasattr(tokenizer, "chat_template")
        and tokenizer.chat_template
        and hasattr(tokenizer, "apply_chat_template")
    )

    if use_chat:
        messages: List[Dict[str, str]] = []
        if system_prompt and system_prompt.strip():
            messages.append({"role": "system", "content": system_prompt.strip()})
        messages.append({"role": "user", "content": user_text})
        messages.append({"role": "assistant", "content": output_text})

        text = tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=False,
        )
    else:
        if input_text:
            text = (
                "### Instruction:\n"
                f"{instruction}\n\n"
                "### Input:\n"
                f"{input_text}\n\n"
                "### Response:\n"
                f"{output_text}\n"
            )
        else:
            text = (
                "### Instruction:\n"
                f"{instruction}\n\n"
                "### Response:\n"
                f"{output_text}\n"
            )

    if add_eos and getattr(tokenizer, "eos_token", None):
        eos = tokenizer.eos_token
        if eos and not text.endswith(eos):
            text += eos
    return text


def build_inference_prompt(
    tokenizer: Any,
    question: str,
    *,
    system_prompt: Optional[str] = None,
    prefer_chat_template: bool = True,
) -> str:
    question = (question or "").strip()
    use_chat = (
        bool(prefer_chat_template)
        and hasattr(tokenizer, "chat_template")
        and tokenizer.chat_template
        and hasattr(tokenizer, "apply_chat_template")
    )

    if use_chat:
        messages: List[Dict[str, str]] = []
        if system_prompt and system_prompt.strip():
            messages.append({"role": "system", "content": system_prompt.strip()})
        messages.append({"role": "user", "content": question})
        return tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )

    # 非 chat 模型：用 Alpaca prompt 进行推理
    return (
        "### Instruction:\n"
        f"{question}\n\n"
        "### Response:\n"
    )


