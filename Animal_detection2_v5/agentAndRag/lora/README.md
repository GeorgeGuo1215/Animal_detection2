## LoRA 本地微调（基于 Alpaca JSONL）

你这份数据 `RAG/data/qa_gen/animals_alpaca.jsonl` 是标准的 Alpaca 三字段：`instruction` / `input` / `output`，可直接用于 SFT（监督微调）+ LoRA。

### 推荐“小模型”（3B/7B 级别）

下面这些都属于市面常用、体量不大的开源模型（是否可下载取决于你本机网络与 HF 镜像/缓存）：

- **3B**：
  - `Qwen/Qwen2.5-3B-Instruct`（通用、中文能力强，推荐优先尝试）
  - `meta-llama/Llama-3.2-3B-Instruct`（通常需要接受协议/权限）
- **7B**：
  - `Qwen/Qwen2.5-7B-Instruct`
  - `mistralai/Mistral-7B-Instruct-v0.3`
 - **CPU 冒烟测试（更小）**：
  - `Qwen/Qwen2.5-0.5B-Instruct`（用来先把流程跑通）

> 提醒：部分模型（如 Llama/Gemma）可能需要 HF 账户授权；Qwen 通常更省心。

### Windows 说明（重要）

- **3B/7B 微调强烈建议 GPU**：如果你当前环境 `torch` 是 CPU 版（`cuda False`），脚本会默认拒绝开始训练，避免你误点后卡很久。
- **不建议在 Windows 原生环境折腾 4bit**：本项目 `requirements.txt` 已对 Windows 关闭 `bitsandbytes`，因此 `--load_in_4bit` 在 Windows 上大概率不可用（这不是你的问题）。
- **想要 4bit/更省显存**：更推荐用 **WSL2 + CUDA** 或 Linux 环境跑；Windows 下先用非 4bit 跑通流程再说。
- **显存粗略参考**（仅经验值，和序列长度/批大小/优化器有关）：
  - 3B：通常 8~12GB 起步更舒服（LoRA + 小 batch）
  - 7B：通常 16GB+ 更稳；显存紧就降低 `--max_seq_len`、减 batch、增累积

---

## 1. 环境安装

### 1.0 先做一次环境自检（强烈建议）

在项目根（`DeepSeek-OCR/`）执行：

```bash
python lora/check_env.py
```

Windows 也可以直接双击：

- `lora/run_check_env.cmd`

如果你是 3080Ti，但这里输出 `cuda available: False`，说明你**还没切到装了 CUDA 版 torch 的虚拟环境**（或装错了 torch 版本）。

在 `DeepSeek-OCR/lora/` 目录下：

```bash
pip install -r requirements.txt
```

> Windows + NVIDIA：建议装好与你 CUDA 匹配的 PyTorch；若不做 4bit，可不装/不启用 bitsandbytes。

---

## 2. 训练（LoRA SFT）

### 2.1 最简单：直接跑 Qwen2.5-3B

在项目根（`DeepSeek-OCR/`）执行：

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-3b-animals ^
  --num_train_epochs 3 ^
  --max_seq_len 1024
```

### 2.1.1 针对 3080Ti Laptop 16GB 的推荐（更稳）

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-3b-animals ^
  --num_train_epochs 3 ^
  --max_seq_len 1024 ^
  --per_device_train_batch_size 1 ^
  --gradient_accumulation_steps 8
```

> 如果显存不够：把 `--max_seq_len` 降到 768/512；或把累积步数调大（训练更慢但更省显存）。

### 2.3（建议）切一份验证集，训练过程中自动输出 eval_loss

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-3b-animals ^
  --val_split 0.1 ^
  --eval_strategy epoch ^
  --num_train_epochs 3 ^
  --max_seq_len 1024
```

> `--eval_strategy epoch` 表示每个 epoch 结束都会跑一次验证并在日志里输出 `eval_loss`（更适合判断是否过拟合）。

### 2.4 每隔 0.1 个 epoch 保存一次 checkpoint，并顺便做一次 eval（你要的）

> 注意：默认 `--save_total_limit 2` 会只保留最后 2 个 checkpoint。你如果想都留着，请调大（例如 50）。

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-7B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-7b-animals ^
  --num_train_epochs 3 ^
  --max_seq_len 512 ^
  --per_device_train_batch_size 1 ^
  --gradient_accumulation_steps 16 ^
  --device_map cuda ^
  --val_split 0.1 ^
  --checkpoint_every_epoch_frac 0.1 ^
  --save_total_limit 50
```

### 2.1（Windows 一键）双击脚本直接跑

- `lora/run_train_qwen2.5_3b_animals.cmd`

### 2.2 显存更省：4bit 量化加载（可选）

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-3b-animals-4bit ^
  --load_in_4bit ^
  --num_train_epochs 3 ^
  --max_seq_len 1024
```

---

## 3. 推理验证（加载 LoRA 适配器）

```bash
python lora/infer.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --adapter_dir lora/out/qwen2.5-3b-animals ^
  --question "羊驼属于哪一类动物？简单介绍一下它的特征。"
```

> 如果你用 7B，建议加 `--device_map cuda`，避免推理时 offload 到 CPU 导致变慢/不稳定。

### 3.1（Windows 一键）双击脚本直接跑

- `lora/run_infer_qwen2.5_3b_animals.cmd`

### 3.2（汇报用）对比多个 checkpoint 的回答差异（问题写死，一键生成对比报告）

```bash
python lora/compare_adapters.py ^
  --model_id Qwen/Qwen2.5-7B-Instruct ^
  --adapter_dirs "F:/.../checkpoint-138,F:/.../checkpoint-204" ^
  --adapter_names "best,final" ^
  --device_map cuda
```

---

## 4.（可选）合并 LoRA 到基座模型并导出

```bash
python lora/merge_lora.py ^
  --model_id Qwen/Qwen2.5-3B-Instruct ^
  --adapter_dir lora/out/qwen2.5-3b-animals ^
  --merged_out lora/out/qwen2.5-3b-animals-merged
```

---

## 6. 7B 在 16GB 上的现实建议

- 在 **Windows 原生环境** 又 **不走 4bit** 的情况下，7B fp16 往往会非常吃紧（甚至直接 OOM）。
- `device_map="auto"` 可能会把部分权重 offload 到 CPU，并出现 **meta device 参数**；这在训练反传时容易直接报错。
- 如果你一定要试 7B：优先把 `--max_seq_len` 降到 512，并打开 `--gradient_checkpointing`（省显存但更慢），并建议显式 `--device_map cuda` 强制整模上 GPU（不够就尽早 OOM，方便判断路线）。

```bash
python lora/train_sft_lora.py ^
  --model_id Qwen/Qwen2.5-7B-Instruct ^
  --data_path RAG/data/qa_gen/animals_alpaca.jsonl ^
  --output_dir lora/out/qwen2.5-7b-animals ^
  --num_train_epochs 3 ^
  --max_seq_len 512 ^
  --per_device_train_batch_size 1 ^
  --gradient_accumulation_steps 16 ^
  --gradient_checkpointing ^
  --device_map cuda
```

## 5. 你可以按需调的关键参数

- **`--max_seq_len`**：建议 512~2048；你的问答偏知识型，1024 比较稳
- **`--per_device_train_batch_size`** / **`--gradient_accumulation_steps`**：显存紧就减 batch、增累积
- **`--learning_rate`**：常用 1e-4 ~ 2e-4；数据量 1200 条不大，别太大
- **`--lora_r`**：8/16 常见；想更强一点可 32（但更吃显存/更易过拟合）


