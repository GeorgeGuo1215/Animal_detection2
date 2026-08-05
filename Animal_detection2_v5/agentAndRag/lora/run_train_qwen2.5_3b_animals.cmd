@echo off
setlocal enabledelayedexpansion

REM 无论从哪里运行/双击，都切回仓库根目录（lora 的上一级）
set ROOT=%~dp0..
pushd "%ROOT%"

set MODEL_ID=Qwen/Qwen2.5-3B-Instruct
set DATA_PATH=RAG\data\qa_gen\animals_alpaca.jsonl
set OUT_DIR=lora\out\qwen2.5-3b-animals

python lora\train_sft_lora.py ^
  --model_id %MODEL_ID% ^
  --data_path %DATA_PATH% ^
  --output_dir %OUT_DIR% ^
  --num_train_epochs 3 ^
  --max_seq_len 1024 ^
  --per_device_train_batch_size 1 ^
  --gradient_accumulation_steps 8

echo.
echo Done. Adapter saved to: %OUT_DIR%
popd
pause


