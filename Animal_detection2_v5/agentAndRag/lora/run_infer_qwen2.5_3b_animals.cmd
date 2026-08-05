@echo off
setlocal enabledelayedexpansion

REM 无论从哪里运行/双击，都切回仓库根目录（lora 的上一级）
set ROOT=%~dp0..
pushd "%ROOT%"

set MODEL_ID=Qwen/Qwen2.5-3B-Instruct
set ADAPTER_DIR=lora\out\qwen2.5-3b-animals

REM 你可以把问题改成你想测的任何动物知识问答
set QUESTION=羊驼属于哪一类动物？简单介绍一下它的特征。

python lora\infer.py ^
  --model_id %MODEL_ID% ^
  --adapter_dir %ADAPTER_DIR% ^
  --question "%QUESTION%"

popd
pause


