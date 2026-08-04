#!/usr/bin/env bash
# === PetHealthAI Agent 启动脚本 (Linux) ===
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 加载 .env（如果存在，作为 ~/.bashrc 的补充）
if [ -f .env ]; then
    set -a
    source .env
    set +a
    echo "[Config] Loaded .env"
fi

# 默认值
export OPENAI_BASE_URL="${OPENAI_BASE_URL:-https://api.deepseek.com}"
export OPENAI_MODEL="${OPENAI_MODEL:-deepseek-chat}"
export AGENT_ENABLE_CORS="${AGENT_ENABLE_CORS:-1}"
export AGENT_WARMUP_DEVICE="${AGENT_WARMUP_DEVICE:-cuda}"
export AGENT_WARMUP_RAG="${AGENT_WARMUP_RAG:-1}"

echo "[Config] OPENAI_BASE_URL=$OPENAI_BASE_URL"
echo "[Config] OPENAI_API_KEY=${OPENAI_API_KEY:+${OPENAI_API_KEY:0:10}...}"
echo "[Config] OPENAI_MODEL=$OPENAI_MODEL"
echo "[Config] AGENT_WARMUP_DEVICE=$AGENT_WARMUP_DEVICE"
echo "[Config] TAVILY_API_KEY=${TAVILY_API_KEY:+${TAVILY_API_KEY:0:10}...}"
echo ""

CONDA_PYTHON="/home/sam/anaconda3/envs/AnimalDetection/bin/python"
if [ ! -f "$CONDA_PYTHON" ]; then
    echo "[Error] Conda env python not found: $CONDA_PYTHON"
    echo "[Error] Please run: conda activate AnimalDetection"
    exit 1
fi

"$CONDA_PYTHON" -m uvicorn agent_api.app.main:app \
    --host "${AGENT_HOST:-0.0.0.0}" \
    --port "${AGENT_PORT:-8000}"
