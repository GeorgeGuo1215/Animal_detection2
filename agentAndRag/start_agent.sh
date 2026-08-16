#!/usr/bin/env bash
# PetMind Agent + Memory Service unified launcher for Linux.
# Usage: bash start_agent.sh [cpu|cuda] [additional run_agent_stack arguments]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ -f .env ]]; then
    set -a
    # shellcheck disable=SC1091
    source .env
    set +a
    echo "[Config] Loaded $SCRIPT_DIR/.env"
fi

if [[ -n "${AGENT_PYTHON:-}" ]]; then
    PYTHON_BIN="$AGENT_PYTHON"
elif [[ -x /home/sam/anaconda3/envs/AnimalDetection/bin/python ]]; then
    PYTHON_BIN=/home/sam/anaconda3/envs/AnimalDetection/bin/python
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python)"
else
    echo "[Error] Python was not found. Set AGENT_PYTHON to the intended interpreter." >&2
    exit 1
fi

export OPENAI_BASE_URL="${OPENAI_BASE_URL:-https://api.deepseek.com}"
export OPENAI_MODEL="${OPENAI_MODEL:-deepseek-chat}"
export AGENT_HOST="${AGENT_HOST:-0.0.0.0}"
export AGENT_PORT="${AGENT_PORT:-8000}"
export MEMORY_HOST="${MEMORY_HOST:-127.0.0.1}"
export MEMORY_PORT="${MEMORY_PORT:-8300}"
export AGENT_MEMORY_REQUIRED="${AGENT_MEMORY_REQUIRED:-1}"
export AGENT_MEMORY_START_TIMEOUT="${AGENT_MEMORY_START_TIMEOUT:-180}"
export MEMORY_WARMUP_EMBEDDING="${MEMORY_WARMUP_EMBEDDING:-1}"
export AGENT_WARMUP_DEVICE="${AGENT_WARMUP_DEVICE:-cuda}"
export AGENT_ENABLE_CORS="${AGENT_ENABLE_CORS:-1}"
export AGENT_WARMUP_RAG="${AGENT_WARMUP_RAG:-1}"
export AGENT_WARMUP_BM25="${AGENT_WARMUP_BM25:-1}"
export AGENT_WARMUP_RERANKER="${AGENT_WARMUP_RERANKER:-1}"
export AGENT_WARMUP_CATEGORIES="${AGENT_WARMUP_CATEGORIES:-1}"

if [[ "${1:-}" == "cpu" || "${1:-}" == "cuda" ]]; then
    export AGENT_WARMUP_DEVICE="$1"
    shift
elif [[ -n "${1:-}" && "${1:0:1}" != "-" ]]; then
    echo "[Error] Unknown mode '$1'. Usage: ./start_agent.sh [cpu|cuda] [options]" >&2
    exit 2
fi

if [[ "$AGENT_WARMUP_DEVICE" == "cpu" ]]; then
    unset CUDA_VISIBLE_DEVICES || true
fi

if [[ ! -x "$PYTHON_BIN" ]] && ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "[Error] Python is not executable: $PYTHON_BIN" >&2
    exit 1
fi

echo "[Config] Python=$PYTHON_BIN"
echo "[Config] Agent=http://$AGENT_HOST:$AGENT_PORT"
echo "[Config] Memory=http://$MEMORY_HOST:$MEMORY_PORT"
echo "[Config] WarmupDevice=$AGENT_WARMUP_DEVICE"
echo "[Config] Warmup=RAG:$AGENT_WARMUP_RAG BM25:$AGENT_WARMUP_BM25 Reranker:$AGENT_WARMUP_RERANKER Categories:$AGENT_WARMUP_CATEGORIES MemoryEmbedding:$MEMORY_WARMUP_EMBEDDING"
echo "[Config] MemoryRequired=$AGENT_MEMORY_REQUIRED"
if [[ -n "${AGENT_PLATFORM_REDIS_URL:-}" ]]; then
    echo "[Config] PlatformWorker=${AGENT_PLATFORM_WORKER:-1}"
fi
echo "[Config] Secrets loaded but not printed."

exec "$PYTHON_BIN" -m agent_api.scripts.run_agent_stack \
    --agent-host "$AGENT_HOST" \
    --agent-port "$AGENT_PORT" \
    --memory-host "$MEMORY_HOST" \
    --memory-port "$MEMORY_PORT" \
    "$@"
