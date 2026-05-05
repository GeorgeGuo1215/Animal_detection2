#!/usr/bin/env bash
# 启动本地静态站点 (默认 8080 端口)。
# 每次执行时会更新 version.js 中的 window.__SERVICE_UPDATE_TIME__，
# 让页面页脚显示「最新更新服务时间」。

set -e

cd "$(dirname "$0")"

PORT="${1:-8080}"
NOW="$(date '+%Y-%m-%d %H:%M:%S %Z')"

cat > version.js <<EOF
// 此文件由 start-server.sh 在每次启动 / 重启本地服务时自动覆盖。
// 切勿手动修改，修改也会被脚本重写。
window.__SERVICE_UPDATE_TIME__ = '${NOW}';
EOF

echo "[start-server] 更新时间写入 version.js: ${NOW}"

# 如果端口已被占用，先尝试杀掉占用该端口的 python -m http.server 进程
PIDS=$(lsof -tiTCP:${PORT} -sTCP:LISTEN 2>/dev/null || true)
if [ -n "${PIDS}" ]; then
    echo "[start-server] 端口 ${PORT} 已被占用 (PID=${PIDS})，先关闭后再启动"
    kill ${PIDS} 2>/dev/null || true
    sleep 0.5
fi

echo "[start-server] 启动 http.server: http://127.0.0.1:${PORT}/"
exec python3 -m http.server "${PORT}"
