@echo off
chcp 65001 >nul
echo ======================================
echo   毫米波雷达Web应用 - 服务器启动
echo ======================================
echo.

REM 检查Python是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未检测到Python！
    echo.
    echo 请先安装Python:
    echo 1. 访问 https://www.python.org/downloads/
    echo 2. 下载并安装最新版本
    echo 3. 安装时勾选 "Add Python to PATH"
    echo.
    pause
    exit /b 1
)

echo [✓] Python已安装
python --version
echo.

REM 切换到脚本所在目录
cd /d "%~dp0"

echo [启动] 正在启动Web服务器...
echo [提示] 服务器地址: http://localhost:8000
echo [提示] 按 Ctrl+C 可停止服务器
echo.
echo ======================================
echo   请在浏览器打开: http://localhost:8000
echo ======================================
echo.

REM 尝试自动打开浏览器（可选）
start http://localhost:8000

REM 启动Python HTTP服务器
python -m http.server 8000

pause