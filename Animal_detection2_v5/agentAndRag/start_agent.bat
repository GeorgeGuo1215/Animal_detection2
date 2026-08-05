@echo off
REM === PetHealthAI Agent 启动脚本 ===
REM 从 Windows 用户环境变量中读取 key，注入到当前进程

REM 读取用户级环境变量（即使当前 CMD 会话是旧的也能拿到最新值）
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_API_KEY','User')"') do set OPENAI_API_KEY=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_BASE_URL','User')"') do set OPENAI_BASE_URL=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_MODEL','User')"') do set OPENAI_MODEL=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('TAVILY_API_KEY','User')"') do set TAVILY_API_KEY=%%a

REM 预热设备（可改为 cpu）
set AGENT_WARMUP_DEVICE=cuda

REM 显示当前配置
echo [Config] OPENAI_BASE_URL=%OPENAI_BASE_URL%
echo [Config] OPENAI_API_KEY=%OPENAI_API_KEY:~0,10%...
echo [Config] OPENAI_MODEL=%OPENAI_MODEL%
echo [Config] AGENT_WARMUP_DEVICE=%AGENT_WARMUP_DEVICE%
echo [Config] TAVILY_API_KEY=%TAVILY_API_KEY:~0,10%...
echo.

cd /d "%~dp0"
python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000
