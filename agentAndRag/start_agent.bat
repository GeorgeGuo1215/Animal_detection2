@echo off
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_API_KEY','User')"') do set OPENAI_API_KEY=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_BASE_URL','User')"') do set OPENAI_BASE_URL=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('OPENAI_MODEL','User')"') do set OPENAI_MODEL=%%a
for /f "tokens=*" %%a in ('powershell -NoProfile -Command "[System.Environment]::GetEnvironmentVariable('TAVILY_API_KEY','User')"') do set TAVILY_API_KEY=%%a

set AGENT_WARMUP_DEVICE=cuda
set CUDA_DEVICE_ORDER=PCI_BUS_ID

echo [Config] OPENAI_BASE_URL=%OPENAI_BASE_URL%
echo [Config] OPENAI_API_KEY=%OPENAI_API_KEY:~0,10%...
echo [Config] OPENAI_MODEL=%OPENAI_MODEL%
echo [Config] AGENT_WARMUP_DEVICE=%AGENT_WARMUP_DEVICE%
echo [Config] CUDA_DEVICE_ORDER=%CUDA_DEVICE_ORDER%
echo [Config] TAVILY_API_KEY=%TAVILY_API_KEY:~0,10%...
echo.

cd /d "%~dp0"
python -m uvicorn agent_api.app.main:app --host 127.0.0.1 --port 8000
