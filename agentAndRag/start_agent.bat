@echo off
setlocal EnableExtensions

rem PetMind Agent + Memory Service unified launcher for Windows.
rem Usage: start_agent.bat [cpu|cuda]

cd /d "%~dp0"

rem Load simple KEY=VALUE entries from the project .env without overriding
rem variables already supplied by the service manager or current terminal.
if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" if not defined %%A set "%%A=%%B"
    )
    echo [Config] Loaded %CD%\.env
)

rem Prefer an explicitly configured interpreter, then this workstation's RAG
rem environment, and finally the Python command inherited from PATH.
if not defined AGENT_PYTHON (
    if exist "C:\Users\ROG\anaconda3\envs\RAG\python.exe" (
        set "AGENT_PYTHON=C:\Users\ROG\anaconda3\envs\RAG\python.exe"
    ) else (
        set "AGENT_PYTHON=python"
    )
)

if not defined OPENAI_BASE_URL set "OPENAI_BASE_URL=https://api.deepseek.com"
if not defined OPENAI_MODEL set "OPENAI_MODEL=deepseek-chat"
if not defined AGENT_HOST set "AGENT_HOST=127.0.0.1"
if not defined AGENT_PORT set "AGENT_PORT=8000"
if not defined MEMORY_HOST set "MEMORY_HOST=127.0.0.1"
if not defined MEMORY_PORT set "MEMORY_PORT=8300"
if not defined AGENT_MEMORY_REQUIRED set "AGENT_MEMORY_REQUIRED=1"
if not defined AGENT_MEMORY_START_TIMEOUT set "AGENT_MEMORY_START_TIMEOUT=180"
if not defined MEMORY_WARMUP_EMBEDDING set "MEMORY_WARMUP_EMBEDDING=1"
if not defined AGENT_WARMUP_DEVICE set "AGENT_WARMUP_DEVICE=cuda"

if /I "%~1"=="cpu" set "AGENT_WARMUP_DEVICE=cpu"
if /I "%~1"=="cuda" set "AGENT_WARMUP_DEVICE=cuda"
if not "%~1"=="" if /I not "%~1"=="cpu" if /I not "%~1"=="cuda" (
    echo [Error] Unknown mode "%~1". Usage: start_agent.bat [cpu^|cuda]
    exit /b 2
)

rem A CUDA build of PyTorch can crash on Windows when this is set to -1.
rem CPU mode is selected only through AGENT_WARMUP_DEVICE.
if /I "%AGENT_WARMUP_DEVICE%"=="cpu" set "CUDA_VISIBLE_DEVICES="

"%AGENT_PYTHON%" --version >nul 2>&1
if errorlevel 1 (
    echo [Error] Python is not available: %AGENT_PYTHON%
    echo [Error] Set AGENT_PYTHON to the intended python.exe path.
    exit /b 1
)

echo [Config] Python=%AGENT_PYTHON%
echo [Config] Agent=http://%AGENT_HOST%:%AGENT_PORT%
echo [Config] Memory=http://%MEMORY_HOST%:%MEMORY_PORT%
echo [Config] WarmupDevice=%AGENT_WARMUP_DEVICE%
echo [Config] MemoryRequired=%AGENT_MEMORY_REQUIRED%
if defined AGENT_PLATFORM_REDIS_URL echo [Config] PlatformWorker=enabled
echo [Config] Secrets loaded but not printed.

"%AGENT_PYTHON%" -m agent_api.scripts.run_agent_stack ^
    --agent-host "%AGENT_HOST%" ^
    --agent-port "%AGENT_PORT%" ^
    --memory-host "%MEMORY_HOST%" ^
    --memory-port "%MEMORY_PORT%"

set "EXIT_CODE=%ERRORLEVEL%"
echo [Exit] Agent stack stopped with code %EXIT_CODE%.
exit /b %EXIT_CODE%
