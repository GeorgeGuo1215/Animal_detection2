"""Run Memory Service and Agent API under one development/service lifecycle.

The services stay in separate processes (so model/DB failures remain isolated), while
this supervisor owns startup ordering, readiness, termination and exit propagation.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Dict, List, Optional


ROOT = Path(__file__).resolve().parents[2]


def _flag(value: Optional[str], default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def build_child_environment(
    base: Dict[str, str],
    *,
    memory_host: str,
    memory_port: int,
    memory_required: bool,
) -> tuple[Dict[str, str], Dict[str, str]]:
    memory_env = dict(base)
    memory_env["MEMORY_HOST"] = memory_host
    memory_env["MEMORY_PORT"] = str(memory_port)

    agent_env = dict(base)
    agent_env["AGENT_MEMORY_ENABLED"] = "1"
    agent_env["AGENT_MEMORY_REQUIRED"] = "1" if memory_required else "0"
    agent_env["AGENT_MEMORY_URL"] = f"http://{memory_host}:{memory_port}"
    return memory_env, agent_env


def wait_for_memory(url: str, timeout_s: float) -> Dict[str, object]:
    deadline = time.monotonic() + timeout_s
    last_error = "not started"
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(f"{url.rstrip('/')}/health", timeout=2.0) as response:
                payload = json.loads(response.read().decode("utf-8"))
            if payload.get("status") == "ok" and payload.get("database") == "ok":
                return dict(payload)
            last_error = f"unhealthy payload: {payload}"
        except (OSError, ValueError, urllib.error.URLError) as exc:
            last_error = str(exc)
        time.sleep(0.25)
    raise RuntimeError(f"memory service did not become ready within {timeout_s}s: {last_error}")


def _stop_process(process: Optional[subprocess.Popen[bytes]], timeout_s: float = 15.0) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def _commands(args: argparse.Namespace) -> tuple[List[str], List[str]]:
    memory = [sys.executable, "-m", "memory_service.app.main"]
    agent = [
        sys.executable,
        "-m",
        "uvicorn",
        "agent_api.app.main:app",
    ]
    if _flag(os.getenv("AGENT_TRUST_PROXY_HEADERS"), False):
        agent.extend([
            "--proxy-headers",
            "--forwarded-allow-ips",
            os.getenv("AGENT_FORWARDED_ALLOW_IPS", "127.0.0.1"),
        ])
    else:
        agent.append("--no-proxy-headers")
    agent.extend(["--host", args.agent_host, "--port", str(args.agent_port)])
    return memory, agent


def _worker_command() -> List[str]:
    return [sys.executable, "-m", "agent_api.scripts.run_platform_worker"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Start Memory Service, then Agent API.")
    parser.add_argument("--agent-host", default=os.getenv("AGENT_HOST", "127.0.0.1"))
    parser.add_argument("--agent-port", type=int, default=int(os.getenv("AGENT_PORT", "8000")))
    parser.add_argument("--memory-host", default=os.getenv("MEMORY_HOST", "127.0.0.1"))
    parser.add_argument("--memory-port", type=int, default=int(os.getenv("MEMORY_PORT", "8300")))
    parser.add_argument(
        "--memory-start-timeout",
        type=float,
        default=float(os.getenv("AGENT_MEMORY_START_TIMEOUT", "180")),
    )
    parser.add_argument(
        "--init-memory-db",
        action="store_true",
        default=_flag(os.getenv("AGENT_MEMORY_INIT_DB"), False),
        help="Create/apply the local memory schema before starting services.",
    )
    parser.add_argument(
        "--memory-required",
        action=argparse.BooleanOptionalAction,
        default=_flag(os.getenv("AGENT_MEMORY_REQUIRED"), True),
    )
    parser.add_argument(
        "--platform-worker",
        action=argparse.BooleanOptionalAction,
        default=_flag(os.getenv("AGENT_PLATFORM_WORKER"), bool(os.getenv("AGENT_PLATFORM_REDIS_URL"))),
        help="Run the Redis-backed Agent worker in this supervised stack.",
    )
    args = parser.parse_args()

    if args.init_memory_db:
        subprocess.run(
            [sys.executable, "-m", "memory_service.scripts.init_local_db"],
            cwd=ROOT,
            check=True,
        )

    memory_env, agent_env = build_child_environment(
        dict(os.environ),
        memory_host=args.memory_host,
        memory_port=args.memory_port,
        memory_required=args.memory_required,
    )
    memory_command, agent_command = _commands(args)
    memory_process: Optional[subprocess.Popen[bytes]] = None
    agent_process: Optional[subprocess.Popen[bytes]] = None
    worker_process: Optional[subprocess.Popen[bytes]] = None
    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    for name in ("SIGINT", "SIGTERM"):
        signum = getattr(signal, name, None)
        if signum is not None:
            signal.signal(signum, request_stop)

    try:
        memory_process = subprocess.Popen(memory_command, cwd=ROOT, env=memory_env)
        memory_url = f"http://{args.memory_host}:{args.memory_port}"
        health = wait_for_memory(memory_url, args.memory_start_timeout)
        print(f"[stack] memory ready: {health}", flush=True)

        agent_process = subprocess.Popen(agent_command, cwd=ROOT, env=agent_env)
        if args.platform_worker:
            if not agent_env.get("AGENT_PLATFORM_REDIS_URL"):
                raise RuntimeError("--platform-worker requires AGENT_PLATFORM_REDIS_URL")
            worker_process = subprocess.Popen(_worker_command(), cwd=ROOT, env=agent_env)
        print(
            f"[stack] agent started on http://{args.agent_host}:{args.agent_port}; "
            f"memory={memory_url}",
            flush=True,
        )
        while not stopping:
            memory_code = memory_process.poll()
            agent_code = agent_process.poll()
            worker_code = worker_process.poll() if worker_process is not None else None
            if memory_code is not None:
                print(f"[stack] memory exited with code {memory_code}", file=sys.stderr)
                return memory_code or 1
            if agent_code is not None:
                print(f"[stack] agent exited with code {agent_code}", file=sys.stderr)
                return agent_code
            if worker_process is not None and worker_code is not None:
                print(f"[stack] platform worker exited with code {worker_code}", file=sys.stderr)
                return worker_code or 1
            time.sleep(0.25)
        return 0
    finally:
        _stop_process(worker_process)
        _stop_process(agent_process)
        _stop_process(memory_process)


if __name__ == "__main__":
    raise SystemExit(main())
