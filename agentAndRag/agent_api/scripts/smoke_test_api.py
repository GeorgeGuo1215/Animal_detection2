#!/usr/bin/env python3
"""
Smoke test for PetMind Agent API: health, tools list, tools/call (debug.echo), tail trace.jsonl.

Run from repo root (agentAndRag as cwd):
  python agent_api/scripts/smoke_test_api.py

Requires: pip install httpx
Environment:
  AGENT_API_BASE  default http://127.0.0.1:8000
  AGENT_API_KEY   optional; if auth enabled, use keys.txt key or set AGENT_DISABLE_AUTH=1 on server
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

try:
    import httpx
except ImportError:
    print("Install httpx: pip install httpx", file=sys.stderr)
    raise

# agent_api_logs lives under agentAndRag/
_HERE = Path(__file__).resolve()
_AGENT_AND_RAG_ROOT = _HERE.parents[2]
_DEFAULT_TRACE = _AGENT_AND_RAG_ROOT / "agent_api_logs" / "trace.jsonl"


def main() -> int:
    base = os.getenv("AGENT_API_BASE", "http://127.0.0.1:8000").rstrip("/")
    api_key = (os.getenv("AGENT_API_KEY") or "").strip()

    headers: dict[str, str] = {}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    trace_path = Path(os.getenv("AGENT_TRACE_DIR", str(_DEFAULT_TRACE.parent))) / "trace.jsonl"

    with httpx.Client(timeout=120.0) as client:
        r = client.get(f"{base}/health")
        print(f"[1] GET /health -> {r.status_code} {r.text[:200]}")
        if r.status_code != 200:
            return 1

        r = client.get(f"{base}/tools", headers=headers)
        print(f"[2] GET /tools -> {r.status_code}")
        if r.status_code != 200:
            print(r.text[:500])
            return 1
        data = r.json()
        names = [t.get("name") for t in data.get("tools", [])]
        print(f"    tools count={len(names)} sample={names[:5]}...")

        body = {"tool_name": "debug.echo", "arguments": {"ping": "smoke"}}
        r = client.post(f"{base}/tools/call", headers=headers, json=body)
        print(f"[3] POST /tools/call debug.echo -> {r.status_code}")
        out = r.json()
        print(f"    response keys={list(out.keys())} ok={out.get('ok')}")
        if r.status_code != 200 or not out.get("ok"):
            print(json.dumps(out, ensure_ascii=False, indent=2)[:800])
            return 1
        tid = out.get("trace_id", "")
        print(f"    trace_id={tid}")

    print("[4] Trace file (last 5 lines):")
    if not trace_path.is_file():
        print(f"    (no file yet: {trace_path})")
        return 0
    lines = trace_path.read_text(encoding="utf-8").strip().splitlines()
    for line in lines[-5:]:
        try:
            obj = json.loads(line)
            print(f"    ts={obj.get('ts')} tool={obj.get('tool')} trace_id={obj.get('trace_id')}")
        except json.JSONDecodeError:
            print(f"    {line[:120]}...")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
