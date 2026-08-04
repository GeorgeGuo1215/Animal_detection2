"""vitals_alert MCP 冒烟脚本 —— 走真实 stdio 子进程调一次 check_vitals。

它验证的是整条链路而不只是业务逻辑：mcp_servers.json 配置能被解析、子进程能起来、
工具能被发现、调用能拿到结构化结果。数据库没就绪时会稳定返回 DB_UNAVAILABLE，
这本身也是一个有效的验证结果。

本脚本不加载任何模型，不会占用显卡。

用法：
  python agent_api/scripts/check_vitals_alert.py --pet-id <petId> [--hours 24] [--species DOG]
  python agent_api/scripts/check_vitals_alert.py --list-only
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_AGENT_API = os.path.abspath(os.path.join(_HERE, ".."))
_REPO_ROOT = os.path.abspath(os.path.join(_AGENT_API, ".."))
for _p in (_AGENT_API, _REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from app.mcp.mcp_client import call_mcp_tool_async, list_mcp_tools_async  # noqa: E402
from app.mcp.mcp_config import load_mcp_servers  # noqa: E402

SERVER_NAME = "vitals_alert"


def _unwrap(result: dict) -> dict:
    """MCP 返回的是 {is_error, content:[{type,text}]}，把内层业务 JSON 取出来。"""
    for item in result.get("content") or []:
        if isinstance(item, dict) and item.get("text"):
            try:
                return json.loads(item["text"])
            except json.JSONDecodeError:
                return {"raw_text": item["text"]}
    return result


async def _check_registry() -> bool:
    """Run the same registration the backend does at startup, then assert our tool landed."""
    from app.tools.tool_registry import ToolRegistry
    from app.tools.tools_mcp import register_mcp_tools_async

    registry = ToolRegistry()
    summary = await register_mcp_tools_async(registry)
    print(f"[ok] registry summary: {summary}")

    spec = registry.get(f"mcp.{SERVER_NAME}.check_vitals")
    if spec is None:
        print(f"[FAIL] mcp.{SERVER_NAME}.check_vitals was not registered")
        return False
    print(f"[ok] registered: {spec.name}; required={spec.input_schema.get('required')}")
    return True


async def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test the vitals_alert MCP server")
    parser.add_argument("--pet-id", default="", help="Pet.id to query")
    parser.add_argument("--hours", type=int, default=24, help="look-back window in hours")
    parser.add_argument("--species", default="", help="optional species override")
    parser.add_argument("--list-only", action="store_true", help="only list tools, do not call")
    parser.add_argument(
        "--via-registry",
        action="store_true",
        help="also run the full register_mcp_tools_async path (all servers)",
    )
    args = parser.parse_args()

    if args.via_registry and not await _check_registry():
        return 1

    server = next((s for s in load_mcp_servers() if s.name == SERVER_NAME), None)
    if server is None:
        print(f"[FAIL] {SERVER_NAME} not found in mcp_servers.json (or disabled)")
        return 1
    print(f"[ok] config: {server.command} {' '.join(server.args)}  cwd={server.cwd}")

    tools = await list_mcp_tools_async(server)
    names = [t.get("name") for t in tools]
    print(f"[ok] subprocess started, tools exposed: {names}")
    if "check_vitals" not in names:
        print("[FAIL] check_vitals is missing")
        return 1

    if args.list_only:
        return 0
    if not args.pet_id:
        print("[skip] no --pet-id given; pass one to run a real query")
        return 0

    payload = {"pet_id": args.pet_id, "hours": args.hours}
    if args.species:
        payload["species"] = args.species

    result = _unwrap(await call_mcp_tool_async(server, "check_vitals", payload))
    print(json.dumps(result, ensure_ascii=False, indent=2))

    status = result.get("status")
    print(f"[done] status={status}")
    return 0 if status else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
