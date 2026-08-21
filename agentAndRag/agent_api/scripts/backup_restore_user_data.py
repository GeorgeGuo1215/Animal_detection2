"""导出或覆盖恢复指定用户的会话与记忆快照。

操作令牌必须属于超级管理员。快照含敏感会话与记忆数据，需加密存放且禁止提交到仓库。
"""
from __future__ import annotations

import argparse
import gzip
import json
import os
from pathlib import Path

import httpx


def main() -> int:
    """命令行入口：按模式导出或覆盖恢复指定用户的会话与记忆快照。"""
    parser = argparse.ArgumentParser(description="PetMind user data backup/restore")
    parser.add_argument("mode", choices=("export", "restore"))
    parser.add_argument("--base-url", default=os.getenv("PETMIND_API_URL", "http://127.0.0.1:8002"))
    parser.add_argument("--token", default=os.getenv("PETMIND_ADMIN_ACCESS_TOKEN", ""))
    parser.add_argument("--user-id", required=True)
    parser.add_argument("--file", required=True, type=Path)
    parser.add_argument("--confirm", action="store_true", help="required for overwrite restore")
    args = parser.parse_args()
    if not args.token:
        parser.error("--token or PETMIND_ADMIN_ACCESS_TOKEN is required")
    endpoint = f"{args.base_url.rstrip('/')}/api/v1/admin/users/{args.user_id}/data-snapshot"
    headers = {"Authorization": f"Bearer {args.token}"}
    with httpx.Client(timeout=120, trust_env=False) as client:
        if args.mode == "export":
            response = client.get(endpoint, headers=headers)
            response.raise_for_status()
            args.file.parent.mkdir(parents=True, exist_ok=True)
            encoded = json.dumps(response.json(), ensure_ascii=False, indent=2).encode("utf-8")
            if args.file.suffix.lower() == ".gz":
                args.file.write_bytes(gzip.compress(encoded, compresslevel=9))
            else:
                args.file.write_bytes(encoded)
            try:
                os.chmod(args.file, 0o600)
            except OSError:
                pass
            print(f"snapshot exported: {args.file.resolve()}")
            return 0
        if not args.confirm:
            parser.error("restore overwrites current conversations and memory; pass --confirm")
        raw = args.file.read_bytes()
        if args.file.suffix.lower() == ".gz":
            response = client.post(
                f"{endpoint}/restore-file",
                headers={
                    **headers,
                    "Content-Type": "application/gzip",
                    "X-Restore-Confirmation": "OVERWRITE_USER_DATA",
                },
                content=raw,
            )
        else:
            snapshot = json.loads(raw.decode("utf-8"))
            response = client.post(
                f"{endpoint}/restore",
                headers=headers,
                json={"confirmation": "覆盖恢复用户数据", "snapshot": snapshot},
            )
        response.raise_for_status()
        print(json.dumps(response.json(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
