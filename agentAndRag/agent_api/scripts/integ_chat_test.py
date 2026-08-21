"""三端联调（server 侧）冒烟：登录 -> /api/chat/stream(useAgent) -> 校验代理到 MoE。

只测 PetHealth-server -> Agent 的链路，不涉及手机。trust_env=False 绕过本机 Mihomo 代理。
"""
from __future__ import annotations

import json
import sys

import httpx

BASE = "http://127.0.0.1:5050/api"
EMAIL = "1411982730@qq.com"
PASSWORD = "12345678"
PET_ID = sys.argv[1] if len(sys.argv) > 1 else "cmr1t462i00011sv8bjr1ixws"


def main() -> None:
    """三端联调冒烟：登录后走 /api/chat/stream，确认请求被代理到 MoE。"""
    c = httpx.Client(trust_env=False, timeout=httpx.Timeout(connect=10, read=180, write=10, pool=30))

    # 1) 登录
    r = c.post(f"{BASE}/auth/login/email", json={"email": EMAIL, "password": PASSWORD})
    print("login:", r.status_code)
    data = r.json().get("data") or {}
    token = data.get("accessToken")
    if not token:
        print("LOGIN FAILED:", r.text[:500])
        return
    print("token ok, userId:", data.get("id"))

    # 2) SSE 聊天（走 agent）
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
    }
    body = {
        "messages": [{"role": "user", "content": "我家猫最近的静息心率和体温正常吗？还有哪些日常护理要注意？"}],
        "petId": PET_ID,
        "useAgent": True,
        "stream": True,
    }

    tool_status = []
    content_chars = 0
    other_events = []
    with c.stream("POST", f"{BASE}/chat/stream", headers=headers, json=body) as resp:
        print("chat stream:", resp.status_code)
        for line in resp.iter_lines():
            if not line or not line.startswith("data:"):
                continue
            payload = line[5:].strip()
            if not payload:
                continue
            try:
                evt = json.loads(payload)
            except json.JSONDecodeError:
                continue
            etype = evt.get("type")
            if etype == "content":
                content_chars += len(evt.get("content") or evt.get("data") or "")
            elif etype == "tool_status":
                st = (evt.get("status") or (evt.get("detail") or {}).get("status") or "")
                tool_status.append(st)
            elif etype == "done":
                other_events.append("done")
            elif etype:
                other_events.append(etype)

    print("=== RESULT ===")
    print("tool_status events:", tool_status)
    print("content chars:", content_chars)
    print("other events:", other_events)


if __name__ == "__main__":
    main()
