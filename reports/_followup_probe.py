"""Follow-up defensive probes. Recycle after the assessment.

Does not send SQL/command injection strings, scan other hosts, or generate floods.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import socket
import ssl
import subprocess
import sys
import time
from pathlib import Path

import httpx

ROOT = Path(r"C:\Users\ROG\Animal_detection2\agentAndRag")
OUT = Path(__file__).with_name("local-prod-followup.json")
sys.path.insert(0, str(ROOT))


def load_env() -> dict[str, str]:
    creds: dict[str, str] = {}
    for line in (ROOT / ".env").read_text(encoding="utf-8").splitlines():
        if "=" in line and not line.startswith("#"):
            k, v = line.split("=", 1)
            creds[k.strip()] = v
    return creds


def tls_probe() -> dict:
    result: dict = {"https_5173": None, "https_8002": None, "listen_443": False, "nginx_ssl": False}
    nginx = Path(r"C:\Users\ROG\Animal_detection2\petmindAgentFrontend\nginx.conf").read_text(encoding="utf-8")
    result["nginx_ssl"] = "ssl_certificate" in nginx or "listen 443" in nginx
    try:
        with socket.create_connection(("127.0.0.1", 443), timeout=2):
            result["listen_443"] = True
    except OSError:
        result["listen_443"] = False
    for name, port in (("https_5173", 5173), ("https_8002", 8002)):
        try:
            ctx = ssl.create_default_context()
            with socket.create_connection(("127.0.0.1", port), timeout=3) as sock:
                with ctx.wrap_socket(sock, server_hostname="127.0.0.1") as ssock:
                    result[name] = {"ok": True, "version": ssock.version(), "cipher": ssock.cipher()}
        except Exception as exc:  # noqa: BLE001
            result[name] = {"ok": False, "error": type(exc).__name__, "detail": str(exc)[:180]}
    hsts = httpx.head("http://127.0.0.1:5173/chat", timeout=5.0)
    result["hsts_header"] = hsts.headers.get("strict-transport-security")
    result["http_chat_status"] = hsts.status_code
    return result


def webhook_probe(secret: str) -> dict:
    url = "http://127.0.0.1:8002/api/v1/payments/test-webhook"
    body = json.dumps(
        {"event_id": f"evt-followup-{int(time.time())}", "order_id": "0" * 32, "event_type": "payment.succeeded"}
    ).encode()
    cases = []

    def post(headers: dict) -> dict:
        r = httpx.post(url, content=body, headers={"Content-Type": "application/json", **headers}, timeout=10.0)
        return {"status": r.status_code, "preview": r.text[:160]}

    cases.append({"name": "missing_sig", **post({})})
    ts = str(int(time.time()))
    cases.append({"name": "wrong_sig", **post({"x-petmind-timestamp": ts, "x-petmind-signature": "0" * 64})})
    old_ts = str(int(time.time()) - 400)
    old_sig = hmac.new(secret.encode(), old_ts.encode() + b"." + body, hashlib.sha256).hexdigest()
    cases.append({"name": "stale_timestamp", **post({"x-petmind-timestamp": old_ts, "x-petmind-signature": old_sig})})
    good_sig = hmac.new(secret.encode(), ts.encode() + b"." + body, hashlib.sha256).hexdigest()
    cases.append({"name": "valid_sig_unknown_order", **post({"x-petmind-timestamp": ts, "x-petmind-signature": good_sig})})
    return {"cases": cases}


def refresh_family_probe(email: str, password: str) -> dict:
    login_url = "http://127.0.0.1:8002/api/v1/auth/login"
    refresh_url = "http://127.0.0.1:8002/api/v1/auth/refresh"
    me_url = "http://127.0.0.1:8002/api/v1/me"
    with httpx.Client(timeout=15.0) as client:
        login = client.post(login_url, json={"email": email, "password": password})
        cookie_a = client.cookies.get("petmind_refresh")
        access_after_login = login.json().get("access_token") if login.status_code == 200 else None
        rot = client.post(refresh_url)
        cookie_b = client.cookies.get("petmind_refresh")
        access_after_rot = rot.json().get("access_token") if rot.status_code == 200 else None
        replay = client.post(refresh_url, headers={"Cookie": f"petmind_refresh={cookie_a}"})
        family_after = client.post(refresh_url, headers={"Cookie": f"petmind_refresh={cookie_b}"})
        me_old_access = client.get(me_url, headers={"Authorization": f"Bearer {access_after_rot}"}) if access_after_rot else None
        return {
            "login": login.status_code,
            "rotate": rot.status_code,
            "cookie_a_present": bool(cookie_a),
            "cookie_b_present": bool(cookie_b),
            "cookie_changed": cookie_a != cookie_b,
            "replay_old": {"status": replay.status_code, "preview": replay.text[:160]},
            "refresh_new_after_replay": {"status": family_after.status_code, "preview": family_after.text[:160]},
            "access_after_family_kill": me_old_access.status_code if me_old_access is not None else None,
            "login_access_still_set": bool(access_after_login),
        }


def sql_canary_probe() -> dict:
    from agent_api.app.context.request_context import set_request_animal_id
    from agent_api.app.sql_search.query_compiler import compile_select
    from agent_api.app.sql_search.schema_catalog import validate_table
    from agent_api.app.sql_search.tool import sql_search_tool

    out: dict = {}
    rejected = []
    for name in ("not_a_whitelisted_table", "information_schema", "mysql", "pg_catalog"):
        try:
            validate_table(name)
            rejected.append({"table": name, "accepted": True})
        except ValueError as exc:
            rejected.append({"table": name, "accepted": False, "error": str(exc)[:80]})
    out["table_whitelist"] = rejected

    sql, params = compile_select(
        table="daily_reports",
        columns=["animal_id", "report_date"],
        where=[{"column": "report_text", "op": "like", "value": "PETMIND_CANARY_VALUE"}],
        order_by=None,
        limit=5,
    )
    out["compiled"] = {
        "sql": sql,
        "param_count": len(params),
        "canary_in_sql_text": "PETMIND_CANARY_VALUE" in sql,
        "canary_is_bound_param": params[0] == "PETMIND_CANARY_VALUE" if params else False,
        "uses_placeholders": "%s" in sql,
    }

    set_request_animal_id(explicit="PETMIND_CANARY_ANIMAL")
    try:
        result = sql_search_tool(
            table="daily_reports",
            where=[{"column": "report_text", "op": "like", "value": "PETMIND_CANARY_VALUE"}],
            limit=1,
        )
        out["tool_call"] = {
            "ok": result.get("ok"),
            "error": result.get("error"),
            "message": str(result.get("message", ""))[:180],
            "row_count": result.get("row_count"),
        }
    finally:
        set_request_animal_id()
    return out


def mcp_static() -> dict:
    tavily = Path(ROOT / "mcp_servers/web_search/tavily_client.py").read_text(encoding="utf-8")
    vitals = Path(ROOT / "mcp_servers/vitals_alert/db.py").read_text(encoding="utf-8")
    return {
        "web_search_uses_json_api": "api.tavily.com/search" in tavily and "subprocess" not in tavily,
        "vitals_uses_bound_params": "%(pet_id)s" in vitals and "sql.Identifier" in vitals,
        "note": "MCP not invoked with attacker strings; web_search would only forward a JSON query field to Tavily.",
    }


def localhost_exposure() -> dict:
    # Bindings only. Does not probe other LAN hosts.
    ps = r"C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"
    try:
        raw = subprocess.check_output(
            [
                ps,
                "-NoProfile",
                "-Command",
                "Get-NetTCPConnection -State Listen | Where-Object { $_.LocalPort -in 5173,8002,8102,8300,5432,15433,16379,443 } | Select-Object LocalAddress,LocalPort | ConvertTo-Json",
            ],
            text=True,
        )
    except Exception as exc:  # noqa: BLE001
        return {"listen": [], "wildcard_binds": [], "scanned_foreign_hosts": False, "error": type(exc).__name__}
    try:
        rows = json.loads(raw) if raw.strip() else []
    except json.JSONDecodeError:
        rows = raw[:500]
    if isinstance(rows, dict):
        rows = [rows]
    publicish = []
    if isinstance(rows, list):
        for row in rows:
            addr = str(row.get("LocalAddress", ""))
            if addr in {"0.0.0.0", "::"}:
                publicish.append(row)
    return {"listen": rows, "wildcard_binds": publicish, "scanned_foreign_hosts": False}


def main() -> None:
    env = load_env()
    report = {
        "skipped": {
            "sql_mcp_attack_payloads": "Not sending injection or exploit strings. Used whitelist + bound-parameter canary only.",
            "cross_host_scan": "Did not scan LAN/other hosts. Only listed local listen addresses.",
            "syn_flood": "Refused. Flood/kernel attack traffic is out of scope.",
        },
        "tls": tls_probe(),
        "webhook": webhook_probe(env["AGENT_PLATFORM_PAYMENT_WEBHOOK_SECRET"]),
        "refresh_family": refresh_family_probe(env["PETMIND_LOCAL_ADMIN_EMAIL"], env["PETMIND_LOCAL_ADMIN_PASSWORD"]),
        "sql_canary": sql_canary_probe(),
        "mcp": mcp_static(),
        "localhost_exposure": localhost_exposure(),
    }
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
