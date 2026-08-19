"""Bounded localhost capacity probe. Recycle after the assessment."""
from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import httpx

OUT = Path(__file__).with_name("local-prod-load.json")
TIMEOUT = 15.0


def pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    idx = min(len(s) - 1, max(0, int(round((p / 100) * (len(s) - 1)))))
    return s[idx]


def wave(url: str, n: int, *, method: str = "GET", expected: set[int] | None = None, headers: dict | None = None) -> dict:
    latencies: list[float] = []
    codes: dict[str, int] = {}
    errors = 0

    def one(_i: int) -> tuple[int | None, float, str]:
        t0 = time.perf_counter()
        try:
            with httpx.Client(timeout=TIMEOUT) as client:
                r = client.request(method, url, headers=headers)
            return r.status_code, (time.perf_counter() - t0) * 1000, ""
        except Exception as exc:  # noqa: BLE001
            return None, (time.perf_counter() - t0) * 1000, type(exc).__name__

    t_wall = time.perf_counter()
    with ThreadPoolExecutor(max_workers=min(n, 64)) as pool:
        futs = [pool.submit(one, i) for i in range(n)]
        for fut in as_completed(futs):
            code, ms, err = fut.result()
            latencies.append(ms)
            key = str(code) if code is not None else f"err:{err}"
            codes[key] = codes.get(key, 0) + 1
            if code is None:
                errors += 1
            elif expected and code not in expected:
                errors += 1
            elif not expected and code >= 500:
                errors += 1
    wall = (time.perf_counter() - t_wall) * 1000
    result = {
        "url": url,
        "concurrency": n,
        "count": n,
        "codes": codes,
        "errors": errors,
        "error_rate": round(errors / n, 4),
        "p50_ms": round(pct(latencies, 50), 2),
        "p95_ms": round(pct(latencies, 95), 2),
        "max_ms": round(max(latencies) if latencies else 0, 2),
        "wall_ms": round(wall, 2),
        "rps": round(n / (wall / 1000), 2) if wall else 0,
    }
    print(f"[wave] n={n} {url} codes={codes} p50={result['p50_ms']} p95={result['p95_ms']} rps={result['rps']}", flush=True)
    return result


def rate_limit_probe() -> dict:
    url = "http://127.0.0.1:8002/api/v1/plans"
    got_429_at = None
    codes: dict[str, int] = {}
    with httpx.Client(timeout=TIMEOUT) as client:
        for i in range(1, 121):
            r = client.get(url)
            codes[str(r.status_code)] = codes.get(str(r.status_code), 0) + 1
            if r.status_code == 429 and got_429_at is None:
                got_429_at = i
                break
    print(f"[rate] first_429={got_429_at} codes={codes}", flush=True)
    return {"url": url, "codes": codes, "first_429_at": got_429_at, "sent": sum(codes.values())}


def xff_probe() -> dict:
    url = "http://127.0.0.1:8002/api/v1/plans"
    codes: dict[str, int] = {}
    with httpx.Client(timeout=TIMEOUT) as client:
        for i in range(40):
            r = client.get(url, headers={"X-Forwarded-For": f"203.0.113.{i + 1}"})
            codes[str(r.status_code)] = codes.get(str(r.status_code), 0) + 1
    print(f"[xff] codes={codes}", flush=True)
    return {"url": url, "unique_xff": 40, "codes": codes}


def gpu_snap() -> str:
    return os.popen("nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader").read().strip()


def execution_probe() -> dict:
    env_path = Path(r"C:\Users\ROG\Animal_detection2\agentAndRag\.env")
    creds: dict[str, str] = {}
    for line in env_path.read_text(encoding="utf-8").splitlines():
        if "=" in line and not line.startswith("#"):
            k, v = line.split("=", 1)
            creds[k.strip()] = v
    result: dict = {"gpu_before": gpu_snap(), "runs": []}
    with httpx.Client(timeout=httpx.Timeout(20.0, read=70.0)) as client:
        login = client.post(
            "http://127.0.0.1:8002/api/v1/auth/login",
            json={"email": creds["PETMIND_LOCAL_ADMIN_EMAIL"], "password": creds["PETMIND_LOCAL_ADMIN_PASSWORD"]},
        )
        result["login_status"] = login.status_code
        if login.status_code != 200:
            result["login_body"] = login.text[:200]
            return result
        token = login.json()["access_token"]
        headers = {"Authorization": f"Bearer {token}"}

        def one_run(label: str) -> dict:
            conv = client.post(
                "http://127.0.0.1:8002/api/v1/conversations",
                headers={**headers, "Idempotency-Key": f"load-{label}-{time.time_ns()}"},
                json={"title": f"load {label}"},
            )
            info = {"label": label, "conv_status": conv.status_code}
            if conv.status_code not in {200, 201}:
                info["conv_body"] = conv.text[:240]
                return info
            cid = conv.json()["id"]
            t0 = time.perf_counter()
            try:
                with client.stream(
                    "POST",
                    f"http://127.0.0.1:8002/api/v1/conversations/{cid}/runs",
                    headers={
                        **headers,
                        "Idempotency-Key": f"run-{label}-{time.time_ns()}",
                        "Last-Event-ID": "0",
                    },
                    json={
                        "message": "一句话：猫尿血要不要急诊？",
                        "client_message_id": f"cmi-{label}-{time.time_ns()}",
                        "delivery": "sse",
                        "user_role": "veterinarian",
                    },
                    timeout=70.0,
                ) as resp:
                    info["run_status"] = resp.status_code
                    info["run_id"] = resp.headers.get("x-petmind-run-id")
                    events = 0
                    first_event_ms = None
                    terminal = None
                    if resp.status_code == 200:
                        for line in resp.iter_lines():
                            if not line:
                                continue
                            events += 1
                            if first_event_ms is None:
                                first_event_ms = (time.perf_counter() - t0) * 1000
                            if line.startswith("event:"):
                                ev = line.split(":", 1)[1].strip()
                                if ev in {"completed", "failed", "cancelled"}:
                                    terminal = ev
                                    break
                            if (time.perf_counter() - t0) > 65:
                                terminal = "timeout_cut"
                                break
                    info["events"] = events
                    info["first_event_ms"] = round(first_event_ms or 0, 1)
                    info["terminal"] = terminal
            except Exception as exc:  # noqa: BLE001
                info["error"] = type(exc).__name__
                info["error_msg"] = str(exc)[:200]
            info["elapsed_ms"] = round((time.perf_counter() - t0) * 1000, 1)
            try:
                ready = client.get("http://127.0.0.1:8002/ready", timeout=15.0)
                info["gateway_ready"] = ready.status_code
            except Exception as exc:  # noqa: BLE001
                info["gateway_ready"] = type(exc).__name__
            print(f"[exec] {info}", flush=True)
            return info

        with ThreadPoolExecutor(max_workers=2) as pool:
            futs = [pool.submit(one_run, "a"), pool.submit(one_run, "b")]
            result["runs"] = [f.result() for f in futs]
    result["gpu_after"] = gpu_snap()
    return result


def load_existing() -> dict:
    if OUT.exists():
        try:
            return json.loads(OUT.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return {}
    return {}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["control", "execution", "all"], default="all")
    args = parser.parse_args()
    report = load_existing()
    if args.mode in {"control", "all"}:
        report["control_plane"] = []
        for n in (10, 50, 100, 200):
            hz = wave("http://127.0.0.1:5173/healthz", n, expected={200})
            report["control_plane"].append(hz)
            report["control_plane"].append(wave("http://127.0.0.1:8002/health", n, expected={200}))
            report["control_plane"].append(wave("http://127.0.0.1:8002/api/v1/me", n, expected={401}))
            report["control_plane"].append(wave("http://127.0.0.1:5173/api/v1/plans", n, expected={200, 429}))
            if hz["error_rate"] >= 0.2:
                print("[stop] healthz error rate high", flush=True)
                break
        report["rate_limit"] = rate_limit_probe()
        report["xff"] = xff_probe()
    if args.mode in {"execution", "all"}:
        report["execution"] = execution_probe()
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("wrote", OUT, flush=True)


if __name__ == "__main__":
    main()
