"""Live API 测试编排：启动真实后端，仅通过 /v1/chat/completions 验证 agent 全功能。

本脚本会：
  1. 用 uvicorn 拉起 agentAndRag 后端（真实 RAG warmup + MCP + MySQL），日志落盘；
  2. 只打 /v1/chat/completions（SSE 流式）端点，覆盖 P1 各项能力：
     - MoE 默认路径的全工具覆盖（rag.search / sql.search / vitals.summary /
       mcp.web_search.* / mcp.nutritional_planner.*）；
     - sql.search 多表 + 物种软过滤（带 animal_id=cat_001 vs 不带）；
     - 越界拒答；
     - 与原 multi-turn / plan 模式同题对比；
  3. 并发管理验证：并发打多条请求，借助连接池 [POOL] 调试打印观测在途连接峰值；
  4. 产出 markdown 报告（tests/api_live/reports/）。

用法：
    # 全量（会拉起后端、消耗少量 LLM 额度）
    python agentAndRag/agent_api/tests/api_live/run_api_live.py

    # 复用已在跑的后端（不自动启动），需自行保证 AGENT_DISABLE_AUTH=1 / AGENT_POOL_DEBUG=1
    python agentAndRag/agent_api/tests/api_live/run_api_live.py --base http://127.0.0.1:8000 --no-spawn

    # 快速子集
    python agentAndRag/agent_api/tests/api_live/run_api_live.py --quick

依赖：httpx、pymysql；后端依赖 OPENAI_API_KEY/DEEPSEEK_API_KEY（chat）与 TAVILY_API_KEY（web_search）。
注意：脚本在 llm 连接池处临时加了 [POOL] 调试打印（AGENT_POOL_DEBUG=1 才生效），测试后请回滚。
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import httpx

_THIS = Path(__file__).resolve()
_AGENT_API = _THIS.parents[2]      # agentAndRag/agent_api
_AGENTANDRAG = _THIS.parents[3]    # agentAndRag
_REPORTS = _THIS.parent / "reports"

ALL_TOOLS = [
    "rag.search",
    "sql.search",
    "vitals.summary",
    "mcp.web_search.web_search",
    "mcp.web_search.ingredient_check",
    "mcp.nutritional_planner.calculate_meal_plan",
    "mcp.nutritional_planner.generate_exercise_plan",
]

# 已知测试宠物（来自 petmind 库实测）
CAT_ID = "cat_001"   # species=cat → 物种软过滤注入“猫”
DOG_ID = "dog_001"   # vitals 样本最丰富


def _load_dotenv() -> None:
    env_path = _AGENTANDRAG / ".env"
    if not env_path.exists():
        return
    try:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k, v = k.strip(), v.strip().strip('"').strip("'")
            if k and k not in os.environ:
                os.environ[k] = v
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- DB ground truth
def _db_species(animal_id: str) -> Optional[str]:
    try:
        import pymysql
        from pymysql.cursors import DictCursor
        conn = pymysql.connect(
            host=os.getenv("PETMIND_MYSQL_HOST", "127.0.0.1"),
            port=int(os.getenv("PETMIND_MYSQL_PORT", "3306")),
            user=os.getenv("PETMIND_MYSQL_USER", "root"),
            password=os.getenv("PETMIND_MYSQL_PASSWORD", ""),
            database=os.getenv("PETMIND_MYSQL_DATABASE", "petmind"),
            charset="utf8mb4", cursorclass=DictCursor,
        )
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT species FROM animals WHERE animal_id=%s LIMIT 1", [animal_id])
                row = cur.fetchone()
                return (row or {}).get("species")
        finally:
            conn.close()
    except Exception as exc:  # noqa: BLE001
        return f"<db error: {exc}>"


# --------------------------------------------------------------------------- SSE result
@dataclass
class ChatResult:
    case_id: str
    title: str
    model: str
    animal_id: Optional[str]
    question: str
    answer: str = ""
    statuses: List[str] = field(default_factory=list)
    details: List[Dict[str, Any]] = field(default_factory=list)
    tools: List[str] = field(default_factory=list)
    out_of_scope: bool = False
    chunks: int = 0
    elapsed_ms: float = 0.0
    http_status: int = 0
    error: str = ""

    def answer_excerpt(self, n: int = 600) -> str:
        a = (self.answer or "").strip()
        return a[:n] + ("…" if len(a) > n else "")


async def call_chat(
    client: httpx.AsyncClient,
    *,
    base: str,
    case_id: str,
    title: str,
    model: str,
    question: str,
    animal_id: Optional[str] = None,
    user_role: str = "pet_owner",
    max_tokens: int = 700,
    temperature: float = 0.3,
    api_key: str = "test-key",
) -> ChatResult:
    res = ChatResult(case_id=case_id, title=title, model=model, animal_id=animal_id, question=question)
    payload: Dict[str, Any] = {
        "model": model,
        "messages": [{"role": "user", "content": question}],
        "stream": True,
        "temperature": temperature,
        "max_tokens": max_tokens,
        "user_role": user_role,
        "debug_timing": False,
    }
    if animal_id:
        payload["animal_id"] = animal_id
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json", "Accept": "text/event-stream"}

    tools: set[str] = set()
    t0 = time.perf_counter()
    try:
        async with client.stream("POST", f"{base}/v1/chat/completions", json=payload, headers=headers) as resp:
            res.http_status = resp.status_code
            if resp.status_code != 200:
                body = (await resp.aread()).decode("utf-8", "replace")
                res.error = f"HTTP {resp.status_code}: {body[:300]}"
                return res
            async for line in resp.aiter_lines():
                if not line or not line.startswith("data: "):
                    continue
                data = line[6:]
                if data == "[DONE]":
                    break
                try:
                    chunk = json.loads(data)
                except json.JSONDecodeError:
                    continue
                res.chunks += 1
                status = chunk.get("agent_status")
                if status:
                    res.statuses.append(status)
                detail = chunk.get("agent_detail")
                if isinstance(detail, dict):
                    res.details.append(detail)
                    if detail.get("tool_name"):
                        tools.add(str(detail["tool_name"]))
                    tu = detail.get("tools_used")
                    if isinstance(tu, list):
                        tools.update(str(x) for x in tu)
                    if detail.get("out_of_scope") is True:
                        res.out_of_scope = True
                try:
                    delta = chunk.get("choices", [{}])[0].get("delta", {})
                    if status == "streaming" and delta.get("content"):
                        res.answer += delta["content"]
                except Exception:  # noqa: BLE001
                    pass
    except Exception as exc:  # noqa: BLE001
        res.error = f"{type(exc).__name__}: {exc}"
    res.elapsed_ms = round((time.perf_counter() - t0) * 1000, 1)
    res.tools = sorted(tools)
    return res


# --------------------------------------------------------------------------- server lifecycle
def _port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def spawn_server(port: int, log_path: Path) -> subprocess.Popen:
    env = dict(os.environ)
    env["AGENT_DISABLE_AUTH"] = "1"        # 测试聚焦功能，跳过鉴权
    env["AGENT_POOL_DEBUG"] = "1"          # 启用连接池 [POOL] 打印
    env["AGENT_RATE_LIMIT"] = "100000"     # 避免并发测试被限流
    env["AGENT_RATE_BURST"] = "100000"
    env.setdefault("AGENT_ENABLE_MCP", "1")
    env.setdefault("AGENT_WARMUP_DEVICE", "cuda")  # 与 start_agent.bat 一致；CPU 上 reranker 过慢
    env.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    env.setdefault("PYTHONUNBUFFERED", "1")

    log_f = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "agent_api.app.main:app", "--host", "127.0.0.1", "--port", str(port)],
        cwd=str(_AGENTANDRAG),
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
    )
    return proc


def wait_health(base: str, timeout_s: float = 300.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            # Prefer /ready (flips true only after RAG warmup); fall back to
            # /health for older builds without the readiness probe.
            r = httpx.get(f"{base}/ready", timeout=3.0, trust_env=False)
            if r.status_code == 200 and r.json().get("ready"):
                return True
            if r.status_code == 404:
                h = httpx.get(f"{base}/health", timeout=3.0, trust_env=False)
                if h.status_code == 200 and h.json().get("ok"):
                    return True
        except Exception:  # noqa: BLE001
            pass
        time.sleep(2.0)
    return False


def stop_server(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=10)
    except Exception:  # noqa: BLE001
        try:
            proc.kill()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- cases
def build_cases(quick: bool) -> List[Dict[str, Any]]:
    """每个 case：通过精心构造的兽医问题触发特定工具/能力。expect_tools 为软期望。"""
    cases: List[Dict[str, Any]] = [
        {
            "id": "moe_rag", "title": "MoE·知识检索 (rag.search)", "model": "agent-moe",
            "q": "犬细小病毒的典型症状有哪些？家庭护理和何时必须就医？",
            "animal_id": None, "expect_tools": ["rag.search"],
            "expect_status": ["routing", "generating"],
        },
        {
            "id": "moe_sql", "title": "MoE·结构化日报 (sql.search 多表)", "model": "agent-moe",
            "q": "请结合它最近几天的健康日报，总结一下我家宠物的整体风险状况。",
            "animal_id": CAT_ID, "expect_tools": ["sql.search"],
            "expect_status": ["expert_calling"],
        },
        {
            "id": "moe_vitals", "title": "MoE·生理趋势 (vitals.summary)", "model": "agent-moe",
            "q": "我家宠物最近的心率和呼吸频率在正常范围吗？有没有需要警惕的异常？",
            "animal_id": DOG_ID, "expect_tools": ["vitals.summary"],
            "expect_status": ["expert_calling"],
        },
        {
            "id": "moe_nutrition", "title": "MoE·营养/运动计划 (nutritional_planner)", "model": "agent-moe",
            "q": "我家成年宠物体重约29公斤，请帮我制定一份每日喂食热量和运动计划。",
            "animal_id": CAT_ID, "expect_tools": ["mcp.nutritional_planner"],
            "expect_status": ["expert_calling"],
        },
        {
            "id": "moe_ingredient", "title": "MoE·成分安全 (ingredient_check/web_search)", "model": "agent-moe",
            "q": "一款宠物零食成分含木糖醇和洋葱粉，给我家宠物吃安全吗？请核查成分风险。",
            "animal_id": CAT_ID, "expect_tools": ["mcp.web_search"],
            "expect_status": ["expert_calling"],
        },
        {
            "id": "moe_websearch", "title": "MoE·实时检索 (web_search)", "model": "agent-moe",
            "q": "2026年关于犬猫狂犬病疫苗接种的最新建议有哪些更新？请检索最新资料。",
            "animal_id": None, "expect_tools": ["mcp.web_search.web_search"],
            "expect_status": ["routing"],
        },
        {
            "id": "moe_oos", "title": "MoE·越界拒答", "model": "agent-moe",
            "q": "帮我写一段快速排序的 Python 代码并解释时间复杂度。",
            "animal_id": None, "expect_tools": [],
            "expect_refusal": True,
        },
        # 物种软过滤：对乙酰氨基酚对猫剧毒，带 cat_001 应出现“猫”特异性强警告
        # 物种软过滤探针：用“食欲下降/没精神”这类不会触发 Critic 安全兜底、
        # 但物种差异明显（如猫长期厌食→肝脂沉积风险）的问题，比较带/不带 animal_id 的差异。
        {
            "id": "species_cat", "title": "物种软过滤·带 cat_001", "model": "agent-moe",
            "q": "我家宠物这两天没什么精神、也不太爱吃东西，可能是什么问题？要注意什么？",
            "animal_id": CAT_ID, "expect_tools": [], "expect_text": ["猫"],
        },
        {
            "id": "species_none", "title": "物种软过滤·无 animal_id（对照）", "model": "agent-moe",
            "q": "我家宠物这两天没什么精神、也不太爱吃东西，可能是什么问题？要注意什么？",
            "animal_id": None, "expect_tools": [],
        },
        # 模式对比
        {
            "id": "mt_rag", "title": "multi-turn·同题对比 (知识检索)", "model": "agent-multi-turn",
            "q": "犬细小病毒的典型症状有哪些？家庭护理和何时必须就医？",
            "animal_id": None, "expect_tools": ["rag.search"], "expect_status": ["tool_calling"],
        },
        {
            "id": "mt_sql", "title": "multi-turn·同题对比 (日报/物种)", "model": "agent-multi-turn",
            "q": "我的宠物发烧了，能用对乙酰氨基酚退烧吗？另外结合它的健康日报看看风险。",
            "animal_id": CAT_ID, "expect_tools": ["sql.search"], "expect_status": ["tool_calling"],
        },
        {
            "id": "plan_rag", "title": "plan·同题对比 (知识检索)", "model": "agent-plan-solve",
            "q": "犬细小病毒的典型症状有哪些？家庭护理和何时必须就医？",
            "animal_id": None, "expect_tools": ["rag.search"], "expect_status": ["planning"],
        },
    ]
    if quick:
        keep = {"moe_rag", "moe_sql", "moe_vitals", "moe_oos", "species_cat", "species_none", "mt_rag"}
        cases = [c for c in cases if c["id"] in keep]
    return cases


def evaluate(res: ChatResult, case: Dict[str, Any]) -> Tuple[str, List[str]]:
    """返回 (verdict, notes)。verdict ∈ PASS/WARN/FAIL。"""
    notes: List[str] = []
    if res.error:
        return "FAIL", [res.error]

    if case.get("expect_refusal"):
        if res.out_of_scope or "无法回答" in res.answer or "宠物健康助手" in res.answer or "暂时无法" in res.answer:
            return "PASS", ["正确拒答越界问题"]
        return "FAIL", ["未对越界问题拒答", f"answer={res.answer_excerpt(120)!r}"]

    if not res.answer.strip():
        return "FAIL", ["最终答案为空"]

    verdict = "PASS"
    # 状态期望（硬：缺失则 WARN）
    for st in case.get("expect_status", []):
        if st not in res.statuses:
            verdict = "WARN"; notes.append(f"未观测到状态 `{st}`")
    # 工具期望（软：LLM 可能未选用）
    for tk in case.get("expect_tools", []):
        if not any(tk in t for t in res.tools):
            verdict = "WARN"; notes.append(f"未触发期望工具 `{tk}`（LLM 未选用/兜底）")
    # 文本期望（软）
    for tx in case.get("expect_text", []):
        if tx not in res.answer:
            verdict = "WARN"; notes.append(f"答案未包含期望关键词 `{tx}`")
    if not notes:
        notes.append("OK")
    return verdict, notes


# --------------------------------------------------------------------------- concurrency
async def concurrency_probe(client: httpx.AsyncClient, base: str, n: int, api_key: str) -> List[ChatResult]:
    q = "我家宠物有点拉肚子，需要注意什么？什么情况下要去医院？"
    tasks = [
        call_chat(client, base=base, case_id=f"conc_{i}", title=f"并发#{i}", model="agent-moe",
                  question=q, max_tokens=400, api_key=api_key)
        for i in range(n)
    ]
    return list(await asyncio.gather(*tasks))


def parse_pool_peak(log_path: Path) -> Dict[str, int]:
    peaks = {"chat": 0, "stream": 0, "max_inflight": 0}
    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        return peaks
    for m in re.finditer(r"\[POOL\]\[(chat|stream)\] inflight=(\d+) peak=(\d+)", text):
        kind, inflight, peak = m.group(1), int(m.group(2)), int(m.group(3))
        peaks[kind] = max(peaks[kind], peak)
        peaks["max_inflight"] = max(peaks["max_inflight"], inflight)
    return peaks


# --------------------------------------------------------------------------- report
def _md_tools(tools: List[str]) -> str:
    return ", ".join(f"`{t}`" for t in tools) if tools else "_（无）_"


def render_report(
    *, results: List[Tuple[ChatResult, str, List[str]]], coverage: Dict[str, bool],
    species_truth: Dict[str, Any], conc: List[ChatResult], pool: Dict[str, int],
    conc_n: int, base: str,
) -> str:
    out: List[str] = []
    out.append("# Live API 测试报告 — /v1/chat/completions")
    out.append("")
    out.append(f"- 生成时间: {datetime.now().isoformat(timespec='seconds')}")
    out.append(f"- 后端: {base}")
    out.append(f"- 物种 ground truth: cat_001 → `{species_truth.get('cat_001')}`，dog_001 → `{species_truth.get('dog_001')}`")
    out.append("")

    n_pass = sum(1 for _, v, _ in results if v == "PASS")
    n_warn = sum(1 for _, v, _ in results if v == "WARN")
    n_fail = sum(1 for _, v, _ in results if v == "FAIL")
    out.append(f"## 总览：PASS {n_pass} / WARN {n_warn} / FAIL {n_fail}")
    out.append("")
    out.append("| 用例 | 模型 | animal_id | 判定 | 状态序列 | 触发工具 | 耗时(ms) |")
    out.append("| --- | --- | --- | --- | --- | --- | --- |")
    for r, v, _notes in results:
        sts = ",".join(dict.fromkeys(r.statuses))  # 去重保序
        out.append(
            f"| {r.case_id} | {r.model} | {r.animal_id or '-'} | **{v}** | {sts or '-'} | "
            f"{_md_tools(r.tools)} | {r.elapsed_ms} |"
        )
    out.append("")

    # 工具覆盖矩阵
    out.append("## P1 工具覆盖矩阵（跨全部用例的并集）")
    out.append("")
    out.append("| 工具 | 是否被触发 |")
    out.append("| --- | --- |")
    for t in ALL_TOOLS:
        out.append(f"| `{t}` | {'✅' if coverage.get(t) else '❌ 未触发'} |")
    out.append("")

    # 物种软过滤对比
    cat = next((r for r, _, _ in results if r.case_id == "species_cat"), None)
    none = next((r for r, _, _ in results if r.case_id == "species_none"), None)
    if cat and none:
        out.append("## 物种软过滤对比（对乙酰氨基酚 / 扑热息痛）")
        out.append("")
        out.append(f"- 注入物种(cat_001→species): `{species_truth.get('cat_001')}`；带 animal_id 答案是否提及“猫”: "
                   f"**{'是' if '猫' in cat.answer else '否'}**")
        out.append("")
        out.append("**带 cat_001：**")
        out.append("")
        out.append("> " + cat.answer_excerpt(500).replace("\n", "\n> "))
        out.append("")
        out.append("**无 animal_id（对照）：**")
        out.append("")
        out.append("> " + none.answer_excerpt(500).replace("\n", "\n> "))
        out.append("")

    # 并发
    out.append("## 并发管理")
    out.append("")
    out.append(f"- 并发请求数: {conc_n}")
    ok = sum(1 for r in conc if not r.error and r.answer.strip())
    out.append(f"- 成功完成: {ok}/{len(conc)}")
    out.append(f"- 连接池在途峰值: chat={pool.get('chat')}, stream={pool.get('stream')}, "
               f"max_inflight={pool.get('max_inflight')}")
    verdict_conc = "PASS（观测到并发在途>1）" if pool.get("max_inflight", 0) >= 2 else \
                   "WARN（未观测到 inflight>1，可能串行或日志未捕获）"
    out.append(f"- 判定: **{verdict_conc}**")
    out.append("")
    out.append("| 并发用例 | 判定 | 状态序列 | 工具 | 耗时(ms) |")
    out.append("| --- | --- | --- | --- | --- |")
    for r in conc:
        sts = ",".join(dict.fromkeys(r.statuses))
        v = "OK" if (not r.error and r.answer.strip()) else f"ERR {r.error[:60]}"
        out.append(f"| {r.case_id} | {v} | {sts or '-'} | {_md_tools(r.tools)} | {r.elapsed_ms} |")
    out.append("")

    # 明细
    out.append("## 用例明细")
    for r, v, notes in results:
        out.append("")
        out.append(f"### [{v}] {r.case_id} · {r.title}")
        out.append(f"- 模型: `{r.model}` | animal_id: `{r.animal_id or '-'}` | HTTP {r.http_status} | "
                   f"chunks={r.chunks} | {r.elapsed_ms}ms")
        out.append(f"- 触发工具: {_md_tools(r.tools)}")
        out.append(f"- 备注: {'; '.join(notes)}")
        out.append(f"- 问题: {r.question}")
        out.append("")
        out.append("<details><summary>答案</summary>")
        out.append("")
        out.append("```text")
        out.append(r.answer_excerpt(1500) or "(空)")
        out.append("```")
        out.append("</details>")
    out.append("")
    return "\n".join(out)


# --------------------------------------------------------------------------- main
async def _amain(args: argparse.Namespace) -> int:
    _load_dotenv()
    base = args.base.rstrip("/")
    host = "127.0.0.1"
    port = int(base.rsplit(":", 1)[-1]) if base.rsplit(":", 1)[-1].isdigit() else 8000

    _REPORTS.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = _REPORTS / f"server_{ts}.log"

    proc: Optional[subprocess.Popen] = None
    if not args.no_spawn:
        if _port_open(host, port):
            print(f"[boot] 端口 {port} 已被占用；如需复用请加 --no-spawn。中止。")
            return 2
        print(f"[boot] 启动后端 uvicorn :{port}（日志 {log_path.name}）…")
        proc = spawn_server(port, log_path)

    try:
        print("[boot] 等待 /health（含 RAG warmup，最多 300s）…")
        if not wait_health(base, timeout_s=args.health_timeout):
            print("[boot] 后端未就绪，放弃。请查看服务日志：", log_path)
            return 3
        print("[boot] 后端就绪。")

        species_truth = {"cat_001": _db_species("cat_001"), "dog_001": _db_species("dog_001")}
        print(f"[db] species ground truth: {species_truth}")

        cases = build_cases(args.quick)
        results: List[Tuple[ChatResult, str, List[str]]] = []
        async with httpx.AsyncClient(
            timeout=httpx.Timeout(connect=10, read=300, write=10, pool=30), trust_env=False,
        ) as client:
            for case in cases:
                print(f"\n[case] {case['id']} · {case['title']} (model={case['model']}, animal_id={case.get('animal_id')})")
                r = await call_chat(
                    client, base=base, case_id=case["id"], title=case["title"], model=case["model"],
                    question=case["q"], animal_id=case.get("animal_id"),
                    user_role=case.get("user_role", "pet_owner"),
                    max_tokens=args.max_tokens, api_key=args.api_key,
                )
                verdict, notes = evaluate(r, case)
                results.append((r, verdict, notes))
                print(f"  -> {verdict} | tools={r.tools} | statuses={list(dict.fromkeys(r.statuses))}")
                print(f"     notes: {'; '.join(notes)}")

            # 并发
            print(f"\n[concurrency] 并发打 {args.concurrency} 条 MoE 请求…")
            conc = await concurrency_probe(client, base, args.concurrency, args.api_key)
            for r in conc:
                print(f"  conc {r.case_id}: {'OK' if (not r.error and r.answer.strip()) else 'ERR ' + r.error[:80]} "
                      f"({r.elapsed_ms}ms)")

        # 覆盖矩阵
        seen: set[str] = set()
        for r, _, _ in results:
            seen.update(r.tools)
        for r in conc:
            seen.update(r.tools)
        coverage = {t: any(t in s for s in seen) for t in ALL_TOOLS}

        time.sleep(1.0)  # 让服务端 flush 日志
        pool_log = log_path if not args.no_spawn else (Path(args.server_log) if args.server_log else None)
        pool = parse_pool_peak(pool_log) if pool_log else {"chat": 0, "stream": 0, "max_inflight": 0}

        report = render_report(
            results=results, coverage=coverage, species_truth=species_truth,
            conc=conc, pool=pool, conc_n=args.concurrency, base=base,
        )
        report_path = _REPORTS / f"api_live_{ts}.md"
        report_path.write_text(report, encoding="utf-8")

        # 控制台总结
        n_fail = sum(1 for _, v, _ in results if v == "FAIL")
        n_warn = sum(1 for _, v, _ in results if v == "WARN")
        print("\n" + "=" * 60)
        print(f"[done] 报告: {report_path}")
        print(f"[done] 工具覆盖: " + ", ".join(f"{t}={'Y' if coverage[t] else 'N'}" for t in ALL_TOOLS))
        print(f"[done] 连接池在途峰值: {pool}")
        print(f"[done] FAIL={n_fail} WARN={n_warn}")
        return 1 if n_fail else 0
    finally:
        if proc is not None:
            print("[boot] 关闭后端…")
            stop_server(proc)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live API 测试（/v1/chat/completions 全功能 + 并发 + 物种软过滤）")
    p.add_argument("--base", default="http://127.0.0.1:8137", help="后端地址（端口默认 8137 以避开 8000 开发服务）")
    p.add_argument("--no-spawn", action="store_true", help="不自动启动后端（复用 --base 上已运行的服务）")
    p.add_argument("--server-log", default=None, help="--no-spawn 时，外部服务日志路径（用于解析连接池 [POOL] 峰值）")
    p.add_argument("--quick", action="store_true", help="只跑核心子集，省额度")
    p.add_argument("--concurrency", type=int, default=5, help="并发请求数")
    p.add_argument("--max-tokens", type=int, default=700)
    p.add_argument("--health-timeout", type=float, default=300.0)
    p.add_argument("--api-key", default="test-key", help="Bearer key（AGENT_DISABLE_AUTH=1 时任意值均可）")
    return p.parse_args()


def main() -> None:
    sys.exit(asyncio.run(_amain(parse_args())))


if __name__ == "__main__":
    main()
