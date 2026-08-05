"""MoE 实时运行脚本：直接预加载真实 RAG 向量库 + 调用真实 LLM API。

与 run_moe_eval.py 的区别：本脚本会像服务端启动那样**先 warmup 加载真实索引**
（numpy 向量库 + 嵌入模型 + BM25 + 可选重排器），再跑 MoE，可流式打印到控制台，
也支持交互式多轮提问，并可选导出 markdown 报告。

用法：
    # 单次提问（流式打印）
    python agentAndRag/agent_api/tests/moe/run_moe_live.py -q "我家3岁布偶猫这两天软便要紧吗？"

    # 交互式 REPL（不带 -q）
    python agentAndRag/agent_api/tests/moe/run_moe_live.py --softmax-temp 2.5

    # 同时导出报告（用非流式 run() 以记录完整 trace）
    python agentAndRag/agent_api/tests/moe/run_moe_live.py -q "猫能吃布洛芬吗" --report

环境变量（与服务端一致）：OPENAI_API_KEY / DEEPSEEK_API_KEY、可选 OPENAI_BASE_URL、OPENAI_MODEL、
AGENT_WARMUP_DEVICE、AGENT_EMBEDDING_MODEL_PATH / AGENT_RERANKER_MODEL_PATH。脚本会读取 agentAndRag/.env。
"""
from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
from datetime import datetime
from pathlib import Path

_THIS = Path(__file__).resolve()
_AGENT_API = _THIS.parents[2]      # agentAndRag/agent_api  (import app)
_AGENTANDRAG = _THIS.parents[3]    # agentAndRag           (import RAG / mcp_servers; models/, RAG/)
for _p in (str(_AGENT_API), str(_AGENTANDRAG)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


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


_load_dotenv()

from app.services.moe import MoEOrchestrator, OrchestratorConfig, RouterConfig, MoETrace  # noqa: E402
from app.tools.tool_registry import get_registry  # noqa: E402
from app.tools.tools_builtin import register_builtin_tools, register_debug_tools  # noqa: E402
from app.tools.rag_tools import warmup_rag_cache  # noqa: E402
from app.hf_local_model import resolve_embedding_model_id, resolve_rerank_model_id, is_local_path  # noqa: E402

from report_writer import render  # noqa: E402


def _slugify(text: str, max_len: int = 30) -> str:
    s = re.sub(r"\s+", "_", (text or "").strip())
    s = re.sub(r"[^\w\u4e00-\u9fff]+", "", s)
    return s[:max_len] or "moe"


def _check_api_key() -> bool:
    return bool(os.getenv("OPENAI_API_KEY") or os.getenv("DEEPSEEK_API_KEY"))


def _register_tools(with_mcp: bool) -> None:
    reg = get_registry()
    if reg.get("rag.search") is None:
        register_builtin_tools(reg)
        register_debug_tools(reg)
        if with_mcp:
            from app.tools.tools_mcp import register_mcp_tools
            register_mcp_tools(reg)


def _warmup_rag(*, device, enable_bm25: bool, enable_reranker: bool) -> None:
    from RAG.simple_rag.config import default_config

    cfg = default_config(_AGENTANDRAG)
    embedding_model = resolve_embedding_model_id(None, _AGENTANDRAG)
    rerank_model = resolve_rerank_model_id(None, _AGENTANDRAG)

    if enable_reranker and not is_local_path(rerank_model) and os.getenv("HF_HUB_OFFLINE", "").strip() in ("1", "true", "yes"):
        print("[warmup] 离线模式且重排模型非本地路径，禁用重排器预热。")
        enable_reranker = False

    print(f"[warmup] 索引目录: {cfg.index_dir}")
    if not Path(cfg.index_dir).exists():
        print(f"[warmup][警告] 索引目录不存在，rag.search 可能无结果。请先用 rag.reindex 构建索引。")

    stats = warmup_rag_cache(
        index_dir=cfg.index_dir,
        embedding_model=embedding_model,
        device=device,
        enable_bm25=enable_bm25,
        enable_reranker=enable_reranker,
        rerank_model=rerank_model,
    )
    try:
        import torch
        if device and device != "cpu" and torch.cuda.is_available():
            dev_str = f"{device} ({torch.cuda.get_device_name(0)})"
        else:
            dev_str = device or "cpu"
    except ImportError:
        dev_str = device or "cpu"
    print(f"[warmup] 设备: {dev_str}")
    print(f"[warmup] 嵌入模型: {embedding_model}")
    print(f"[warmup] 重排器: {rerank_model if enable_reranker else 'disabled'}")
    print(f"[warmup] BM25: {'enabled' if enable_bm25 else 'disabled'}")
    print(f"[warmup] 索引规模: {stats['index_size']} chunks\n")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MoE 实时运行：预加载真实 RAG 向量库 + 真实 API")
    p.add_argument("--question", "-q", default=None, help="单次提问；不提供则进入交互式 REPL")
    p.add_argument("--user-role", default="pet_owner", choices=["pet_owner", "veterinarian"])
    p.add_argument("--gating-threshold", type=float, default=None)
    p.add_argument("--max-experts", type=int, default=None)
    p.add_argument("--min-relevance", type=float, default=None)
    p.add_argument("--softmax-temp", type=float, default=None)
    p.add_argument("--rag-top-k", type=int, default=5)
    p.add_argument("--temperature", type=float, default=0.3)
    p.add_argument("--max-tokens", type=int, default=900)
    p.add_argument("--device", default=os.getenv("AGENT_WARMUP_DEVICE") or None, help="推理设备，如 cuda / cpu")
    p.add_argument("--no-bm25", action="store_true", help="warmup 不加载 BM25")
    p.add_argument("--no-reranker", action="store_true", help="warmup 不加载重排器")
    p.add_argument("--no-warmup", action="store_true", help="跳过 warmup（首个查询时懒加载）")
    p.add_argument("--with-mcp", action="store_true", help="注册 MCP 工具")
    p.add_argument("--report", action="store_true", help="用非流式 run() 跑并导出 markdown 报告")
    p.add_argument("--out-dir", default=str(_THIS.parent / "reports"))
    return p.parse_args()


def _build_config(args: argparse.Namespace) -> OrchestratorConfig:
    router_cfg = RouterConfig()
    if args.gating_threshold is not None:
        router_cfg.gating_threshold = args.gating_threshold
    if args.max_experts is not None:
        router_cfg.max_experts = args.max_experts
    if args.min_relevance is not None:
        router_cfg.min_relevance = args.min_relevance
    if args.softmax_temp is not None:
        router_cfg.softmax_temp = args.softmax_temp
    return OrchestratorConfig(
        router=router_cfg,
        rag_top_k=args.rag_top_k,
        temperature=args.temperature,
        max_tokens=args.max_tokens,
        user_role=args.user_role,
        device=args.device,
    )


def _config_snapshot(cfg: OrchestratorConfig) -> dict:
    return {
        "user_role": cfg.user_role,
        "gating_threshold": cfg.router.gating_threshold,
        "max_experts": cfg.router.max_experts,
        "min_relevance": cfg.router.min_relevance,
        "softmax_temp": cfg.router.softmax_temp,
        "rag_top_k": cfg.rag_top_k,
        "temperature": cfg.temperature,
        "max_tokens": cfg.max_tokens,
        "device": cfg.device or "cpu",
        "llm_model": os.getenv("OPENAI_MODEL") or os.getenv("DEEPSEEK_MODEL") or "deepseek-chat",
        "llm_base_url": os.getenv("OPENAI_BASE_URL") or "https://api.deepseek.com",
    }


async def _stream_to_console(orch: MoEOrchestrator, question: str, user_role: str) -> None:
    print(f"\n=== 提问 ===\n{question}\n\n=== MoE 进度 / 回答 ===")
    async for ev in orch.stream(query=question):
        content = ev.get("content") or ""
        if content:
            print(content, end="", flush=True)
        else:
            status = ev.get("status")
            detail = ev.get("detail") or {}
            if status and detail:
                print(f"\n  · [{status}] {detail}", flush=True)
    print("\n")


async def _run_with_report(orch: MoEOrchestrator, args: argparse.Namespace, cfg: OrchestratorConfig) -> Path:
    trace = MoETrace(question=args.question, user_role=args.user_role, config=_config_snapshot(cfg))
    answer, _ = await orch.run(query=args.question, recorder=trace)
    print(f"\n=== 最终答案 ===\n{answer}\n")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"{ts}_{_slugify(args.question)}.md"
    out_path.write_text(render(trace), encoding="utf-8")
    return out_path


async def _repl(orch: MoEOrchestrator, user_role: str) -> None:
    print("进入交互式 MoE（输入空行或 exit/quit 退出）")
    loop = asyncio.get_event_loop()
    while True:
        try:
            q = await loop.run_in_executor(None, lambda: input("\n你> ").strip())
        except (EOFError, KeyboardInterrupt):
            print("\n再见。")
            return
        if not q or q.lower() in ("exit", "quit"):
            print("再见。")
            return
        await _stream_to_console(orch, q, user_role)


async def _amain(args: argparse.Namespace) -> None:
    if not _check_api_key():
        print("[错误] 未检测到 OPENAI_API_KEY / DEEPSEEK_API_KEY，请设置后重试（或写入 agentAndRag/.env）。")
        return

    _register_tools(with_mcp=args.with_mcp)

    if not args.no_warmup:
        try:
            _warmup_rag(device=args.device, enable_bm25=not args.no_bm25, enable_reranker=not args.no_reranker)
        except Exception as exc:  # noqa: BLE001
            print(f"[warmup] 预热失败（将懒加载继续）：{exc}\n")

    cfg = _build_config(args)
    orch = MoEOrchestrator(registry=get_registry(), config=cfg)

    if args.question:
        if args.report:
            out_path = await _run_with_report(orch, args, cfg)
            print(f"[OK] 报告已生成: {out_path}")
        else:
            await _stream_to_console(orch, args.question, args.user_role)
    else:
        await _repl(orch, args.user_role)


def main() -> None:
    args = parse_args()
    asyncio.run(_amain(args))


if __name__ == "__main__":
    main()
