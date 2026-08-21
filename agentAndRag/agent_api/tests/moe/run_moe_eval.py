"""MoE 离线评测脚本。

用法（在任意目录）：
    python agentAndRag/agent_api/tests/moe/run_moe_eval.py \
        --question "我家3岁布偶猫这两天一直软便，要紧吗？" \
        --user-role pet_owner --gating-threshold 0.15 --max-experts 3

跑完会在 reports/ 下生成一个 markdown 报告，包含：运行配置、路由决策、
每次 LLM 调用的顺序/输入/输出、各专家意见、RAG 指标、Critic 裁决、最终答案与汇总指标。

依赖环境变量（与服务端一致）：OPENAI_API_KEY 或 DEEPSEEK_API_KEY（可选 OPENAI_BASE_URL / OPENAI_MODEL）。
脚本会尝试读取 agentAndRag/.env 自动注入未设置的变量。
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
_AGENT_API = _THIS.parents[2]          # agentAndRag/agent_api  (for `import app`)
_AGENTANDRAG = _THIS.parents[3]        # agentAndRag           (for `RAG`, `mcp_servers`)
for _p in (str(_AGENT_API), str(_AGENTANDRAG)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_dotenv() -> None:
    """极简 .env 加载：只填充尚未设置的变量，无第三方依赖。"""
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

from report_writer import render  # noqa: E402


def _slugify(text: str, max_len: int = 30) -> str:
    """把标题转成适合做文件名的短名。"""
    s = re.sub(r"\s+", "_", (text or "").strip())
    s = re.sub(r"[^\w\u4e00-\u9fff]+", "", s)
    return s[:max_len] or "moe"


def _ensure_tools(with_mcp: bool) -> None:
    """若注册表还没有 rag.search，则注册内置工具，可选再挂 MCP。"""
    reg = get_registry()
    if reg.get("rag.search") is None:
        register_builtin_tools(reg)
        register_debug_tools(reg)
        if with_mcp:
            from app.tools.tools_mcp import register_mcp_tools
            register_mcp_tools(reg)


def parse_args() -> argparse.Namespace:
    """解析命令行参数并返回配置。"""
    p = argparse.ArgumentParser(description="MoE 离线评测：调超参跑单问题并产出 markdown 报告")
    p.add_argument("--question", "-q", required=True, help="输入问题")
    p.add_argument("--user-role", default="pet_owner", choices=["pet_owner", "veterinarian"])
    p.add_argument("--gating-threshold", type=float, default=None, help="门控阈值 τ（默认读 MOE_GATING_THRESHOLD 或 0.15）")
    p.add_argument("--max-experts", type=int, default=None, help="最大激活专家数（默认 MOE_MAX_EXPERTS 或 3）")
    p.add_argument("--min-relevance", type=float, default=None, help="拒答阈值：最高分低于它则 out_of_scope（默认 3.0）")
    p.add_argument("--softmax-temp", type=float, default=None, help="softmax 温度（默认 1.0）")
    p.add_argument("--rag-top-k", type=int, default=5, help="每个专家 RAG top_k")
    p.add_argument("--temperature", type=float, default=0.3, help="最终生成温度")
    p.add_argument("--max-tokens", type=int, default=900, help="最终生成 max_tokens")
    p.add_argument("--with-mcp", action="store_true", help="注册 MCP 工具（默认不注册，更快）")
    p.add_argument("--no-tools", action="store_true", help="禁用专家工具，用于纯 LLM 链路与低资源评测")
    p.add_argument("--out-dir", default=str(_THIS.parent / "reports"), help="报告输出目录")
    return p.parse_args()


def _build_router_config(args: argparse.Namespace) -> RouterConfig:
    """构造本次评测用的路由配置。"""
    cfg = RouterConfig()
    if args.gating_threshold is not None:
        cfg.gating_threshold = args.gating_threshold
    if args.max_experts is not None:
        cfg.max_experts = args.max_experts
    if args.min_relevance is not None:
        cfg.min_relevance = args.min_relevance
    if args.softmax_temp is not None:
        cfg.softmax_temp = args.softmax_temp
    return cfg


async def _amain(args: argparse.Namespace) -> Path:
    """内部异步主流程。"""
    _ensure_tools(with_mcp=args.with_mcp)

    router_cfg = _build_router_config(args)
    orch = MoEOrchestrator(
        registry=get_registry(),
        config=OrchestratorConfig(
            router=router_cfg,
            rag_top_k=args.rag_top_k,
            temperature=args.temperature,
            max_tokens=args.max_tokens,
            user_role=args.user_role,
            allowed_tools=[] if args.no_tools else None,
        ),
    )

    config_snapshot = {
        "user_role": args.user_role,
        "gating_threshold": router_cfg.gating_threshold,
        "max_experts": router_cfg.max_experts,
        "min_relevance": router_cfg.min_relevance,
        "softmax_temp": router_cfg.softmax_temp,
        "rag_top_k": args.rag_top_k,
        "temperature": args.temperature,
        "max_tokens": args.max_tokens,
        "allowed_tools": "none" if args.no_tools else "service defaults",
        "llm_model": os.getenv("OPENAI_MODEL") or os.getenv("DEEPSEEK_MODEL") or "deepseek-v4-flash",
        "llm_base_url": os.getenv("OPENAI_BASE_URL") or "https://api.deepseek.com",
    }
    trace = MoETrace(question=args.question, user_role=args.user_role, config=config_snapshot)

    await orch.run(query=args.question, recorder=trace)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"{ts}_{_slugify(args.question)}.md"
    out_path.write_text(render(trace), encoding="utf-8")
    return out_path


def main() -> None:
    """脚本入口，解析参数并执行主流程。"""
    args = parse_args()
    try:
        out_path = asyncio.run(_amain(args))
    except KeyboardInterrupt:
        print("中断。")
        return
    print(f"[OK] 报告已生成: {out_path}")
    print(f"     总 LLM 调用、专家意见、Critic 裁决与指标见报告。")


if __name__ == "__main__":
    main()
