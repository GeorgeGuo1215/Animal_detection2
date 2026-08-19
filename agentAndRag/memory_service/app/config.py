"""记忆服务配置。

风格对齐 mcp_servers/vitals_alert/config.py：dataclass + os.getenv，空串视为未设置，
不引入 pydantic-settings。所有变量统一 MEMORY_ 前缀；LLM 相关允许回退到仓库里已有的
OPENAI_* 变量，免得同一套凭据配两遍。
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field

from .heat import DEFAULT_HEAT_THRESHOLD, HeatParams

logger = logging.getLogger(__name__)

DEFAULT_DSN = "postgresql://postgres:postgres@127.0.0.1:5432/petmemory_dev"
DEFAULT_EMBEDDING_MODEL = "intfloat/multilingual-e5-small"
DEFAULT_EMBEDDING_DIM = 384
DEFAULT_LLM_MODEL = "deepseek-chat"
DEFAULT_LLM_BASE_URL = "https://api.deepseek.com/v1"

# 话题段合并阈值，判据是 topic_score = 语义余弦 + keyword_weight * 关键词 Jaccard。
#
# 纯语义区分不了话题：e5 给中文短文本的余弦基线极高，真实模型生成的段摘要之间，
# 跨话题余弦 P50 就有 0.96、同话题 0.99，两者几乎重叠，靠关键词 Jaccard 才拉得开。
#
# 1.2 由 scripts/calibrate_similarity.py --real-llm 标定，语料是真实模型生成的段摘要
# ——与线上实际参与比较的对象同分布。这一点是踩坑换来的：上一版 1.1 拿手写对话原文
# 标定，而摘要的句式远比原文雷同，余弦基线高出一截，结果 200 轮真实跑把 18 个话题段
# 挤成了 4 个，出现"摘要写着鸡肉过敏、段里装着 33 页体重与喂养"的失真。
#
# 权衡表显示 1.00~1.15 是平台期（误合并率恒为 12%），1.2 是拐点（降到 3.7%），代价
# 是同话题拆散率升到 28%。选它是因为两类错误代价不对称：段被拆散只是多比几个向量，
# 页级检索照样召回；糅在一起则会让段摘要失去指向性，汰换时把错误概括沉淀进长期知识。
#
# 换 embedding 模型或改摘要提示词都必须重跑标定，两者都会移动这个分布。
DEFAULT_SEGMENT_SIMILARITY_THRESHOLD = 1.2


def _env(name: str, default: str) -> str:
    """读环境变量，空串按未设置处理。

    容器编排与 .env 常把未赋值的变量渲染成空串，若不这样处理会拿着空 DSN 去连库。
    """
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip()
    return value if value else default


def _env_int(name: str, default: int) -> int:
    raw = _env(name, "")
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        logger.warning("%s=%r is not an integer; falling back to %s", name, raw, default)
        return default


def _env_float(name: str, default: float) -> float:
    raw = _env(name, "")
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        logger.warning("%s=%r is not a number; falling back to %s", name, raw, default)
        return default


def _positive_int(name: str, default: int) -> int:
    value = _env_int(name, default)
    if value < 1:
        logger.warning("%s=%s must be >= 1; falling back to %s", name, value, default)
        return default
    return value


@dataclass(frozen=True)
class MemoryConfig:
    # --- 数据库 ---
    dsn: str
    connect_timeout: int
    pool_min: int
    pool_max: int

    # --- 向量化 ---
    embedding_model: str
    embedding_device: str
    embedding_dim: int

    # --- LLM ---
    llm_base_url: str
    llm_api_key: str
    llm_model: str
    llm_timeout: float

    # --- 各层容量 ---
    short_term_capacity: int
    mid_term_capacity: int
    knowledge_capacity: int

    # --- 提升节流 ---
    promotion_batch: int
    analysis_min_pages: int

    # --- 热度 ---
    heat: HeatParams
    heat_threshold: float

    # --- 话题合并与连续性 ---
    segment_similarity_threshold: float
    keyword_weight: float
    continuity_gap_minutes: float
    continuity_similarity: float

    # --- 检索 ---
    top_k_segments: int
    top_k_pages: int
    top_k_knowledge: int
    short_term_context_size: int

    # --- worker ---
    worker_concurrency: int
    worker_poll_interval: float
    task_max_attempts: int
    management_token: str = ""

    def redacted_dsn(self) -> str:
        """给日志用的 DSN，去掉密码。"""
        dsn = self.dsn
        if "@" not in dsn or "//" not in dsn:
            return dsn
        scheme, rest = dsn.split("//", 1)
        credentials, host = rest.rsplit("@", 1)
        user = credentials.split(":", 1)[0]
        return f"{scheme}//{user}:***@{host}"


def load_config() -> MemoryConfig:
    return MemoryConfig(
        dsn=_env("MEMORY_DB_DSN", DEFAULT_DSN),
        connect_timeout=_positive_int("MEMORY_DB_CONNECT_TIMEOUT", 5),
        pool_min=_positive_int("MEMORY_DB_POOL_MIN", 1),
        pool_max=_positive_int("MEMORY_DB_POOL_MAX", 8),
        embedding_model=_env("MEMORY_EMBEDDING_MODEL", DEFAULT_EMBEDDING_MODEL),
        # 默认强制 CPU：这台机器的显卡留给别的项目，记忆服务不该去抢。
        embedding_device=_env("MEMORY_EMBEDDING_DEVICE", "cpu"),
        embedding_dim=_positive_int("MEMORY_EMBEDDING_DIM", DEFAULT_EMBEDDING_DIM),
        llm_base_url=_env("MEMORY_LLM_BASE_URL", _env("OPENAI_BASE_URL", DEFAULT_LLM_BASE_URL)),
        llm_api_key=_env("MEMORY_LLM_API_KEY", _env("OPENAI_API_KEY", "")),
        llm_model=_env("MEMORY_LLM_MODEL", _env("OPENAI_MODEL", DEFAULT_LLM_MODEL)),
        llm_timeout=_env_float("MEMORY_LLM_TIMEOUT", 60.0),
        short_term_capacity=_positive_int("MEMORY_SHORT_TERM_CAPACITY", 10),
        # 200 轮对话（模拟中约两个月）自然稳定在 18 个话题段，原来的 200 意味着
        # 汰换永远不会触发，那条代码路径等同死代码。50 大致对应半年左右的活跃使用，
        # 让汰换成为长期用户的兜底而不是日常路径。scripts/simulate_usage.py 的容量
        # 扫描显示即使压到 5，误杀率仍是 0%——因为驱逐前会把段摘要沉淀为长期知识，
        # 所以这个值偏小的风险远低于偏大。
        mid_term_capacity=_positive_int("MEMORY_MID_TERM_CAPACITY", 50),
        knowledge_capacity=_positive_int("MEMORY_KNOWLEDGE_CAPACITY", 200),
        # 攒够一批再提升。每溢出一条就跑一次提升的话，每轮对话都要付一次摘要
        # LLM 调用；攒到 5 条再跑，同样的对话量只需五分之一的调用，而代价只是
        # 短期队列多留几条尚未归档的对话（它们本来就在检索上下文里）。
        promotion_batch=_positive_int("MEMORY_PROMOTION_BATCH", 5),
        # 段内攒够这么多条未分析的新对话，才值得重新做一次画像与知识抽取。
        #
        # 这个门槛曾经设成 5，用意是省 LLM 调用，结果把最该记住的信息挡在了门外：
        # 话题段做细之后，"去年打疫苗过敏"这种一次性的关键陈述会独占一个只有 1 页
        # 的段，热度明明达标却永远凑不满 5 页，于是全库 37% 的对话从未被分析，
        # 过敏史一条都没沉淀进长期知识。
        #
        # 宠物医疗场景里，低频恰恰是重要信息的特征——用户只会提一次过敏史，但下次
        # 开药就得靠它。所以这里选择不省这笔钱：有新对话就分析。
        analysis_min_pages=_positive_int("MEMORY_ANALYSIS_MIN_PAGES", 1),
        heat=HeatParams(
            alpha=_env_float("MEMORY_HEAT_ALPHA", HeatParams.alpha),
            beta=_env_float("MEMORY_HEAT_BETA", HeatParams.beta),
            gamma=_env_float("MEMORY_HEAT_GAMMA", HeatParams.gamma),
            tau_hours=_env_float("MEMORY_HEAT_TAU_HOURS", HeatParams.tau_hours),
        ),
        heat_threshold=_env_float("MEMORY_HEAT_THRESHOLD", DEFAULT_HEAT_THRESHOLD),
        segment_similarity_threshold=_env_float(
            "MEMORY_SEGMENT_SIMILARITY_THRESHOLD", DEFAULT_SEGMENT_SIMILARITY_THRESHOLD
        ),
        keyword_weight=_env_float("MEMORY_KEYWORD_WEIGHT", 1.0),
        continuity_gap_minutes=_env_float("MEMORY_CONTINUITY_GAP_MINUTES", 30.0),
        continuity_similarity=_env_float("MEMORY_CONTINUITY_SIMILARITY", 0.5),
        top_k_segments=_positive_int("MEMORY_TOP_K_SEGMENTS", 5),
        top_k_pages=_positive_int("MEMORY_TOP_K_PAGES", 7),
        top_k_knowledge=_positive_int("MEMORY_TOP_K_KNOWLEDGE", 5),
        short_term_context_size=_positive_int("MEMORY_SHORT_TERM_CONTEXT_SIZE", 6),
        worker_concurrency=_positive_int("MEMORY_WORKER_CONCURRENCY", 2),
        worker_poll_interval=_env_float("MEMORY_WORKER_POLL_INTERVAL", 1.0),
        task_max_attempts=_positive_int("MEMORY_TASK_MAX_ATTEMPTS", 3),
        management_token=_env("MEMORY_MANAGEMENT_TOKEN", ""),
    )
