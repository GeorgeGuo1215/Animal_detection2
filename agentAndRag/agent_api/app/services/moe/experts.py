"""兼容模块别名；专家运行时已迁移到 services.moe.expert_runtime。"""
import sys
from .expert_runtime import service as _implementation
sys.modules[__name__] = _implementation
