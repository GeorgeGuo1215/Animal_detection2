"""兼容模块别名；MoE 编排器已迁移到 services.moe.orchestration。"""
import sys
from .orchestration import service as _implementation
sys.modules[__name__] = _implementation
