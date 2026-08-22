"""兼容模块别名；Run 生命周期已迁移到 platform.runs。"""
import sys
from .runs import service as _implementation
sys.modules[__name__] = _implementation
