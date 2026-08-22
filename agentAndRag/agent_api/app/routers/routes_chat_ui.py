"""兼容模块别名；Chat-MoE 路由实现已迁移到 features.chat_moe。"""

import sys

from ..features.chat_moe import router as _implementation

sys.modules[__name__] = _implementation
