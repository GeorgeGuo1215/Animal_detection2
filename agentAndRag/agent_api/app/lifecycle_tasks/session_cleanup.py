"""兼容模块别名；Chat-MoE 清理任务已迁移到 features.chat_moe。"""

import sys

from ..features.chat_moe import cleanup as _implementation

sys.modules[__name__] = _implementation
