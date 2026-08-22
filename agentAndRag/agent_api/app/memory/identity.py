"""兼容模块别名；Chat-MoE 测试身份映射已迁移到 features.chat_moe。"""

import sys

from ..features.chat_moe import identity as _implementation

sys.modules[__name__] = _implementation
