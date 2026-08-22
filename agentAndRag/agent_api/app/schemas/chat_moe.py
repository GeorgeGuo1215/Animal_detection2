"""兼容模块别名；Chat-MoE Schema 已迁移到 features.chat_moe。"""

import sys

from ..features.chat_moe import schemas as _implementation

sys.modules[__name__] = _implementation
