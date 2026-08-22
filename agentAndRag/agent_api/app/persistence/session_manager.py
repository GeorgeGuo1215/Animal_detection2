"""兼容模块别名；Chat-MoE SQLite Session 已迁移到 features.chat_moe。"""

import sys

from ..features.chat_moe import session_store as _implementation

sys.modules[__name__] = _implementation
