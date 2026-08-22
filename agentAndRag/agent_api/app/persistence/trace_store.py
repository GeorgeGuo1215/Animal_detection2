"""兼容模块别名；JSONL Trace 已迁移到 observability。"""

import sys

from ..observability import jsonl_trace as _implementation

sys.modules[__name__] = _implementation
