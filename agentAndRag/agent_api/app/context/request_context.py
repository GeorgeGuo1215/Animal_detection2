"""兼容模块别名；工具请求作用域已迁移到 tools.request_scope。"""

import sys

from ..tools import request_scope as _implementation

sys.modules[__name__] = _implementation
