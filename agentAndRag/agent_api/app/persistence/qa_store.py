"""兼容模块别名；QA 审计存储已迁移到 features.qa_audit。"""

import sys

from ..features.qa_audit import repository as _implementation

sys.modules[__name__] = _implementation
