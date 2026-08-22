"""兼容模块别名；实现已迁移到 integrations.petmind_mysql。"""
import sys
from ..integrations.petmind_mysql import query_compiler as _implementation
sys.modules[__name__] = _implementation
