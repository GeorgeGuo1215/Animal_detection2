"""兼容模块别名；LLM 客户端已迁移到 integrations.llm.client。"""
import sys
from ..integrations.llm import client as _implementation
sys.modules[__name__] = _implementation
