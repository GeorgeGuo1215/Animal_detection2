"""兼容模块别名；OpenAI 配置已迁移到 integrations.llm.config。"""
import sys
from ..integrations.llm import config as _implementation
sys.modules[__name__] = _implementation
