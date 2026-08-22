"""兼容模块别名；流式和非流式调用现共用 integrations.llm.client。"""
import sys
from ..integrations.llm import client as _implementation
sys.modules[__name__] = _implementation
