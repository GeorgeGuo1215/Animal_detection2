"""LLM 配置、协议解析和统一聊天客户端。"""

from .client import OpenAIChatClient, get_shared_async_client

__all__ = ["OpenAIChatClient", "get_shared_async_client"]
