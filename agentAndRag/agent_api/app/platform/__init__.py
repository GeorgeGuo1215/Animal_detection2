"""PetMind 兽医 Agent 生产级 Web 平台的基础组件。"""

from .config import PlatformSettings, get_platform_settings
from .database import close_platform_database, init_platform_database

__all__ = [
    "PlatformSettings",
    "close_platform_database",
    "get_platform_settings",
    "init_platform_database",
]
