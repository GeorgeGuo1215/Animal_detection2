"""Production web-platform building blocks for the PetMind veterinary Agent."""

from .config import PlatformSettings, get_platform_settings
from .database import close_platform_database, init_platform_database

__all__ = [
    "PlatformSettings",
    "close_platform_database",
    "get_platform_settings",
    "init_platform_database",
]
