from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class MysqlConfig:
    host: str
    port: int
    user: str
    password: str
    database: str
    max_limit: int
    debug_sql: bool
    pool_size: int
    pool_timeout: float


def load_mysql_config() -> MysqlConfig:
    return MysqlConfig(
        host=os.getenv("PETMIND_MYSQL_HOST", "127.0.0.1"),
        port=int(os.getenv("PETMIND_MYSQL_PORT", "3306")),
        user=os.getenv("PETMIND_MYSQL_USER", "root"),
        password=os.getenv("PETMIND_MYSQL_PASSWORD", ""),
        database=os.getenv("PETMIND_MYSQL_DATABASE", "petmind"),
        max_limit=max(1, min(500, int(os.getenv("PETMIND_SQL_MAX_LIMIT", "200")))),
        debug_sql=os.getenv("AGENT_SQL_DEBUG", "0") == "1",
        pool_size=max(1, int(os.getenv("PETMIND_MYSQL_POOL_SIZE", "5"))),
        pool_timeout=max(0.1, float(os.getenv("PETMIND_MYSQL_POOL_TIMEOUT", "10"))),
    )
