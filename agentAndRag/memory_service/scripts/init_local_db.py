"""建本地调试库并建表。

复刻 pet-server 里记忆系统依赖的最小结构（User / Pet / ChatSession），再叠加记忆表。
默认建独立的 petmemory_dev 库，不碰 pet-server 的开发库。

    python -m memory_service.scripts.init_local_db            # 建库建表
    python -m memory_service.scripts.init_local_db --drop     # 先删库再重建
    python -m memory_service.scripts.init_local_db --seed     # 顺带插入调试用户与宠物

在 agentAndRag/ 目录下执行。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import psycopg
from psycopg import sql
from psycopg.conninfo import conninfo_to_dict, make_conninfo

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from memory_service.app.config import load_config  # noqa: E402

SQL_DIR = Path(__file__).resolve().parent.parent / "sql"

SEED_USER_ID = "seed_user_local"
SEED_PET_ID = "seed_pet_local"


def _split_dsn(dsn: str) -> tuple[str, str]:
    """拆出目标库名，以及一个指向 maintenance 库(postgres)的 DSN。"""
    parts = conninfo_to_dict(dsn)
    dbname = parts.pop("dbname", None)
    if not dbname:
        raise SystemExit(f"DSN 里没有数据库名: {dsn}")
    return dbname, make_conninfo(**parts, dbname="postgres")


def _database_exists(admin_dsn: str, dbname: str) -> bool:
    with psycopg.connect(admin_dsn, autocommit=True) as conn:
        row = conn.execute(
            "SELECT 1 FROM pg_database WHERE datname = %s", (dbname,)
        ).fetchone()
        return row is not None


def _create_database(admin_dsn: str, dbname: str, drop: bool) -> None:
    # CREATE/DROP DATABASE 不能跑在事务块里，必须 autocommit。
    with psycopg.connect(admin_dsn, autocommit=True) as conn:
        if drop:
            conn.execute(
                sql.SQL("DROP DATABASE IF EXISTS {} WITH (FORCE)").format(
                    sql.Identifier(dbname)
                )
            )
            print(f"已删除数据库 {dbname}")
        exists = conn.execute(
            "SELECT 1 FROM pg_database WHERE datname = %s", (dbname,)
        ).fetchone()
        if exists:
            print(f"数据库 {dbname} 已存在，跳过创建")
            return
        conn.execute(sql.SQL("CREATE DATABASE {}").format(sql.Identifier(dbname)))
        print(f"已创建数据库 {dbname}")


def _run_sql_file(dsn: str, path: Path) -> None:
    statements = path.read_text(encoding="utf-8")
    with psycopg.connect(dsn, autocommit=True) as conn:
        conn.execute(statements)
    print(f"已执行 {path.name}")


def _verify(dsn: str) -> None:
    with psycopg.connect(dsn) as conn:
        row = conn.execute(
            "SELECT extversion FROM pg_extension WHERE extname = 'vector'"
        ).fetchone()
        if not row:
            raise SystemExit("pgvector 扩展未启用，记忆表无法工作")
        print(f"pgvector 已启用，版本 {row[0]}")

        tables = conn.execute(
            """
            SELECT table_name FROM information_schema.tables
            WHERE table_schema = 'public' AND table_name LIKE 'memory_%'
            ORDER BY table_name
            """
        ).fetchall()
        names = [t[0] for t in tables]
        print(f"记忆表 {len(names)} 张: {', '.join(names)}")

        expected = {
            "memory_short_term",
            "memory_segments",
            "memory_pages",
            "memory_profiles",
            "memory_knowledge",
            "memory_tasks",
        }
        missing = expected - set(names)
        if missing:
            raise SystemExit(f"缺少表: {', '.join(sorted(missing))}")

        # HNSW 索引建不出来的话向量检索会退化成全表扫描，这里显式确认一次。
        idx = conn.execute(
            """
            SELECT count(*) FROM pg_indexes
            WHERE schemaname = 'public' AND indexdef LIKE '%hnsw%'
            """
        ).fetchone()
        print(f"HNSW 向量索引 {idx[0]} 个")


def _seed(dsn: str) -> None:
    with psycopg.connect(dsn, autocommit=True) as conn:
        conn.execute(
            """
            INSERT INTO "User" ("id", "username", "passwordHash")
            VALUES (%s, %s, %s)
            ON CONFLICT ("id") DO NOTHING
            """,
            (SEED_USER_ID, "本地调试用户", "not-a-real-hash"),
        )
        conn.execute(
            """
            INSERT INTO "Pet" ("id", "name", "type", "userId")
            VALUES (%s, %s, %s, %s)
            ON CONFLICT ("id") DO NOTHING
            """,
            (SEED_PET_ID, "咪咪", "CAT", SEED_USER_ID),
        )
    print(f"已写入调试数据: user={SEED_USER_ID}, pet={SEED_PET_ID}")


def main() -> int:
    parser = argparse.ArgumentParser(description="初始化记忆服务的本地调试库")
    parser.add_argument("--dsn", default=None, help="目标库 DSN，默认取 MEMORY_DB_DSN")
    parser.add_argument("--drop", action="store_true", help="先删除已有数据库再重建")
    parser.add_argument("--seed", action="store_true", help="插入调试用的用户与宠物")
    args = parser.parse_args()

    dsn = args.dsn or load_config().dsn
    dbname, admin_dsn = _split_dsn(dsn)

    print(f"目标数据库: {dbname}")
    _create_database(admin_dsn, dbname, drop=args.drop)

    for name in ("000_petserver_fixture.sql", "001_schema.sql"):
        _run_sql_file(dsn, SQL_DIR / name)

    _verify(dsn)
    if args.seed:
        _seed(dsn)

    print("本地调试库就绪")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
