"""仅内置浏览器测试 UI 使用的、带 TTL 的持久化会话。"""
from __future__ import annotations

import asyncio
import copy
import json
import os
import sqlite3
import time
import uuid
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, AsyncIterator, Dict, List, Optional, Sequence, Tuple


def _default_db_path() -> Path:
    # agent_api/app/persistence/session_manager.py -> agentAndRag is parents[3].
    """默认会话库路径。"""
    repo_root = Path(__file__).resolve().parents[3]
    return repo_root / "agent_api_logs" / "petmind_sessions.db"


def _env_int(name: str, default: int) -> int:
    """读取正整数环境变量。"""
    try:
        return max(1, int(os.getenv(name, "") or default))
    except (TypeError, ValueError):
        return default


def _env_float(name: str, default: float) -> float:
    """读取正浮点环境变量。"""
    try:
        return max(1.0, float(os.getenv(name, "") or default))
    except (TypeError, ValueError):
        return default


@dataclass
class Session:
    """一条测试会话的消息、专家上下文与元数据。"""
    session_id: str
    messages: List[Dict[str, str]] = field(default_factory=list)
    tool_results: List[Dict[str, Any]] = field(default_factory=list)
    expert_contexts: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    created_at: float = field(default_factory=time.time)
    last_active: float = field(default_factory=time.time)

    def touch(self) -> None:
        """刷新最后活跃时间。"""
        self.last_active = time.time()


def _complete_turns(messages: Sequence[Dict[str, str]]) -> List[Tuple[Dict[str, str], Dict[str, str]]]:
    """返回完整的 user/assistant 轮次对，不暴露孤立消息。"""
    turns: List[Tuple[Dict[str, str], Dict[str, str]]] = []
    pending_user: Optional[Dict[str, str]] = None
    for raw in messages:
        role = str(raw.get("role") or "")
        content = str(raw.get("content") or "").strip()
        if not content:
            continue
        message = {"role": role, "content": content}
        if role == "user":
            pending_user = message
        elif role == "assistant" and pending_user is not None:
            turns.append((pending_user, message))
            pending_user = None
    return turns


def select_complete_turn_context(
    messages: Sequence[Dict[str, str]],
    *,
    max_turns: int,
    max_chars: int,
) -> List[Dict[str, str]]:
    """选取最近的完整轮次，并保证每对消息不被拆开。"""
    turns = _complete_turns(messages)[-max(1, int(max_turns)):]
    selected: List[Tuple[Dict[str, str], Dict[str, str]]] = []
    used = 0
    for turn in reversed(turns):
        size = sum(len(message["content"]) for message in turn)
        if selected and used + size > max_chars:
            break
        selected.append(turn)
        used += size
    selected.reverse()
    return [dict(message) for turn in selected for message in turn]


class SessionManager:
    """带内存热缓存与会话锁的 SQLite 测试会话存储。"""

    def __init__(
        self,
        ttl_seconds: Optional[float] = None,
        max_sessions: Optional[int] = None,
        db_path: Optional[str | Path] = None,
        context_max_turns: Optional[int] = None,
        context_max_chars: Optional[int] = None,
    ) -> None:
        """按 TTL、容量与上下文预算初始化会话管理器。"""
        self._sessions: Dict[str, Session] = {}
        self._session_locks: Dict[str, asyncio.Lock] = {}
        self._session_lock_users: Dict[str, int] = {}
        self._ttl = ttl_seconds if ttl_seconds is not None else _env_float("AGENT_SESSION_TTL_SEC", 3600.0)
        self._max = max_sessions if max_sessions is not None else _env_int("AGENT_SESSION_MAX", 10_000)
        self.context_max_turns = (
            context_max_turns
            if context_max_turns is not None
            else _env_int("AGENT_SESSION_CONTEXT_MAX_TURNS", 24)
        )
        self.context_max_chars = (
            context_max_chars
            if context_max_chars is not None
            else _env_int("AGENT_SESSION_CONTEXT_MAX_CHARS", 48_000)
        )
        self._db_path = str(db_path or os.getenv("AGENT_SESSION_DB_PATH") or _default_db_path())
        self._lock = asyncio.Lock()
        self._init_db_sync()

    def _connect(self) -> sqlite3.Connection:
        """打开会话库 SQLite 连接。"""
        Path(self._db_path).parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(self._db_path, timeout=10)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA busy_timeout=5000")
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db_sync(self) -> None:
        """同步创建会话表。"""
        conn = self._connect()
        try:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS agent_sessions (
                    session_id      TEXT PRIMARY KEY,
                    messages        TEXT NOT NULL DEFAULT '[]',
                    tool_results    TEXT NOT NULL DEFAULT '[]',
                    expert_contexts TEXT NOT NULL DEFAULT '[]',
                    metadata        TEXT NOT NULL DEFAULT '{}',
                    created_at      REAL NOT NULL,
                    last_active     REAL NOT NULL
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_agent_sessions_last_active "
                "ON agent_sessions(last_active)"
            )
            conn.commit()
        finally:
            conn.close()

    @staticmethod
    def _json_load(value: str, fallback: Any) -> Any:
        """安全解析 JSON 文本。"""
        try:
            parsed = json.loads(value)
            return parsed
        except (TypeError, ValueError):
            return fallback

    def _row_to_session(self, row: sqlite3.Row) -> Session:
        """将数据库行转为 Session。"""
        return Session(
            session_id=str(row["session_id"]),
            messages=list(self._json_load(row["messages"], [])),
            tool_results=list(self._json_load(row["tool_results"], [])),
            expert_contexts=list(self._json_load(row["expert_contexts"], [])),
            metadata=dict(self._json_load(row["metadata"], {})),
            created_at=float(row["created_at"]),
            last_active=float(row["last_active"]),
        )

    def _load_sync(self, session_id: str) -> Optional[Session]:
        """同步按 id 加载会话。"""
        conn = self._connect()
        try:
            row = conn.execute(
                "SELECT * FROM agent_sessions WHERE session_id = ?", (session_id,)
            ).fetchone()
            return self._row_to_session(row) if row is not None else None
        finally:
            conn.close()

    def _save_sync(self, session: Session) -> None:
        """同步持久化会话。"""
        conn = self._connect()
        try:
            conn.execute(
                """
                INSERT INTO agent_sessions
                    (session_id, messages, tool_results, expert_contexts, metadata,
                     created_at, last_active)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(session_id) DO UPDATE SET
                    messages=excluded.messages,
                    tool_results=excluded.tool_results,
                    expert_contexts=excluded.expert_contexts,
                    metadata=excluded.metadata,
                    last_active=excluded.last_active
                """,
                (
                    session.session_id,
                    json.dumps(session.messages, ensure_ascii=False, default=str),
                    json.dumps(session.tool_results, ensure_ascii=False, default=str),
                    json.dumps(session.expert_contexts, ensure_ascii=False, default=str),
                    json.dumps(session.metadata, ensure_ascii=False, default=str),
                    session.created_at,
                    session.last_active,
                ),
            )
            conn.commit()
        finally:
            conn.close()

    def _delete_sync(self, session_id: str) -> bool:
        """同步删除会话。"""
        conn = self._connect()
        try:
            cur = conn.execute("DELETE FROM agent_sessions WHERE session_id = ?", (session_id,))
            conn.commit()
            return cur.rowcount > 0
        finally:
            conn.close()

    def _evict_sync(self, now: float, protected_session_ids: Sequence[str] = ()) -> List[str]:
        """从 SQLite 删除未受锁保护的过期会话，并按最久未活跃顺序收缩容量。

        返回实际删除的 session_id；活跃请求保护的会话即使过期或处于容量候选中也会
        跳过，避免并发提交过程中被清理。
        """
        cutoff = now - self._ttl
        protected = set(protected_session_ids)
        conn = self._connect()
        try:
            expired = [
                str(row[0])
                for row in conn.execute(
                    "SELECT session_id FROM agent_sessions WHERE last_active < ?", (cutoff,)
                ).fetchall()
                if str(row[0]) not in protected
            ]
            if expired:
                conn.executemany(
                    "DELETE FROM agent_sessions WHERE session_id = ?",
                    [(session_id,) for session_id in expired],
                )
            count = int(conn.execute("SELECT COUNT(*) FROM agent_sessions").fetchone()[0])
            overflow: List[str] = []
            if count > self._max:
                candidates = conn.execute(
                    "SELECT session_id FROM agent_sessions ORDER BY last_active ASC"
                ).fetchall()
                overflow = [
                    str(row[0]) for row in candidates if str(row[0]) not in protected
                ][:count - self._max]
                conn.executemany(
                    "DELETE FROM agent_sessions WHERE session_id = ?",
                    [(session_id,) for session_id in overflow],
                )
            conn.commit()
            return expired + overflow
        finally:
            conn.close()

    def _protected_session_ids(self) -> List[str]:
        """返回当前仍被请求锁保护的会话 id。"""
        return [
            session_id
            for session_id, users in self._session_lock_users.items()
            if users > 0
        ]

    async def cleanup(self) -> List[str]:
        """清理过期/超量会话，不打断进行中的请求。"""
        async with self._lock:
            removed = await asyncio.to_thread(
                self._evict_sync,
                time.time(),
                self._protected_session_ids(),
            )
            for session_id in removed:
                self._sessions.pop(session_id, None)
                self._session_locks.pop(session_id, None)
                self._session_lock_users.pop(session_id, None)
            return removed

    async def create(self, metadata: Optional[Dict[str, Any]] = None) -> Session:
        """创建新会话并写入存储。"""
        async with self._lock:
            removed = await asyncio.to_thread(
                self._evict_sync, time.time(), self._protected_session_ids()
            )
            for session_id in removed:
                self._sessions.pop(session_id, None)
                self._session_locks.pop(session_id, None)
                self._session_lock_users.pop(session_id, None)
            session = Session(session_id=uuid.uuid4().hex, metadata=dict(metadata or {}))
            await asyncio.to_thread(self._save_sync, session)
            removed = await asyncio.to_thread(
                self._evict_sync,
                time.time(),
                [*self._protected_session_ids(), session.session_id],
            )
            for session_id in removed:
                self._sessions.pop(session_id, None)
                self._session_locks.pop(session_id, None)
                self._session_lock_users.pop(session_id, None)
            self._sessions[session.session_id] = session
            self._session_locks.setdefault(session.session_id, asyncio.Lock())
            return session

    async def get(self, session_id: str, *, touch: bool = True) -> Optional[Session]:
        """读取会话；过期且未锁定则删除。"""
        async with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                session = await asyncio.to_thread(self._load_sync, session_id)
                if session is None:
                    return None
            expired = time.time() - session.last_active > self._ttl
            protected = self._session_lock_users.get(session_id, 0) > 0
            if expired and not protected:
                self._sessions.pop(session_id, None)
                self._session_locks.pop(session_id, None)
                await asyncio.to_thread(self._delete_sync, session_id)
                return None
            if touch or expired:
                session.touch()
                await asyncio.to_thread(self._save_sync, session)
            self._sessions[session_id] = session
            self._session_locks.setdefault(session_id, asyncio.Lock())
            return session

    async def get_or_create(
        self,
        session_id: Optional[str],
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Session:
        """按 id 获取会话，不存在则新建。"""
        if session_id:
            session = await self.get(session_id)
            if session:
                return session
        return await self.create(metadata)

    @asynccontextmanager
    async def session_lock(self, session_id: str) -> AsyncIterator[None]:
        """获取针对单个会话的异步锁。"""
        async with self._lock:
            lock = self._session_locks.setdefault(session_id, asyncio.Lock())
            self._session_lock_users[session_id] = self._session_lock_users.get(session_id, 0) + 1
        try:
            async with lock:
                yield
        finally:
            async with self._lock:
                users = self._session_lock_users.get(session_id, 0) - 1
                if users > 0:
                    self._session_lock_users[session_id] = users
                else:
                    self._session_lock_users.pop(session_id, None)

    async def context(self, session_id: str) -> Tuple[List[Dict[str, str]], List[Dict[str, Any]]]:
        """返回裁剪后的对话历史与对应专家上下文。"""
        session = await self.get(session_id)
        if session is None:
            return [], []
        messages = select_complete_turn_context(
            session.messages,
            max_turns=self.context_max_turns,
            max_chars=self.context_max_chars,
        )
        selected_turns = len(messages) // 2
        total_turns = len(_complete_turns(session.messages))
        first_turn = max(1, total_turns - selected_turns + 1)
        expert_contexts = [
            dict(item)
            for item in session.expert_contexts
            if int(item.get("turn_index") or 0) >= first_turn
        ]
        return messages, expert_contexts

    async def commit_turn(
        self,
        session_id: str,
        *,
        user_message: str,
        assistant_message: str,
        expert_context: Optional[Dict[str, Any]] = None,
        tool_results: Optional[List[Dict[str, Any]]] = None,
    ) -> Optional[Session]:
        """在会话锁内原子追加一轮完整 user/assistant 消息并持久化。

        仅提交成对消息，避免恢复时出现孤儿半轮；可同时保存专家上下文与工具结果。
        会话不存在或等待锁期间已被清理时返回 ``None``，容量/TTL 清理不会驱逐活跃锁。
        """
        async with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                session = await asyncio.to_thread(self._load_sync, session_id)
            protected = self._session_lock_users.get(session_id, 0) > 0
            if session is None or (
                time.time() - session.last_active > self._ttl and not protected
            ):
                self._sessions.pop(session_id, None)
                await asyncio.to_thread(self._delete_sync, session_id)
                return None
            updated = copy.deepcopy(session)
            updated.messages.extend([
                {"role": "user", "content": str(user_message)},
                {"role": "assistant", "content": str(assistant_message)},
            ])
            turn_index = len(_complete_turns(updated.messages))
            if expert_context:
                stored_context = dict(expert_context)
                stored_context.setdefault("turn_index", turn_index)
                updated.expert_contexts.append(stored_context)
            if tool_results:
                updated.tool_results.append({
                    "turn_index": turn_index,
                    "results": list(tool_results),
                })
            updated.touch()
            await asyncio.to_thread(self._save_sync, updated)
            self._sessions[session_id] = updated
            return updated

    async def delete(self, session_id: str) -> bool:
        """删除会话，返回是否曾存在。"""
        async with self._lock:
            cached = self._sessions.pop(session_id, None) is not None
            self._session_locks.pop(session_id, None)
            self._session_lock_users.pop(session_id, None)
            persisted = await asyncio.to_thread(self._delete_sync, session_id)
            return cached or persisted


_MANAGER: Optional[SessionManager] = None


def get_session_manager() -> SessionManager:
    """获取全局 SessionManager 单例。"""
    global _MANAGER  # noqa: PLW0603
    if _MANAGER is None:
        _MANAGER = SessionManager()
    return _MANAGER
