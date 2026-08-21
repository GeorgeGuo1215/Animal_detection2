"""生产与测试聊天路由共用的失败开放（fail-open）记忆操作。"""
from __future__ import annotations

from typing import Any, Dict, Optional

from fastapi import HTTPException

from ..prompts.memory import build_memory_context_injection
from .client import get_memory_client


async def ensure_memory_subject(
    *,
    user_id: str,
    display_name: Optional[str] = None,
    source: str,
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """确保记忆服务中存在该用户主体；服务关闭或失败时 fail-open。"""
    client = get_memory_client()
    if client is None:
        return {"enabled": False, "ensured": False, "reason": "memory_disabled"}
    try:
        result = await client.ensure_subject(
            user_id=user_id,
            display_name=display_name,
            source=source,
            metadata=metadata,
        )
        return {
            "enabled": True,
            "ensured": True,
            "user_id": user_id,
            "source": result.get("source") or source,
        }
    except Exception as exc:  # noqa: BLE001
        if client.config.required:
            raise HTTPException(status_code=503, detail=f"required memory unavailable: {exc}") from exc
        return {
            "enabled": True,
            "ensured": False,
            "user_id": user_id,
            "reason": "memory_unavailable",
            "error": str(exc),
        }


async def load_user_memory(
    *,
    user_id: Optional[str],
    query: str,
    pet_id: Optional[str],
) -> tuple[str, Dict[str, Any]]:
    """按认证用户、当前查询和可选宠物 ID 加载隔离的记忆上下文。

    未启用、缺少 user_id 或服务失败时按配置 fail-open，返回空注入及原因元数据；
    ``required`` 模式下不可用错误向上抛出。记忆正文只用于模型注入，不写入响应元数据。
    """
    client = get_memory_client()
    if client is None:
        return "", {"enabled": False, "loaded": False, "reason": "memory_disabled"}
    if not user_id:
        return "", {"enabled": True, "loaded": False, "reason": "missing_user_id"}
    try:
        context = await client.context(user_id=user_id, query=query, pet_id=pet_id)
        text = str(context.get("text") or "")
        return build_memory_context_injection(text), {
            "enabled": True,
            "loaded": True,
            "user_id": user_id,
            "profile_version": int(context.get("profile_version") or 0),
            "knowledge_items": len(context.get("knowledge") or []),
            "related_pages": len(context.get("related_pages") or []),
            "recent_turns": len(context.get("recent_dialogue") or []),
            "context_chars": len(text),
        }
    except Exception as exc:  # noqa: BLE001
        if client.config.required:
            raise HTTPException(status_code=503, detail=f"required memory unavailable: {exc}") from exc
        return "", {
            "enabled": True,
            "loaded": False,
            "user_id": user_id,
            "reason": "memory_unavailable",
            "error": str(exc),
        }


async def write_user_memory(
    *,
    user_id: Optional[str],
    query: str,
    answer: str,
    pet_id: Optional[str],
    session_id: Optional[str],
    turn_id: Optional[str],
) -> Dict[str, Any]:
    """以稳定 turn_id 将完成的一轮问答幂等写入认证用户的记忆队列。

    空用户、空终答或未启用时跳过；普通不可用按 fail-open 返回状态，``required`` 模式
    则抛错。session_id 仅作为来源上下文，不改变用户级记忆归属。
    """
    client = get_memory_client()
    if client is None or not user_id or not answer.strip():
        return {"stored": False, "reason": "disabled_or_incomplete"}
    try:
        result = await client.write_turn(
            user_id=user_id,
            user_input=query,
            agent_response=answer,
            pet_id=pet_id,
            session_id=session_id,
            turn_id=turn_id,
        )
        return {
            "stored": True,
            "user_id": user_id,
            "message_id": result.get("id"),
            "duplicate": bool(result.get("duplicate")),
            "queued": bool(result.get("queued")),
            "short_term_size": int(result.get("short_term_size") or 0),
        }
    except Exception as exc:  # noqa: BLE001
        if client.config.required:
            raise HTTPException(status_code=503, detail=f"required memory write failed: {exc}") from exc
        return {
            "stored": False,
            "user_id": user_id,
            "reason": "memory_unavailable",
            "error": str(exc),
        }
