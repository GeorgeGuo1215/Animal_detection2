-- 会话历史与记忆使用独立生命周期。中长期记忆不保存原始 turn 关联，
-- 用户隐藏/删除会话不会触发记忆删除或重建。
DELETE FROM memory_derivations WHERE "sourceType" IN ('turn', 'legacy_import');

DROP INDEX IF EXISTS "memory_pages_user_source_turn_idx";
ALTER TABLE memory_pages DROP COLUMN IF EXISTS "sourceTurnId";
