-- 将旧版依赖 PetHealth "User" 表的记忆库迁移为独立 memory_subjects。
-- 本文件可重复执行，并保留已有记忆数据。

CREATE TABLE IF NOT EXISTS memory_subjects (
    "id"          TEXT NOT NULL,
    "displayName" TEXT,
    "source"      TEXT NOT NULL DEFAULT 'external',
    "metadata"    JSONB NOT NULL DEFAULT '{}'::jsonb,
    "createdAt"   TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt"   TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_subjects_pkey" PRIMARY KEY ("id")
);

ALTER TABLE memory_short_term ADD COLUMN IF NOT EXISTS "turnId" TEXT;

-- 先为所有已有 userId 建主体，再切换外键，避免迁移时丢数据。
INSERT INTO memory_subjects ("id", "source")
SELECT DISTINCT "userId", 'legacy'
FROM (
    SELECT "userId" FROM memory_short_term
    UNION SELECT "userId" FROM memory_segments
    UNION SELECT "userId" FROM memory_pages
    UNION SELECT "userId" FROM memory_profiles
    UNION SELECT "userId" FROM memory_knowledge
    UNION SELECT "userId" FROM memory_tasks
) existing
WHERE "userId" IS NOT NULL
ON CONFLICT ("id") DO NOTHING;

CREATE TABLE IF NOT EXISTS memory_ingest_receipts (
    "userId"    TEXT NOT NULL,
    "turnId"    TEXT NOT NULL,
    "messageId" TEXT NOT NULL,
    "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_ingest_receipts_pkey" PRIMARY KEY ("userId", "turnId")
);

ALTER TABLE memory_short_term
    DROP CONSTRAINT IF EXISTS "memory_short_term_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_short_term_subject_fkey";
ALTER TABLE memory_short_term ADD CONSTRAINT "memory_short_term_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_segments
    DROP CONSTRAINT IF EXISTS "memory_segments_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_segments_subject_fkey";
ALTER TABLE memory_segments ADD CONSTRAINT "memory_segments_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_pages
    DROP CONSTRAINT IF EXISTS "memory_pages_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_pages_subject_fkey";
ALTER TABLE memory_pages ADD CONSTRAINT "memory_pages_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_profiles
    DROP CONSTRAINT IF EXISTS "memory_profiles_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_profiles_subject_fkey";
ALTER TABLE memory_profiles ADD CONSTRAINT "memory_profiles_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_knowledge
    DROP CONSTRAINT IF EXISTS "memory_knowledge_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_knowledge_subject_fkey";
ALTER TABLE memory_knowledge ADD CONSTRAINT "memory_knowledge_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_tasks
    DROP CONSTRAINT IF EXISTS "memory_tasks_userId_fkey",
    DROP CONSTRAINT IF EXISTS "memory_tasks_subject_fkey";
ALTER TABLE memory_tasks ADD CONSTRAINT "memory_tasks_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;

ALTER TABLE memory_ingest_receipts
    DROP CONSTRAINT IF EXISTS "memory_ingest_receipts_subject_fkey";
ALTER TABLE memory_ingest_receipts ADD CONSTRAINT "memory_ingest_receipts_subject_fkey"
    FOREIGN KEY ("userId") REFERENCES memory_subjects("id") ON DELETE CASCADE;
