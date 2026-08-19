-- 中期主题到长期知识/画像的生成依赖图。会话历史与记忆使用独立生命周期，
-- 因此不保存原始 turn_id 到中长期记忆的关联。
CREATE TABLE IF NOT EXISTS memory_derivations (
    "id"            BIGSERIAL NOT NULL,
    "userId"        TEXT NOT NULL,
    "sourceType"    TEXT NOT NULL,
    "sourceId"      TEXT NOT NULL,
    "targetType"    TEXT NOT NULL,
    "targetId"      TEXT NOT NULL,
    "generationTag" TEXT NOT NULL,
    "metadata"      JSONB NOT NULL DEFAULT '{}'::jsonb,
    "createdAt"     TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_derivations_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_derivations_subject_fkey" FOREIGN KEY ("userId")
        REFERENCES memory_subjects("id") ON DELETE CASCADE,
    CONSTRAINT "memory_derivations_unique_edge" UNIQUE
        ("userId", "sourceType", "sourceId", "targetType", "targetId", "generationTag")
);

CREATE INDEX IF NOT EXISTS "memory_derivations_source_idx"
    ON memory_derivations ("userId", "sourceType", "sourceId");
CREATE INDEX IF NOT EXISTS "memory_derivations_target_idx"
    ON memory_derivations ("userId", "targetType", "targetId");

-- 存量长期项没有可验证的中期段来源，不伪造依赖边；管理接口会使用长期项
-- 自身的 source 字段作为展示标签。新生成项由运行时代码写入 segment 依赖。
