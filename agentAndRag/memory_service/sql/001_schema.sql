-- 记忆系统表结构。
--
-- 命名对齐 pet-server 中较新的领域表（health_scores / pet_profiles）：
-- snake_case 表名 + camelCase 字段 + TEXT 主键 + timestamp(3) without time zone。
--
-- 向量维度 384 对应 intfloat/multilingual-e5-small。换 embedding 模型必须同步
-- 改这里的 vector(384) 并重建索引，否则写入会因维度不匹配报错。

CREATE EXTENSION IF NOT EXISTS vector;

-- ---------------------------------------------------------------- 短期记忆
-- 最近若干轮原始对话，先进先出。溢出的部分由 consolidator 提升为中期记忆。
CREATE TABLE IF NOT EXISTS memory_short_term (
    "id"            TEXT NOT NULL,
    "userId"        TEXT NOT NULL,
    "petId"         TEXT,
    "sessionId"     TEXT,
    "userInput"     TEXT NOT NULL,
    "agentResponse" TEXT NOT NULL,
    "createdAt"     TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_short_term_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_short_term_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

-- 取队列头尾都靠这个索引；createdAt 相同时用 id 兜底保证顺序稳定。
CREATE INDEX IF NOT EXISTS "memory_short_term_user_time_idx"
    ON memory_short_term ("userId", "createdAt", "id");

-- ---------------------------------------------------------------- 中期记忆：话题段
-- 热度载体。visitCount / pageCount / lastVisitAt 三个因子决定 heat，
-- heat 落列而不是内存堆，多 worker 才能看到同一份排序。
CREATE TABLE IF NOT EXISTS memory_segments (
    "id"               TEXT NOT NULL,
    "userId"           TEXT NOT NULL,
    "petId"            TEXT,
    "summary"          TEXT NOT NULL,
    "keywords"         TEXT[] NOT NULL DEFAULT '{}',
    "summaryEmbedding" vector(384),
    "visitCount"       INTEGER NOT NULL DEFAULT 0,
    "pageCount"        INTEGER NOT NULL DEFAULT 0,
    "heat"             DOUBLE PRECISION NOT NULL DEFAULT 0,
    "lastVisitAt"      TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "lastAnalyzedAt"   TIMESTAMP(3),
    "createdAt"        TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt"        TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_segments_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_segments_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

-- 汰换扫最冷（ASC）、提升扫最热（DESC），B-tree 双向可扫，一个索引够用。
CREATE INDEX IF NOT EXISTS "memory_segments_user_heat_idx"
    ON memory_segments ("userId", "heat");

CREATE INDEX IF NOT EXISTS "memory_segments_user_pet_idx"
    ON memory_segments ("userId", "petId");

CREATE INDEX IF NOT EXISTS "memory_segments_embedding_idx"
    ON memory_segments USING hnsw ("summaryEmbedding" vector_cosine_ops);

-- ---------------------------------------------------------------- 中期记忆：段内页
CREATE TABLE IF NOT EXISTS memory_pages (
    "id"            TEXT NOT NULL,
    "segmentId"     TEXT NOT NULL,
    "userId"        TEXT NOT NULL,
    "petId"         TEXT,
    "userInput"     TEXT NOT NULL,
    "agentResponse" TEXT NOT NULL,
    "embedding"     vector(384),
    "analyzed"      BOOLEAN NOT NULL DEFAULT false,
    "prevPageId"    TEXT,
    "createdAt"     TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_pages_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_pages_segmentId_fkey" FOREIGN KEY ("segmentId")
        REFERENCES memory_segments("id") ON DELETE CASCADE,
    CONSTRAINT "memory_pages_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS "memory_pages_segment_idx"
    ON memory_pages ("segmentId");

-- 画像提升只关心未分析页，用部分索引避免扫全表。
CREATE INDEX IF NOT EXISTS "memory_pages_unanalyzed_idx"
    ON memory_pages ("segmentId") WHERE "analyzed" = false;

CREATE INDEX IF NOT EXISTS "memory_pages_embedding_idx"
    ON memory_pages USING hnsw ("embedding" vector_cosine_ops);

-- ---------------------------------------------------------------- 长期记忆：用户画像
-- 一用户一行。profile 是结构化 JSONB，LLM 只产出字段增量，写入走 JSONB 合并，
-- 避免上游那种"整段重写"导致的历史信息丢失。version 做乐观锁。
CREATE TABLE IF NOT EXISTS memory_profiles (
    "userId"    TEXT NOT NULL,
    "profile"   JSONB NOT NULL DEFAULT '{}'::jsonb,
    "version"   INTEGER NOT NULL DEFAULT 0,
    "updatedAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_profiles_pkey" PRIMARY KEY ("userId"),
    CONSTRAINT "memory_profiles_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

-- ---------------------------------------------------------------- 长期记忆：知识条目
-- 被汰换的中期段摘要也会沉淀到这里，所以 source 区分来源。
CREATE TABLE IF NOT EXISTS memory_knowledge (
    "id"        TEXT NOT NULL,
    "userId"    TEXT NOT NULL,
    "petId"     TEXT,
    "content"   TEXT NOT NULL,
    "embedding" vector(384),
    "source"    TEXT NOT NULL DEFAULT 'extraction',
    "hitCount"  INTEGER NOT NULL DEFAULT 0,
    "lastHitAt" TIMESTAMP(3),
    "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_knowledge_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_knowledge_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

-- 同一用户重复抽出同一条知识时直接命中冲突，靠它做幂等写入。
CREATE UNIQUE INDEX IF NOT EXISTS "memory_knowledge_user_content_key"
    ON memory_knowledge ("userId", md5("content"));

-- 知识库超容量时按"冷且旧"淘汰。
CREATE INDEX IF NOT EXISTS "memory_knowledge_user_hit_idx"
    ON memory_knowledge ("userId", "hitCount", "createdAt");

CREATE INDEX IF NOT EXISTS "memory_knowledge_embedding_idx"
    ON memory_knowledge USING hnsw ("embedding" vector_cosine_ops);

-- ---------------------------------------------------------------- 任务队列
-- 写入路径只入队，LLM 处理全部由 worker 异步消费。
CREATE TABLE IF NOT EXISTS memory_tasks (
    "id"         BIGSERIAL NOT NULL,
    "userId"     TEXT NOT NULL,
    "kind"       TEXT NOT NULL,
    "payload"    JSONB NOT NULL DEFAULT '{}'::jsonb,
    "status"     TEXT NOT NULL DEFAULT 'pending',
    "attempts"   INTEGER NOT NULL DEFAULT 0,
    "lastError"  TEXT,
    "lockedAt"   TIMESTAMP(3),
    "createdAt"  TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt"  TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "memory_tasks_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "memory_tasks_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

-- 出队只看 pending，部分索引让队列深度不影响扫描成本。
CREATE INDEX IF NOT EXISTS "memory_tasks_pending_idx"
    ON memory_tasks ("createdAt") WHERE "status" = 'pending';

-- 同一用户同一类型只留一个待处理任务：高频对话不会把队列撑爆，
-- 入队用 ON CONFLICT DO NOTHING 天然合并。
CREATE UNIQUE INDEX IF NOT EXISTS "memory_tasks_user_kind_pending_key"
    ON memory_tasks ("userId", "kind") WHERE "status" = 'pending';
