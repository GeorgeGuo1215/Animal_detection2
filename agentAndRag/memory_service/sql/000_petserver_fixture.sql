-- 本地调试用：复刻 pet-server 中记忆系统依赖的最小表结构。
--
-- 生产库里这些表由 pet-server 的 Prisma migration 创建，这个文件只在本地
-- 独立调试库（默认 petmemory_dev）中执行，用来让外键与真实环境一致。
-- 列定义抄自运行中的 PetHealth 库，包括 Prisma 特有的 timestamp(3) without time zone。

CREATE EXTENSION IF NOT EXISTS vector;

DO $$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'Role') THEN
        CREATE TYPE "Role" AS ENUM ('USER', 'ADMIN');
    END IF;
    IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'PetType') THEN
        CREATE TYPE "PetType" AS ENUM ('DOG', 'CAT', 'OTHER');
    END IF;
    IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'Gender') THEN
        CREATE TYPE "Gender" AS ENUM ('MALE', 'FEMALE', 'UNKNOWN');
    END IF;
END
$$;

CREATE TABLE IF NOT EXISTS "User" (
    "id"            TEXT NOT NULL,
    "phone"         TEXT,
    "email"         TEXT,
    "username"      TEXT,
    "passwordHash"  TEXT NOT NULL DEFAULT '',
    "avatar"        TEXT,
    "role"          "Role" NOT NULL DEFAULT 'USER',
    "phoneVerified" BOOLEAN NOT NULL DEFAULT false,
    "emailVerified" BOOLEAN NOT NULL DEFAULT false,
    "createdAt"     TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "User_pkey" PRIMARY KEY ("id")
);

CREATE TABLE IF NOT EXISTS "Pet" (
    "id"        TEXT NOT NULL,
    "name"      TEXT NOT NULL,
    "type"      "PetType" NOT NULL DEFAULT 'OTHER',
    "breed"     TEXT,
    "gender"    "Gender" NOT NULL DEFAULT 'UNKNOWN',
    "userId"    TEXT NOT NULL,
    "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "Pet_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "Pet_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS "ChatSession" (
    "id"        TEXT NOT NULL,
    "userId"    TEXT NOT NULL,
    "title"     TEXT NOT NULL DEFAULT '',
    "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "updatedAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT "ChatSession_pkey" PRIMARY KEY ("id"),
    CONSTRAINT "ChatSession_userId_fkey" FOREIGN KEY ("userId")
        REFERENCES "User"("id") ON DELETE CASCADE
);
