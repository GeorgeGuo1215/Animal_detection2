export const STREAM_RENDER_INTERVAL_MS = 32
export const STREAM_RENDER_CHARS = 72

export interface StreamRenderChunk {
  chunk: string
  rest: string
}

/**
 * 将可能很大的 SSE 文本增量切成适合一帧渲染的小段。
 *
 * 优先在换行、中文标点或空格后断开，让 Markdown 的临时渲染更稳定；
 * 找不到自然边界时按长度切分，同时避免拆开 UTF-16 代理对。
 */
export function takeStreamRenderChunk(
  pending: string,
  maxChars = STREAM_RENDER_CHARS,
): StreamRenderChunk {
  if (!pending || pending.length <= maxChars) return { chunk: pending, rest: '' }

  const minimumBoundary = Math.max(1, Math.floor(maxChars * 0.55))
  const candidate = pending.slice(0, maxChars + 1)
  const boundaries = ['\n\n', '\n', '。', '！', '？', '；', '，', '、', ' ']
  let end = 0

  for (const boundary of boundaries) {
    const index = candidate.lastIndexOf(boundary, maxChars - 1)
    if (index >= minimumBoundary) {
      end = Math.max(end, index + boundary.length)
    }
  }
  if (!end) end = maxChars

  // 不在 emoji 等 Unicode 字符的高、低代理项之间断开。
  const previous = pending.charCodeAt(end - 1)
  const next = pending.charCodeAt(end)
  if (previous >= 0xd800 && previous <= 0xdbff && next >= 0xdc00 && next <= 0xdfff) end -= 1

  return { chunk: pending.slice(0, end), rest: pending.slice(end) }
}
