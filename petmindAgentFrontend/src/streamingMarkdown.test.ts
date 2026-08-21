import { describe, expect, it } from 'vitest'
import { takeStreamRenderChunk } from './streamingMarkdown'

describe('takeStreamRenderChunk', () => {
  it('小增量直接进入下一次渲染', () => {
    expect(takeStreamRenderChunk('## 检查建议', 20)).toEqual({ chunk: '## 检查建议', rest: '' })
  })

  it('大增量优先在自然语言边界分批渲染', () => {
    const result = takeStreamRenderChunk('第一项检查建议，需要立即完成。第二项检查建议随后完成。', 18)
    expect(result.chunk.length).toBeLessThanOrEqual(18)
    expect(result.chunk.endsWith('，') || result.chunk.endsWith('。')).toBe(true)
    expect(result.chunk + result.rest).toBe('第一项检查建议，需要立即完成。第二项检查建议随后完成。')
  })

  it('不会拆开 emoji 的 UTF-16 代理对', () => {
    const result = takeStreamRenderChunk('12345🐾后续内容', 6)
    expect(result.chunk).toBe('12345')
    expect(result.rest.startsWith('🐾')).toBe(true)
    expect(result.chunk + result.rest).toBe('12345🐾后续内容')
  })
})
