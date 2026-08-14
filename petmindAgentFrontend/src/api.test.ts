import { describe, expect, it } from 'vitest'
import { parseSseBlock } from './api'

describe('SSE parser', () => {
  it('keeps persisted sequence and sanitized event payload', () => {
    expect(parseSseBlock('id: 17\nevent: status\ndata: {"phase":"reviewing","message":"正在进行安全复核"}')).toEqual({
      id: 17,
      event: 'status',
      data: { phase: 'reviewing', message: '正在进行安全复核' },
    })
  })

  it('ignores keep-alive blocks', () => {
    expect(parseSseBlock(': keep-alive')).toBeNull()
  })
})
