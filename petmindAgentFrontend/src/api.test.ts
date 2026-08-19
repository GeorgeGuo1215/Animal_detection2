import { afterEach, describe, expect, it, vi } from 'vitest'
import { getAccessToken, parseSseBlock, restoreSession, setAccessToken } from './api'

afterEach(() => {
  vi.unstubAllGlobals()
  setAccessToken('')
})

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

describe('session bootstrap', () => {
  it('restores both the access token and user with one refresh request', async () => {
    const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
    const fetchMock = vi.fn().mockResolvedValue(new Response(JSON.stringify({ access_token: 'access-1', user }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    }))
    vi.stubGlobal('fetch', fetchMock)

    await expect(restoreSession()).resolves.toEqual(user)
    expect(getAccessToken()).toBe('access-1')
    expect(fetchMock).toHaveBeenCalledTimes(1)
    expect(fetchMock.mock.calls[0][0]).toContain('/api/v1/auth/refresh')
  })

  it('retries once when another tab has just rotated the refresh cookie', async () => {
    const user = { id: 'user-2', email: 'second@petmind.local', display_name: '周医生', role: 'VET', status: 'active' }
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(new Response('{"detail":"refresh token already rotated"}', { status: 409 }))
      .mockResolvedValueOnce(new Response(JSON.stringify({ access_token: 'access-2', user }), {
        status: 200,
        headers: { 'Content-Type': 'application/json' },
      }))
    vi.stubGlobal('fetch', fetchMock)

    await expect(restoreSession()).resolves.toEqual(user)
    expect(fetchMock).toHaveBeenCalledTimes(2)
    expect(getAccessToken()).toBe('access-2')
  })
})
