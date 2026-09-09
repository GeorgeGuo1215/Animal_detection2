import { afterEach, describe, expect, it, vi } from 'vitest'
import { api, ApiError, consumeSse, getAccessToken, parseSseBlock, restoreSession, setAccessToken, resumeRun } from './api'

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
  it('shares a single refresh between expired SSE connections and preserves cursors', async () => {
    setAccessToken('expired')
    const fetchMock = vi.fn(async (url: string, init: RequestInit) => {
      if (url.endsWith('/auth/refresh')) return new Response(JSON.stringify({ access_token: 'fresh', user: { id: 'test-user' } }))
      if (new Headers(init.headers).get('Authorization') !== 'Bearer fresh') return new Response('{}', { status: 401 })
      return new Response('id: 13\nevent: completed\ndata: {}\n\n')
    })
    vi.stubGlobal('fetch', fetchMock)
    const event = vi.fn()
    await Promise.all(['a', 'b'].map(id => resumeRun(id, 12, event, new AbortController().signal)))
    expect(fetchMock.mock.calls.filter(([url]) => url.endsWith('/auth/refresh'))).toHaveLength(1)
    for (const [url, init] of fetchMock.mock.calls) if (url.endsWith('/events')) expect(new Headers(init.headers).get('Last-Event-ID')).toBe('12')
    expect(event).toHaveBeenCalledTimes(2)
  })
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
  it('rejects malformed event shapes and nonfinite sequence ids', () => {
    for (const block of ['id: Infinity\nevent: delta\ndata: {"content":"bad"}', 'id: 1\nevent: delta\ndata: []', 'id: 2\nevent: delta\ndata: {"content":{}}', 'id: 3\nevent: trace\ndata: {"node_id":"x","details":null}']) expect(parseSseBlock(block)).toBeNull()
  })
  it('releases the reader lock at EOF and stops delivering buffered frames after abort', async () => {
    const controller = new AbortController()
    const body = new ReadableStream<Uint8Array>({ start(c) { c.enqueue(new TextEncoder().encode('id: 1\nevent: delta\ndata: {"content":"first"}\n\nid: 2\nevent: delta\ndata: {"content":"late"}\n\n')); c.close() } })
    const event = vi.fn(() => controller.abort())
    await consumeSse(new Response(body), event, controller.signal)
    expect(event).toHaveBeenCalledTimes(1)
    expect(body.locked).toBe(false)
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

it('preserves HTTP rate limit metadata and never retries a rejected write', async () => {
  const fetchMock = vi.fn().mockResolvedValue(new Response('{"code":"rate_limited","message":"慢一点"}', {
    status: 429, headers: { 'Retry-After': '3' },
  }))
  vi.stubGlobal('fetch', fetchMock)
  const error = await api('/api/v1/messages/test/feedback', { method: 'PUT', body: '{"rating":"down"}' }).catch(e => e)
  expect(error).toBeInstanceOf(ApiError)
  expect(error).toMatchObject({ status: 429, code: 'rate_limited', retryAfter: 3 })
  expect(fetchMock).toHaveBeenCalledTimes(1)
})
