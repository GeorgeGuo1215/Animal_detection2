import type { Conversation, Message, Plan, RunEvent, User } from './types'

const API_ROOT = (import.meta.env.VITE_API_ROOT || '').replace(/\/$/, '')

/** UUID for HTTP public IPs where crypto.randomUUID is a secure-context-only API. */
export function newId(): string {
  const c = globalThis.crypto
  if (c && typeof c.randomUUID === 'function') return c.randomUUID()
  const bytes = new Uint8Array(16)
  if (c && typeof c.getRandomValues === 'function') c.getRandomValues(bytes)
  else for (let i = 0; i < 16; i++) bytes[i] = (Math.random() * 256) | 0
  bytes[6] = (bytes[6] & 0x0f) | 0x40
  bytes[8] = (bytes[8] & 0x3f) | 0x80
  const hex = Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('')
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`
}
let accessToken = ''
let refreshing: Promise<User | null> | null = null

export function setAccessToken(token: string) { accessToken = token }
export function getAccessToken() { return accessToken }

function wait(milliseconds: number) {
  return new Promise<void>(resolve => globalThis.setTimeout(resolve, milliseconds))
}

async function requestRefresh(retryRotated = true): Promise<User | null> {
  const response = await fetch(`${API_ROOT}/api/v1/auth/refresh`, {
    method: 'POST', credentials: 'include',
  })
  if (response.status === 409 && retryRotated) {
    // Another tab won refresh rotation. Give the shared cookie jar a moment
    // to apply that response, then retry once with the replacement cookie.
    await wait(150)
    return requestRefresh(false)
  }
  if (!response.ok) return null
  const data = await response.json() as { access_token: string; user: User }
  setAccessToken(data.access_token)
  return data.user
}

async function refresh(): Promise<User | null> {
  if (!refreshing) {
    refreshing = requestRefresh().finally(() => { refreshing = null })
  }
  return refreshing
}

export function restoreSession(): Promise<User | null> { return refresh() }

export async function api<T>(path: string, init: RequestInit = {}, retry = true): Promise<T> {
  const headers = new Headers(init.headers)
  if (accessToken) headers.set('Authorization', `Bearer ${accessToken}`)
  if (init.body && !headers.has('Content-Type')) headers.set('Content-Type', 'application/json')
  if (init.method && ['POST', 'PUT', 'PATCH', 'DELETE'].includes(init.method.toUpperCase()) && !headers.has('Idempotency-Key')) {
    headers.set('Idempotency-Key', newId())
  }
  const response = await fetch(`${API_ROOT}${path}`, { ...init, headers, credentials: 'include' })
  if (response.status === 401 && retry && await refresh()) return api<T>(path, init, false)
  if (!response.ok) {
    const payload = await response.json().catch(() => ({}))
    throw new Error(payload.message || payload.detail || `请求失败 (${response.status})`)
  }
  if (response.status === 204) return undefined as T
  return response.json()
}

export const client = {
  login: (email: string, password: string) => api<{access_token: string; user: User}>('/api/v1/auth/login', { method: 'POST', body: JSON.stringify({ email, password }) }),
  logout: () => api<void>('/api/v1/auth/logout', { method: 'POST' }),
  me: () => api<User>('/api/v1/me'),
  plans: () => api<{items: Plan[]}>('/api/v1/plans'),
  conversations: () => api<{items: Conversation[]}>('/api/v1/conversations'),
  search: (q: string) => api<{items: Conversation[]}>(`/api/v1/conversations/search?q=${encodeURIComponent(q)}`),
  createConversation: () => api<Conversation>('/api/v1/conversations', { method: 'POST', body: JSON.stringify({ title: '新会诊' }) }),
  messages: (id: string) => api<{items: Message[]}>(`/api/v1/conversations/${id}/messages`),
  rename: (id: string, title: string) => api<Conversation>(`/api/v1/conversations/${id}`, { method: 'PATCH', body: JSON.stringify({ title }) }),
  archive: (id: string) => api<Conversation>(`/api/v1/conversations/${id}`, { method: 'PATCH', body: JSON.stringify({ status: 'archived' }) }),
  remove: (id: string) => api<void>(`/api/v1/conversations/${id}`, { method: 'DELETE' }),
  forkConversation: (id: string, messageId: string) => api<Conversation>(`/api/v1/conversations/${id}/forks`, { method: 'POST', body: JSON.stringify({ message_id: messageId }) }),
  setMessageFeedback: (messageId: string, rating: 'up' | 'down' | null) => api<{message_id: string; rating: 'up' | 'down' | null; updated_at: string | null}>(`/api/v1/messages/${messageId}/feedback`, { method: 'PUT', body: JSON.stringify({ rating }) }),
}

export function parseSseBlock(block: string): RunEvent | null {
  let id = 0, event = 'message', raw = ''
  for (const line of block.split('\n')) {
    if (line.startsWith('id:')) id = Number(line.slice(3).trim())
    else if (line.startsWith('event:')) event = line.slice(6).trim()
    else if (line.startsWith('data:')) raw += line.slice(5).trim()
  }
  if (!raw) return null
  return { id, event, data: JSON.parse(raw) }
}

async function consumeSse(response: Response, onEvent: (event: RunEvent) => void) {
  if (!response.ok || !response.body) throw new Error(`会诊连接失败 (${response.status})`)
  const reader = response.body.getReader()
  const decoder = new TextDecoder()
  let buffer = ''
  while (true) {
    const { value, done } = await reader.read()
    buffer += decoder.decode(value || new Uint8Array(), { stream: !done }).replace(/\r/g, '')
    const blocks = buffer.split('\n\n')
    buffer = blocks.pop() || ''
    for (const block of blocks) {
      const parsed = parseSseBlock(block)
      if (parsed) onEvent(parsed)
    }
    if (done) break
  }
}

export async function streamRun(conversationId: string, message: string, userRole: 'pet_owner' | 'veterinarian', onEvent: (event: RunEvent) => void, signal: AbortSignal, onRunId: (id: string) => void, rewriteMessageId?: string) {
  const clientMessageId = newId()
  const response = await fetch(`${API_ROOT}/api/v1/conversations/${conversationId}/runs`, {
    method: 'POST', credentials: 'include', signal,
    headers: {
      'Content-Type': 'application/json', Authorization: `Bearer ${accessToken}`,
      'Idempotency-Key': clientMessageId, 'Last-Event-ID': '0',
    },
    body: JSON.stringify({ message, client_message_id: clientMessageId, delivery: 'sse', user_role: userRole, ...(rewriteMessageId ? { rewrite_message_id: rewriteMessageId } : {}) }),
  })
  const runId = response.headers.get('X-PetMind-Run-Id') || ''
  if (runId) onRunId(runId)
  await consumeSse(response, onEvent)
}

export async function resumeRun(runId: string, after: number, onEvent: (event: RunEvent) => void, signal: AbortSignal) {
  const response = await fetch(`${API_ROOT}/api/v1/runs/${runId}/events`, {
    credentials: 'include', signal,
    headers: { Authorization: `Bearer ${accessToken}`, 'Last-Event-ID': String(after) },
  })
  await consumeSse(response, onEvent)
}
