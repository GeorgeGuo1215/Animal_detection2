import type { Conversation, Message, Plan, RunEvent, User } from './types'

const API_ROOT = (import.meta.env.VITE_API_ROOT || '').replace(/\/$/, '')
let accessToken = ''
let refreshing: Promise<boolean> | null = null

export function setAccessToken(token: string) { accessToken = token }
export function getAccessToken() { return accessToken }

async function refresh(): Promise<boolean> {
  if (!refreshing) {
    refreshing = fetch(`${API_ROOT}/api/v1/auth/refresh`, {
      method: 'POST', credentials: 'include',
    }).then(async response => {
      if (!response.ok) return false
      const data = await response.json()
      setAccessToken(data.access_token)
      return true
    }).finally(() => { refreshing = null })
  }
  return refreshing
}

export async function api<T>(path: string, init: RequestInit = {}, retry = true): Promise<T> {
  const headers = new Headers(init.headers)
  if (accessToken) headers.set('Authorization', `Bearer ${accessToken}`)
  if (init.body && !headers.has('Content-Type')) headers.set('Content-Type', 'application/json')
  if (init.method && ['POST', 'PUT', 'PATCH', 'DELETE'].includes(init.method.toUpperCase()) && !headers.has('Idempotency-Key')) {
    headers.set('Idempotency-Key', crypto.randomUUID())
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

export async function streamRun(conversationId: string, message: string, userRole: 'pet_owner' | 'veterinarian', onEvent: (event: RunEvent) => void, signal: AbortSignal, onRunId: (id: string) => void) {
  const clientMessageId = crypto.randomUUID()
  const response = await fetch(`${API_ROOT}/api/v1/conversations/${conversationId}/runs`, {
    method: 'POST', credentials: 'include', signal,
    headers: {
      'Content-Type': 'application/json', Authorization: `Bearer ${accessToken}`,
      'Idempotency-Key': clientMessageId, 'Last-Event-ID': '0',
    },
    body: JSON.stringify({ message, client_message_id: clientMessageId, delivery: 'sse', user_role: userRole }),
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
