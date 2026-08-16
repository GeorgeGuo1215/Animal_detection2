export type UserRole = 'VET' | 'SUPPORT_ADMIN' | 'BILLING_ADMIN' | 'SUPER_ADMIN'

export interface User {
  id: string
  email: string
  display_name: string
  role: UserRole
  status: string
}

export interface Conversation {
  id: string
  title: string
  status: string
  created_at: string
  last_active_at: string
  snippet?: string
}

export interface Message {
  id: string
  run_id?: string | null
  role: 'user' | 'assistant'
  content: string
  status: string
  created_at: string
  expert_consultations?: ExpertTrace[]
}

export interface ExpertToolTrace {
  kind: 'tool'
  tool_name: string
  ok: boolean
  latency_ms: number
  result?: { hits?: number; sources?: string[]; results?: number; titles?: string[]; code?: string; status?: string; alert_level?: string }
  error?: string
}

export interface ExpertTrace {
  expert: string
  name: string
  status: 'running' | 'completed'
  task?: string
  required_tools?: string[]
  recommended_tools?: string[]
  tools?: ExpertToolTrace[]
  opinion?: { conclusion: string; evidence: string[]; risks: string[]; confidence: number }
  execution?: 'single_pass'
}

export interface Plan {
  code: string
  name: string
  description: string
  billing_period: string
  price_cents: number
  currency: string
  credit_grant: number
  duration_days: number
  features: Record<string, boolean>
}

export interface RunEvent {
  id: number
  event: string
  data: Record<string, unknown>
}
