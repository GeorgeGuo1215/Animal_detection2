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
  trace_nodes?: TraceNode[]
}

export interface TraceNode {
  version: number
  node_id: string
  parent_id: string
  node_type: 'decision' | 'goal' | 'expert' | 'query' | 'review' | 'answer'
  status: 'pending' | 'running' | 'completed' | 'degraded' | 'failed' | 'cancelled'
  wave: number
  goal_id: string
  details: {
    intent_id?: string
    intent_name?: string
    output_variant?: string
    selected_experts?: string[]
    emergency?: boolean
    owner?: string
    capability?: string
    requirement?: string
    goal?: string
    queries?: string[]
    query?: string
    tool_name?: string
    latency_ms?: number
    scope?: string
    result?: { hits?: number; sources?: string[]; results?: number; titles?: string[] }
    sufficiency?: { status?: string; reason?: string } | null
    error?: string
    verdict?: string
    issues?: string[]
    message?: string
    finish_reason?: string
  }
}

export interface ExpertToolTrace {
  kind: 'tool'
  tool_name: string
  ok: boolean
  latency_ms: number
  result?: { hits?: number; sources?: string[]; results?: number; titles?: string[]; code?: string; status?: string; alert_level?: string }
  error?: string
  query?: string
  goal?: string
  wave?: number
  scope?: string
  sufficiency?: { status: string; reason: string }
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
