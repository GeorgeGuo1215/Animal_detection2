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
  source_conversation_id?: string | null
  forked_from_message_id?: string | null
  copied_messages?: number
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
  feedback_rating?: 'up' | 'down' | null
  feedback_updated_at?: string | null
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

export interface RunStatus { phase: string; message?: string; expert?: ExpertTrace }
export type TerminalStatus = 'completed' | 'failed' | 'cancelled'
export type RunEvent =
  | { id: number; event: 'status'; data: RunStatus }
  | { id: number; event: 'delta'; data: { content: string } }
  | { id: number; event: 'trace'; data: TraceNode }
  | { id: number; event: 'reset'; data: Record<string, unknown> }
  | { id: number; event: TerminalStatus; data: { message?: unknown; finish_reason?: unknown; [key: string]: unknown } }

export function isTerminalStatus(value: string): value is TerminalStatus {
  return value === 'completed' || value === 'failed' || value === 'cancelled'
}

function fields(value: Record<string, unknown>, strings: string[], arrays: string[] = [], numbers: string[] = []) {
  return strings.every(key => value[key] === undefined || typeof value[key] === 'string')
    && arrays.every(key => value[key] === undefined || (Array.isArray(value[key]) && value[key].every(x => typeof x === 'string')))
    && numbers.every(key => value[key] === undefined || (typeof value[key] === 'number' && Number.isFinite(value[key])))
}

function toolDetails(value: Record<string, unknown>) {
  if (value.result !== undefined) {
    const r = value.result
    if (!r || typeof r !== 'object' || Array.isArray(r) || !fields(r as Record<string, unknown>, ['code', 'status', 'alert_level'], ['sources', 'titles'], ['hits', 'results'])) return false
  }
  if (value.sufficiency !== undefined && value.sufficiency !== null) {
    const s = value.sufficiency
    if (typeof s !== 'object' || Array.isArray(s) || !fields(s as Record<string, unknown>, ['status', 'reason'])) return false
  }
  return true
}

export function isExpert(value: unknown): value is ExpertTrace {
  if (!value || typeof value !== 'object') return false
  const v = value as Record<string, unknown>
  if (typeof v.expert !== 'string' || typeof v.name !== 'string' || !['running', 'completed'].includes(String(v.status))) return false
  if (!fields(v, ['task', 'execution'], ['required_tools', 'recommended_tools'])) return false
  if (v.tools !== undefined && (!Array.isArray(v.tools) || !v.tools.every(t => t && typeof t === 'object' && typeof t.tool_name === 'string' && typeof t.ok === 'boolean' && fields(t, ['error', 'query', 'goal', 'scope'], [], ['wave', 'latency_ms']) && toolDetails(t)))) return false
  if (v.opinion !== undefined) {
    const p = v.opinion as Record<string, unknown> | null
    if (!p || typeof p.conclusion !== 'string' || !Array.isArray(p.evidence) || !p.evidence.every(x => typeof x === 'string') || !Array.isArray(p.risks) || !p.risks.every(x => typeof x === 'string') || !fields(p, [], [], ['confidence'])) return false
  }
  return true
}

export function isTrace(value: unknown): value is TraceNode {
  if (!value || typeof value !== 'object') return false
  const v = value as Record<string, unknown>
  if (typeof v.node_id !== 'string' || !['decision', 'goal', 'expert', 'query', 'review', 'answer'].includes(String(v.node_type)) || !['pending', 'running', 'completed', 'degraded', 'failed', 'cancelled'].includes(String(v.status))) return false
  if (!v.details || typeof v.details !== 'object' || Array.isArray(v.details)) return false
  const d = v.details as Record<string, unknown>
  return fields(v, ['parent_id', 'goal_id'], [], ['version', 'wave'])
    && fields(d, ['intent_id', 'intent_name', 'output_variant', 'owner', 'capability', 'requirement', 'goal', 'query', 'tool_name', 'scope', 'error', 'verdict', 'message', 'finish_reason'], ['selected_experts', 'queries', 'issues'], ['latency_ms'])
    && toolDetails(d)
}
