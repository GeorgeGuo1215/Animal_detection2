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
  role: 'user' | 'assistant'
  content: string
  status: string
  created_at: string
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
