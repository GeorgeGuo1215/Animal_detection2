import type { User } from './types'

export interface AdminData {
  users: User[]
  invitations: { id: string; email: string; accepted_at: string | null; revoked_at: string | null }[]
  plans: { code: string; name: string; price_cents: number; credit_grant: number; active: boolean }[]
  orders: { id: string; user_id: string; plan_code: string; status: string }[]
  subscriptions: { id: string; user_id: string; plan_code: string; status: string; expires_at: string | null }[]
  runs: { id: string; user_id: string; status: string; credits: number }[]
  keys: { id: string; name: string; user_id: string; prefix: string; revoked_at: string | null }[]
  audit: { id: string; action: string; resource_type: string; resource_id: string | null; created_at: string }[]
}
export interface AdminOverview { users: number; pending_orders: number; active_runs: number }
