import { createContext, useContext, useEffect, useMemo, useState, type ReactNode } from 'react'
import { client, restoreSession, setAccessToken } from './api'
import type { User } from './types'

interface AuthValue { user: User | null; loading: boolean; login(email: string, password: string): Promise<void>; logout(): Promise<void> }
const AuthContext = createContext<AuthValue | null>(null)

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null)
  const [loading, setLoading] = useState(true)
  useEffect(() => { restoreSession().then(setUser).catch(() => setUser(null)).finally(() => setLoading(false)) }, [])
  const value = useMemo<AuthValue>(() => ({
    user, loading,
    async login(email, password) { const data = await client.login(email, password); setAccessToken(data.access_token); setUser(data.user) },
    async logout() { await client.logout(); setAccessToken(''); setUser(null) },
  }), [user, loading])
  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>
}

export function useAuth() {
  const value = useContext(AuthContext)
  if (!value) throw new Error('AuthProvider is missing')
  return value
}
