import { useCallback, useEffect, useRef, useState, type FormEvent } from 'react'
import { Link } from 'react-router-dom'
import { LayoutDashboard, UserRound, WalletCards, Clipboard, KeyRound, ShieldCheck, Check, Copy } from 'lucide-react'
import { api, newId } from './api'
import { writeClipboard } from './clipboard'
import type { AdminData, AdminOverview } from './adminTypes'
import { useAuth } from './auth'
function CopyButton({ text, className = 'copy', label = '复制' }: { text: string; className?: string; label?: string }) {
  const [copied, setCopied] = useState(false)
  const timer = useRef<number | null>(null)
  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current) }, [])
  async function copy() {
    await writeClipboard(text)
    setCopied(true)
    if (timer.current) window.clearTimeout(timer.current)
    timer.current = window.setTimeout(() => setCopied(false), 1800)
  }
  return <button type="button" className={`${className} ${copied ? 'copied' : ''}`} onClick={copy} aria-live="polite" aria-label={copied ? '已复制' : label}>{copied ? <Check /> : <Copy />}<span>{copied ? '已复制' : label}</span></button>
}

export default function AdminPage() {
  const { user } = useAuth()
  const superAdmin = user?.role === 'SUPER_ADMIN'
  const support = superAdmin || user?.role === 'SUPPORT_ADMIN'
  const billing = superAdmin || user?.role === 'BILLING_ADMIN'
  type Tab = 'overview' | 'users' | 'billing' | 'runs' | 'keys' | 'audit'
  const [tab, setTab] = useState<Tab>('overview'); const [overview, setOverview] = useState<Partial<AdminOverview>>({}); const [email, setEmail] = useState(''); const [token, setToken] = useState(''); const [data, setData] = useState<AdminData>({ users: [], invitations: [], plans: [], orders: [], subscriptions: [], runs: [], keys: [], audit: [] })
  const [error, setError] = useState('')
  const [inviting, setInviting] = useState(false)
  const [copyableToken, setCopyableToken] = useState(false)
  const request = useRef<AbortController | null>(null)
  const load = useCallback(async () => {
    request.current?.abort()
    const controller = new AbortController(); request.current = controller
    const items = async <K extends keyof AdminData>(key: K, path: string) => {
      const allowed = key === 'audit' ? superAdmin : ['plans', 'orders'].includes(key) ? billing : key === 'subscriptions' ? true : support
      if (!allowed) return [key, []] as const
      try { return [key, (await api<{items: AdminData[K]}>(path, { signal: controller.signal })).items] as const }
      catch (e) { if (!controller.signal.aborted) setError(e instanceof Error ? e.message : '管理数据加载失败'); return [key, []] as const }
    }
    const [overview, ...entries] = await Promise.all([
      api<AdminOverview>('/api/v1/admin/overview', { signal: controller.signal }).catch(() => null),
      items('users', '/api/v1/admin/users'), items('invitations', '/api/v1/admin/invitations'),
      items('plans', '/api/v1/admin/plans'), items('orders', '/api/v1/admin/orders'),
      items('subscriptions', '/api/v1/admin/subscriptions'), items('runs', '/api/v1/admin/runs'),
      items('keys', '/api/v1/admin/api-keys'), items('audit', '/api/v1/admin/audit'),
    ])
    if (controller.signal.aborted) return
    if (overview) setOverview(overview)
    setData(Object.fromEntries(entries) as unknown as AdminData)
  }, [superAdmin, support, billing])
  useEffect(() => { void load(); return () => request.current?.abort() }, [load])
  async function mutate(path: string, init: RequestInit) {
    try { setError(''); await api(path, init); await load() }
    catch (error) { if (!request.current?.signal.aborted) setError(error instanceof Error ? error.message : '操作失败') }
  }
  async function invite(e: FormEvent) {
    e.preventDefault(); if (inviting) return; setInviting(true); setError(''); setToken('')
    try { const result = await api<{development_token?: string}>('/api/v1/admin/invitations', { method: 'POST', body: JSON.stringify({ email, role: 'VET', initial_plan_code: 'trial' }) }); setCopyableToken(Boolean(result.development_token)); setToken(result.development_token || '邀请已创建，请通过邮件中的链接完成注册。'); setEmail(''); await load() }
    catch (e) { setError(e instanceof Error ? e.message : '创建邀请失败') }
    finally { setInviting(false) }
  }
  const nav: Array<[Tab, React.ReactNode, string]> = [['overview', <LayoutDashboard />, '总览与邀请'], ['users', <UserRound />, '用户'], ['billing', <WalletCards />, '套餐与订单'], ['runs', <Clipboard />, '任务'], ['keys', <KeyRound />, 'API Key'], ['audit', <ShieldCheck />, '审计']]
  const rows = <K extends keyof AdminData>(name: K, render: (row: AdminData[K][number]) => React.ReactNode) => <div className="admin-list">{(data[name] as Array<AdminData[K][number]>).map(row => <div key={'id' in row ? row.id : row.code}>{render(row)}</div>)}</div>
  return <div className="admin-shell"><aside><strong className="admin-nav-title">管理功能</strong>{nav.filter(([id]) => id === 'overview' || (id === 'audit' ? superAdmin : id === 'billing' ? billing : support)).map(([id, icon, label]) => <button key={id} className={tab === id ? 'active' : ''} onClick={() => setTab(id)}>{icon}{label}</button>)}</aside><main><header><div><p className="eyebrow">OPERATIONS</p><h1>平台管理</h1></div><Link to="/chat">返回会诊台</Link></header>{error && <div className="form-error" role="alert">{error}</div>}{tab === 'overview' && <><div className="metrics">{[['用户', overview.users], ['待确认订单', overview.pending_orders], ['运行中任务', overview.active_runs]].map(([label, value]) => <article key={String(label)}><span>{label}</span><strong>{value ?? '—'}</strong></article>)}</div>{support && <section className="admin-grid"><article className="panel"><h2>邀请新兽医</h2><form onSubmit={invite}><label>邮箱<input required type="email" value={email} onChange={e => setEmail(e.target.value)} /></label><button className="primary" disabled={inviting}>{inviting ? '正在创建…' : '创建邀请'}</button></form>{token && <div className="token-box">{token}{copyableToken && <CopyButton text={token} className="icon-copy" label="复制邀请令牌" />}</div>}</article><article className="panel"><h2>最近邀请</h2>{rows('invitations', row => <><span>{String(row.email)}</span><strong>{row.accepted_at ? '已接受' : row.revoked_at ? '已撤销' : '待接受'}</strong></>)}</article></section>}</>}{tab === 'users' && <section className="panel admin-full"><h2>用户与积分</h2>{rows('users', row => <><span><strong>{String(row.display_name)}</strong><small>{String(row.email)} · {String(row.role)}</small></span><em>{String(row.status)}</em><button disabled={!billing} onClick={() => mutate(`/api/v1/admin/users/${row.id}/credits`, { method: 'POST', body: JSON.stringify({ amount: 100, reason: 'admin_web_adjustment', idempotency_key: newId() }) })}>+100 积分</button><button disabled={!superAdmin} onClick={() => mutate(`/api/v1/admin/users/${row.id}`, { method: 'PATCH', body: JSON.stringify({ status: row.status === 'active' ? 'suspended' : 'active' }) })}>{row.status === 'active' ? '暂停' : '启用'}</button></>)}</section>}{tab === 'billing' && <section className="admin-grid"><article className="panel"><h2>套餐</h2>{rows('plans', row => <><span><strong>{String(row.name)}</strong><small>¥{(Number(row.price_cents) / 100).toFixed(2)} · {String(row.credit_grant)} 积分</small></span><button onClick={() => mutate(`/api/v1/admin/plans/${row.code}`, { method: 'PATCH', body: JSON.stringify({ active: !row.active }) })}>{row.active ? '停用' : '启用'}</button></>)}</article><article className="panel"><h2>订单</h2>{rows('orders', row => <><span><strong>{String(row.plan_code)}</strong><small>{String(row.user_id).slice(0, 10)}</small></span><em>{String(row.status)}</em>{row.status === 'pending_payment' && <button onClick={() => mutate(`/api/v1/admin/orders/${row.id}/confirm`, { method: 'POST' })}>确认到账</button>}</>)}</article><article className="panel wide"><h2>会员</h2>{rows('subscriptions', row => <><span>{String(row.user_id).slice(0, 10)} · {String(row.plan_code)}</span><em>{String(row.status)}</em><small>{row.expires_at ? new Date(String(row.expires_at)).toLocaleDateString() : '长期'}</small></>)}</article></section>}{tab === 'runs' && <section className="panel admin-full"><h2>Agent 任务</h2>{rows('runs', row => <><code>{String(row.id).slice(0, 12)}</code><span>{String(row.user_id).slice(0, 10)}</span><em>{String(row.status)}</em><small>{String(row.credits)} 积分</small></>)}</section>}{tab === 'keys' && <section className="panel admin-full"><h2>API Key</h2>{rows('keys', row => <><span><strong>{String(row.name)}</strong><small>pm_live_{String(row.prefix)}_… · {String(row.user_id).slice(0, 10)}</small></span><em>{row.revoked_at ? '已撤销' : '有效'}</em>{!row.revoked_at && <button onClick={() => mutate(`/api/v1/admin/api-keys/${row.id}`, { method: 'DELETE' })}>撤销</button>}</>)}</section>}{tab === 'audit' && <section className="panel admin-full"><h2>审计日志</h2>{rows('audit', row => <><span><strong>{String(row.action)}</strong><small>{String(row.resource_type)} · {String(row.resource_id || '')}</small></span><time>{new Date(String(row.created_at)).toLocaleString()}</time></>)}</section>}</main></div>
}
