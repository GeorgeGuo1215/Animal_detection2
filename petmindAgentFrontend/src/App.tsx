import { useEffect, useMemo, useRef, useState, type FormEvent } from 'react'
import { Navigate, Route, Routes, useLocation, useNavigate, useParams } from 'react-router-dom'
import ReactMarkdown from 'react-markdown'
import rehypeSanitize from 'rehype-sanitize'
import {
  Archive, BookOpen, Check, ChevronDown, CircleHelp, Clipboard, Coins, Copy,
  Download, KeyRound, LayoutDashboard, LogOut, Menu, MessageSquarePlus, MoreHorizontal,
  PanelLeftClose, PanelLeftOpen, Search, Send, Settings, ShieldCheck, Sparkles, Square,
  Stethoscope, Trash2, UserRound, WalletCards, X,
} from 'lucide-react'
import { api, client, newId, resumeRun, setAccessToken, streamRun } from './api'
import { useAuth } from './auth'
import type { Conversation, Message, Plan, RunEvent } from './types'

const logoUrl = '/brand/petmind-logo-cropped.png'

function Brand({ compact = false }: { compact?: boolean }) {
  return <div className={`brand ${compact ? 'compact' : ''}`}><img src={logoUrl} alt="PetMind 团队标志" /><div><strong>PetMind</strong>{!compact && <span>兽医智能会诊</span>}</div></div>
}

function CopyButton({ text, className = 'copy', label = '复制' }: { text: string; className?: string; label?: string }) {
  const [copied, setCopied] = useState(false)
  const timer = useRef<number | null>(null)
  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current) }, [])
  async function copy() {
    await navigator.clipboard.writeText(text)
    setCopied(true)
    if (timer.current) window.clearTimeout(timer.current)
    timer.current = window.setTimeout(() => setCopied(false), 1800)
  }
  return <button type="button" className={`${className} ${copied ? 'copied' : ''}`} onClick={copy} aria-live="polite" aria-label={copied ? '已复制' : label}>{copied ? <Check /> : <Copy />}<span>{copied ? '已复制' : label}</span></button>
}

function PublicShell({ children }: { children: React.ReactNode }) {
  const { user, logout } = useAuth()
  const navigate = useNavigate()
  async function signOut() { await logout(); navigate('/login', { replace: true }) }
  return <main className="public-shell"><div className="paper-grain" /><nav><Brand compact /><div className="public-nav-actions">{user && <a href="/chat">返回会诊台</a>}<a href="/plans">会员计划</a>{user && <button type="button" className="public-logout" aria-label="退出登录" onClick={signOut}><LogOut /><span>退出登录</span></button>}</div></nav>{children}</main>
}

function LoginPage() {
  const { user, login } = useAuth()
  const navigate = useNavigate()
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const [busy, setBusy] = useState(false)
  if (user) return <Navigate to="/chat" replace />
  async function submit(event: FormEvent) {
    event.preventDefault(); setError(''); setBusy(true)
    try { await login(email, password); navigate('/chat') } catch (e) { setError(e instanceof Error ? e.message : '登录失败') } finally { setBusy(false) }
  }
  return <PublicShell><section className="login-layout">
    <div className="login-story"><Brand /><p className="eyebrow">PETMIND CLINICAL INTELLIGENCE</p><h1>把复杂病例，<br />梳理成清晰判断。</h1><p>汇集临床资料、知识检索与多专家复核，为每一次判断补充更完整的证据链。</p><div className="story-points"><span><Check />多专家协同复核</span><span><Check />对话与证据可追溯</span><span><Check />临床知识持续更新</span></div></div>
    <form className="auth-card" onSubmit={submit}><div className="sketch-corner" /><p className="eyebrow">YOUR CLINICAL COPILOT</p><h2>让判断更有依据，<br />让沟通更加从容。</h2><p>进入你的 PetMind 智能会诊工作台</p><label>邮箱<input autoComplete="email" type="email" required value={email} onChange={e => setEmail(e.target.value)} placeholder="name@clinic.com" /></label><label>密码<input autoComplete="current-password" type="password" required value={password} onChange={e => setPassword(e.target.value)} placeholder="至少 10 位" /></label>{error && <div className="form-error">{error}</div>}<button className="primary" disabled={busy}>{busy ? '正在验证…' : '登录工作台'}</button><a className="text-link" href="/forgot-password">忘记密码？</a><div className="invite-hint"><ShieldCheck size={17} />专业账号 · 安全访问 · 数据可追溯</div></form>
  </section></PublicShell>
}

function AcceptInvitePage() {
  const token = new URLSearchParams(useLocation().search).get('token') || ''
  const navigate = useNavigate()
  const [name, setName] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  async function submit(event: FormEvent) {
    event.preventDefault()
    try {
      const data = await api<{access_token: string}>('/api/v1/auth/invitations/accept', { method: 'POST', body: JSON.stringify({ token, display_name: name, password }) })
      setAccessToken(data.access_token); window.location.assign('/chat')
    } catch (e) { setError(e instanceof Error ? e.message : '邀请无效') }
  }
  return <PublicShell><div className="center-card"><Brand /><h1>完成兽医账号</h1><p>邀请链接同时完成邮箱验证。</p><form onSubmit={submit}><label>称呼<input required value={name} onChange={e => setName(e.target.value)} /></label><label>设置密码<input type="password" minLength={10} required value={password} onChange={e => setPassword(e.target.value)} /></label>{error && <div className="form-error">{error}</div>}<button className="primary">接受邀请</button></form></div></PublicShell>
}

function PasswordPage({ reset = false }: { reset?: boolean }) {
  const [email, setEmail] = useState(''); const [password, setPassword] = useState(''); const [done, setDone] = useState(false)
  const token = new URLSearchParams(useLocation().search).get('token') || ''
  async function submit(e: FormEvent) { e.preventDefault(); await api(reset ? '/api/v1/auth/password/reset' : '/api/v1/auth/password/forgot', { method: 'POST', body: JSON.stringify(reset ? { token, password } : { email }) }); setDone(true) }
  return <PublicShell><div className="center-card"><Brand /><h1>{reset ? '设置新密码' : '找回密码'}</h1>{done ? <><Check className="success-mark" /><p>操作已受理，请按提示继续。</p></> : <form onSubmit={submit}><label>{reset ? '新密码' : '账号邮箱'}<input required type={reset ? 'password' : 'email'} value={reset ? password : email} onChange={e => reset ? setPassword(e.target.value) : setEmail(e.target.value)} /></label><button className="primary">确认</button></form>}</div></PublicShell>
}

function PlansPage() {
  const { user } = useAuth(); const [plans, setPlans] = useState<Plan[]>([]); const [message, setMessage] = useState('')
  useEffect(() => { client.plans().then(r => setPlans(r.items)).catch(() => setPlans([])) }, [])
  async function order(code: string) { try { await api('/api/v1/orders', { method: 'POST', body: JSON.stringify({ plan_code: code }) }); setMessage('订单已创建，可在工作台查看进度。') } catch (e) { setMessage(e instanceof Error ? e.message : '创建失败') } }
  return <PublicShell><section className="plans"><p className="eyebrow">PETMIND MEMBERSHIP</p><h1>按临床工作节奏选择</h1><p>价格、积分与有效期均由平台后台配置，结算记录全程可追溯。</p>{message && <div className="notice">{message}</div>}<div className="plan-grid">{plans.map((plan, i) => <article className={i === 1 ? 'featured' : ''} key={plan.code}><span>{plan.billing_period === 'year' ? '年度' : plan.billing_period === 'month' ? '月度' : '试用'}</span><h2>{plan.name}</h2><div className="price">{plan.price_cents ? `¥${(plan.price_cents / 100).toFixed(0)}` : '免费'}</div><p>{plan.description}</p><ul><li><Check />{plan.credit_grant} 会诊积分</li><li><Check />有效 {plan.duration_days} 天</li><li><Check />对话与记忆持久化</li></ul><button disabled={!user || plan.price_cents === 0} onClick={() => order(plan.code)}>{!user ? '登录后选择' : plan.price_cents === 0 ? '邀请时开通' : '创建订单'}</button></article>)}</div></section></PublicShell>
}

function UserMenu() {
  const { user, logout } = useAuth(); const [open, setOpen] = useState(false)
  return <div className="user-menu" onMouseEnter={() => setOpen(true)} onMouseLeave={() => setOpen(false)}><button className="user-trigger" onClick={() => setOpen(!open)}><span className="avatar">{user?.display_name?.[0] || '医'}</span><span><strong>{user?.display_name}</strong><small>{user?.role}</small></span><ChevronDown size={15} /></button>{open && <div className="user-popover"><a href="/settings"><Settings />设置</a><a href="/help"><CircleHelp />帮助</a><a href="/plans"><WalletCards />会员计划</a><a href="/settings#api-keys"><KeyRound />API Key</a><button onClick={() => logout()}><LogOut />退出登录</button></div>}</div>
}

function Sidebar({ current, collapsed = false, onCollapsedChange, onSelect, onNew, onDeleted }: { current?: string; collapsed?: boolean; onCollapsedChange(collapsed: boolean): void; onSelect(id: string): void; onNew(): void; onDeleted(id: string): void }) {
  const [items, setItems] = useState<Conversation[]>([]); const [query, setQuery] = useState(''); const [actionOpen, setActionOpen] = useState(''); const [actionError, setActionError] = useState('')
  const deletedIds = useRef(new Set<string>())
  async function load(q = '') { const result = q ? await client.search(q) : await client.conversations(); setItems(result.items.filter(item => !deletedIds.current.has(item.id))) }
  async function exportConversation(item: Conversation) {
    try {
      setActionError('')
      const result = await client.messages(item.id)
      const content = [`# ${item.title}`, '', `导出时间：${new Date().toLocaleString()}`, '', ...result.items.flatMap(message => [`## ${message.role === 'user' ? '我的问题' : 'PetMind 会诊意见'}`, '', message.content, ''])].join('\n')
      const blob = new Blob([content], { type: 'text/markdown;charset=utf-8' })
      const url = URL.createObjectURL(blob)
      const link = document.createElement('a')
      link.href = url
      link.download = `${item.title.replace(/[<>:"/\\|?*\u0000-\u001F]/g, '_').slice(0, 60) || 'PetMind会诊'}.md`
      document.body.appendChild(link); link.click(); link.remove(); URL.revokeObjectURL(url)
      setActionOpen('')
    } catch (error) { setActionError(error instanceof Error ? error.message : '导出失败') }
  }
  async function deleteConversation(item: Conversation) {
    if (!window.confirm(`确定删除“${item.title}”吗？删除后将无法在对话列表中恢复。`)) return
    try {
      setActionError(''); await client.remove(item.id); deletedIds.current.add(item.id); setItems(previous => previous.filter(row => row.id !== item.id)); setActionOpen(''); onDeleted(item.id)
    } catch (error) { setActionError(error instanceof Error ? error.message : '删除失败') }
  }
  useEffect(() => { load().catch(() => undefined) }, [current])
  useEffect(() => { const timer = setTimeout(() => load(query).catch(() => undefined), 250); return () => clearTimeout(timer) }, [query])
  return <aside className={`sidebar ${collapsed ? 'collapsed' : ''}`}><header><Brand compact /><button type="button" aria-label={collapsed ? '展开侧边栏' : '收起侧边栏'} aria-expanded={!collapsed} onClick={() => onCollapsedChange(!collapsed)}>{collapsed ? <PanelLeftOpen /> : <PanelLeftClose />}</button></header><button className="new-chat" aria-label="新建会诊" title={collapsed ? '新建会诊' : undefined} onClick={onNew}><MessageSquarePlus /> <span>新建会诊</span></button><div className="search-box"><Search /><input aria-label="搜索对话" placeholder="搜索病例与回答" value={query} onChange={e => setQuery(e.target.value)} /></div><div className="history"><small>最近会诊</small>{actionError && <div className="history-error">{actionError}</div>}{items.map(item => <div className={`history-row ${current === item.id ? 'active' : ''}`} key={item.id}><button className="history-main" onClick={() => onSelect(item.id)}><span>{item.title}</span><time>{new Date(item.last_active_at).toLocaleDateString()}</time></button><div className="history-actions"><button type="button" aria-label={`管理对话：${item.title}`} aria-expanded={actionOpen === item.id} onClick={() => setActionOpen(actionOpen === item.id ? '' : item.id)}><MoreHorizontal /></button>{actionOpen === item.id && <div className="history-menu" role="menu"><button type="button" role="menuitem" onClick={() => exportConversation(item)}><Download />导出 Markdown</button><button type="button" role="menuitem" className="danger" onClick={() => deleteConversation(item)}><Trash2 />删除对话</button></div>}</div></div>)}</div><UserMenu /></aside>
}

const phaseLabels: Record<string, string> = { queued: '等待会诊资源', reconnecting: '正在恢复会诊流', understanding: '理解问题', routing: '组织会诊路径', consulting: '专家会诊', reviewing: '安全复核', generating: '整理答复' }
type AudienceRole = 'veterinarian' | 'pet_owner'

interface ExpertToolTrace {
  kind: 'tool'
  tool_name: string
  ok: boolean
  latency_ms: number
  result?: { hits?: number; sources?: string[]; results?: number; titles?: string[]; code?: string; status?: string; alert_level?: string }
  error?: string
}

interface ExpertTrace {
  expert: string
  name: string
  status: 'running' | 'completed'
  task?: string
  tools?: ExpertToolTrace[]
  opinion?: { conclusion: string; evidence: string[]; risks: string[]; confidence: number }
  execution?: 'single_pass'
}

function ExpertConsultation({ experts }: { experts: ExpertTrace[] }) {
  if (!experts.length) return null
  return <section className="expert-consultation" aria-label="专家会诊过程">
    <div className="expert-consultation-heading"><Stethoscope /><div><strong>专家会诊</strong><small>可展开查看脱敏后的任务、工具与专家意见</small></div></div>
    {experts.map(expert => <details className="expert-thread" key={expert.expert}>
      <summary><span className={`expert-dot ${expert.status}`} /><span><strong>{expert.name}</strong><small>{expert.status === 'completed' ? '已提交结构化意见' : '正在执行任务'}</small></span><ChevronDown /></summary>
      <div className="expert-thread-body">
        <div className="expert-step"><span>任务</span><p>{expert.task || '根据统一任务策略分析当前病例'}</p></div>
        {(expert.tools || []).map((tool, index) => <div className="expert-step" key={`${tool.tool_name}-${index}`}><span>工具</span><div><strong>{tool.tool_name}</strong><small>{tool.ok ? '调用完成' : '调用失败'} · {(tool.latency_ms / 1000).toFixed(1)} 秒</small>{tool.result?.hits !== undefined && <p>知识库命中 {tool.result.hits} 条{tool.result.sources?.length ? ` · ${tool.result.sources.join('、')}` : ''}</p>}{tool.result?.results !== undefined && <p>网络结果 {tool.result.results} 条{tool.result.titles?.length ? ` · ${tool.result.titles.join('、')}` : ''}</p>}{tool.error && <p className="expert-step-error">{tool.error}</p>}</div></div>)}
        {expert.opinion && <div className="expert-step"><span>意见</span><div><p>{expert.opinion.conclusion}</p>{expert.opinion.evidence.length > 0 && <ul>{expert.opinion.evidence.map(item => <li key={item}>{item}</li>)}</ul>}{expert.opinion.risks.length > 0 && <small>风险边界：{expert.opinion.risks.join('；')}</small>}</div></div>}
        <p className="expert-privacy">仅展示任务结果，不展示系统提示词或模型内部推理。</p>
      </div>
    </details>)}
  </section>
}

function ChatModeSelector({ role, onRoleChange }: { role: AudienceRole; onRoleChange(role: AudienceRole): void }) {
  const [open, setOpen] = useState(false)
  const root = useRef<HTMLDivElement | null>(null)
  useEffect(() => {
    function close(event: MouseEvent) { if (!root.current?.contains(event.target as Node)) setOpen(false) }
    document.addEventListener('mousedown', close)
    return () => document.removeEventListener('mousedown', close)
  }, [])
  return <div className="chat-mode" ref={root}>
    <button type="button" className="chat-mode-trigger" onClick={() => setOpen(!open)} aria-expanded={open} aria-label="选择模型和回答身份">
      <span className="online-dot" /><span><strong>PetMind Clinical MoE</strong><small>{role === 'veterinarian' ? '兽医专业模式' : '宠物主沟通模式'}</small></span><ChevronDown />
    </button>
    {open && <div className="chat-mode-popover">
      <p className="eyebrow">CONVERSATION MODE</p>
      <label>模型<select value="petmind-clinical-moe" disabled><option value="petmind-clinical-moe">PetMind Clinical MoE</option></select></label>
      <fieldset><legend>回答身份</legend><button type="button" className={role === 'veterinarian' ? 'selected' : ''} onClick={() => onRoleChange('veterinarian')}><Stethoscope /><span><strong>兽医</strong><small>专业术语、鉴别诊断与证据边界</small></span>{role === 'veterinarian' && <Check />}</button><button type="button" className={role === 'pet_owner' ? 'selected' : ''} onClick={() => onRoleChange('pet_owner')}><UserRound /><span><strong>宠物主</strong><small>通俗解释、观察重点与就医建议</small></span>{role === 'pet_owner' && <Check />}</button></fieldset>
      <p className="mode-note">身份会影响回答表达和安全边界，不改变病例原始事实。</p>
    </div>}
  </div>
}

function ChatPage() {
  const navigate = useNavigate(); const { conversationId } = useParams(); const [messages, setMessages] = useState<Message[]>([]); const [input, setInput] = useState(''); const [phase, setPhase] = useState(''); const [busy, setBusy] = useState(false); const [error, setError] = useState(''); const [expertTraces, setExpertTraces] = useState<ExpertTrace[]>([]); const [mobileNav, setMobileNav] = useState(false); const [sidebarCollapsed, setSidebarCollapsed] = useState(false); const [detailsOpen, setDetailsOpen] = useState(false)
  const [audienceRole, setAudienceRole] = useState<AudienceRole>(() => localStorage.getItem('petmind-audience-role') === 'pet_owner' ? 'pet_owner' : 'veterinarian')
  const controller = useRef<AbortController | null>(null); const activeRun = useRef(''); const activeConversation = useRef(conversationId || ''); const pendingConversationNavigation = useRef(''); const lastEvent = useRef(0); const draftAnswer = useRef('')
  function changeAudienceRole(role: AudienceRole) { setAudienceRole(role); localStorage.setItem('petmind-audience-role', role) }
  useEffect(() => {
    if (!conversationId) { activeConversation.current = ''; setMessages([]); setExpertTraces([]); setError(''); return }
    activeConversation.current = conversationId
    setError('')
    if (pendingConversationNavigation.current === conversationId) {
      pendingConversationNavigation.current = ''
      return
    }
    void refreshMessages(conversationId).catch(error => setError(error instanceof Error ? error.message : '会话加载失败'))
    const saved = sessionStorage.getItem(`petmind-run:${conversationId}`)
    const recoveryController = new AbortController()
    if (saved) {
      try {
        const state = JSON.parse(saved) as { runId: string; lastEvent: number }
        controller.current = recoveryController
        activeRun.current = state.runId; lastEvent.current = state.lastEvent; setBusy(true); setPhase('queued'); draftAnswer.current = ''
        void recoverRun(state.runId, recoveryController.signal)
      } catch { sessionStorage.removeItem(`petmind-run:${conversationId}`) }
    }
    return () => recoveryController.abort()
  }, [conversationId])
  async function create() { const c = await client.createConversation(); navigate(`/chat/${c.id}`) }
  async function refreshMessages(id: string) {
    const result = await client.messages(id)
    if (activeConversation.current === id) setMessages(result.items)
    return result.items
  }
  function finishRun(event: RunEvent) {
    const id = activeConversation.current
    if (id) sessionStorage.removeItem(`petmind-run:${id}`)
    activeRun.current = ''; draftAnswer.current = ''; setPhase(''); setBusy(false)
    if (event.event === 'failed') setError(String(event.data.message || '会诊任务执行失败，请稍后重试'))
    else if (event.event === 'completed' && event.data.finish_reason === 'truncated') setError('回答已达到长度上限，内容可能不完整，请继续追问。')
    else setError('')
    if (id) void refreshMessages(id).catch(error => {
      if (event.event === 'completed') setError(error instanceof Error ? error.message : '最终回答加载失败，请刷新重试')
    })
  }
  function handleEvent(event: RunEvent) {
    lastEvent.current = Math.max(lastEvent.current, event.id)
    if (activeConversation.current && activeRun.current) sessionStorage.setItem(`petmind-run:${activeConversation.current}`, JSON.stringify({ runId: activeRun.current, lastEvent: lastEvent.current }))
    if (event.event === 'status') {
      setError(''); setPhase(String(event.data.phase || ''))
      const incoming = event.data.expert as ExpertTrace | undefined
      if (incoming?.expert) setExpertTraces(previous => {
        const next = previous.filter(item => item.expert !== incoming.expert)
        return [...next, incoming]
      })
    }
    if (event.event === 'delta') {
      setError('')
      draftAnswer.current += String(event.data.content || '')
      setMessages(prev => [...prev.filter(m => m.id !== 'streaming'), { id: 'streaming', role: 'assistant', content: draftAnswer.current, status: 'streaming', created_at: new Date().toISOString() }])
    }
    if (['completed', 'failed', 'cancelled'].includes(event.event)) finishRun(event)
  }
  async function recoverRun(runId: string, signal: AbortSignal) {
    let attempts = 0
    while (!signal.aborted && activeRun.current === runId) {
      try {
        await resumeRun(runId, lastEvent.current, handleEvent, signal)
        if (signal.aborted || activeRun.current !== runId) return
      } catch {
        if (signal.aborted || activeRun.current !== runId) return
      }
      try {
        const snapshot = await api<{status: string; error?: {message?: string} | null}>(`/api/v1/runs/${runId}`)
        if (['completed', 'failed', 'cancelled'].includes(snapshot.status)) {
          finishRun({ id: lastEvent.current, event: snapshot.status, data: { message: snapshot.error?.message || '' } })
          return
        }
      } catch { /* the next retry also refreshes authentication when needed */ }
      if (signal.aborted || activeRun.current !== runId) return
      attempts += 1; setError(''); setPhase('reconnecting')
      await new Promise<void>(resolve => {
        const timer = window.setTimeout(done, Math.min(5000, 800 + attempts * 500))
        function done() { window.clearTimeout(timer); signal.removeEventListener('abort', done); resolve() }
        signal.addEventListener('abort', done, { once: true })
      })
    }
  }
  async function send() {
    const text = input.trim(); if (!text || busy) return
    let id = conversationId
    if (!id) { const c = await client.createConversation(); id = c.id; activeConversation.current = id; pendingConversationNavigation.current = id; navigate(`/chat/${id}`, { replace: true }) }
    setMessages(prev => [...prev, { id: newId(), role: 'user', content: text, status: 'complete', created_at: new Date().toISOString() }]); setInput(''); setBusy(true); setError(''); setExpertTraces([]); setPhase('queued'); draftAnswer.current = ''; lastEvent.current = 0
    controller.current = new AbortController()
    try {
      await streamRun(id!, text, audienceRole, handleEvent, controller.current.signal, runId => { activeRun.current = runId; sessionStorage.setItem(`petmind-run:${id}`, JSON.stringify({ runId, lastEvent: lastEvent.current })) })
    } catch (e) {
      if (controller.current?.signal.aborted) return
      if (activeRun.current) {
        await recoverRun(activeRun.current, controller.current!.signal); return
      }
      setError(e instanceof Error ? e.message : '连接中断'); setBusy(false)
    }
  }
  async function stop() { if (activeRun.current) await api(`/api/v1/runs/${activeRun.current}`, { method: 'DELETE' }).catch(() => undefined); if (activeConversation.current) sessionStorage.removeItem(`petmind-run:${activeConversation.current}`); controller.current?.abort(); setBusy(false); setPhase('') }
  function handleDeleted(id: string) { if (id === conversationId) { setMessages([]); setExpertTraces([]); setMobileNav(false); navigate('/chat', { replace: true }) } }
  return <div className={`workspace ${sidebarCollapsed ? 'sidebar-collapsed' : ''}`}><div className={mobileNav ? 'mobile-sidebar shown' : 'mobile-sidebar'}><Sidebar current={conversationId} onCollapsedChange={() => setMobileNav(false)} onSelect={id => { navigate(`/chat/${id}`); setMobileNav(false) }} onNew={create} onDeleted={handleDeleted} /></div><Sidebar current={conversationId} collapsed={sidebarCollapsed} onCollapsedChange={setSidebarCollapsed} onSelect={id => navigate(`/chat/${id}`)} onNew={create} onDeleted={handleDeleted} /><section className="chat-main"><header className="chat-top"><button className="mobile-menu" aria-label="打开侧边栏" onClick={() => setMobileNav(!mobileNav)}><Menu /></button><ChatModeSelector role={audienceRole} onRoleChange={changeAudienceRole} /><div className="top-actions"><a href="/plans"><Coins />积分</a><div className="conversation-more"><button type="button" aria-label="会诊详情" aria-expanded={detailsOpen} onClick={() => setDetailsOpen(!detailsOpen)}><MoreHorizontal /></button>{detailsOpen && <div className="conversation-details"><p className="eyebrow">SESSION DETAILS</p><h3>当前会诊</h3><dl><div><dt>模型</dt><dd>PetMind Clinical MoE</dd></div><div><dt>回答身份</dt><dd>{audienceRole === 'veterinarian' ? '兽医专业模式' : '宠物主沟通模式'}</dd></div><div><dt>会话状态</dt><dd>{busy ? phaseLabels[phase] || '处理中' : '随时可用'}</dd></div></dl><a href="/settings"><Settings />个人设置</a><a href="/help"><CircleHelp />使用帮助</a><a href="/plans"><WalletCards />会员与积分</a></div>}</div></div></header><div className="message-scroll">{messages.length === 0 ? <div className="empty-chat"><img src={logoUrl} alt="" /><p className="eyebrow">PETMIND CLINICAL DESK</p><h1>今天需要一起梳理<br />哪个病例？</h1><p>请提供物种、年龄、主诉、症状时间线和已有检查。系统会组织资料检索与专家复核。</p><div className="suggestions">{['猫频繁进出猫砂盆，如何排急症？', '犬持续咳嗽的鉴别诊断路径', '帮我解读这组肝功能指标'].map(q => <button key={q} onClick={() => setInput(q)}>{q}</button>)}</div></div> : <div className="messages">{messages.map(message => <article key={message.id} className={`message ${message.role}`}><div className="message-label">{message.role === 'user' ? '我的问题' : <><Stethoscope />PetMind 会诊意见</>}</div><div className="message-body">{message.role === 'assistant' ? <ReactMarkdown rehypePlugins={[rehypeSanitize]}>{message.content}</ReactMarkdown> : message.content}</div>{message.role === 'assistant' && message.status !== 'streaming' && <CopyButton text={message.content} />}</article>)}<ExpertConsultation experts={expertTraces} />{phase && <div className="phase-card"><span className="phase-spinner" /><div><strong>{phaseLabels[phase] || '处理中'}</strong><small>仅展示任务阶段，不暴露模型内部推理</small></div></div>}{error && <div className="form-error">{error}</div>}</div>}</div><div className="composer-wrap"><div className="composer"><textarea aria-label="输入病例" rows={1} value={input} onChange={e => setInput(e.target.value)} onKeyDown={e => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); send() } }} placeholder={audienceRole === 'veterinarian' ? '描述病例，Shift + Enter 换行' : '描述宠物的症状和变化，Shift + Enter 换行'} /><button className={busy ? 'stop' : 'send'} aria-label={busy ? '停止生成' : '发送'} onClick={busy ? stop : send}>{busy ? <Square /> : <Send />}</button></div><small>PetMind 可能出错，请结合体检、实验室与影像结果独立判断。</small></div></section></div>
}

function AdminPage() {
  type Tab = 'overview' | 'users' | 'billing' | 'runs' | 'keys' | 'audit'
  const [tab, setTab] = useState<Tab>('overview'); const [overview, setOverview] = useState<Record<string, number>>({}); const [email, setEmail] = useState(''); const [token, setToken] = useState(''); const [data, setData] = useState<Record<string, Array<Record<string, unknown>>>>({})
  async function items(path: string) { return (await api<{items: Array<Record<string, unknown>>}>(path)).items }
  async function load() {
    const results = await Promise.allSettled([
      api<Record<string, number>>('/api/v1/admin/overview'), items('/api/v1/admin/users'), items('/api/v1/admin/invitations'), items('/api/v1/admin/plans'),
      items('/api/v1/admin/orders'), items('/api/v1/admin/subscriptions'), items('/api/v1/admin/runs'), items('/api/v1/admin/api-keys'), items('/api/v1/admin/audit'),
    ])
    if (results[0].status === 'fulfilled') setOverview(results[0].value)
    const names = ['users', 'invitations', 'plans', 'orders', 'subscriptions', 'runs', 'keys', 'audit']
    setData(Object.fromEntries(names.map((name, index) => {
      const outcome = results[index + 1]
      return [name, outcome.status === 'fulfilled' && Array.isArray(outcome.value) ? outcome.value : []]
    })))
  }
  useEffect(() => { load().catch(() => undefined) }, [])
  async function invite(e: FormEvent) { e.preventDefault(); const result = await api<{development_token?: string}>('/api/v1/admin/invitations', { method: 'POST', body: JSON.stringify({ email, role: 'VET', initial_plan_code: 'trial' }) }); setToken(result.development_token || '邀请已写入邮件 Outbox'); setEmail(''); await load() }
  const nav: Array<[Tab, React.ReactNode, string]> = [['overview', <LayoutDashboard />, '总览与邀请'], ['users', <UserRound />, '用户'], ['billing', <WalletCards />, '套餐与订单'], ['runs', <Clipboard />, '任务'], ['keys', <KeyRound />, 'API Key'], ['audit', <ShieldCheck />, '审计']]
  const rows = (name: string, render: (row: Record<string, unknown>) => React.ReactNode) => <div className="admin-list">{(data[name] || []).map(row => <div key={String(row.id || row.code)}>{render(row)}</div>)}</div>
  return <div className="admin-shell"><aside><Brand compact />{nav.map(([id, icon, label]) => <button key={id} className={tab === id ? 'active' : ''} onClick={() => setTab(id)}>{icon}{label}</button>)}</aside><main><header><div><p className="eyebrow">OPERATIONS</p><h1>平台管理</h1></div><a href="/chat">返回会诊台</a></header>{tab === 'overview' && <><div className="metrics">{[['用户', overview.users], ['待确认订单', overview.pending_orders], ['运行中任务', overview.active_runs]].map(([label, value]) => <article key={String(label)}><span>{label}</span><strong>{value ?? '—'}</strong></article>)}</div><section className="admin-grid"><article className="panel"><h2>邀请新兽医</h2><form onSubmit={invite}><label>邮箱<input required type="email" value={email} onChange={e => setEmail(e.target.value)} /></label><button className="primary">创建邀请</button></form>{token && <div className="token-box">{token}<CopyButton text={token} className="icon-copy" label="复制邀请令牌" /></div>}</article><article className="panel"><h2>最近邀请</h2>{rows('invitations', row => <><span>{String(row.email)}</span><strong>{row.accepted_at ? '已接受' : row.revoked_at ? '已撤销' : '待接受'}</strong></>)}</article></section></>}{tab === 'users' && <section className="panel admin-full"><h2>用户与积分</h2>{rows('users', row => <><span><strong>{String(row.display_name)}</strong><small>{String(row.email)} · {String(row.role)}</small></span><em>{String(row.status)}</em><button onClick={() => api(`/api/v1/admin/users/${row.id}/credits`, { method: 'POST', body: JSON.stringify({ amount: 100, reason: 'admin_web_adjustment', idempotency_key: newId() }) }).then(load)}>+100 积分</button><button onClick={() => api(`/api/v1/admin/users/${row.id}`, { method: 'PATCH', body: JSON.stringify({ status: row.status === 'active' ? 'suspended' : 'active' }) }).then(load)}>{row.status === 'active' ? '暂停' : '启用'}</button></>)}</section>}{tab === 'billing' && <section className="admin-grid"><article className="panel"><h2>套餐</h2>{rows('plans', row => <><span><strong>{String(row.name)}</strong><small>¥{(Number(row.price_cents) / 100).toFixed(2)} · {String(row.credit_grant)} 积分</small></span><button onClick={() => api(`/api/v1/admin/plans/${row.code}`, { method: 'PATCH', body: JSON.stringify({ active: !row.active }) }).then(load)}>{row.active ? '停用' : '启用'}</button></>)}</article><article className="panel"><h2>订单</h2>{rows('orders', row => <><span><strong>{String(row.plan_code)}</strong><small>{String(row.user_id).slice(0, 10)}</small></span><em>{String(row.status)}</em>{row.status === 'pending_payment' && <button onClick={() => api(`/api/v1/admin/orders/${row.id}/confirm`, { method: 'POST' }).then(load)}>确认到账</button>}</>)}</article><article className="panel wide"><h2>会员</h2>{rows('subscriptions', row => <><span>{String(row.user_id).slice(0, 10)} · {String(row.plan_code)}</span><em>{String(row.status)}</em><small>{row.expires_at ? new Date(String(row.expires_at)).toLocaleDateString() : '长期'}</small></>)}</article></section>}{tab === 'runs' && <section className="panel admin-full"><h2>Agent 任务</h2>{rows('runs', row => <><code>{String(row.id).slice(0, 12)}</code><span>{String(row.user_id).slice(0, 10)}</span><em>{String(row.status)}</em><small>{String(row.credits)} 积分</small></>)}</section>}{tab === 'keys' && <section className="panel admin-full"><h2>API Key</h2>{rows('keys', row => <><span><strong>{String(row.name)}</strong><small>pm_live_{String(row.prefix)}_… · {String(row.user_id).slice(0, 10)}</small></span><em>{row.revoked_at ? '已撤销' : '有效'}</em>{!row.revoked_at && <button onClick={() => api(`/api/v1/admin/api-keys/${row.id}`, { method: 'DELETE' }).then(load)}>撤销</button>}</>)}</section>}{tab === 'audit' && <section className="panel admin-full"><h2>审计日志</h2>{rows('audit', row => <><span><strong>{String(row.action)}</strong><small>{String(row.resource_type)} · {String(row.resource_id || '')}</small></span><time>{new Date(String(row.created_at)).toLocaleString()}</time></>)}</section>}</main></div>
}

interface UserApiKey {
  id: string
  name: string
  prefix: string
  scopes: string[]
  expires_at: string | null
  revoked_at: string | null
  last_used_at: string | null
  created_at: string
}

function AccountPage() {
  const { user } = useAuth()
  const [credits, setCredits] = useState<{balance: number; reserved: number; ledger: Array<Record<string, string | number>>}>({ balance: 0, reserved: 0, ledger: [] })
  const [subscription, setSubscription] = useState<Record<string, string> | null>(null)
  const [orders, setOrders] = useState<Array<Record<string, string | number>>>([])
  const [keys, setKeys] = useState<UserApiKey[]>([])
  const [keyName, setKeyName] = useState('诊所系统')
  const [newKey, setNewKey] = useState('')
  const [keyNotice, setKeyNotice] = useState('')
  const [keyError, setKeyError] = useState('')
  const [revokingKey, setRevokingKey] = useState('')
  async function load() {
    const [creditData, subscriptionData, orderData, keyData] = await Promise.all([
      api<typeof credits>('/api/v1/credits'), api<{subscription: Record<string, string> | null}>('/api/v1/subscription'),
      api<{items: Array<Record<string, string | number>>}>('/api/v1/orders'), api<{items: UserApiKey[]}>('/api/v1/me/api-keys'),
    ])
    setCredits(creditData); setSubscription(subscriptionData.subscription); setOrders(orderData.items); setKeys(keyData.items)
  }
  useEffect(() => { load().catch(() => undefined) }, [])
  async function createKey(e: FormEvent) {
    e.preventDefault()
    setKeyError(''); setKeyNotice('')
    try {
      const created = await api<{key: string}>('/api/v1/me/api-keys', { method: 'POST', body: JSON.stringify({ name: keyName, scopes: ['chat:write', 'models:read', 'runs:read'] }) })
      setNewKey(created.key); setKeyNotice('API Key 已创建，请立即保存完整密钥。'); await load()
    } catch (error) { setKeyError(error instanceof Error ? error.message : 'API Key 创建失败') }
  }
  async function revokeKey(key: UserApiKey) {
    if (key.revoked_at || !window.confirm(`确定撤销“${key.name}”吗？使用该 Key 的客户端将立即无法访问。`)) return
    setRevokingKey(key.id); setKeyError(''); setKeyNotice('')
    try {
      await api(`/api/v1/me/api-keys/${key.id}`, { method: 'DELETE' })
      await load(); setNewKey(''); setKeyNotice(`“${key.name}”已撤销，原密钥已立即失效。`)
    } catch (error) { setKeyError(error instanceof Error ? error.message : 'API Key 撤销失败') }
    finally { setRevokingKey('') }
  }
  return <div className="account-page"><header><Brand compact /><nav><a href="/chat">会诊台</a><a href="/plans">会员计划</a></nav></header><main><div className="account-heading"><p className="eyebrow">ACCOUNT & BILLING</p><h1>个人设置</h1><p>{user?.display_name} · {user?.email}</p></div><div className="account-grid"><section className="panel"><h2>会员与积分</h2><div className="credit-balance"><Coins /><strong>{credits.balance}</strong><span>可用积分</span><small>{credits.reserved} 积分正在预占</small></div><p className="muted">当前套餐：{subscription ? `${subscription.plan_code}，有效至 ${new Date(subscription.expires_at).toLocaleDateString()}` : '暂无有效套餐'}</p><a className="outline-link" href="/plans">查看会员计划</a></section><section className="panel" id="api-keys"><h2>API Key</h2><p className="muted">完整密钥只在创建时显示一次，仅用于 OpenAI 兼容接口。</p><form className="inline-form" onSubmit={createKey}><input required value={keyName} onChange={e => setKeyName(e.target.value)} maxLength={80} /><button className="primary">创建</button></form>{keyNotice && <div className="key-feedback success">{keyNotice}</div>}{keyError && <div className="key-feedback error">{keyError}</div>}{newKey && <div className="one-time-key"><strong>请立即安全保存</strong><code>{newKey}</code><CopyButton text={newKey} /></div>}<div className="key-list">{keys.map(key => <div className={key.revoked_at ? 'revoked' : ''} key={key.id}><span><strong>{key.name}</strong><small>pm_live_{key.prefix}_… · 创建于 {new Date(key.created_at).toLocaleDateString()}</small></span><div className="key-actions"><em>{key.revoked_at ? `已撤销 · ${new Date(key.revoked_at).toLocaleDateString()}` : '有效'}</em>{!key.revoked_at && <button disabled={revokingKey === key.id} onClick={() => revokeKey(key)}><X />{revokingKey === key.id ? '撤销中…' : '撤销'}</button>}</div></div>)}</div></section><section className="panel wide"><h2>订单记录</h2>{orders.length ? orders.map(order => <div className="order-row" key={String(order.id)}><span>{order.plan_code}<small>{new Date(String(order.created_at)).toLocaleString()}</small></span><strong>{order.status}</strong><span>¥{(Number(order.amount_cents) / 100).toFixed(2)}</span></div>) : <p className="muted">暂无订单</p>}</section><section className="panel wide"><h2>最近积分流水</h2>{credits.ledger.slice(0, 8).map(row => <div className="ledger-row" key={String(row.id)}><span>{row.reason}</span><strong>{Number(row.amount) > 0 ? '+' : ''}{row.amount}</strong><small>余额 {row.balance_after}</small></div>)}</section></div></main></div>
}

function HelpPage() {
  return <PublicShell><div className="help-page"><Brand /><p className="eyebrow">CLINICAL SUPPORT GUIDE</p><h1>让信息更完整，判断更可靠</h1><div className="help-grid"><article><BookOpen /><h2>如何描述病例</h2><p>依次提供物种、年龄、性别与绝育状态、主诉、时间线、体检和已有检查。缺失信息可以明确写“未知”。</p></article><article><Sparkles /><h2>如何理解回答</h2><p>系统会给出支持证据、反对证据与缺失信息。它是临床决策支持，不替代查体、化验、影像与兽医最终判断。</p></article><article><ShieldCheck /><h2>数据与安全</h2><p>不要提交无关个人信息。系统不会向网页暴露内部提示词、模型推理过程或原始工具载荷。</p></article></div><a className="primary back-chat" href="/chat">返回会诊台</a></div></PublicShell>
}

function Protected({ children, admin = false }: { children: React.ReactNode; admin?: boolean }) {
  const { user, loading } = useAuth(); if (loading) return <div className="app-loading"><Brand /><span /></div>; if (!user) return <Navigate to="/login" replace />
  if (admin && user.role === 'VET') return <Navigate to="/chat" replace />
  return children
}

export default function App() {
  return <Routes><Route path="/" element={<Navigate to="/chat" replace />} /><Route path="/login" element={<LoginPage />} /><Route path="/accept-invite" element={<AcceptInvitePage />} /><Route path="/forgot-password" element={<PasswordPage />} /><Route path="/reset-password" element={<PasswordPage reset />} /><Route path="/plans" element={<PlansPage />} /><Route path="/help" element={<HelpPage />} /><Route path="/settings" element={<Protected><AccountPage /></Protected>} /><Route path="/chat" element={<Protected><ChatPage /></Protected>} /><Route path="/chat/:conversationId" element={<Protected><ChatPage /></Protected>} /><Route path="/admin" element={<Protected admin><AdminPage /></Protected>} /><Route path="*" element={<Navigate to="/chat" replace />} /></Routes>
}
