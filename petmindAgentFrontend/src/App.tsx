import { createContext, useContext, useEffect, useRef, useState, lazy, Suspense, type FormEvent } from 'react'
import { Link, Navigate, Outlet, Route, Routes, useLocation, useNavigate } from 'react-router-dom'
import { AssistantAnswer } from './AssistantAnswer'
import {
  BookOpen, Check, ChevronDown, CircleHelp, Coins,
  Download, KeyRound, LogOut, Menu, MessageSquarePlus, MoreHorizontal,
  PanelLeftClose, PanelLeftOpen, Search, Send, Settings, ShieldCheck, Sparkles, Square,
  Stethoscope, Trash2, UserRound, WalletCards, X,
} from 'lucide-react'
import { api, client, newId } from './api'
import { phaseLabels } from './agentProcess'
import { useAuth } from './auth'
import { useConfirmation } from './Confirmation'
import { useRunStream } from './useRunStream'
import { CONVERSATION_UPSERT_EVENT, CONVERSATION_DELETE_EVENT, announceConversationDeleted, type ConversationUpdate } from './conversationEvents'
import type { Conversation, Plan } from './types'
const SettingsPage = lazy(() => import('./SettingsPage').then(m => ({ default: m.SettingsPage })))
const ActivationPage = lazy(() => import('./SettingsPage').then(m => ({ default: m.ActivationPage })))
const FeedbackPage = lazy(() => import('./SettingsPage').then(m => ({ default: m.FeedbackPage })))
const LegalPage = lazy(() => import('./SettingsPage').then(m => ({ default: m.LegalPage })))
const AdminPage = lazy(() => import('./AdminPage'))
import { MessageActions } from './MessageActions'
import { MessageList } from './MessageList'
import { useUrlToken } from './useUrlToken'
import type { User } from './types'

const logoUrl = '/brand/petmind-logo-cropped.png'

function Brand({ compact = false }: { compact?: boolean }) {
  return <div className={`brand ${compact ? 'compact' : ''}`}><span className="brand-logo"><img src={logoUrl} alt="PetMind 团队标志" width="76" height="68" loading="eager" decoding="async" fetchPriority="high" /></span><div><strong>PetMind</strong>{!compact && <span>兽医智能会诊</span>}</div></div>
}

function PublicShell({ children }: { children: React.ReactNode }) {
  const { user, logout } = useAuth()
  const navigate = useNavigate()
  async function signOut() { await logout(); navigate('/login', { replace: true }) }
  return <main className="public-shell"><div className="paper-grain" /><nav><Brand compact /><div className="public-nav-actions"><Link to="/plans">会员计划</Link>{user && <button type="button" className="public-logout" aria-label="退出登录" onClick={signOut}><LogOut /><span>退出登录</span></button>}</div></nav>{children}</main>
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
    <form className="auth-card" onSubmit={submit}><div className="sketch-corner" /><p className="eyebrow">YOUR CLINICAL COPILOT</p><h2>让判断更有依据，<br />让沟通更加从容。</h2><p>进入你的 PetMind 智能会诊工作台</p><label>邮箱<input autoComplete="email" type="email" required value={email} onChange={e => setEmail(e.target.value)} placeholder="name@clinic.com" /></label><label>密码<input autoComplete="current-password" type="password" required value={password} onChange={e => setPassword(e.target.value)} placeholder="至少 10 位" /></label>{error && <div className="form-error">{error}</div>}<button className="primary" disabled={busy}>{busy ? '正在验证…' : '登录工作台'}</button><Link className="text-link" to="/forgot-password">忘记密码？</Link><div className="invite-hint"><ShieldCheck size={17} />专业账号 · 安全访问 · 数据可追溯</div></form>
  </section></PublicShell>
}

function AcceptInvitePage() {
  const token = useUrlToken()
  const { acceptSession } = useAuth()
  const navigate = useNavigate()
  const [name, setName] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const [accepted, setAccepted] = useState(false)
  const [versions, setVersions] = useState({ terms: '', privacy: '' })
  useEffect(() => { Promise.all([api<{version: string}>('/api/v1/legal/terms'), api<{version: string}>('/api/v1/legal/privacy')]).then(([terms, privacy]) => setVersions({ terms: terms.version, privacy: privacy.version })) }, [])
  async function submit(event: FormEvent) {
    event.preventDefault()
    try {
      const data = await api<{access_token: string; user: User}>('/api/v1/auth/invitations/accept', { method: 'POST', body: JSON.stringify({ token, display_name: name, password, terms_version: versions.terms, privacy_version: versions.privacy, accept_terms: accepted, accept_privacy: accepted }) })
      acceptSession(data.access_token, data.user); navigate('/chat', { replace: true })
    } catch (e) { setError(e instanceof Error ? e.message : '邀请无效') }
  }
  return <PublicShell><div className="center-card"><Brand /><h1>完成兽医账号</h1><p>邀请链接同时完成邮箱验证。</p><form onSubmit={submit}><label>称呼<input required value={name} onChange={e => setName(e.target.value)} /></label><label>设置密码<input type="password" minLength={10} required value={password} onChange={e => setPassword(e.target.value)} /></label><label className="legal-check"><input type="checkbox" required checked={accepted} onChange={e => setAccepted(e.target.checked)} />我已阅读并同意<Link to="/legal/terms" target="_blank">用户协议</Link>与<Link to="/legal/privacy" target="_blank">隐私政策</Link></label>{error && <div className="form-error">{error}</div>}<button className="primary" disabled={!accepted || !versions.terms}>接受邀请</button></form></div></PublicShell>
}

function PasswordPage({ reset = false }: { reset?: boolean }) {
  const [email, setEmail] = useState(''); const [password, setPassword] = useState(''); const [done, setDone] = useState(false)
  const [error, setError] = useState(''); const [busy, setBusy] = useState(false)
  const token = useUrlToken()
  async function submit(e: FormEvent) {
    e.preventDefault(); if (busy) return; setError(''); setBusy(true)
    try { await api(reset ? '/api/v1/auth/password/reset' : '/api/v1/auth/password/forgot', { method: 'POST', body: JSON.stringify(reset ? { token, password } : { email }) }); setDone(true) }
    catch (e) { setError(e instanceof Error ? e.message : '操作失败，请重试') }
    finally { setBusy(false) }
  }
  return <PublicShell><div className="center-card"><Brand /><h1>{reset ? '设置新密码' : '找回密码'}</h1>{done ? <><Check className="success-mark" /><p>{reset ? '密码已更新，请重新登录。' : '若账号存在，将收到密码重置邮件。'}</p><Link to="/login">返回登录</Link></> : <form onSubmit={submit}><label>{reset ? '新密码' : '账号邮箱'}<input required minLength={reset ? 10 : undefined} type={reset ? 'password' : 'email'} value={reset ? password : email} onChange={e => reset ? setPassword(e.target.value) : setEmail(e.target.value)} /></label>{error && <div className="form-error" role="alert">{error}</div>}<button className="primary" disabled={busy}>{busy ? '正在提交…' : '确认'}</button></form>}</div></PublicShell>
}

function PlansPage() {
  const { user } = useAuth(); const navigate = useNavigate(); const [plans, setPlans] = useState<Plan[]>([]); const [message, setMessage] = useState(''); const [ordering, setOrdering] = useState(false); const orderBusy = useRef(false); const orderKeys = useRef(new Map<string, string>())
  useEffect(() => { client.plans().then(r => setPlans(r.items)).catch(() => setPlans([])) }, [])
  async function order(code: string) { if (orderBusy.current) return; orderBusy.current = true; setOrdering(true); const key = orderKeys.current.get(code) || newId(); orderKeys.current.set(code, key); try { const result = await api<{id: string}>('/api/v1/orders', { method: 'POST', headers: { 'Idempotency-Key': key }, body: JSON.stringify({ plan_code: code }) }); setMessage(`订单 ${result.id} 已创建，等待管理员确认到账。请保留订单编号。`) } catch (e) { setMessage(e instanceof Error ? e.message : '创建失败') } finally { orderBusy.current = false; setOrdering(false) } }
  return <main className="plans-standalone"><div className="paper-grain" /><header><Brand compact /><button className="page-close" aria-label="关闭会员计划" onClick={() => navigate(user ? '/chat' : '/login')}><X /></button></header><section className="plans"><p className="eyebrow">PETMIND MEMBERSHIP</p><h1>按临床工作节奏选择</h1><p>价格、积分与有效期均由平台后台配置，结算记录全程可追溯。</p>{message && <div className="notice">{message}</div>}<div className="plan-grid">{plans.map((plan, i) => <article className={i === 1 ? 'featured' : ''} key={plan.code}><span>{plan.billing_period === 'year' ? '年度' : plan.billing_period === 'month' ? '月度' : '试用'}</span><h2>{plan.name}</h2><div className="price">{plan.price_cents ? `¥${(plan.price_cents / 100).toFixed(0)}` : '免费'}</div><p>{plan.description}</p><ul><li><Check />{plan.credit_grant} 会诊积分</li><li><Check />有效 {plan.duration_days} 天</li><li><Check />对话与记忆持久化</li></ul><button disabled={ordering || !user || plan.price_cents === 0} onClick={() => order(plan.code)}>{!user ? '登录后选择' : plan.price_cents === 0 ? '邀请时开通' : '创建订单'}</button></article>)}</div></section></main>
}

function UserMenu() {
  const { user, logout } = useAuth(); const [open, setOpen] = useState(false)
  const location = useLocation()
  useEffect(() => { setOpen(false) }, [location.pathname, location.hash])
  return <div className="user-menu" onBlur={e => { if (!e.currentTarget.contains(e.relatedTarget)) setOpen(false) }} onKeyDown={e => { if (e.key === 'Escape') setOpen(false) }}><button className="user-trigger" aria-expanded={open} onClick={() => setOpen(!open)}><span className="avatar">{user?.display_name?.[0] || '医'}</span><span><strong>{user?.display_name}</strong><small>{user?.role}</small></span><ChevronDown size={15} /></button>{open && <div className="user-popover">{user?.role !== 'VET' && <Link to="/admin"><ShieldCheck />平台管理</Link>}<Link to="/settings"><Settings />设置</Link><Link to="/help"><CircleHelp />帮助</Link><Link to="/plans"><WalletCards />会员计划</Link><Link to="/settings#api-keys"><KeyRound />API Key</Link><button onClick={() => logout()}><LogOut />退出登录</button></div>}</div>
}

function Sidebar({ current, collapsed = false, onCollapsedChange, onSelect, onNew, onDeleted }: { current?: string; collapsed?: boolean; onCollapsedChange(collapsed: boolean): void; onSelect(id: string): void; onNew(): void; onDeleted(id: string): void }) {
  const confirm = useConfirmation()
  const [items, setItems] = useState<Conversation[]>([]); const [query, setQuery] = useState(''); const [actionOpen, setActionOpen] = useState(''); const [actionError, setActionError] = useState('')
  const deletedIds = useRef(new Set<string>())
  // Search requests can resolve out of order; only the newest one may update the list.
  const loadRequest = useRef(0)
  async function load(q = '') {
    const request = ++loadRequest.current
    const result = q ? await client.search(q) : await client.conversations()
    if (request !== loadRequest.current) return
    const serverItems = result.items.filter(item => !deletedIds.current.has(item.id))
    setItems(previous => {
      if (q.trim()) return serverItems
      const serverIds = new Set(serverItems.map(item => item.id))
      // A list refresh can finish just after the user submitted the first
      // message but before the server-side list query observes that write.
      // Preserve locally announced conversations until a later refresh merges
      // the authoritative row instead of making the optimistic title vanish.
      const optimistic = previous.filter(item => !serverIds.has(item.id) && !deletedIds.current.has(item.id))
      return [...serverItems, ...optimistic]
    })
  }
  async function exportConversation(item: Conversation) {
    try {
      setActionError('')
      const result = await client.allMessages(item.id)
      const content = [`# ${item.title}`, '', `导出时间：${new Date().toLocaleString()}`, '', ...result.items.flatMap(message => [`## ${message.role === 'user' ? '我的问题' : 'PetMind 会诊意见'}`, '', message.content, ''])].join('\n')
      const blob = new Blob([content], { type: 'text/markdown;charset=utf-8' })
      const url = URL.createObjectURL(blob)
      const link = document.createElement('a')
      link.href = url
      const safeTitle = [...item.title]
        .map(char => char.charCodeAt(0) < 32 || /[<>:"/\\|?*]/.test(char) ? '_' : char)
        .join('')
      link.download = `${safeTitle.slice(0, 60) || 'PetMind会诊'}.md`
      document.body.appendChild(link); link.click(); link.remove(); URL.revokeObjectURL(url)
      setActionOpen('')
    } catch (error) { setActionError(error instanceof Error ? error.message : '导出失败') }
  }
  async function deleteConversation(item: Conversation) {
    if (!await confirm(`确定删除“${item.title}”吗？删除后将无法在对话列表中恢复。`)) return
    try {
      setActionError(''); await client.remove(item.id); announceConversationDeleted(item.id); onDeleted(item.id)
    } catch (error) { setActionError(error instanceof Error ? error.message : '删除失败') }
  }
  useEffect(() => { load().catch(() => undefined) }, [])
  useEffect(() => { const timer = setTimeout(() => load(query).catch(() => undefined), 250); return () => clearTimeout(timer) }, [query])
  useEffect(() => {
    function upsert(event: Event) {
      const update = (event as CustomEvent<ConversationUpdate>).detail
      if (!update?.id || deletedIds.current.has(update.id)) return
      setItems(previous => {
        const existing = previous.find(item => item.id === update.id)
        const now = new Date().toISOString()
        const merged: Conversation = {
          id: update.id,
          title: update.title || existing?.title || '新会诊',
          status: update.status || existing?.status || 'active',
          created_at: update.created_at || existing?.created_at || now,
          last_active_at: update.last_active_at || now,
          snippet: update.snippet || existing?.snippet,
        }
        return [merged, ...previous.filter(item => item.id !== update.id)]
      })
    }
    window.addEventListener(CONVERSATION_UPSERT_EVENT, upsert)
    return () => window.removeEventListener(CONVERSATION_UPSERT_EVENT, upsert)
  }, [])
  useEffect(() => {
    function removeDeleted(event: Event) {
      const id = (event as CustomEvent<string>).detail
      if (!id) return
      deletedIds.current.add(id)
      setItems(previous => previous.filter(item => item.id !== id))
      setActionOpen(previous => previous === id ? '' : previous)
    }
    window.addEventListener(CONVERSATION_DELETE_EVENT, removeDeleted)
    return () => window.removeEventListener(CONVERSATION_DELETE_EVENT, removeDeleted)
  }, [])
  return <aside className={`sidebar ${collapsed ? 'collapsed' : ''}`}><header><Brand compact /><button type="button" aria-label={collapsed ? '展开侧边栏' : '收起侧边栏'} aria-expanded={!collapsed} onClick={() => onCollapsedChange(!collapsed)}>{collapsed ? <PanelLeftOpen /> : <PanelLeftClose />}</button></header><button className="new-chat" aria-label="新建会诊" title={collapsed ? '新建会诊' : undefined} onClick={onNew}><MessageSquarePlus /> <span>新建会诊</span></button><div className="search-box"><Search /><input aria-label="搜索对话" placeholder="搜索病例与回答" value={query} onChange={e => setQuery(e.target.value)} /></div><div className="history"><small>最近会诊</small>{actionError && <div className="history-error">{actionError}</div>}{items.map(item => <div className={`history-row ${current === item.id ? 'active' : ''}`} key={item.id}><button className="history-main" onClick={() => onSelect(item.id)}><span>{item.title}</span><time>{new Date(item.last_active_at).toLocaleDateString()}</time></button><div className="history-actions"><button type="button" aria-label={`管理对话：${item.title}`} aria-expanded={actionOpen === item.id} onClick={() => setActionOpen(actionOpen === item.id ? '' : item.id)}><MoreHorizontal /></button>{actionOpen === item.id && <div className="history-menu" role="menu"><button type="button" role="menuitem" onClick={() => exportConversation(item)}><Download />导出 Markdown</button><button type="button" role="menuitem" className="danger" onClick={() => deleteConversation(item)}><Trash2 />删除对话</button></div>}</div></div>)}</div><UserMenu /></aside>
}

interface WorkspaceValue { openMobileSidebar(): void }
const WorkspaceContext = createContext<WorkspaceValue>({ openMobileSidebar() {} })
function useWorkspace() { return useContext(WorkspaceContext) }

function WorkspaceFrame() {
  const { user, loading } = useAuth()
  const location = useLocation()
  const navigate = useNavigate()
  const [collapsed, setCollapsed] = useState(false)
  const [mobileOpen, setMobileOpen] = useState(false)
  const [mobile, setMobile] = useState(() => window.matchMedia('(max-width: 800px)').matches)
  useEffect(() => { setMobileOpen(false) }, [location.pathname, location.hash])
  useEffect(() => {
    const query = window.matchMedia('(max-width: 800px)')
    const change = () => { setMobile(query.matches); setMobileOpen(false) }
    query.addEventListener('change', change)
    return () => query.removeEventListener('change', change)
  }, [])
  const workspacePath = /^\/(chat|help|settings|admin)(\/|$)/.test(location.pathname)
  const match = location.pathname.match(/^\/chat\/([^/]+)/)
  const current = match?.[1]
  if (loading && workspacePath) return <div className="app-loading"><Brand /><span /></div>
  if (!user || !workspacePath) return <Outlet />
  function deleted(id: string) { if (id === current) navigate('/chat', { replace: true }) }
  return <WorkspaceContext.Provider value={{ openMobileSidebar: () => setMobileOpen(true) }}>
    <div className={`workspace ${collapsed ? 'sidebar-collapsed' : ''}`}>
      <div className={`sidebar-host ${mobile ? 'mobile-sidebar' : ''} ${mobileOpen ? 'shown' : ''}`} onClick={event => { if (event.target === event.currentTarget) setMobileOpen(false) }}>
        <Sidebar current={current} collapsed={!mobile && collapsed} onCollapsedChange={value => mobile ? setMobileOpen(false) : setCollapsed(value)} onSelect={id => { navigate(`/chat/${id}`); setMobileOpen(false) }} onNew={() => { navigate('/chat'); setMobileOpen(false) }} onDeleted={deleted} />
      </div>
      <div className="workspace-view">{mobile && !location.pathname.startsWith('/chat') && <header className="workspace-mobile-nav"><button type="button" aria-label="打开侧边栏" onClick={() => setMobileOpen(true)}><Menu /></button><Link to="/chat">返回会诊台</Link></header>}<Outlet /></div>
    </div>
  </WorkspaceContext.Provider>
}

type AudienceRole = 'veterinarian' | 'pet_owner'

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
  const { openMobileSidebar } = useWorkspace()
  const [detailsOpen, setDetailsOpen] = useState(false)
  const { messages, nextCursor, loadingOlder, loadOlder, messagesLoading, input, setInput, editingMessageId, setEditingMessageId, editingContent, setEditingContent, phase, busy, error, audienceRole, changeAudienceRole, commonPhrases, submitMessage, send, stop, updateMessageFeedback, forkFromMessage, beginRewrite } = useRunStream()
  return <section className="chat-main">
    <header className="chat-top">
      <button className="mobile-menu" aria-label="打开侧边栏" onClick={openMobileSidebar}><Menu /></button>
      <ChatModeSelector role={audienceRole} onRoleChange={changeAudienceRole} />
      <div className="top-actions"><Link to="/plans"><Coins />积分</Link><div className="conversation-more"><button type="button" aria-label="会诊详情" aria-expanded={detailsOpen} onClick={() => setDetailsOpen(!detailsOpen)}><MoreHorizontal /></button>{detailsOpen && <div className="conversation-details"><p className="eyebrow">SESSION DETAILS</p><h3>当前会诊</h3><dl><div><dt>模型</dt><dd>PetMind Clinical MoE</dd></div><div><dt>回答身份</dt><dd>{audienceRole === 'veterinarian' ? '兽医专业模式' : '宠物主沟通模式'}</dd></div><div><dt>会话状态</dt><dd>{busy ? phaseLabels[phase] || '处理中' : '随时可用'}</dd></div></dl><Link to="/settings"><Settings />个人设置</Link><Link to="/help"><CircleHelp />使用帮助</Link><Link to="/plans"><WalletCards />会员与积分</Link></div>}</div></div>
    </header>
    <div className="message-scroll">
      {messagesLoading ? <div className="conversation-loading" role="status" aria-label="正在加载对话"><span className="phase-spinner" /><div><strong>正在加载对话</strong><small>正在读取消息与专家会诊记录…</small></div></div> : messages.length === 0 ? <div className="empty-chat"><img src={logoUrl} alt="" width="148" height="135" loading="eager" /><p className="eyebrow">PETMIND CLINICAL DESK</p><h1>今天需要一起梳理<br />哪个病例？</h1><p>请提供物种、年龄、主诉、症状时间线和已有检查。系统会组织资料检索与专家复核。</p><div className="suggestions">{['猫频繁进出猫砂盆，如何排急症？', '犬持续咳嗽的鉴别诊断路径', '帮我解读这组肝功能指标'].map(q => <button key={q} onClick={() => setInput(q)}>{q}</button>)}</div></div> : <div className="messages">
        {nextCursor && <button type="button" className="load-older" disabled={loadingOlder} onClick={loadOlder}>{loadingOlder ? '正在加载…' : '加载更早消息'}</button>}
        <MessageList messages={messages}>{message => <article key={message.id} className={`message ${message.role}`}>
          {message.role === 'user' ? <div className={`user-message-bubble ${editingMessageId === message.id ? 'editing' : ''}`}><div className="message-label">我的问题</div>{editingMessageId === message.id ? <div className="message-rewrite"><textarea autoFocus aria-label="编辑用户消息" value={editingContent} onChange={event => setEditingContent(event.target.value)} onKeyDown={event => { if (event.key === 'Enter' && !event.shiftKey) { event.preventDefault(); void submitMessage(editingContent.trim(), message.id) } }} /><div><button type="button" onClick={() => { setEditingMessageId(''); setEditingContent('') }}>取消</button><button type="button" className="primary" disabled={!editingContent.trim()} onClick={() => submitMessage(editingContent.trim(), message.id)}>发送</button></div></div> : message.content && <div className="message-body">{message.content}</div>}</div> : <AssistantAnswer message={message} phase={message.status === 'streaming' ? phase : ''} />}
          {editingMessageId !== message.id && message.status !== 'streaming' && message.content && message.id !== 'streaming' && <MessageActions message={message} onFeedback={rating => updateMessageFeedback(message.id, rating)} onFork={() => forkFromMessage(message.id)} onRewrite={() => beginRewrite(message)} />}
        </article>}</MessageList>
      </div>}
    </div>
    {error && <div className="form-error" role="alert">{error}</div>}
    <div className="composer-wrap">{commonPhrases.length > 0 && <select className="phrase-picker" aria-label="插入常用语" defaultValue="" onChange={e => { const phrase = commonPhrases.find(item => item.id === e.target.value); if (phrase) setInput(previous => previous ? `${previous}\n${phrase.content}` : phrase.content); e.target.value = '' }}><option value="">插入常用语</option>{commonPhrases.map(item => <option value={item.id} key={item.id}>{item.title || item.content.slice(0, 24)}</option>)}</select>}<div className="composer"><textarea aria-label="输入病例" rows={1} value={input} onChange={e => setInput(e.target.value)} onKeyDown={e => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); send() } }} placeholder={audienceRole === 'veterinarian' ? '描述病例，Shift + Enter 换行' : '描述宠物的症状和变化，Shift + Enter 换行'} /><button className={busy ? 'stop' : 'send'} aria-label={busy ? '停止生成' : '发送'} onClick={busy ? stop : send}>{busy ? <Square /> : <Send />}</button></div><small>AI 生成 · PetMind 可能出错，请结合体检、实验室与影像结果独立判断。</small></div>
  </section>
}

function HelpPage() {
  const { user } = useAuth()
  const content = <div className="help-page"><p className="eyebrow">CLINICAL SUPPORT GUIDE</p><h1>让信息更完整，判断更可靠</h1><div className="help-grid"><article><BookOpen /><h2>如何描述病例</h2><p>依次提供物种、年龄、性别与绝育状态、主诉、时间线、体检和已有检查。缺失信息可以明确写“未知”。</p></article><article><Sparkles /><h2>如何理解回答</h2><p>系统会给出支持证据、反对证据与缺失信息。它是临床决策支持，不替代查体、化验、影像与兽医最终判断。</p></article><article><ShieldCheck /><h2>数据与安全</h2><p>不要提交无关个人信息。系统不会向网页暴露内部提示词、模型推理过程或原始工具载荷。</p></article></div></div>
  return user ? content : <PublicShell>{content}</PublicShell>
}

function Protected({ children, admin = false }: { children: React.ReactNode; admin?: boolean }) {
  const { user, loading } = useAuth(); if (loading) return <div className="app-loading"><Brand /><span /></div>; if (!user) return <Navigate to="/login" replace />
  if (admin && user.role === 'VET') return <Navigate to="/chat" replace />
  return children
}

export default function App() {
  return <Suspense fallback={<div className="app-loading" role="status">正在加载…</div>}><Routes><Route element={<WorkspaceFrame />}><Route path="/" element={<Navigate to="/chat" replace />} /><Route path="/login" element={<LoginPage />} /><Route path="/accept-invite" element={<AcceptInvitePage />} /><Route path="/forgot-password" element={<PasswordPage />} /><Route path="/reset-password" element={<PasswordPage reset />} /><Route path="/plans" element={<PlansPage />} /><Route path="/help" element={<HelpPage />} /><Route path="/settings" element={<Protected><SettingsPage /></Protected>} /><Route path="/activate-code" element={<Protected><ActivationPage /></Protected>} /><Route path="/feedback" element={<Protected><FeedbackPage /></Protected>} /><Route path="/legal/terms" element={<LegalPage type="terms" />} /><Route path="/legal/privacy" element={<LegalPage type="privacy" />} /><Route path="/chat" element={<Protected><ChatPage /></Protected>} /><Route path="/chat/:conversationId" element={<Protected><ChatPage /></Protected>} /><Route path="/admin" element={<Protected admin><AdminPage /></Protected>} /><Route path="*" element={<Navigate to="/chat" replace />} /></Route></Routes></Suspense>
}
