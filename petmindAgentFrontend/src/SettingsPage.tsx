import { useEffect, useState, type FormEvent, type ReactNode } from 'react'
import { Check, ChevronRight, Coins, Copy, LogOut, MessageSquareText, Moon, ShieldCheck, Sparkles, Sun, Trash2, UserRound } from 'lucide-react'
import { Link, useNavigate } from 'react-router-dom'
import ReactMarkdown from 'react-markdown'
import rehypeSanitize from 'rehype-sanitize'
import { api } from './api'
import { useAuth } from './auth'

type Preference = { theme: 'light' | 'dark' | 'system'; locale: string; default_expand_experts: boolean; memory_recall_enabled: boolean; memory_write_enabled: boolean }
type Phrase = { id: string; title: string; content: string; sort_order: number }
type MemoryItem = { id: string; type: string; label: string; content: unknown; created_at?: string; source_count: number; generation_tags: string[] }
type ApiKey = { id: string; name: string; prefix: string; revoked_at: string | null; created_at: string }

const defaults: Preference = { theme: 'system', locale: 'zh-CN', default_expand_experts: true, memory_recall_enabled: true, memory_write_enabled: true }

function applyTheme(theme: Preference['theme']) {
  document.documentElement.dataset.theme = theme
  localStorage.setItem('petmind-theme', theme)
}

function AnimatedCopy({ text, label = '复制 API Key' }: { text: string; label?: string }) {
  const [copied, setCopied] = useState(false)
  async function copy() {
    await navigator.clipboard.writeText(text)
    setCopied(true)
    window.setTimeout(() => setCopied(false), 1600)
  }
  return <button type="button" className={copied ? 'copy-action copied' : 'copy-action'} onClick={copy} aria-label={label}>{copied ? <><Check />已复制</> : <><Copy />复制</>}</button>
}

function memoryText(content: unknown) {
  if (typeof content === 'string') return content
  if (Array.isArray(content)) return content.join('；')
  return JSON.stringify(content)
}

function profileLabel(label: string) {
  const aliases: Record<string, string> = { concerns: '关注事项', healthWatch: '健康关注', communication: '沟通偏好', petFacts: '宠物信息' }
  const parts = label.split('.')
  return parts.length > 1 ? parts.slice(1).join(' · ') : aliases[label] || label
}

type MemoryScope = 'short_term' | 'knowledge' | 'profile'

function MemoryGroups({ items, busy, onDelete, onClear }: { items: MemoryItem[]; busy: boolean; onDelete(item: MemoryItem): void; onClear(scope: MemoryScope): void }) {
  if (busy) return <p>正在读取…</p>
  const shortItems = items.filter(item => item.type === 'short_term')
  const profileItems = items.filter(item => item.type === 'profile')
  const knowledgeItems = items.filter(item => item.type === 'knowledge')
  return <div className="memory-scroll" tabIndex={0} aria-label="记忆条目列表"><div className="memory-groups">
    <section><h3>短期记忆 <span>{shortItems.length}</span><button disabled={!shortItems.length} aria-label="清空短期记忆" title="清空短期记忆" onClick={() => onClear('short_term')}><Trash2 /></button></h3><div className="memory-list">{shortItems.length ? shortItems.map(item => <article key={item.id}><div><p>{memoryText(item.content)}</p>{item.created_at && <small>{new Date(item.created_at).toLocaleString()}</small>}</div><button aria-label="删除短期记忆" onClick={() => onDelete(item)}><Trash2 /></button></article>) : <p className="muted">暂无短期记忆。</p>}</div></section>
    <section><h3>长期记忆 <span>{knowledgeItems.length}</span><button disabled={!knowledgeItems.length} aria-label="清空长期记忆" title="清空长期记忆" onClick={() => onClear('knowledge')}><Trash2 /></button></h3><div className="memory-list">
      {knowledgeItems.map(item => <article key={item.id}><div><p>{memoryText(item.content)}</p></div><button aria-label="删除长期记忆" onClick={() => onDelete(item)}><Trash2 /></button></article>)}
      {!knowledgeItems.length && <p className="muted">暂无长期记忆。</p>}
    </div></section>
    <section><h3>用户画像 <span>{profileItems.length}</span><button disabled={!profileItems.length} aria-label="清空用户画像" title="清空用户画像" onClick={() => onClear('profile')}><Trash2 /></button></h3><div className="memory-list">
      {profileItems.length > 0 && <article className="profile-memory-card"><div><div className="profile-memory-fields">{profileItems.map(item => <div key={item.id}><span><small>{profileLabel(item.label)}</small><p>{memoryText(item.content)}</p></span><button aria-label={`删除用户画像：${profileLabel(item.label)}`} onClick={() => onDelete(item)}><Trash2 /></button></div>)}</div></div></article>}
      {!profileItems.length && <p className="muted">暂无用户画像。</p>}
    </div></section>
  </div></div>
}

export function SettingsPage() {
  const { user, logout } = useAuth(); const navigate = useNavigate()
  const [preference, setPreference] = useState(defaults); const [panel, setPanel] = useState('')
  const [phrases, setPhrases] = useState<Phrase[]>([]); const [phraseTitle, setPhraseTitle] = useState(''); const [phraseContent, setPhraseContent] = useState('')
  const [memories, setMemories] = useState<MemoryItem[]>([]); const [memoryBusy, setMemoryBusy] = useState(false)
  const [keys, setKeys] = useState<ApiKey[]>([]); const [keyName, setKeyName] = useState(''); const [newKey, setNewKey] = useState('')
  const [credits, setCredits] = useState({ balance: 0, reserved: 0 }); const [notice, setNotice] = useState('')
  const [displayName, setDisplayName] = useState(user?.display_name || '')
  const [editingPhrase, setEditingPhrase] = useState<Phrase | null>(null)

  async function load() {
    const [p, phraseData, keyData, creditData] = await Promise.allSettled([
      api<Preference>('/api/v1/me/preferences'), api<{items: Phrase[]}>('/api/v1/me/common-phrases'),
      api<{items: ApiKey[]}>('/api/v1/me/api-keys'), api<{balance: number; reserved: number}>('/api/v1/credits'),
    ])
    if (p.status === 'fulfilled') { setPreference(p.value); applyTheme(p.value.theme) }
    if (phraseData.status === 'fulfilled') setPhrases(phraseData.value.items)
    if (keyData.status === 'fulfilled') setKeys(keyData.value.items)
    if (creditData.status === 'fulfilled') setCredits(creditData.value)
  }
  useEffect(() => { if (location.hash === '#api-keys') setPanel('billing'); load().catch(error => setNotice(error instanceof Error ? error.message : '设置加载失败')) }, [])
  async function patchPreference(changes: Partial<Preference>) {
    const next = await api<Preference>('/api/v1/me/preferences', { method: 'PATCH', body: JSON.stringify(changes) })
    setPreference(next); localStorage.setItem('petmind-default-expand-experts', String(next.default_expand_experts)); applyTheme(next.theme)
  }
  async function addPhrase(event: FormEvent) { event.preventDefault(); await api('/api/v1/me/common-phrases', { method: 'POST', body: JSON.stringify({ title: phraseTitle, content: phraseContent, sort_order: phrases.length }) }); setPhraseTitle(''); setPhraseContent(''); await load() }
  async function updatePhrase(event: FormEvent) { event.preventDefault(); if (!editingPhrase) return; await api(`/api/v1/me/common-phrases/${editingPhrase.id}`, { method: 'PATCH', body: JSON.stringify({ title: editingPhrase.title, content: editingPhrase.content, sort_order: editingPhrase.sort_order }) }); setEditingPhrase(null); await load() }
  async function deletePhrase(id: string) { await api(`/api/v1/me/common-phrases/${id}`, { method: 'DELETE' }); setPhrases(rows => rows.filter(row => row.id !== id)) }
  async function loadMemories() { setMemoryBusy(true); try { setMemories((await api<{items: MemoryItem[]}>('/api/v1/me/memories')).items) } finally { setMemoryBusy(false) } }
  async function deleteMemory(item: MemoryItem) { const kind = item.type === 'short_term' ? '短期' : '长期'; if (!confirm(`这会永久删除所选${kind}记忆，但不会删除历史对话。是否继续？`)) return; await api(`/api/v1/me/memories/${encodeURIComponent(item.id)}`, { method: 'DELETE' }); await loadMemories() }
  async function clearMemories(scope: MemoryScope) { const labels: Record<MemoryScope, string> = { short_term: '短期记忆', knowledge: '长期记忆', profile: '用户画像' }; const label = labels[scope]; if (!confirm(`确定清空全部${label}吗？此操作无法恢复。`)) return; await api('/api/v1/me/memories', { method: 'DELETE', body: JSON.stringify({ scope }) }); await loadMemories(); setNotice(`${label}已清空`) }
  async function createKey(event: FormEvent) { event.preventDefault(); const created = await api<{key: string}>('/api/v1/me/api-keys', { method: 'POST', body: JSON.stringify({ name: keyName, scopes: ['chat:write', 'models:read', 'runs:read'] }) }); setNewKey(created.key); setKeyName(''); await load() }
  async function revokeKey(id: string) { if (!confirm('撤销后使用该密钥的客户端会立即失效。')) return; await api(`/api/v1/me/api-keys/${id}`, { method: 'DELETE' }); await load(); setNotice('原密钥已立即失效，无法恢复。') }
  async function signOut() { await logout(); navigate('/login', { replace: true }) }
  async function saveProfile(event: FormEvent) { event.preventDefault(); await api('/api/v1/me', { method: 'PATCH', body: JSON.stringify({ display_name: displayName }) }); setNotice('账号信息已保存'); setPanel('') }

  const row = (icon: ReactNode, label: string, value: string, action: () => void) => <button className="setting-row" onClick={action}>{icon}<span>{label}</span><small>{value}</small><ChevronRight /></button>
  return <main className="settings-page"><header><p className="eyebrow">PETMIND PREFERENCES</p><h1>设置</h1></header>
    <button className="settings-account" onClick={() => setPanel('profile')}><span className="avatar">{user?.display_name?.[0] || '医'}</span><span><strong>{user?.display_name}</strong><small>{user?.email}</small></span><ChevronRight /></button>
    <section><h2>通用</h2><div className="settings-group"><div className="setting-row static"><Sun /><span>界面主题</span><select value={preference.theme} onChange={event => patchPreference({ theme: event.target.value as Preference['theme'] })}><option value="system">跟随系统</option><option value="light">暖色浅色</option><option value="dark">暖色深色</option></select></div><div className="setting-row static"><MessageSquareText /><span>语言</span><small>简体中文</small></div></div></section>
    <section><h2>会话</h2><div className="settings-group">{row(<Sparkles />, '常用语', `${phrases.length} 条`, () => setPanel('phrases'))}<label className="setting-row static"><MessageSquareText /><span>默认展开专家输出</span><input type="checkbox" checked={preference.default_expand_experts} onChange={event => patchPreference({ default_expand_experts: event.target.checked })} /></label></div></section>
    <section><h2>管理</h2><div className="settings-group">{row(<Coins />, '额度和 API 管理', `${credits.balance} 可用额度`, () => setPanel('billing'))}{row(<ShieldCheck />, '邀请码激活', '', () => navigate('/activate-code'))}</div></section>
    <section><h2>个性化</h2><div className="settings-group">{row(<Moon />, '记忆', '查看与管理', () => { setPanel('memory'); void loadMemories() })}</div></section>
    <section><h2>帮助与关于</h2><div className="settings-group">{row(<UserRound />, '帮助中心', '', () => setPanel('help'))}{row(<MessageSquareText />, '反馈问题', '', () => navigate('/feedback'))}<Link className="setting-row" to="/legal/terms"><ShieldCheck /><span>用户协议</span><ChevronRight /></Link><Link className="setting-row" to="/legal/privacy"><ShieldCheck /><span>隐私政策</span><ChevronRight /></Link>{row(<Sparkles />, '功能介绍', '', () => setPanel('features'))}</div></section>
    <button className="settings-logout" onClick={signOut}><LogOut />退出登录</button>{notice && <div className="notice">{notice}</div>}

    {panel && <div className="settings-modal" onMouseDown={event => { if (event.target === event.currentTarget) setPanel('') }}><div className={`settings-modal-card${panel === 'memory' ? ' memory-modal-card' : ''}`}><button className="modal-close" aria-label="关闭设置弹窗" onClick={() => setPanel('')}>×</button>
      {panel === 'profile' && <div className="account-settings"><h2>账号设置</h2><div className="account-settings-avatar" aria-label="当前头像">{user?.display_name?.[0] || '医'}</div><form onSubmit={saveProfile}><label>显示名称<input required value={displayName} onChange={e => setDisplayName(e.target.value)} maxLength={100} placeholder="填写你的显示名称" /></label><label>登录邮箱<input readOnly value={user?.email || ''} /></label><button className="primary">保存</button></form></div>}
      {panel === 'phrases' && <><h2>常用语</h2>{editingPhrase ? <form onSubmit={updatePhrase}><input value={editingPhrase.title} onChange={e => setEditingPhrase({ ...editingPhrase, title: e.target.value })} placeholder="名称（可选）" maxLength={100} /><textarea required value={editingPhrase.content} onChange={e => setEditingPhrase({ ...editingPhrase, content: e.target.value })} maxLength={2000} /><div className="inline-form"><button type="button" onClick={() => setEditingPhrase(null)}>取消</button><button className="primary">保存修改</button></div></form> : <form onSubmit={addPhrase}><input value={phraseTitle} onChange={e => setPhraseTitle(e.target.value)} placeholder="名称（可选）" maxLength={100} /><textarea required value={phraseContent} onChange={e => setPhraseContent(e.target.value)} placeholder="输入常用病例描述或要求" maxLength={2000} /><button className="primary">保存常用语</button></form>}<div className="memory-list">{phrases.map(item => <article key={item.id}><button className="phrase-main" onClick={() => setEditingPhrase(item)}><strong>{item.title || '未命名常用语'}</strong><p>{item.content}</p></button><button aria-label="删除常用语" onClick={() => deletePhrase(item.id)}><Trash2 /></button></article>)}</div></>}
      {panel === 'memory' && <><h2>记忆</h2><p className="muted">系统会自动整理短期和长期记忆；删除对话不会影响已经形成的记忆。</p><div className="toggle-grid"><label>允许调用记忆<input type="checkbox" checked={preference.memory_recall_enabled} onChange={e => patchPreference({ memory_recall_enabled: e.target.checked })} /></label><label>允许写入记忆<input type="checkbox" checked={preference.memory_write_enabled} onChange={e => patchPreference({ memory_write_enabled: e.target.checked })} /></label></div><MemoryGroups items={memories} busy={memoryBusy} onDelete={deleteMemory} onClear={clearMemories} /></>}
      {panel === 'billing' && <><h2>额度和 API 管理</h2><p className="muted">额度用于网页会诊和 API 调用。任务开始时会暂时预占额度，完成后按实际用量结算。</p><div className="credit-metrics"><div><Coins /><span>可用额度</span><strong>{credits.balance}</strong><small>可用于新的会诊任务</small></div><div><span>预占额度</span><strong>{credits.reserved}</strong><small>正在执行的任务暂时占用</small></div></div><section className="api-guide"><h3>连接院内系统或第三方客户端</h3><p>API Key 相当于应用密码，可用于 OpenAI 兼容客户端、院内工作台或自动化程序，不影响网页账号登录。</p><div className="api-endpoint"><span>兼容接口地址</span><code>{window.location.origin}/v1</code><AnimatedCopy text={`${window.location.origin}/v1`} label="复制兼容接口地址" /></div><ol><li>填写一个便于识别的密钥名称。</li><li>创建后立即复制完整密钥，它只显示一次。</li><li>在客户端中使用 Bearer API Key，并妥善保管。</li></ol></section><form className="api-key-create" onSubmit={createKey}><label>密钥名称<input required value={keyName} onChange={e => setKeyName(e.target.value)} placeholder="例如：病例工作台" maxLength={80} /></label><button className="primary">创建 API Key</button></form>{newKey && <div className="one-time-key"><strong>仅显示一次，请立即保存</strong><p>关闭弹窗后无法再次查看完整密钥。</p><code>{newKey}</code><AnimatedCopy text={newKey} /></div>}<section className="api-key-section"><h3>已创建的密钥 <span>{keys.length}</span></h3><div className="key-list">{keys.length ? keys.map(key => <div key={key.id}><span><strong>{key.name}</strong><small>pm_live_{key.prefix}_… · 创建于 {new Date(key.created_at).toLocaleDateString()}</small></span>{key.revoked_at ? <em>已撤销 · {new Date(key.revoked_at).toLocaleDateString()}</em> : <button onClick={() => revokeKey(key.id)}>撤销</button>}</div>) : <p className="muted">尚未创建 API Key。</p>}</div></section></>}
      {panel === 'help' && <><h2>帮助中心</h2><p>描述病例时请提供物种、年龄、性别、绝育状态、主诉、时间线和已有检查。PetMind 仅提供临床决策支持，不替代线下检查。</p></>}
      {panel === 'features' && <><h2>PetMind 功能</h2><p>多专家会诊、兽医知识检索、联网补证、会话恢复、长期记忆和 OpenAI 兼容 API。</p></>}
    </div></div>}
  </main>
}

export function ActivationPage() {
  const navigate = useNavigate(); const [code, setCode] = useState(''); const [message, setMessage] = useState('')
  async function submit(event: FormEvent) { event.preventDefault(); setMessage(''); try { const result = await api<{credits_granted: number}>('/api/v1/activation-codes/redeem', { method: 'POST', body: JSON.stringify({ code }) }); setMessage(`激活成功，已发放 ${result.credits_granted} 额度。`) } catch (error) { setMessage(error instanceof Error ? error.message : '邀请码无效或当前不可用') } }
  return <main className="form-page"><button className="page-close" onClick={() => navigate('/settings')}>×</button><section><p className="eyebrow">MEMBERSHIP ACTIVATION</p><h1>邀请码激活</h1><form onSubmit={submit}><input required minLength={8} maxLength={64} value={code} onChange={e => setCode(e.target.value)} placeholder="输入邀请码" /><button className="primary">立即激活</button></form>{message && <div className="notice">{message}</div>}</section></main>
}

export function FeedbackPage() {
  const navigate = useNavigate(); const [category, setCategory] = useState('product'); const [content, setContent] = useState(''); const [contact, setContact] = useState(''); const [done, setDone] = useState(false)
  async function submit(event: FormEvent) { event.preventDefault(); await api('/api/v1/feedback', { method: 'POST', body: JSON.stringify({ category, content, contact: contact || null, page_path: location.pathname }) }); setDone(true) }
  return <main className="form-page"><button className="page-close" onClick={() => navigate('/settings')}>×</button><section><p className="eyebrow">PETMIND FEEDBACK</p><h1>反馈问题</h1>{done ? <div className="notice"><Check />反馈已提交，我们会认真查看。</div> : <form onSubmit={submit}><select value={category} onChange={e => setCategory(e.target.value)}><option value="product">产品体验</option><option value="answer">回答质量</option><option value="billing">额度与订单</option><option value="security">安全问题</option><option value="other">其他</option></select><textarea required minLength={5} maxLength={4000} value={content} onChange={e => setContent(e.target.value)} placeholder="请描述问题、出现时间和期望结果" /><input type="text" value={contact} onChange={e => setContact(e.target.value)} maxLength={320} placeholder="联系方式（选填）" /><button className="primary">提交反馈</button></form>}</section></main>
}

export function LegalPage({ type }: { type: 'terms' | 'privacy' }) {
  const navigate = useNavigate(); const [document, setDocument] = useState({ version: '', content: '' })
  useEffect(() => { api<typeof document>(`/api/v1/legal/${type}`).then(setDocument) }, [type])
  return <main className="legal-page"><button className="page-close" onClick={() => navigate(-1)}>×</button><article><ReactMarkdown rehypePlugins={[rehypeSanitize]}>{document.content}</ReactMarkdown></article></main>
}
