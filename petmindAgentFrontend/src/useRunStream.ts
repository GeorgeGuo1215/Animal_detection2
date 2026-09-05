import { useEffect, useRef, useState } from 'react'
import { useNavigate, useParams } from 'react-router-dom'
import { api, client, newId, resumeRun, streamRun } from './api'
import { STREAM_RENDER_INTERVAL_MS, takeStreamRenderChunk } from './streamingMarkdown'
import { announceConversation } from './conversationEvents'
import { isTerminalStatus, type TerminalStatus, type Conversation, type ExpertTrace, type Message, type RunEvent, type TraceNode } from './types'
type AudienceRole = 'veterinarian' | 'pet_owner'

export function useRunStream() {
  const navigate = useNavigate(); const { conversationId } = useParams(); const [messages, setMessages] = useState<Message[]>([]); const [messagesLoading, setMessagesLoading] = useState(false); const [input, setInput] = useState(''); const [editingMessageId, setEditingMessageId] = useState(''); const [editingContent, setEditingContent] = useState(''); const [phase, setPhase] = useState(''); const [busy, setBusy] = useState(false); const [error, setError] = useState('')
  const [audienceRole, setAudienceRole] = useState<AudienceRole>(() => localStorage.getItem('petmind-audience-role') === 'pet_owner' ? 'pet_owner' : 'veterinarian')
  const [streaming, setStreaming] = useState<Message | null>(null)
  const streamingRef = useRef<Message | null>(null)
  const [nextCursor, setNextCursor] = useState<string | null>(null)
  const [loadingOlder, setLoadingOlder] = useState(false)
  function draft(value: Message | null) { streamingRef.current = value; setStreaming(value) }
  const [commonPhrases, setCommonPhrases] = useState<Array<{id: string; title: string; content: string}>>([])
  const controller = useRef<AbortController | null>(null); const activeRun = useRef(''); const activeConversation = useRef(conversationId || ''); const pendingConversationNavigation = useRef(''); const lastEvent = useRef(0); const draftAnswer = useRef(''); const pendingAnswer = useRef(''); const draftFlushTimer = useRef<number | null>(null); const activeExperts = useRef<ExpertTrace[]>([]); const activeTrace = useRef<TraceNode[]>([])
  // Every SSE stream (send, resume, reconnect) gets a generation number. Events
  // that arrive after a newer stream started, or after stop/switch, are stale.
  const streamGeneration = useRef(0)
  // `busy` is React state and lags behind rapid double submits; this ref locks synchronously.
  const submitting = useRef(false)
  function changeAudienceRole(role: AudienceRole) { setAudienceRole(role); localStorage.setItem('petmind-audience-role', role) }
  function beginStream(): AbortController {
    controller.current?.abort()
    streamGeneration.current += 1
    const next = new AbortController()
    controller.current = next
    return next
  }
  function guardedHandler(generation: number) {
    return (event: RunEvent) => { if (streamGeneration.current === generation) handleEvent(event) }
  }
  useEffect(() => { const request = new AbortController(); api<{items: Array<{id: string; title: string; content: string}>}>('/api/v1/me/common-phrases', { signal: request.signal }).then(result => { if (!request.signal.aborted) setCommonPhrases(result.items) }).catch(() => undefined); return () => request.abort() }, [])
  function updateActiveExpert(incoming: ExpertTrace) {
    activeExperts.current = [...activeExperts.current.filter(item => item.expert !== incoming.expert), incoming]
    updateStreamingAnswer(draftAnswer.current)
  }
  function updateActiveTrace(incoming: TraceNode) {
    const index = activeTrace.current.findIndex(item => item.node_id === incoming.node_id)
    activeTrace.current = index < 0
      ? [...activeTrace.current, incoming]
      : activeTrace.current.map(item => item.node_id === incoming.node_id ? incoming : item)
    updateStreamingAnswer(draftAnswer.current)
  }
  function cancelDraftFlush() {
    if (draftFlushTimer.current !== null) window.clearTimeout(draftFlushTimer.current)
    draftFlushTimer.current = null
  }
  function updateStreamingAnswer(content: string) {
    draft({ id: 'streaming', run_id: activeRun.current || null, role: 'assistant', content,
      status: 'streaming', created_at: streamingRef.current?.created_at || new Date().toISOString(),
      expert_consultations: activeExperts.current, trace_nodes: activeTrace.current })
  }
  function scheduleDraftFlush() {
    if (draftFlushTimer.current !== null || !pendingAnswer.current) return
    draftFlushTimer.current = window.setTimeout(() => {
      draftFlushTimer.current = null
      flushPendingAnswer()
    }, STREAM_RENDER_INTERVAL_MS)
  }
  function flushPendingAnswer(force = false) {
    cancelDraftFlush()
    if (!pendingAnswer.current) return
    const next = force
      ? { chunk: pendingAnswer.current, rest: '' }
      : takeStreamRenderChunk(pendingAnswer.current)
    pendingAnswer.current = next.rest
    draftAnswer.current += next.chunk
    updateStreamingAnswer(draftAnswer.current)
    if (pendingAnswer.current) scheduleDraftFlush()
  }
  useEffect(() => () => { controller.current?.abort(); streamGeneration.current += 1; cancelDraftFlush() }, [])
  useEffect(() => {
    const previousConversation = activeConversation.current
    if (!conversationId) {
      controller.current?.abort(); streamGeneration.current += 1; submitting.current = false
      cancelDraftFlush(); pendingAnswer.current = ''; draftAnswer.current = ''; activeConversation.current = ''; activeRun.current = ''; activeExperts.current = []; activeTrace.current = []; setMessages([]); draft(null); setNextCursor(null); setMessagesLoading(false); setBusy(false); setPhase(''); setError(''); return
    }
    if (previousConversation !== conversationId) {
      // Disconnect only this browser stream. The persisted server task keeps
      // running and can be resumed when the user returns to the conversation.
      controller.current?.abort()
      streamGeneration.current += 1
      submitting.current = false
      activeRun.current = ''
      setBusy(false)
      setPhase('')
    }
    activeConversation.current = conversationId
    setError('')
    if (pendingConversationNavigation.current === conversationId) {
      pendingConversationNavigation.current = ''
      setMessagesLoading(false)
      return
    }
    cancelDraftFlush(); pendingAnswer.current = ''; draftAnswer.current = ''; activeExperts.current = []; activeTrace.current = []
    const saved = sessionStorage.getItem(`petmind-run:${conversationId}`)
    const recoveryController = new AbortController()
    setMessages([]); draft(null); setNextCursor(null); setLoadingOlder(false)
    setMessagesLoading(true)
    void (async () => {
      try {
        await refreshMessages(conversationId, recoveryController.signal)
        if (!recoveryController.signal.aborted) setMessagesLoading(false)
        if (!saved || recoveryController.signal.aborted) return
        const state = JSON.parse(saved) as { runId?: unknown }
        if (typeof state.runId !== 'string' || !state.runId) throw new SyntaxError('无效的恢复记录')
        controller.current?.abort()
        controller.current = recoveryController
        streamGeneration.current += 1
        const generation = streamGeneration.current
        activeRun.current = state.runId
        // A full reload has no in-memory draft. Replay all durable events so
        // expert cards and answer deltas are reconstructed together.
        lastEvent.current = 0; setBusy(true); setPhase('queued'); draftAnswer.current = ''; pendingAnswer.current = ''
        await recoverRun(state.runId, recoveryController.signal, generation)
      } catch (loadError) {
        if (!recoveryController.signal.aborted) { setMessagesLoading(false); setError(loadError instanceof Error ? loadError.message : '会话加载失败') }
        if (saved && !recoveryController.signal.aborted && loadError instanceof SyntaxError) sessionStorage.removeItem(`petmind-run:${conversationId}`)
      }
    })()
    return () => recoveryController.abort()
    // Recovery is keyed only by the route; the helper reads mutable run state
    // from refs so adding its per-render identity would restart the SSE loop.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [conversationId])
  async function refreshMessages(id: string, signal?: AbortSignal) {
    const generation = streamGeneration.current
    const result = await client.messages(id, signal)
    if (!signal?.aborted && activeConversation.current === id && streamGeneration.current === generation) { setMessages(result.items); setNextCursor(result.next_cursor || null) }
    return result.items
  }
  function finishRun(event: Extract<RunEvent, { event: TerminalStatus }>) {
    const id = activeConversation.current
    const generation = streamGeneration.current
    if (id) sessionStorage.removeItem(`petmind-run:${id}`)
    flushPendingAnswer(true)
    const finalDraft = streamingRef.current
    if (finalDraft) setMessages(previous => [...previous, { ...finalDraft, status: event.event === 'completed' ? 'complete' : event.event }])
    draft(null)
    activeRun.current = ''; draftAnswer.current = ''; pendingAnswer.current = ''; activeExperts.current = []; activeTrace.current = []; setPhase(''); setBusy(false)
    if (event.event === 'failed') setError(String(event.data.message || '会诊任务执行失败，请稍后重试'))
    else if (event.event === 'cancelled') setError('生成已停止。')
    else if (event.event === 'completed' && event.data.finish_reason === 'truncated') setError('回答已达到长度上限，内容可能不完整，请继续追问。')
    else setError('')
    if (id) void refreshMessages(id).catch(error => {
      if (activeConversation.current === id && streamGeneration.current === generation && event.event === 'completed') setError(error instanceof Error ? error.message : '最终回答加载失败，请刷新重试')
    })
    if (id) announceConversation({ id, last_active_at: new Date().toISOString() })
  }
  function handleEvent(event: RunEvent) {
    if (event.id <= lastEvent.current) return
    lastEvent.current = event.id
    if (activeConversation.current && activeRun.current) sessionStorage.setItem(`petmind-run:${activeConversation.current}`, JSON.stringify({ runId: activeRun.current, lastEvent: lastEvent.current }))
    if (event.event === 'status') {
      setError(''); setPhase(String(event.data.phase || ''))
      const incoming = event.data.expert
      if (incoming?.expert) updateActiveExpert(incoming)
    }
    if (event.event === 'trace' && event.data.node_id) updateActiveTrace(event.data)
    if (event.event === 'delta') {
      setError('')
      pendingAnswer.current += String(event.data.content || '')
      scheduleDraftFlush()
    }
    if (event.event === 'reset') {
      setError('')
      cancelDraftFlush(); pendingAnswer.current = ''; draftAnswer.current = ''; activeExperts.current = []; activeTrace.current = []
      updateStreamingAnswer('')
    }
    if (event.event === 'completed' || event.event === 'failed' || event.event === 'cancelled') finishRun(event)
  }
  async function recoverRun(runId: string, signal: AbortSignal, generation: number) {
    let attempts = 0
    const onEvent = guardedHandler(generation)
    while (!signal.aborted && activeRun.current === runId && streamGeneration.current === generation) {
      try {
        await resumeRun(runId, lastEvent.current, onEvent, signal)
        if (signal.aborted || activeRun.current !== runId || streamGeneration.current !== generation) return
      } catch {
        if (signal.aborted || activeRun.current !== runId || streamGeneration.current !== generation) return
      }
      try {
        const snapshot = await api<{status: string; error?: {message?: string} | null}>(`/api/v1/runs/${runId}`, { signal })
        if (signal.aborted || activeRun.current !== runId || streamGeneration.current !== generation) return
        if (isTerminalStatus(snapshot.status)) {
          finishRun({ id: lastEvent.current, event: snapshot.status, data: { message: snapshot.error?.message || '' } })
          return
        }
      } catch { /* the next retry also refreshes authentication when needed */ }
      if (signal.aborted || activeRun.current !== runId || streamGeneration.current !== generation) return
      attempts += 1; setError(''); setPhase('reconnecting')
      await new Promise<void>(resolve => {
        const timer = window.setTimeout(done, Math.min(5000, 800 + attempts * 500))
        function done() { window.clearTimeout(timer); signal.removeEventListener('abort', done); resolve() }
        signal.addEventListener('abort', done, { once: true })
      })
    }
  }
  async function submitMessage(text: string, rewriteMessageId?: string) {
    if (!text || busy || submitting.current) return
    submitting.current = true
    const generation = streamGeneration.current + 1
    try {
      await submitMessageLocked(text, rewriteMessageId)
    } finally {
      if (streamGeneration.current === generation) submitting.current = false
    }
  }
  async function submitMessageLocked(text: string, rewriteMessageId?: string) {
    const streamController = beginStream()
    const generation = streamGeneration.current
    setBusy(true)
    let id = conversationId
    let created: Conversation | null = null
    if (!id) {
      try {
        created = await client.createConversation(streamController.signal, text)
        if (streamController.signal.aborted || generation !== streamGeneration.current) return
      } catch (createError) {
        if (streamController.signal.aborted || generation !== streamGeneration.current) return
        setBusy(false)
        setError(createError instanceof Error ? createError.message : '创建会诊失败，请重试')
        return
      }
      id = created.id; activeConversation.current = id; pendingConversationNavigation.current = id; navigate(`/chat/${id}`, { replace: true })
    }
    announceConversation({
      ...(created || {}),
      id: id!,
      ...(created || (rewriteMessageId && !nextCursor && messages.find(item => item.role === 'user')?.id === rewriteMessageId) ? { title: text.slice(0, 60) } : {}),
      snippet: text,
      last_active_at: new Date().toISOString(),
    })
    activeExperts.current = []
    setMessages(previous => {
      if (!rewriteMessageId) return [...previous, { id: newId(), role: 'user', content: text, status: 'complete', created_at: new Date().toISOString() }]
      const index = previous.findIndex(message => message.id === rewriteMessageId)
      if (index < 0) return previous
      return [...previous.slice(0, index), { ...previous[index], content: text }]
    }); setInput(''); setEditingMessageId(''); setEditingContent(''); setBusy(true); setError(''); setPhase('queued'); cancelDraftFlush(); draftAnswer.current = ''; pendingAnswer.current = ''; activeExperts.current = []; activeTrace.current = []; lastEvent.current = 0; updateStreamingAnswer('')
    try {
      await streamRun(id!, text, audienceRole, guardedHandler(generation), streamController.signal, runId => { if (streamController.signal.aborted || streamGeneration.current !== generation) return; activeRun.current = runId; sessionStorage.setItem(`petmind-run:${id}`, JSON.stringify({ runId, lastEvent: lastEvent.current })) }, rewriteMessageId)
      if (!streamController.signal.aborted && generation === streamGeneration.current && activeRun.current) await recoverRun(activeRun.current, streamController.signal, generation)
    } catch (e) {
      if (streamController.signal.aborted || streamGeneration.current !== generation) return
      if (activeRun.current) {
        await recoverRun(activeRun.current, streamController.signal, generation); return
      }
      setError(e instanceof Error ? e.message : '连接中断'); setBusy(false)
      if (rewriteMessageId && id) await refreshMessages(id).catch(() => undefined)
    }
  }
  async function send() { await submitMessage(input.trim()) }
  async function stop() {
    const runId = activeRun.current
    const generation = streamGeneration.current
    if (!runId) return
    setPhase('cancelling')
    try {
      const snapshot = await api<{status: string}>(`/api/v1/runs/${runId}`, { method: 'DELETE' })
      if (generation !== streamGeneration.current || activeRun.current !== runId) return
      if (isTerminalStatus(snapshot.status)) finishRun({ id: lastEvent.current, event: snapshot.status, data: {} })
      // Keep the live reader and durable resume marker until the server confirms a terminal state.
    } catch (error) {
      if (generation === streamGeneration.current && activeRun.current === runId) {
        setError(error instanceof Error ? `停止失败：${error.message}，任务仍可恢复` : '停止失败，请重试')
        setPhase('reconnecting')
      }
    }
  }
  async function updateMessageFeedback(messageId: string, rating: 'up' | 'down' | null) {
    const generation = streamGeneration.current
    const result = await client.setMessageFeedback(messageId, rating)
    if (generation !== streamGeneration.current) return
    setMessages(previous => previous.map(message => message.id === messageId
      ? { ...message, feedback_rating: result.rating, feedback_updated_at: result.updated_at }
      : message))
  }
  async function forkFromMessage(messageId: string) {
    if (!conversationId) throw new Error('请先开始一个会诊')
    const generation = streamGeneration.current
    const forked = await client.forkConversation(conversationId, messageId)
    if (generation !== streamGeneration.current) return
    announceConversation(forked)
    navigate(`/chat/${forked.id}`)
  }
  function beginRewrite(message: Message) {
    if (busy) return
    setEditingMessageId(message.id)
    setEditingContent(message.content)
  }
  async function loadOlder() {
    const id = activeConversation.current
    const generation = streamGeneration.current
    if (!id || !nextCursor || loadingOlder) return
    setLoadingOlder(true)
    try {
      const result = await client.messages(id, undefined, nextCursor)
      if (generation !== streamGeneration.current || activeConversation.current !== id) return
      setMessages(previous => [...result.items.filter(item => !previous.some(p => p.id === item.id)), ...previous])
      setNextCursor(result.next_cursor || null)
    } catch (error) {
      if (generation === streamGeneration.current) setError(error instanceof Error ? error.message : '历史加载失败')
    } finally { if (generation === streamGeneration.current) setLoadingOlder(false) }
  }
  return { messages: streaming ? [...messages, streaming] : messages, nextCursor, loadingOlder, loadOlder, messagesLoading, input, setInput, editingMessageId, setEditingMessageId, editingContent, setEditingContent, phase, busy, error, audienceRole, changeAudienceRole, commonPhrases, submitMessage, send, stop, updateMessageFeedback, forkFromMessage, beginRewrite }
}
