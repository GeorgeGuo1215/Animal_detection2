import { useEffect, useRef, useState } from 'react'
import { Check, Copy, GitFork, LoaderCircle, Pencil, ThumbsDown, ThumbsUp } from 'lucide-react'
import { writeClipboard } from './clipboard'
import type { Message } from './types'


interface MessageActionsProps {
  message: Message
  onFeedback(rating: 'up' | 'down' | null): Promise<void>
  onFork(): Promise<void>
  onRewrite(): void
}


export function MessageActions({ message, onFeedback, onFork, onRewrite }: MessageActionsProps) {
  const [copied, setCopied] = useState(false)
  const [pending, setPending] = useState<'' | 'up' | 'down' | 'fork'>('')
  const [notice, setNotice] = useState('')
  const timer = useRef<number | null>(null)

  useEffect(() => () => {
    if (timer.current !== null) window.clearTimeout(timer.current)
  }, [])

  function showNotice(value: string) {
    setNotice(value)
    if (timer.current !== null) window.clearTimeout(timer.current)
    timer.current = window.setTimeout(() => {
      setNotice('')
      setCopied(false)
    }, 1800)
  }

  async function copyMessage() {
    try {
      const prefix = message.role === 'assistant' ? 'AI 生成 · 仅供兽医临床决策支持\n\n' : ''
      await writeClipboard(`${prefix}${message.content}`)
      setCopied(true)
      showNotice('已复制')
    } catch {
      showNotice('复制失败，请手动选择文字')
    }
  }

  async function updateFeedback(rating: 'up' | 'down') {
    if (pending) return
    const next = message.feedback_rating === rating ? null : rating
    setPending(rating)
    try {
      await onFeedback(next)
      showNotice(next === 'up' ? '已点赞' : next === 'down' ? '已点踩' : '已取消评价')
    } catch (error) {
      showNotice(error instanceof Error ? error.message : '评价保存失败')
    } finally {
      setPending('')
    }
  }

  async function forkMessage() {
    if (pending) return
    setPending('fork')
    try {
      await onFork()
    } catch (error) {
      showNotice(error instanceof Error ? error.message : '创建分支失败')
      setPending('')
    }
  }

  return <div className="message-actions" aria-label="消息操作">
    <button type="button" className={copied ? 'selected copied' : ''} onClick={copyMessage} aria-label={copied ? '已复制' : '复制消息'} title={copied ? '已复制' : '复制消息'}>{copied ? <Check /> : <Copy />}</button>
    {message.role === 'user' ? <button type="button" onClick={onRewrite} aria-label="编辑消息" title="编辑这条消息并重新生成后续回答"><Pencil /></button> : <>
      <button type="button" className={message.feedback_rating === 'up' ? 'selected' : ''} disabled={Boolean(pending)} onClick={() => updateFeedback('up')} aria-pressed={message.feedback_rating === 'up'} aria-label="赞同回答" title="赞同回答">{pending === 'up' ? <LoaderCircle className="spin" /> : <ThumbsUp />}</button>
      <button type="button" className={message.feedback_rating === 'down' ? 'selected' : ''} disabled={Boolean(pending)} onClick={() => updateFeedback('down')} aria-pressed={message.feedback_rating === 'down'} aria-label="不赞同回答" title="不赞同回答">{pending === 'down' ? <LoaderCircle className="spin" /> : <ThumbsDown />}</button>
      <button type="button" disabled={Boolean(pending)} onClick={forkMessage} aria-label="从此消息创建分支" title="从此消息创建新对话分支">{pending === 'fork' ? <LoaderCircle className="spin" /> : <GitFork />}</button>
    </>}
    <span className={notice.includes('失败') ? 'action-notice error' : 'action-notice'} aria-live="polite">{notice}</span>
  </div>
}
