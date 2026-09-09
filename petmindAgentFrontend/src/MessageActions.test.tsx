// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { MessageActions } from './MessageActions'
import { ApiError } from './api'
import { writeClipboard } from './clipboard'
import type { Message } from './types'

vi.mock('./clipboard', () => ({ writeClipboard: vi.fn().mockResolvedValue(undefined) }))
const answer: Message = { id: 'answer', role: 'assistant', content: '合成测试回答', status: 'complete', created_at: '' }
afterEach(() => { cleanup(); vi.useRealTimers(); vi.clearAllMocks() })

describe('answer rating actions', () => {
  it('maps selected up/down to cancellation and allows switching', async () => {
    const submit = vi.fn().mockResolvedValue(undefined)
    const props = { onFeedback: submit, onFork: vi.fn(), onRewrite: vi.fn() }
    const view = render(<MessageActions message={{ ...answer, feedback_rating: 'up' }} {...props} />)
    expect(screen.getByRole('button', { name: '赞同回答' })).toHaveAttribute('aria-pressed', 'true')
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '赞同回答' })) })
    expect(submit).toHaveBeenLastCalledWith(null)
    view.rerender(<MessageActions message={{ ...answer, feedback_rating: 'down' }} {...props} />)
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '赞同回答' })) })
    expect(submit).toHaveBeenLastCalledWith('up')
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '不赞同回答' })) })
    expect(submit).toHaveBeenLastCalledWith(null)
  })

  it('synchronously prevents duplicate submissions before rerender', async () => {
    let finish!: () => void
    const submit = vi.fn(() => new Promise<void>(resolve => { finish = resolve }))
    render(<MessageActions message={answer} onFeedback={submit} onFork={vi.fn()} onRewrite={vi.fn()} />)
    const up = screen.getByRole('button', { name: '赞同回答' })
    act(() => { fireEvent.click(up); fireEvent.click(up) })
    expect(submit).toHaveBeenCalledTimes(1)
    expect(up).toBeDisabled()
    await act(async () => { finish() })
    expect(up).toBeEnabled()
  })

  it('preserves the selected rating after failure and allows explicit retry', async () => {
    const submit = vi.fn().mockRejectedValueOnce(new Error('保存失败')).mockResolvedValue(undefined)
    render(<MessageActions message={{ ...answer, feedback_rating: 'up' }} onFeedback={submit} onFork={vi.fn()} onRewrite={vi.fn()} />)
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '不赞同回答' })) })
    expect(screen.getByText('保存失败')).toBeVisible()
    expect(screen.getByRole('button', { name: '赞同回答' })).toHaveAttribute('aria-pressed', 'true')
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '不赞同回答' })) })
    expect(submit).toHaveBeenCalledTimes(2)
  })

  it('shows Retry-After countdown without replay and cleans all timers', async () => {
    vi.useFakeTimers()
    const submit = vi.fn().mockRejectedValue(new ApiError('操作频繁', 429, 'rate_limited', 3))
    const view = render(<MessageActions message={answer} onFeedback={submit} onFork={vi.fn()} onRewrite={vi.fn()} />)
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '赞同回答' })) })
    expect(screen.getByText('操作频繁，3 秒后可再次评价')).toBeVisible()
    expect(screen.getByRole('button', { name: '不赞同回答' })).toBeDisabled()
    act(() => { vi.advanceTimersByTime(3000) })
    expect(screen.getByRole('button', { name: '赞同回答' })).toBeEnabled()
    expect(submit).toHaveBeenCalledTimes(1)
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '赞同回答' })) })
    view.unmount()
    expect(vi.getTimerCount()).toBe(0)
  })

  it('keeps copy available and does not permit rating incomplete answers', async () => {
    render(<MessageActions message={{ ...answer, status: 'cancelled' }} onFeedback={vi.fn()} onFork={vi.fn()} onRewrite={vi.fn()} />)
    expect(screen.getByRole('button', { name: '赞同回答' })).toBeDisabled()
    await act(async () => { fireEvent.click(screen.getByRole('button', { name: '复制消息' })) })
    expect(writeClipboard).toHaveBeenCalledWith(expect.stringContaining(answer.content))
    expect(screen.getByRole('button', { name: '已复制' })).toBeEnabled()
  })
})
