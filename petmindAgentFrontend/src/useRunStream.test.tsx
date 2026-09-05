// @vitest-environment jsdom
import { act, cleanup, renderHook, waitFor } from '@testing-library/react'
import { MemoryRouter, Route, Routes, useNavigate } from 'react-router-dom'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { ReactNode } from 'react'
import { useRunStream } from './useRunStream'

const mocks = vi.hoisted(() => ({ api: vi.fn(), messages: vi.fn(), create: vi.fn(), stream: vi.fn(), resume: vi.fn() }))
vi.mock('./api', () => ({
  api: mocks.api, newId: () => 'local-id', streamRun: mocks.stream, resumeRun: mocks.resume,
  client: { messages: mocks.messages, createConversation: mocks.create },
}))
const never = () => new Promise<void>(() => undefined)
function wrapper({ children }: { children: ReactNode }) {
  return <MemoryRouter initialEntries={['/chat/a']}><Routes><Route path="/chat/:conversationId" element={children} /></Routes></MemoryRouter>
}
beforeEach(() => {
  vi.clearAllMocks(); sessionStorage.clear(); localStorage.clear()
  mocks.messages.mockResolvedValue({ items: [] })
  mocks.api.mockResolvedValue({ items: [] })
  mocks.resume.mockImplementation(never)
})
afterEach(cleanup)

describe('durable run stream', () => {
  it('recovers clean EOF without a terminal event and ignores duplicate sequences', async () => {
    mocks.stream.mockImplementation(async (_id, _text, _role, event, _signal, id) => {
      id('run-1'); event({ id: 1, event: 'delta', data: { content: 'once' } })
      event({ id: 1, event: 'delta', data: { content: 'once' } })
    })
    const { result } = renderHook(useRunStream, { wrapper })
    await waitFor(() => expect(result.current.messagesLoading).toBe(false))
    act(() => { void result.current.submitMessage('test case') })
    await waitFor(() => expect(mocks.resume).toHaveBeenCalledWith('run-1', 1, expect.any(Function), expect.any(AbortSignal)))
    await waitFor(() => expect(result.current.messages.at(-1)?.content).toBe('once'))
    expect(result.current.busy).toBe(true)
  })

  it('preserves the reader and resume marker when cancellation fails', async () => {
    let signal: AbortSignal | undefined
    mocks.stream.mockImplementation((_id, _text, _role, _event, currentSignal, id) => {
      signal = currentSignal; id('run-2'); return never()
    })
    const { result } = renderHook(useRunStream, { wrapper })
    await waitFor(() => expect(result.current.messagesLoading).toBe(false))
    act(() => { void result.current.submitMessage('case') })
    await waitFor(() => expect(result.current.busy).toBe(true))
    mocks.api.mockRejectedValueOnce(new Error('offline'))
    await act(async () => result.current.stop())
    expect(signal?.aborted).toBe(false)
    expect(result.current.busy).toBe(true)
    expect(sessionStorage.getItem('petmind-run:a')).toContain('run-2')
    expect(result.current.error).toContain('停止失败')
  })

  it('drops a late run id and events after switching conversations', async () => {
    let late: (() => void) | undefined
    mocks.stream.mockImplementation((_id, _text, _role, event, _signal, id) => {
      late = () => { id('old-run'); event({ id: 9, event: 'delta', data: { content: 'stale' } }) }
      return never()
    })
    const { result } = renderHook(() => ({ run: useRunStream(), navigate: useNavigate() }), { wrapper })
    await waitFor(() => expect(result.current.run.messagesLoading).toBe(false))
    act(() => { void result.current.run.submitMessage('case') })
    await waitFor(() => expect(mocks.stream).toHaveBeenCalled())
    act(() => result.current.navigate('/chat/b'))
    await waitFor(() => expect(result.current.run.messagesLoading).toBe(false))
    act(() => late?.())
    expect(sessionStorage.getItem('petmind-run:a')).toBeNull()
    expect(result.current.run.messages).toEqual([])
    expect(result.current.run.busy).toBe(false)
  })

  it('aborts a pending reader when the page unmounts', async () => {
    let signal: AbortSignal | undefined
    mocks.stream.mockImplementation((_id, _text, _role, _event, currentSignal, id) => {
      signal = currentSignal; id('run-3'); return never()
    })
    const { result, unmount } = renderHook(useRunStream, { wrapper })
    await waitFor(() => expect(result.current.messagesLoading).toBe(false))
    act(() => { void result.current.submitMessage('case') })
    await waitFor(() => expect(signal).toBeDefined())
    unmount()
    expect(signal?.aborted).toBe(true)
    expect(sessionStorage.getItem('petmind-run:a')).toContain('run-3')
  })
})
