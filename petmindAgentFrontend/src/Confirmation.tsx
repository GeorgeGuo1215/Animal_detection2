import { createContext, useCallback, useContext, useEffect, useRef, useState, type ReactNode } from 'react'

type Confirm = (message: string) => Promise<boolean>
const ConfirmationContext = createContext<Confirm | null>(null)

/** Accessible page dialog: traps focus, supports Escape, and cancels on unmount. */
export function ConfirmationProvider({ children }: { children: ReactNode }) {
  const element = useRef<HTMLDialogElement>(null)
  const resolver = useRef<((answer: boolean) => void) | null>(null)
  const [message, setMessage] = useState('')
  const confirm = useCallback<Confirm>(text => {
    if (resolver.current) return Promise.resolve(false)
    return new Promise(resolve => { resolver.current = resolve; setMessage(text) })
  }, [])
  function finish(answer: boolean) {
    resolver.current?.(answer); resolver.current = null
    element.current?.close(); setMessage('')
  }
  useEffect(() => { if (message && !element.current?.open) element.current?.showModal() }, [message])
  useEffect(() => () => { resolver.current?.(false); resolver.current = null }, [])
  return <ConfirmationContext.Provider value={confirm}>{children}
    <dialog ref={element} className="confirmation-dialog" aria-label="确认操作" onCancel={event => { event.preventDefault(); finish(false) }}>
      <h2>确认操作</h2><p>{message}</p><div className="confirmation-actions"><button type="button" autoFocus onClick={() => finish(false)}>取消</button><button type="button" className="primary" onClick={() => finish(true)}>确认继续</button></div>
    </dialog>
  </ConfirmationContext.Provider>
}

// eslint-disable-next-line react-refresh/only-export-components
export function useConfirmation() {
  const confirm = useContext(ConfirmationContext)
  if (!confirm) throw new Error('ConfirmationProvider is missing')
  return confirm
}
