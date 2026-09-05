import { useRef, type ReactNode } from 'react'
import { useVirtualizer } from '@tanstack/react-virtual'
import type { Message } from './types'

export function MessageList({ messages, children }: { messages: Message[]; children(message: Message): ReactNode }) {
  const root = useRef<HTMLDivElement>(null)
  const virtual = messages.length > 100
  // This component intentionally uses the library's mutable instance without compiler memoization.
  // eslint-disable-next-line react-hooks/incompatible-library
  const list = useVirtualizer({
    count: messages.length,
    enabled: virtual,
    getScrollElement: () => root.current?.closest<HTMLDivElement>('.message-scroll') || null,
    getItemKey: index => messages[index].id,
    estimateSize: () => 320,
    overscan: 5,
  })
  const rows = list.getVirtualItems()
  // Assign individual CSSOM properties: compatible with style-src 'self'.
  const space = (height: number) => <div aria-hidden="true" ref={element => { if (element) element.style.height = `${height}px` }} />
  return <div ref={root} className="message-list" data-virtualized={virtual}>
    {virtual ? <>
      {space(rows[0]?.start || 0)}
      {rows.map(row => <div className="measured-message" data-index={row.index} key={row.key} ref={list.measureElement}>{children(messages[row.index])}</div>)}
      {space(Math.max(0, list.getTotalSize() - (rows.at(-1)?.end || 0)))}
    </> : messages.map(message => <div className="measured-message" key={message.id}>{children(message)}</div>)}
  </div>
}
