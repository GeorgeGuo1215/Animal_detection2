import { useState } from 'react'

let captured: { url: string; value: string } | null = null

/** Capture once and immediately remove credentials from the address bar and referrer. */
export function useUrlToken() {
  const [token] = useState(() => {
    const url = new URL(window.location.href)
    if (captured?.url === url.href) return captured.value
    const fragment = new URLSearchParams(url.hash.slice(1))
    const value = fragment.get('token') || url.searchParams.get('token') || ''
    url.searchParams.delete('token')
    fragment.delete('token')
    url.hash = fragment.toString()
    captured = value ? { url: url.href, value } : null
    if (value) window.history.replaceState(window.history.state, '', url)
    return value
  })
  return token
}
