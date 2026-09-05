// @vitest-environment jsdom
import { StrictMode, type ReactNode } from 'react'
import { cleanup, renderHook } from '@testing-library/react'
import { afterEach, expect, it } from 'vitest'
import { useUrlToken } from './useUrlToken'

afterEach(cleanup)
it('removes invitation credentials without losing them during StrictMode initialization', () => {
  window.history.replaceState({}, '', '/accept-invite?token=test-invite')
  const wrapper = ({ children }: { children: ReactNode }) => <StrictMode>{children}</StrictMode>
  const { result } = renderHook(useUrlToken, { wrapper })
  expect(result.current).toBe('test-invite')
  expect(window.location.href).not.toContain('token=')
})
