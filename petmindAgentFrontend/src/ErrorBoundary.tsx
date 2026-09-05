import { Component, type ErrorInfo, type ReactNode } from 'react'

interface Props {
  children: ReactNode
}

interface State {
  error: Error | null
}

/**
 * Last line of defence for render-time exceptions. Without it a single thrown
 * error (e.g. a malformed trace payload) unmounts the whole React tree and the
 * user is left with a blank page and no way back to the conversation list.
 */
export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null }

  static getDerivedStateFromError(error: Error): State {
    return { error }
  }

  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error('Unhandled render error', error, info.componentStack)
  }

  render() {
    if (!this.state.error) return this.props.children
    return (
      <main className="error-boundary" role="alert">
        <h1>页面出现了异常</h1>
        <p>已记录错误信息。你可以刷新页面继续，或返回首页重新开始。</p>
        <p className="error-boundary-detail">{this.state.error.message}</p>
        <div className="error-boundary-actions">
          <button type="button" onClick={() => window.location.reload()}>刷新页面</button>
          <button type="button" onClick={() => { window.location.href = '/' }}>返回首页</button>
        </div>
      </main>
    )
  }
}
