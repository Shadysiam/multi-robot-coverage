import { Component } from 'react'

/**
 * Catches React render errors and shows them instead of going blank.
 * Wrap the root <App /> with this so crashes are visible.
 */
export default class ErrorBoundary extends Component {
  constructor(props) {
    super(props)
    this.state = { error: null }
  }

  static getDerivedStateFromError(error) {
    return { error }
  }

  componentDidCatch(error, info) {
    console.error('[ErrorBoundary]', error, info.componentStack)
  }

  render() {
    if (this.state.error) {
      return (
        <div className="h-screen w-screen flex flex-col items-center justify-center bg-surface-900 gap-4 p-8">
          <span className="text-4xl">⚠️</span>
          <h1 className="text-red-400 font-bold text-lg">Dashboard crashed</h1>
          <pre className="text-xs text-slate-400 bg-surface-700 rounded-xl p-4 max-w-2xl overflow-auto whitespace-pre-wrap">
            {this.state.error.message}
            {'\n\n'}
            {this.state.error.stack}
          </pre>
          <button
            className="px-4 py-2 bg-blue-700 hover:bg-blue-600 text-white rounded-lg text-sm"
            onClick={() => this.setState({ error: null })}
          >
            Retry
          </button>
        </div>
      )
    }
    return this.props.children
  }
}
