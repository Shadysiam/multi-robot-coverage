import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App'
import ErrorBoundary from './components/ErrorBoundary'
import './index.css'

// StrictMode intentionally double-fires effects in dev, which causes ros.close()
// to terminate the WebSocket before the second mount re-connects. Keep it off.
ReactDOM.createRoot(document.getElementById('root')).render(
  <ErrorBoundary>
    <App />
  </ErrorBoundary>
)
