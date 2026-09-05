import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter } from 'react-router-dom'
import { AuthProvider } from './auth'
import { ErrorBoundary } from './ErrorBoundary'
import { ConfirmationProvider } from './Confirmation'
import App from './App'
import './styles.css'

document.documentElement.dataset.theme = localStorage.getItem('petmind-theme') || 'system'

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode><ErrorBoundary><BrowserRouter><AuthProvider><ConfirmationProvider><App /></ConfirmationProvider></AuthProvider></BrowserRouter></ErrorBoundary></React.StrictMode>,
)
