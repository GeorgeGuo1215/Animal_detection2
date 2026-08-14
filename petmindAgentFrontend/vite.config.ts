import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'node:path'

export default defineConfig({
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    strictPort: true,
    allowedHosts: ['119.29.87.145', 'localhost', '127.0.0.1'],
    fs: { allow: [path.resolve(__dirname, '..')] },
    proxy: { '/api': 'http://127.0.0.1:8002' },
  },
  build: { sourcemap: process.env.VITE_SOURCEMAP === '1' },
})
