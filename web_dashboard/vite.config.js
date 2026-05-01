import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: 5173,
  },
  // roslib ships as CommonJS — pre-bundle it so Vite's ESM transform works
  optimizeDeps: {
    include: ['roslib'],
  },
})
