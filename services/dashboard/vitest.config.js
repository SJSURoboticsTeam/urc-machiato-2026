/// <reference types="vitest" />
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  test: {
    globals: true,
    environment: 'jsdom',
    setupFiles: ['./src/test/setup.js'],
    exclude: [
      '**/node_modules/**',
      '**/useROS.test.js',
      '**/rosbridge.test.js',
      '**/NetworkTab.test.jsx',
      '**/MessageTester.test.js',
      '**/testing-dashboard/MessageTester.test.jsx'
    ],
    coverage: {
      provider: 'v8',
      reporter: ['text', 'text-summary', 'html'],
      exclude: [
        'node_modules/',
        'src/test/',
        '**/*.test.{js,jsx}',
        '**/__tests__/**',
        '**/*.d.ts',
        '**/setupTests.js',
        'vitest.config.js'
      ],
      // Phase 4: thresholds at current coverage; target 90%+ over time.
      thresholds: {
        statements: 82,
        branches: 70,
        functions: 80,
        lines: 84
      }
    }
  },
  esbuild: {
    jsxFactory: 'React.createElement',
    jsxFragment: 'React.Fragment',
  },
})
