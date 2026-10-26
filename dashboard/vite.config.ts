import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The dev server proxies /api to the gateway, so `npm run dev` and the built
// bundle served by the gateway itself talk to exactly the same endpoints and
// the client never carries a base URL.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      '/api': {
        target: `http://127.0.0.1:${process.env.RAL_DASHBOARD_PORT ?? 8771}`,
        changeOrigin: false,
      },
    },
  },
  build: { outDir: 'dist', sourcemap: false },
});
