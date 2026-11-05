import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The dev server proxies /api to the gateway, so `npm run dev` and the built
// bundle served by the gateway itself talk to exactly the same endpoints and
// the client never carries a base URL.
//
// `base: './'` makes every emitted asset and fixture URL relative to the
// document, so one bundle serves from the gateway's root (`/`) AND from
// `site/dist/console/` on a static host under an arbitrary prefix
// (GitHub Pages project pages mount at `/<repo>/`). Root-absolute `/assets/...`
// would 404 there.
export default defineConfig({
  base: './',
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
