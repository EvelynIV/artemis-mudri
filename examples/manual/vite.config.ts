import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const webPort = Number(process.env.ARTEMIS_MANUAL_WEB_PORT ?? 8765);
const vitePort = Number(process.env.ARTEMIS_MANUAL_VITE_PORT ?? 5173);

export default defineConfig({
  plugins: [react()],
  server: {
    port: Number.isFinite(vitePort) ? vitePort : 5173,
    proxy: {
      "/config.json": `http://127.0.0.1:${Number.isFinite(webPort) ? webPort : 8765}`,
      "/stop": `http://127.0.0.1:${Number.isFinite(webPort) ? webPort : 8765}`
    }
  },
  build: {
    outDir: "dist",
    emptyOutDir: true
  }
});
