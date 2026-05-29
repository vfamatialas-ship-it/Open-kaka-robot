import { defineConfig } from "vite";
import path from "node:path";
import { fileURLToPath } from "node:url";

const webRoot = path.dirname(fileURLToPath(import.meta.url));
const projectRoot = path.resolve(webRoot, "../..");

export default defineConfig({
  root: webRoot,
  server: {
    host: "127.0.0.1",
    port: 5173,
    strictPort: true,
    fs: {
      allow: [projectRoot],
    },
  },
  build: {
    outDir: path.join(webRoot, "dist"),
    emptyOutDir: true,
  },
});
