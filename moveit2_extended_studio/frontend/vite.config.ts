import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Built output goes straight into the ROS package's web/ directory, which studio_server serves.
// Relative base so the page works whatever path it is mounted at.
export default defineConfig({
  plugins: [react()],
  base: "./",
  build: {
    outDir: "../web",
    emptyOutDir: false, // the package's other web assets live here too
    sourcemap: false,
    chunkSizeWarningLimit: 1200,
  },
  server: {
    // `npm run dev` against a running studio_server on the robot.
    proxy: { "/api": "http://127.0.0.1:8080" },
  },
});
