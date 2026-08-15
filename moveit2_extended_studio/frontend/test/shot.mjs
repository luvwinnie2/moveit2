// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Drive the built Studio in a real browser and photograph it.
//
// A build that succeeds proves the bundler was happy, not that the page renders -- a bad hook, a
// null dereference in a render path, or a missing asset all pass `vite build` and produce a blank
// screen. So this loads the page, fails on any console error or page error, drives the editor, and
// writes screenshots to look at.
//
//   node test/shot.mjs [baseUrl] [outDir]

import { mkdirSync } from "node:fs";
import { chromium } from "playwright";

const base = process.argv[2] ?? "http://127.0.0.1:8080";
const outDir = process.argv[3] ?? "/tmp/studio-shots";
mkdirSync(outDir, { recursive: true });

const problems = [];

const browser = await chromium.launch({ args: ["--no-sandbox"] });
const page = await browser.newPage({ viewport: { width: 1600, height: 950 } });

page.on("console", (message) => {
  if (message.type() === "error") problems.push(`console: ${message.text()}`);
});
page.on("pageerror", (error) => problems.push(`pageerror: ${error.message}`));
page.on("requestfailed", (request) => problems.push(`request failed: ${request.url()}`));

console.log(`opening ${base}`);
// NOT networkidle: the app polls /api/state several times a second, so the network is never
// idle and that wait can only ever time out. Wait for the thing that actually matters instead.
await page.goto(base, { waitUntil: "domcontentloaded", timeout: 30000 });
await page.waitForSelector("header h1", { timeout: 15000 });
await page.waitForTimeout(1200); // one poll cycle, so the Objective list is populated

const objectives = await page.locator("aside.sidebar select option").allTextContents();
console.log("objectives in the list:", objectives.map((o) => o.trim()));
await page.screenshot({ path: `${outDir}/1-watch.png` });

// Load an Objective: this is the whole editor path -- fetch XML, parse, lay out, render.
const target = process.env.OBJECTIVE ?? "PruneGrapeCluster";
await page.locator("aside.sidebar select").selectOption(target);
await page.waitForSelector(".bt-node", { timeout: 15000 });
await page.waitForTimeout(900);

const nodeCount = await page.locator(".bt-node").count();
const orderBadges = await page.locator(".bt-order").count();
console.log(`rendered ${nodeCount} nodes, ${orderBadges} order badges`);
// Measured, not eyeballed: how much of the tree is actually inside the canvas. A screenshot that
// looks fine at a glance can still be cut off at both edges.
const framing = await page.evaluate(() => {
  const pane = document.querySelector(".react-flow__viewport");
  const canvas = document.querySelector(".canvas");
  if (!pane || !canvas) return null;
  const view = canvas.getBoundingClientRect();
  let inside = 0;
  const nodes = document.querySelectorAll(".bt-node");
  for (const node of nodes) {
    const box = node.getBoundingClientRect();
    if (box.left >= view.left - 1 && box.right <= view.right + 1 && box.top >= view.top - 1 && box.bottom <= view.bottom + 1)
      inside += 1;
  }
  const transform = getComputedStyle(pane).transform;
  const zoom = transform && transform !== "none" ? Number(transform.split("(")[1].split(",")[0]) : 1;
  return { inside, total: nodes.length, zoom: Number(zoom.toFixed(3)) };
});
console.log("framing:", framing);
if (framing && framing.inside < framing.total) {
  problems.push(`fitView left ${framing.total - framing.inside} of ${framing.total} nodes outside the canvas (zoom ${framing.zoom})`);
}
await page.screenshot({ path: `${outDir}/2-editor.png` });

// Select a node so the inspector fills in, then validate through the server.
// force, because React Flow draws nodes inside a transformed pane and one of them can sit
// outside the visible viewport -- Playwright then waits for a scroll that will never happen.
await page.locator(".bt-node").first().click({ force: true });
await page.waitForTimeout(400);
await page.screenshot({ path: `${outDir}/3-inspector.png` });

await page.locator("footer button", { hasText: "Validate" }).click();
await page.waitForTimeout(1500);
const verdict = (await page.locator("footer span").last().textContent()) ?? "";
console.log("validation footer:", verdict.trim().slice(0, 160));
await page.screenshot({ path: `${outDir}/4-validated.png` });

await browser.close();

if (problems.length) {
  console.log(`\n${problems.length} problem(s):`);
  for (const problem of problems.slice(0, 12)) console.log("  " + problem);
  process.exit(1);
}
console.log(`\nno console or page errors. screenshots in ${outDir}`);
