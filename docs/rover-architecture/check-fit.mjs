// Does anything on these pages spill out of the box it was written into?
//
//     node make-pdf.mjs && node check-fit.mjs
//
// Exits non-zero and names the box when something does not fit, and leaves a
// PNG of each page beside print.html either way.
//
// The reason this exists rather than an eye on the PDF: every box on these
// slides is absolutely positioned with a height in its inline style and its
// overflow left visible, so a sentence one line too long does not clip or
// scroll -- it prints over the border below it, and on a 1280-pixel page that
// is a couple of millimetres nobody notices until it is on a wall. An element
// whose scrollHeight has passed its clientHeight is exactly that condition, and
// it is worth asking a real browser rather than counting characters, because
// the answer depends on the webfont having loaded.
//
// Chrome is driven over the DevTools protocol on a port of its own. CHROME in
// the environment overrides where it is.
import { spawn } from "node:child_process";
import { writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

const dir = process.argv[2] ?? path.dirname(fileURLToPath(import.meta.url));
const CHROME = process.env.CHROME
  ?? "C:/Program Files/Google/Chrome/Application/chrome.exe";
const port = 9331;

const chrome = spawn(CHROME, [
  "--headless=new", "--disable-gpu", "--hide-scrollbars",
  `--remote-debugging-port=${port}`,
  `--user-data-dir=${mkdtempSync(path.join(tmpdir(), "checkfit-"))}`,
  "--window-size=1280,720", "about:blank",
], { stdio: "ignore" });

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// The port is open before the page target exists, so wait for the target.
async function debuggerUrl() {
  for (let i = 0; i < 60; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const page = list.find((t) => t.type === "page");
      if (page?.webSocketDebuggerUrl) return page.webSocketDebuggerUrl;
    } catch { /* not up yet */ }
    await sleep(250);
  }
  throw new Error("chrome never answered on the debugging port");
}

const ws = new WebSocket(await debuggerUrl());
await new Promise((r) => ws.addEventListener("open", r, { once: true }));

let nextId = 1;
const pending = new Map();
ws.addEventListener("message", (ev) => {
  const msg = JSON.parse(ev.data);
  const waiting = msg.id && pending.get(msg.id);
  if (!waiting) return;
  pending.delete(msg.id);
  msg.error ? waiting.reject(new Error(JSON.stringify(msg.error))) : waiting.resolve(msg.result);
});
const send = (method, params = {}) => new Promise((resolve, reject) => {
  const id = nextId++;
  pending.set(id, { resolve, reject });
  ws.send(JSON.stringify({ id, method, params }));
});

async function evaluate(expression) {
  const { result, exceptionDetails } = await send("Runtime.evaluate", {
    expression, returnByValue: true, awaitPromise: true,
  });
  if (exceptionDetails) throw new Error(JSON.stringify(exceptionDetails));
  return result.value;
}

await send("Page.enable");
await send("Page.navigate", {
  url: "file:///" + path.resolve(dir, "print.html").replace(/\\/g, "/"),
});
await sleep(1500);
// IBM Plex arrives over the network and every line count here depends on it.
await evaluate("document.fonts.ready.then(() => 0)");
await sleep(500);

const report = await evaluate(`(() => {
  const name = (el) => {
    const heading = el.querySelector("div");
    return ((heading ?? el).textContent ?? "").trim().slice(0, 44) || el.tagName;
  };
  const bad = [];
  for (const el of document.querySelectorAll('[style*="height:"]')) {
    if (!/(^|;)\\s*height:/.test(el.getAttribute("style") || "")) continue;
    if (el.scrollHeight > el.clientHeight + 1) {
      bad.push({ box: name(el), fits: el.clientHeight, needs: el.scrollHeight });
    }
  }
  const pages = [...document.querySelectorAll(".dc-page")]
    .map((p, i) => ({ page: i + 1, height: p.getBoundingClientRect().height }));
  return { bad, pages };
})()`);

for (const { page, height } of report.pages) {
  const shot = await send("Page.captureScreenshot", {
    format: "png",
    clip: { x: 0, y: (page - 1) * 720, width: 1280, height: 720, scale: 1 },
    captureBeyondViewport: true,
  });
  writeFileSync(path.join(dir, `page-${page}.png`), Buffer.from(shot.data, "base64"));
  if (height !== 720) console.log(`page ${page} is ${height} tall, not 720`);
}

for (const { box, fits, needs } of report.bad) {
  console.log(`does not fit: ${box} -- ${needs}px of words in ${fits}px of box`);
}
console.log(report.bad.length
  ? `${report.bad.length} box(es) overflowing`
  : `${report.pages.length} page(s), everything fits`);

ws.close();
chrome.kill();
process.exit(report.bad.length ? 1 : 0);
