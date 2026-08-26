/**
 * The camera's rate drop-down, driven the way a hand drives it.
 *
 *     node drive_web/test_rate_box.mjs
 *
 * Everything else in this repository that checks the console checks the model on
 * the rover's side. This one is here because the fault is not there: the rate is
 * chosen in a browser, and what goes wrong to it goes wrong between the moment
 * the box is let go of and the moment the rover's answer comes back down the
 * event stream. That gap only exists in a page.
 *
 * So this is the real page. The script is lifted out of drive_web.html and run
 * against a document that is just enough of a browser for it -- with one honest
 * element in it, a select that takes a value only if one of its options carries
 * it, exactly as a real one does. Behind it is a real console serving a real
 * event stream and talking to the mock rover, so the ordering under test is the
 * console's own and not one invented here.
 *
 * The hand is modelled twice, because the two differ and a page can pass one
 * while failing the other: choosing a rate and leaving the pointer on the box,
 * and choosing a rate and looking away in the same breath.
 *
 * The link is modelled twice as well, and that is the half that matters. A
 * console on this desk answers the POST in a millisecond and there is barely a
 * gap to get wrong. The rover's own console is not that: measured from the desk
 * on 2026-08-26, a request took between eight and eleven seconds to be answered
 * while the same request took 74 ms on the board itself, because the console
 * pushes its whole 7.6 kB state ten times a second to every browser -- 91 kB/s
 * each -- and had eight streams open to one desk with a megabyte queued behind
 * them. So the slow ask here is not a pessimistic invention; it is the rover on
 * an ordinary evening, and a page that only works on the fast one is a page that
 * does not work.
 */
import {spawn} from "node:child_process";
import {existsSync, readFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = join(HERE, "..");
const PAGE = join(HERE, "drive_web.html");
const ROVER_PORT = 18769, CONSOLE_PORT = 18899;
const BASE = `http://127.0.0.1:${CONSOLE_PORT}/`;

const pass = [], fail = [];
const check = (what, got, want) => {
  const ok = JSON.stringify(got) === JSON.stringify(want);
  (ok ? pass : fail).push(
    ok ? what
       : `${what}\n         got ${JSON.stringify(got)}, wanted ${JSON.stringify(want)}`);
};
const after = (ms) => new Promise((done) => setTimeout(done, ms));

// --- a browser, as far as this page can tell ---------------------------------

function element(id) {
  return {
    id, tag: "", style: {}, dataset: {},
    classList: {add() {}, remove() {}, toggle() {}},
    textContent: "", className: "", value: "", src: "", innerHTML: "",
    hidden: false, disabled: false, checked: false, selected: false,
    naturalWidth: 640, naturalHeight: 480, scrollTop: 0, scrollHeight: 0,
    children: [], options: [], onclick: null, onchange: null, onload: null,
    onerror: null,
    append(...kids) {
      for (const kid of kids) {
        this.children.push(kid);
        if (kid && kid.tag === "option") this.options.push(kid);
      }
    },
    appendChild(kid) { this.append(kid); },
    replaceChildren(...kids) {
      this.children = [];
      this.options = [];
      this.append(...kids);
    },
    remove() {}, focus() {}, blur() {}, scrollIntoView() {},
    querySelector() { return element(""); },
    querySelectorAll() { return []; },
    addEventListener() {}, removeEventListener() {}, dispatchEvent() {},
    getBoundingClientRect: () => ({left: 0, top: 0, width: 640, height: 480}),
  };
}

/** The one element here that is not a stub.
 *
 * A real select is picky in a way that matters: assigning it a string no option
 * carries does not leave the old choice standing, it empties the box. A page
 * that wrote "2" into a list built out of "2.0" would show a blank rather than
 * the wrong rung -- a different fault with the same complaint behind it -- so
 * the model has to be able to tell the two apart.
 */
function selectBox(id) {
  const box = element(id);
  let picked = "";
  Object.defineProperty(box, "value", {
    get: () => picked,
    set: (wanted) => {
      picked = box.options.some((o) => o.value === String(wanted)) ? String(wanted) : "";
    },
  });
  return box;
}

/** How long an ask takes to reach the rover, and whether it gets there at all. */
const link = {askMs: 0, askFails: false};

const nodes = new Map();
const document = {
  activeElement: null,
  getElementById(id) {
    if (!nodes.has(id)) nodes.set(id, id === "cameraRate" ? selectBox(id) : element(id));
    return nodes.get(id);
  },
  createElement(tag) { const made = element(""); made.tag = tag; return made; },
  createTextNode(text) { return {tag: "#text", textContent: text}; },
  querySelectorAll() { return []; },
};

/** The event stream, over a real socket to a real console. */
let lastState = null;
class Stream {
  constructor(path) {
    this.handlers = {};
    this.onerror = null;
    this.pump(path).catch((error) => { if (this.onerror) this.onerror(error); });
  }
  addEventListener(name, handler) { this.handlers[name] = handler; }
  close() { this.closed = true; }
  async pump(path) {
    const response = await fetch(new URL(path, BASE));
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = "";
    while (!this.closed) {
      const {value, done} = await reader.read();
      if (done) return;
      buffer += decoder.decode(value, {stream: true});
      let cut;
      while ((cut = buffer.indexOf("\n\n")) >= 0) {
        const block = buffer.slice(0, cut);
        buffer = buffer.slice(cut + 2);
        const name = /^event: (.*)$/m.exec(block);
        const data = /^data: (.*)$/m.exec(block);
        if (!name || !data) continue;
        if (name[1] === "state") lastState = JSON.parse(data[1]);
        if (this.handlers[name[1]]) this.handlers[name[1]]({data: data[1]});
      }
    }
  }
}

const browser = {
  document,
  EventSource: Stream,
  location: {protocol: "http:", host: `127.0.0.1:${CONSOLE_PORT}`},
  navigator: {
    mediaDevices: {getUserMedia: async () => { throw new Error("no microphone here"); }},
  },
  localStorage: {getItem: () => null, setItem() {}, removeItem() {}},
  addEventListener() {},
  removeEventListener() {},
  AudioContext: class { constructor() { throw new Error("no audio here"); } },
  AudioWorkletNode: class {},
  WebSocket: class {},
  Blob: class {},
  alert() {},
  // Relative, the way a page means them -- and held back by however long the
  // link under test takes to carry an ask. Held on the way out rather than on
  // the way back, because that is where the rover's delay actually is: the
  // console queues the action and answers in the same breath, so a request that
  // takes ten seconds to be answered is a request that took ten seconds to
  // arrive, and the rate is not set until it does.
  fetch: async (url, options) => {
    if (!String(url).includes("/do")) return fetch(new URL(url, BASE), options);
    await after(link.askMs);
    if (link.askFails) throw new TypeError("the ask never got out");
    return fetch(new URL(url, BASE), options);
  },
};

// --- the page itself ---------------------------------------------------------

function loadPage() {
  const html = readFileSync(PAGE, "utf-8");
  const opened = html.lastIndexOf("<script>");
  const closed = html.lastIndexOf("</script>");
  if (opened < 0 || closed < opened) throw new Error("no script in drive_web.html");
  const source = html.slice(opened + "<script>".length, closed);
  const names = Object.keys(browser);
  // The page's own top-level names stay inside this function, which is all a
  // script tag gets anyway. Nothing here reaches in; it goes through the document.
  new Function(...names, source)(...names.map((name) => browser[name]));
}

/** Choosing a rung, as a pointer does it: the value moves first and the change
 *  event follows. Whether the box keeps the focus afterwards is the difference
 *  between a hand still resting on it and one that has already moved on. */
function pick(box, seconds, keepFocus) {
  box.value = String(seconds);
  document.activeElement = keepFocus ? box : null;
  box.onchange();
}

/** Every value the box takes over a stretch of time, in order, each with how
 *  long it stood -- because how long a wrong value is on screen is the whole
 *  difference between a flicker nobody sees and a setting that did not take. */
async function watchBox(box, ms) {
  const runs = [];
  for (let waited = 0; waited < ms; waited += 10) {
    const last = runs[runs.length - 1];
    if (last && last.value === box.value) last.ms += 10;
    else runs.push({value: box.value, ms: 10});
    await after(10);
  }
  return runs;
}

// --- the hands, and the links they are on ------------------------------------

/** Put the rover back on a known rung without going through the page, so each
 *  run starts where the last one was meant to end whatever it actually did. */
async function reset(box, seconds) {
  link.askMs = 0;
  link.askFails = false;
  await fetch(new URL("/do", BASE), {
    method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({do: "camera_rate", seconds}),
  });
  for (let waited = 0; waited < 3000 && box.value !== String(seconds); waited += 20) {
    await after(20);
  }
}

async function tryPicking(box, from, to, {keepFocus, askMs, askFails, what}) {
  await reset(box, from);
  check(`${what}: the box starts on the rate the rover holds`, box.value, String(from));

  link.askMs = askMs;
  link.askFails = Boolean(askFails);
  pick(box, to, keepFocus);
  const runs = await watchBox(box, askMs + 1500);
  const seen = runs.map((run) => run.value);
  const reverted = runs.filter((run) => run.value === String(from))
                       .reduce((total, run) => total + run.ms, 0);

  if (askFails) {
    // An ask that never got out must not leave the box telling the driver the
    // camera is on a rate the rover was never told about. Better a box that
    // goes back to the truth than one that quietly lies about the rover.
    check(`${what}: the box goes back to what the rover actually holds`,
          box.value, String(from));
    return;
  }
  check(`${what}: choosing ${to} s leaves the box on ${to} s`, box.value, String(to));
  check(`${what}: ...and it never falls back to ${from} s on the way`
        + (reverted ? ` -- it did, for ${(reverted / 1000).toFixed(1)} s` : ""),
        seen, [String(to)]);
  check(`${what}: ...and the rover ends up on ${to} s`,
        lastState && lastState.frame.every_s, to);
}

// --- running it --------------------------------------------------------------

function python() {
  for (const path of [join(ROOT, ".venv/Scripts/python.exe"),
                      join(ROOT, ".venv/bin/python")]) {
    if (existsSync(path)) return path;
  }
  return process.platform === "win32" ? "python" : "python3";
}

function launch(args, ready) {
  // Unbuffered, or "drive console on" sits in a pipe buffer until the process
  // ends and this waits fifteen seconds for a line that was printed at once.
  const child = spawn(python(), args, {cwd: ROOT,
                                       env: {...process.env, PYTHONUNBUFFERED: "1"}});
  let said = "";
  const collect = (chunk) => { said += chunk; };
  child.stdout.on("data", collect);
  child.stderr.on("data", collect);
  child.said = () => said;
  child.ready = new Promise((done, blew) => {
    const watch = setInterval(() => {
      if (said.includes(ready)) { clearInterval(watch); done(); }
    }, 50);
    setTimeout(() => {
      clearInterval(watch);
      blew(new Error(`never said "${ready}":\n${said}`));
    }, 15000);
  });
  return child;
}

async function main() {
  const rover = launch(["voice_chat/mock_rover.py", "--port", String(ROVER_PORT), "--drive"],
                       "mock rover on");
  const box_server = launch(["drive_web/drive_web.py", "--rover", `127.0.0.1:${ROVER_PORT}`,
                             "--port", String(CONSOLE_PORT), "--bind", "127.0.0.1",
                             "--no-tls", "--no-omni", "--no-browser", "--no-idle"],
                            "drive console on");
  try {
    await rover.ready;
    await box_server.ready;
    loadPage();
    const box = document.getElementById("cameraRate");
    for (let waited = 0; waited < 5000 && !(box.options.length && box.value); waited += 50) {
      await after(50);
    }
    if (!box.options.length) throw new Error("the page never built the drop-down");

    // A tenth of a second is one tick of the console's pump; 1.2 s is a dozen
    // states crossing the ask, which is the rover's link scaled down to
    // something a test can wait for.
    await tryPicking(box, 1.0, 2.0, {keepFocus: true, askMs: 0,
                                     what: "on this desk, pointer still on the box"});
    await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 0,
                                     what: "on this desk, having looked away"});
    await tryPicking(box, 1.0, 2.0, {keepFocus: true, askMs: 1200,
                                     what: "on the rover's link, pointer still on the box"});
    await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 1200,
                                     what: "on the rover's link, having looked away"});
    await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 200, askFails: true,
                                     what: "when the ask never gets out"});
  } finally {
    rover.kill();
    box_server.kill();
  }
}

main().then(() => {
  for (const what of pass) process.stdout.write(`  ok   ${what}\n`);
  for (const what of fail) process.stdout.write(`  FAIL ${what}\n`);
  process.stdout.write(`\n${pass.length} passed, ${fail.length} failed\n`);
  process.exit(fail.length ? 1 : 0);
}, (error) => {
  process.stdout.write(`the harness itself fell over: ${error.stack}\n`);
  process.exit(2);
});
