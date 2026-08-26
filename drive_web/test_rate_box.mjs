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
import http from "node:http";
import https from "node:https";
import {existsSync, readFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = join(HERE, "..");
const PAGE = join(HERE, "drive_web.html");
const ROVER_PORT = 18769, CONSOLE_PORT = 18899;

/** With no arguments this brings up its own mock and its own console and models
 *  the slow link. Given a console that is already running -- the rover's --  it
 *  drives that one instead and lets the real link supply its own delay, which is
 *  how the fix gets proved on the machine that serves the page rather than on a
 *  copy of it:
 *
 *      NODE_TLS_REJECT_UNAUTHORIZED=0 \
 *        node drive_web/test_rate_box.mjs --console https://192.168.1.80:8771/
 *
 *  --wait is how long an ask is given to cross that link before the run calls it
 *  lost. The rover's has been seen taking eleven seconds. */
const argv = process.argv.slice(2);
const option = (name, fallback) => {
  const at = argv.indexOf(name);
  return at < 0 ? fallback : argv[at + 1];
};
const REMOTE = option("--console", null);
const WAIT_MS = Number(option("--wait", REMOTE ? 20000 : 1500));
const BASE = REMOTE || `http://127.0.0.1:${CONSOLE_PORT}/`;

process.on("unhandledRejection", (why) => {
  const said = (why && why.stack) || why;
  process.stdout.write(`the page threw and nobody caught it: ${said}\n`);
});

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
  box.refused = [];
  Object.defineProperty(box, "value", {
    get: () => picked,
    set: (wanted) => {
      const known = box.options.some((o) => o.value === String(wanted));
      if (!known && box.options.length) box.refused.push(String(wanted));
      picked = known ? String(wanted) : "";
    },
  });
  return box;
}

/** How long an ask takes to reach the rover, and whether it gets there at all. */
const link = {askMs: 0, askFails: false};
/** Every ask the page put on the wire, so that a rate nobody chose can be traced
 *  to the page having sent it rather than to the rover having invented it. */
const asked = [];
/** Every transcript line the console sent down, for the same reason. */
const heard = [];

/** One request, on Node's own HTTP rather than fetch.
 *
 * Not a preference. fetch gives up on a connection that takes more than ten
 * seconds to open and will not be talked out of it, and opening a connection to
 * the rover takes longer than that whenever its own event stream is in the way
 * -- which is exactly the condition this file exists to test. The console's
 * certificate is its own, and this is a rover on a LAN rather than the web.
 */
// Kept open and reused, the way a browser keeps its handful of sockets to an
// origin. It matters here: on a link the console is already saturating, opening
// a fresh connection means a fresh TLS handshake competing with the stream, and
// what would be measured is the handshake rather than the page.
const pools = {
  "http:": new http.Agent({keepAlive: true, maxSockets: 6}),
  "https:": new https.Agent({keepAlive: true, maxSockets: 6,
                             rejectUnauthorized: false}),
};

function ask(url, {method = "GET", body = null, quietFor = WAIT_MS + 30000,
                   stream = null} = {}) {
  const target = new URL(url, BASE);
  const agent = target.protocol === "https:" ? https : http;
  return new Promise((answered, blew) => {
    const sent = agent.request(target, {
      method, agent: pools[target.protocol], rejectUnauthorized: false,
      headers: body ? {"Content-Type": "application/json",
                       "Content-Length": Buffer.byteLength(body)} : {},
    }, (reply) => {
      const ok = reply.statusCode < 400;
      if (stream) { stream(reply); answered({ok}); return; }
      let text = "";
      reply.setEncoding("utf-8");
      reply.on("data", (chunk) => { text += chunk; });
      reply.on("end", () => answered({ok, text}));
    });
    // The event stream is meant to be quiet between states, and on a rover that
    // has nothing to say it can be quiet for a long time, so only the one-shot
    // requests are given a deadline.
    if (!stream) {
      sent.setTimeout(quietFor,
                      () => sent.destroy(new Error(`${target.pathname}: nothing back `
                                                   + `in ${quietFor / 1000} s`)));
    }
    sent.on("error", blew);
    if (body) sent.write(body);
    sent.end();
  });
}

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
    this.pump(path).catch((error) => {
      process.stdout.write(`  (the event stream fell over: ${error.message})\n`);
      if (this.onerror) this.onerror(error);
    });
  }
  addEventListener(name, handler) { this.handlers[name] = handler; }
  close() { this.closed = true; }
  async pump(path) {
    let buffer = "";
    await ask(path, {stream: (reply) => {
      reply.setEncoding("utf-8");
      reply.on("data", (chunk) => {
        if (this.closed) { reply.destroy(); return; }
        buffer += chunk;
        let cut;
        while ((cut = buffer.indexOf("\n\n")) >= 0) {
          const block = buffer.slice(0, cut);
          buffer = buffer.slice(cut + 2);
          const name = /^event: (.*)$/m.exec(block);
          const data = /^data: (.*)$/m.exec(block);
          if (!name || !data) continue;
          if (name[1] === "state") lastState = JSON.parse(data[1]);
          // Kept so that a rate nobody asked for can be laid at the door of
          // whatever actually asked for it.
          if (name[1] === "log") heard.push(...JSON.parse(data[1]));
          if (this.handlers[name[1]]) this.handlers[name[1]]({data: data[1]});
        }
      });
    }});
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
  fetch: async (url, options = {}) => {
    if (String(url).includes("/do")) {
      asked.push(String(options.body));
      await after(link.askMs);
      if (link.askFails) throw new TypeError("the ask never got out");
    }
    const reply = await ask(url, {method: options.method || "GET",
                                  body: options.body || null});
    return {ok: reply.ok, text: async () => reply.text,
            json: async () => JSON.parse(reply.text)};
  },
};

// --- the page itself ---------------------------------------------------------

async function loadPage() {
  // From the console being driven when there is a real one, because the file on
  // this desk is not evidence about the rover: what is being checked there is
  // the page the rover actually serves.
  const html = REMOTE ? (await ask("/")).text : readFileSync(PAGE, "utf-8");
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
  await ask("/do", {method: "POST",
                    body: JSON.stringify({do: "camera_rate", seconds})});
  const patience = WAIT_MS + 3000;
  for (let waited = 0; waited < patience && box.value !== String(seconds); waited += 20) {
    await after(20);
  }
}

async function tryPicking(box, from, to, {keepFocus, askMs, askFails, what}) {
  await reset(box, from);
  check(`${what}: the box starts on the rate the rover holds`, box.value, String(from));

  link.askMs = askMs;
  link.askFails = Boolean(askFails);
  pick(box, to, keepFocus);
  const runs = await watchBox(box, askMs + WAIT_MS);
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
  // A console that is already running is driven as it stands; nothing is started
  // and nothing is killed, because on the rover that console is the real one.
  const started = REMOTE ? [] : [
    launch(["voice_chat/mock_rover.py", "--port", String(ROVER_PORT), "--drive"],
           "mock rover on"),
    launch(["drive_web/drive_web.py", "--rover", `127.0.0.1:${ROVER_PORT}`,
            "--port", String(CONSOLE_PORT), "--bind", "127.0.0.1",
            "--no-tls", "--no-omni", "--no-browser", "--no-idle"],
           "drive console on"),
  ];
  try {
    for (const child of started) await child.ready;
    process.stdout.write(`driving the console at ${BASE}\n`);
    await loadPage();
    const box = document.getElementById("cameraRate");
    for (let waited = 0; waited < 5000 && !(box.options.length && box.value); waited += 50) {
      await after(50);
    }
    if (!box.options.length) throw new Error("the page never built the drop-down");

    // A tenth of a second is one tick of the console's pump; 1.2 s is a dozen
    // states crossing the ask, which is the rover's link scaled down to
    // something a test can wait for. Against a real rover none of that is
    // needed -- the link is already the slow one, and pretending otherwise
    // would only be measuring this file.
    const slow = REMOTE ? 0 : 1200;
    const where = REMOTE ? "over the real link" : "on the rover's link";
    if (!REMOTE) {
      await tryPicking(box, 1.0, 2.0, {keepFocus: true, askMs: 0,
                                       what: "on this desk, pointer still on the box"});
      await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 0,
                                       what: "on this desk, having looked away"});
    }
    await tryPicking(box, 1.0, 2.0, {keepFocus: true, askMs: slow,
                                     what: `${where}, pointer still on the box`});
    await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: slow,
                                     what: `${where}, having looked away`});
    // Longer than the page's own patience with an unconfirmed rate, which is the
    // case the rover taught: an ask that takes over fifteen seconds to arrive is
    // still an ask, and a page that gives up waiting for it puts the box back
    // exactly where the driver moved it from. Watched happening on 2026-08-26,
    // for 10.5 s of a 30 s window, before the timer was made to start when the
    // ask lands rather than when it is made.
    if (!REMOTE) {
      await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 17000,
                                       what: "when the ask is slower than the page's patience"});
    }
    await tryPicking(box, 1.0, 2.0, {keepFocus: false, askMs: 200, askFails: true,
                                     what: "when the ask never gets out"});
    // Left as it was found. This one is somebody's rover, not a fixture.
    if (REMOTE) await reset(box, 1.0);
  } finally {
    for (const child of started) child.kill();
  }
}

main().then(() => {
  for (const what of pass) process.stdout.write(`  ok   ${what}\n`);
  for (const what of fail) process.stdout.write(`  FAIL ${what}\n`);
  if (fail.length) {
    // A rate nobody chose has to be traceable to the page having asked for it
    // rather than to the rover having invented it, and a box showing nothing at
    // all has to be distinguishable from a box showing the wrong rung.
    const box = document.getElementById("cameraRate");
    process.stdout.write(`\n  what the page put on the wire: ${asked.join("  ")}\n`);
    const rates = heard.filter((line) => JSON.stringify(line).includes("camera"));
    if (rates.length) {
      process.stdout.write(`  what the console said about the camera:
`);
      for (const line of rates.slice(-8)) {
        process.stdout.write(`    ${JSON.stringify(line)}
`);
      }
    }
    if (box.refused.length) {
      process.stdout.write(`  values the box would not take: ${box.refused.join("  ")}\n`);
    }
  }
  process.stdout.write(`\n${pass.length} passed, ${fail.length} failed\n`);
  process.exit(fail.length ? 1 : 0);
}, (error) => {
  process.stdout.write(`the harness itself fell over: ${error.stack}\n`);
  process.exit(2);
});
