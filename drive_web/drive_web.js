// The drive console's script: one page, six connections, one state object.
//
// Split out of drive_web.html, which was 1948 lines with this and the
// stylesheet inside it. Served by drive_web.py at /drive_web.js and deployed
// alongside the page.
//
// Every element this reaches for by name has to exist in the markup, which is
// checked in test_page.py rather than by a browser.

"use strict";

const $ = (id) => document.getElementById(id);
let setup = null;          // the ladders and the colour key, fetched once
let state = null;          // the last thing the server said
let mapGen = -1, frameGen = -1, wifiGen = "";
let networks = [];         // fetched from /wifi.json when wifiGen moves
let mapArrivedAt = 0;      // when this browser received the map on screen
let noticeSeq = -1, noticeTimer = 0;
// How long a notice stays on screen. Timed here rather than counted down in the
// state, because a number the rover publishes is a number that changes, and a state
// that changes is a state that goes down the wire.
const NOTICE_S = 25;

// --- talking to the server --------------------------------------------------

// Answers with whether the ask got out, which every control here throws away: a
// button that has been pressed has nothing left to decide, and what became of it
// arrives in the next state like everything else.
function post(action) {
  return fetch("/do", {method: "POST", headers: {"Content-Type": "application/json"},
                       body: JSON.stringify(action)})
    .then((reply) => reply.ok, () => false);
}

// What the boxes currently say, sent with the action rather than remembered by
// the server: they are the only things on this page that are genuinely local,
// because a half-typed distance is not a fact about the rover.
const number = (id) => $(id).value.trim();

// The map's age is the one thing on this page that changes while nothing happens,
// so it is the one thing drawn on a clock. A second is as fine as it is ever shown.
function tick() {
  if (state) render(state);
}

function listen() {
  const stream = new EventSource("/events");
  stream.addEventListener("state", (event) => render(JSON.parse(event.data)));
  stream.onerror = () => {
    // EventSource reconnects by itself; say so rather than looking frozen.
    $("link").textContent = "lost the console server -- reconnecting";
    $("link").className = "down";
  };
}

// --- drawing it -------------------------------------------------------------

function render(next) {
  state = next;

  const link = $("link");
  link.textContent = next.link.text;
  link.className = next.link.connected ? "up" : "down";
  if (document.activeElement !== $("address")) $("address").value = next.link.address;
  // A superseded move is still the move in flight, so it keeps its name and its
  // stopwatch -- but a click that has been taken and is waiting for the wheels
  // would otherwise look for a second like a click that went nowhere.
  $("busy").textContent = next.busy
    ? `${next.busy.name}  ${next.busy.seconds} s`
      + (next.busy.superseded ? "  \u2014 new target waiting" : "") : "";
  $("watchers").textContent = next.watching > 1
    ? `${next.watching} browsers watching` : "";

  drawVoice(next.omni);
  drawStatus(next.status);
  // The button is for a lidar that has stopped talking, and it costs the camera and
  // the OAK a few seconds -- so it is offered when it would help and out of the way
  // when it would not.
  $("resetLidar").disabled = !next.lidar.offer;
  $("lidarNote").textContent = next.lidar.note;
  $("plan").textContent = next.plan;

  if (next.map.gen && next.map.gen !== mapGen) mapArrivedAt = Date.now();
  drawPicture($("mapImg"), $("mapEmpty"), "/map.png", next.map.gen,
              () => mapGen, (g) => mapGen = g, next.map.error);
  if (next.map.width) {
    $("mapShot").style.aspectRatio = `${next.map.width} / ${next.map.height}`;
  }
  // How old the picture is, said out loud, because without a number there is no
  // telling a map that is two seconds behind from one that stopped arriving a
  // minute ago. Two sources, and neither of them ticks in the state: normally this
  // browser counts from when the picture arrived, and the rover speaks up only once
  // the map is late enough that its own account is the honest one -- which is also
  // the case a page that has just opened cannot work out for itself.
  const counted = mapArrivedAt ? (Date.now() - mapArrivedAt) / 1000 : null;
  const late = next.map.age_s;
  const age = late !== null && (counted === null || late > counted) ? late : counted;
  const drawn = age === null ? "" : age < 1.5 ? "just now" : `${age.toFixed(0)} s ago`;
  $("mapNote").textContent = next.map.drawing
    ? (drawn ? `drawing... (showing one from ${drawn})` : "drawing...")
    : [drawn && `drawn ${drawn}`, next.map.settings, next.map.note]
        .filter(Boolean).join(" -- ");
  $("mapError").textContent = next.map.error || "";
  $("roverUp").checked = next.map.rover_up;
  // What it takes with it, said on the armed press rather than in a dialog: the
  // world state is measured entirely in the map's own frame, so it goes too.
  $("clearMap").textContent = next.clear_armed
    ? "clear map and world state -- press again" : "clear map";

  drawPicture($("frameImg"), $("frameEmpty"), "/frame.jpg", next.frame.gen,
              () => frameGen, (g) => frameGen = g, next.frame.error);
  $("frameNote").textContent = next.frame.note;
  $("frameError").textContent = next.frame.error || "";

  $("tracking").textContent = next.tracking;
  $("lights").textContent = next.lights.text;

  $("battery").textContent = next.battery.text;
  $("battery").className = "reading " + verdictOf(next.battery.state);
  $("batteryNote").textContent = next.battery.note;

  drawWifi(next.wifi);
  drawNotice(next.notice);
  drawWorld(next.world);

  // Only the moves go out while one is running; stop, the map and the camera
  // stay live, which is the whole reason they have their own connections.
  //
  // Exploring counts as the wheels being taken even though this console is not
  // waiting on a call for it -- the rover holds the same mutex either way, so a
  // drive sent now comes back "busy". Greying them is the console saying that
  // before the rover has to.
  const wheelsTaken = next.busy !== null || next.exploring;
  for (const button of document.querySelectorAll("[data-move]")) {
    button.disabled = !next.link.can_drive || wheelsTaken;
  }
  // Face tracking is not driving and is not held up by it. It goes out on the
  // status connection rather than the move one, the rover runs the camera while
  // the wheels turn, and a daemon offering no driving tools still has a camera --
  // so what these two buttons wait for is the rover admitting it has them.
  for (const button of document.querySelectorAll("[data-track]")) {
    button.disabled = !next.link.tools.includes("start_tracking");
  }
  $("clearMap").disabled = !next.link.connected || wheelsTaken;

  // The explore toggle. Its rules are not the driving buttons' rules, which is
  // why it is not one of them: those go out while anything is running, and this
  // one has to stay live *while exploring*, because pressing it again is how you
  // stop. So it is greyed only when some **other** move has the wheels, or when
  // the rover is too old to offer the tool at all.
  //
  // `next.exploring` is the rover's own answer, not "this console has a call in
  // flight" -- starting one returns in a moment and leaves the rover driving for
  // ten minutes, and the voice model or a second browser can start one too. A
  // toggle drawn from anything else would be a button disagreeing with the room.
  const explore = $("explore");
  explore.classList.toggle("on", next.exploring);
  explore.textContent = next.exploring ? "stop exploring" : "explore";
  explore.disabled = !next.link.can_drive
      || !next.link.tools.includes("explore")
      || (next.busy !== null && !next.exploring);
}

const verdictOf = (state) =>
  ({full: "good", ok: "", low: "warn", critical: "bad", absent: "bad"})[state] || "";

function drawVoice(omni) {
  if (!omni) return;
  const button = $("micButton"), where = $("voiceState");
  if (!omni.available) {
    button.disabled = true;
    where.textContent = "not on this console";
    $("voiceError").textContent = omni.why || "";
    return;
  }
  button.disabled = false;
  const live = omni.state === "live" || omni.state === "starting";
  button.textContent = live ? "stop talking" : "start talking";
  where.textContent = omni.state
    + (omni.state === "live" && !omni.listening ? " (no microphone attached)" : "")
    + (omni.seconds ? `  ${Math.round(omni.seconds)}s` : "");
  where.className = "mono " + (omni.state === "live" ? "up"
    : omni.state === "error" ? "down" : "");
  $("voiceHeard").textContent = omni.heard ? `you: ${omni.heard}` : "";
  $("voiceSaid").textContent = omni.said ? `rover: ${omni.said}` : "";
  // **Only what the rover said.** This runs on every state push, which is ten
  // times a second, and it used to write into the box the page also used for its
  // own errors -- so "you have not entered a token" appeared and was overwritten
  // within a tenth of a second, which reads as a button that flashes something
  // and does nothing. The page's own messages go to #voiceNote and stay there
  // until the next thing the person does.
  $("voiceError").textContent = omni.error || "";
}

// --- the microphone ---------------------------------------------------------
//
// **The rover holds the conversation; this holds a microphone and a speaker.**
// Audio goes up as PCM16 at setup.mic_rate and comes back as PCM16 at
// setup.play_rate, and the only other thing on the socket is a line saying where
// playback has actually got to -- which is what lets an interruption tell the
// model how much of its reply was audible. Sending what was *received* instead
// would teach it, every single time, that it said more than anybody heard.
//
// getUserMedia needs a secure context, which is why this console serves https at
// all. The echo cancellation is the browser's, and it is the reason a laptop
// with open speakers works here where the desk client wanted headphones.
const CAPTURE = `
class Capture extends AudioWorkletProcessor {
  constructor() { super(); this.buffer = []; this.have = 0; }
  process(inputs) {
    const channel = inputs[0] && inputs[0][0];
    if (!channel) return true;
    this.buffer.push(new Float32Array(channel));
    this.have += channel.length;
    // A tenth of a second a message. A frame per 128 samples is 125 messages a
    // second down a socket that is carrying a conversation, and the service
    // cannot act on a block that small anyway.
    if (this.have >= 1600) {
      const out = new Int16Array(this.have);
      let at = 0;
      for (const block of this.buffer) {
        for (let i = 0; i < block.length; i++) {
          const clipped = Math.max(-1, Math.min(1, block[i]));
          out[at++] = clipped < 0 ? clipped * 0x8000 : clipped * 0x7fff;
        }
      }
      this.port.postMessage(out.buffer, [out.buffer]);
      this.buffer = []; this.have = 0;
    }
    return true;
  }
}
registerProcessor("capture", Capture);
`;

const voice = {
  socket: null, capture: null, play: null, media: null, node: null,
  gen: 0, queued: 0, nextStart: 0, sources: [], ticker: null,
  token: localStorage.getItem("omniToken") || "",

  wanted() { return !!this.socket; },

  note(text) { $("voiceNote").textContent = text; },

  async toggle() {
    this.note("");
    if (this.wanted()) { this.stop(true); return; }
    if (!this.token) {
      // Open the box rather than pointing at it. A message about a field that is
      // folded away is a message about something invisible.
      $("voiceTokenBox").open = true;
      $("voiceToken").focus();
      this.note("paste the token from ~/.ugv/console.token below");
      return;
    }
    this.note("opening the microphone...");
    try {
      await this.listen();
    } catch (error) {
      this.note(this.explain(error));
      this.stop(true);
      return;
    }
    this.note("");
    post({omni: true, token: this.token});
  },

  explain(error) {
    // The four that actually happen, said as what to do rather than as what the
    // specification calls them. Anything else is passed through verbatim, which
    // is more use than a sentence that guesses wrong.
    const name = (error && error.name) || "";
    if (!navigator.mediaDevices) {
      return "no microphone: this page is not a secure context";
    }
    if (name === "NotAllowedError") {
      return "the browser refused the microphone -- allow it from the address bar";
    }
    if (name === "NotFoundError") {
      return "no microphone the browser can see";
    }
    if (name === "NotReadableError") {
      return "something else is holding the microphone";
    }
    return `${error}`;
  },

  async listen() {
    // The microphone first, because it is the half that asks permission and the
    // half that fails on an http page -- and a session started before it is
    // known to work is a session that has to be torn down again.
    this.media = await navigator.mediaDevices.getUserMedia({
      audio: {echoCancellation: true, noiseSuppression: true,
              autoGainControl: true, channelCount: 1}});
    this.capture = new AudioContext({sampleRate: setup.mic_rate});
    await this.capture.audioWorklet.addModule(
      URL.createObjectURL(new Blob([CAPTURE], {type: "text/javascript"})));
    this.play = new AudioContext({sampleRate: setup.play_rate});
    // **Both contexts are resumed by hand.** A context made inside a click
    // handler starts running; one made after an `await` -- and asking for the
    // microphone is an await -- may not, because the user activation that
    // permits it can have expired by then. A suspended capture context is the
    // quiet failure of this whole page: the graph is built, the socket is open,
    // the session is live, and the worklet is never called, so the rover listens
    // to silence and nobody is told anything is wrong.
    await Promise.all([this.capture.resume(), this.play.resume()]);
    this.queued = 0; this.nextStart = 0; this.sources = [];

    // Built as a string rather than by editing a URL's protocol, which is a
    // setter with rules about which schemes may replace which and no way to
    // find out that it declined.
    const scheme = location.protocol === "https:" ? "wss:" : "ws:";
    const socket = new WebSocket(
      `${scheme}//${location.host}/audio?k=${encodeURIComponent(this.token)}`);
    socket.binaryType = "arraybuffer";
    this.socket = socket;
    await new Promise((ready, failed) => {
      socket.onopen = ready;
      socket.onerror = () => failed(new Error(
        "the rover would not open the audio socket"));
      // A socket that is closed before it opens never fires onerror in some
      // browsers, so the close is a failure too until the open has happened.
      socket.onclose = () => failed(new Error(
        "the rover closed the audio socket at once -- probably a wrong token"));
    });
    socket.onclose = () => this.stop(false);
    socket.onmessage = (event) => this.heard(event.data);

    const source = this.capture.createMediaStreamSource(this.media);
    this.node = new AudioWorkletNode(this.capture, "capture");
    this.node.port.onmessage = (event) => {
      if (socket.readyState === WebSocket.OPEN) socket.send(event.data);
    };
    // The worklet has to be pulled by something for `process` to be called at
    // all, and the destination is the only thing that pulls -- so it is
    // connected there even though nothing is meant to come out of it. It writes
    // nothing to its outputs, so what comes out is silence rather than the
    // microphone played back into the room it is listening to.
    source.connect(this.node).connect(this.capture.destination);

    this.ticker = setInterval(() => this.report(), 200);
  },

  heard(data) {
    if (typeof data === "string") { this.control(JSON.parse(data)); return; }
    const pcm = new Int16Array(data);
    const audio = this.play.createBuffer(1, pcm.length, setup.play_rate);
    const channel = audio.getChannelData(0);
    for (let i = 0; i < pcm.length; i++) channel[i] = pcm[i] / 0x8000;
    const source = this.play.createBufferSource();
    source.buffer = audio;
    source.connect(this.play.destination);
    // Scheduled back to back rather than played on arrival, because "play it
    // now" for every block that lands is a seam in the middle of every word.
    const now = this.play.currentTime;
    if (this.nextStart < now) this.nextStart = now + 0.02;
    source.start(this.nextStart);
    this.nextStart += audio.duration;
    this.queued += audio.duration;
    this.sources.push(source);
    source.onended = () => {
      const at = this.sources.indexOf(source);
      if (at >= 0) this.sources.splice(at, 1);
    };
  },

  control(message) {
    if (message.t === "begin") {
      this.gen = message.gen;
      this.queued = 0;
      this.nextStart = 0;
    } else if (message.t === "flush") {
      for (const source of this.sources) { try { source.stop(); } catch (e) {} }
      this.sources = [];
      // Everything still in the future stops existing, so what was queued is now
      // exactly what was played.
      this.queued -= Math.max(0, this.nextStart - this.play.currentTime);
      this.nextStart = this.play.currentTime;
      this.report();
    } else if (message.t === "evicted") {
      this.note(message.why || "another browser took the microphone");
      this.stop(false);
    }
  },

  played() {
    if (!this.play) return 0;
    // What is still scheduled ahead of the clock has not been heard. Everything
    // else has, whatever the network did on the way here.
    const ahead = Math.max(0, this.nextStart - this.play.currentTime);
    return Math.max(0, (this.queued - ahead) * 1000);
  },

  report() {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(
        {t: "played", gen: this.gen, ms: Math.round(this.played())}));
    }
  },

  stop(tell) {
    if (this.ticker) { clearInterval(this.ticker); this.ticker = null; }
    if (this.socket) {
      const socket = this.socket;
      this.socket = null;
      socket.onclose = null;
      try { socket.close(); } catch (e) {}
    }
    if (this.node) { try { this.node.disconnect(); } catch (e) {} this.node = null; }
    if (this.media) {
      for (const track of this.media.getTracks()) track.stop();
      this.media = null;
    }
    for (const context of [this.capture, this.play]) {
      if (context) { try { context.close(); } catch (e) {} }
    }
    this.capture = this.play = null;
    this.sources = [];
    if (tell) post({omni: false, token: this.token});
  },
};

function drawStatus(status) {
  const holder = $("statusRows");
  if (holder.childElementCount !== status.rows.length * 2) {
    holder.replaceChildren();
    for (const row of status.rows) {
      const key = document.createElement("div");
      key.className = "k"; key.textContent = row[0];
      const value = document.createElement("div");
      value.className = "v";
      holder.append(key, value);
    }
  }
  status.rows.forEach((row, index) => {
    const value = holder.children[index * 2 + 1];
    if (value.textContent !== row[1]) value.textContent = row[1];
    value.className = row[2] ? "v alarm" : "v";
  });
  $("pose").textContent = status.pose;
  $("statusError").textContent = status.error || "";
}

function drawPicture(img, empty, url, gen, get, set, error) {
  if (gen && gen !== get()) {
    set(gen);
    // The generation is in the URL, so the browser fetches each picture once and
    // never has to be told not to cache the last one. It carries the console's own
    // run with it -- `?gen=8f3a1c02-7` rather than `?gen=7` -- because the counter
    // alone starts again at 1 in every new console, and these are cached for a
    // year: the plain number handed a browser the *previous* run's pictures back in
    // order, which reads as a recorded run replaying over a live rover.
    img.src = `${url}?gen=${gen}`;
    img.hidden = false;
    empty.hidden = true;
  } else if (!gen) {
    empty.textContent = error || empty.textContent;
  }
}

function drawWifi(wifi) {
  $("wifi").textContent = wifi.text;
  $("wifi").className = "reading " + (wifi.verdict || "");
  $("wifiWhere").textContent = wifi.where;
  $("wifiNote").textContent = wifi.note;
  $("wifiScan").disabled = wifi.scanning || wifi.supported === false;
  $("wifiScan").textContent = wifi.scanning ? "scanning..." : "look for networks";
  // A join in flight greys every join button, and that can change without the list
  // changing, so it is applied on every state rather than only when the rows are
  // rebuilt.
  for (const button of $("wifiList").querySelectorAll("button")) {
    button.disabled = !!wifi.joining;
  }

  // The rows themselves are fetched rather than pushed -- three and a half kB that
  // used to ride in a state going out many times a second -- and rebuilt only when
  // the rover says the list has actually changed, so that a poll does not tear the
  // rows out from under a finger reaching for one of them.
  const gen = wifi.networks_gen;
  if (!gen || gen === wifiGen) return;
  wifiGen = gen;
  fetch(`/wifi.json?gen=${gen}`)
    .then((reply) => reply.ok ? reply.json() : null)
    .then((heard) => { if (heard) { networks = heard; drawNetworks(); } })
    .catch(() => {});   // the next state carrying a new count asks again
}

function drawNetworks() {
  const list = $("wifiList");
  list.replaceChildren();
  const joining = state && state.wifi.joining;
  for (const network of networks) {
    const row = document.createElement("tr");
    row.className = network.in_use ? "here" : (network.configured ? "" : "stranger");
    const name = document.createElement("td");
    name.textContent = network.ssid;
    const signal = document.createElement("td");
    signal.className = "num"; signal.textContent = network.signal;
    const action = document.createElement("td");
    if (network.joinable) {
      const join = document.createElement("button");
      join.textContent = "join";
      join.onclick = () => post({do: "wifi_join", ssid: network.ssid});
      join.disabled = !!joining;
      action.append(join);
    } else {
      action.textContent = network.note;
      action.className = "wrap";
    }
    row.append(name, signal, action);
    list.append(row);
  }
}

// The console's own line, which the rover replaces rather than appends to. The
// count is what makes a new notice new: the same sentence said twice is two
// notices, and re-reading an unchanged one on every state would restart the fade
// of a line that has been sitting there for twenty seconds.
function drawNotice(notice) {
  if (notice.seq === noticeSeq) return;
  noticeSeq = notice.seq;
  const box = $("notice");
  box.textContent = notice.text;
  box.className = notice.tag;
  box.hidden = !notice.text;
  clearTimeout(noticeTimer);
  if (notice.text) {
    noticeTimer = setTimeout(() => { box.hidden = true; }, NOTICE_S * 1000);
  }
}

// --- the controls -----------------------------------------------------------

function wire() {
  $("stop").onclick = () => post({do: "stop"});
  $("connect").onclick = () => post({do: "connect", address: $("address").value});
  $("address").onkeydown = (e) => {
    if (e.key === "Enter") post({do: "connect", address: $("address").value});
  };

  // The preset turns bind their own angle in start(); only these two read a box.
  $("driveCard").querySelector('[data-move="drive"]').onclick = () => drive();
  $("driveCard").querySelector('[data-move="turn"]').onclick = () => turn();
  // No arguments and no keyboard shortcut, both deliberately. This is the one
  // control that hands the rover ten minutes of its own work, and it should take
  // a deliberate click rather than a key pressed next to the arrows.
  //
  // Turning it off is an ordinary stop rather than an explore-specific one. It
  // is the same act -- cancel whatever has the wheels -- and the rover already
  // reports the run as "stopped" when it lands, so a second way of saying stop
  // would be a second thing that could be wrong about the first.
  $("explore").onclick = () => post(
      {do: $("explore").classList.contains("on") ? "stop" : "explore"});
  for (const button of document.querySelectorAll("[data-zoom]")) {
    button.onclick = () => post({do: "map", zoom: +button.dataset.zoom});
  }
  for (const button of document.querySelectorAll("[data-track]")) {
    button.onclick = () => post({do: "track", name: button.dataset.track});
  }
  for (const button of document.querySelectorAll("[data-world-build]")) {
    button.onclick = () => post({do: "world", what: "build",
                                 on: button.dataset.worldBuild === "on"});
  }
  for (const button of document.querySelectorAll("[data-light]")) {
    button.onclick = () => post({do: "lights",
      level: button.dataset.light === "on" ? setup.light_max : 0});
  }
  $("refreshMap").onclick = () => post({do: "map"});
  $("roverUp").onchange = () => post({do: "map", rover_up: $("roverUp").checked});
  $("resetLidar").onclick = () => post({do: "reset_lidar"});
  $("describe").onclick = () => post({do: "describe"});
  $("clearMap").onclick = () => post({do: "clear_map"});
  $("wifiScan").onclick = () => post({do: "wifi_scan"});

  $("mapImg").onclick = (event) => {
    // The picture is scaled to whatever width the panel came out at, so a click
    // has to be divided back into the picture's own pixels before it goes to the
    // server -- which is the only arithmetic this page does about the map. The
    // metres are the renderer's business, on the other end.
    const img = event.currentTarget;
    const box = img.getBoundingClientRect();
    // object-fit: contain, so the picture is centred in whatever letterboxing the
    // panel's aspect ratio leaves over.
    const scale = Math.min(box.width / img.naturalWidth,
                           box.height / img.naturalHeight);
    const col = (event.clientX - box.left - (box.width - img.naturalWidth * scale) / 2)
                / scale;
    const row = (event.clientY - box.top - (box.height - img.naturalHeight * scale) / 2)
                / scale;
    if (col < 0 || row < 0 || col >= img.naturalWidth || row >= img.naturalHeight) return;
    post({do: "tap", col: col, row: row, speed_ms: number("speed")});
  };
}

const drive = () => post({do: "drive", distance_m: number("distance"),
                          speed_ms: number("speed")});
const turn = (angle) => post({do: "turn",
                              angle_deg: angle === undefined ? number("angle") : angle});

function keys() {
  addEventListener("keydown", (event) => {
    if (event.key === " " || event.key === "Escape") {
      // Stop is the one key that works while a distance is being typed. Nothing
      // else does: without that guard, typing 0.5 sends an arrow to the motors on
      // the way to the decimal point.
      event.preventDefault();
      post({do: "stop"});
      return;
    }
    if (event.target.tagName === "INPUT") return;
    const angle = Number(number("angle")) || 90;
    const moves = {
      ArrowUp: () => drive(),
      ArrowLeft: () => turn(angle),
      ArrowRight: () => turn(-angle),
      // Plus zooms in, the way it does everywhere else, which means asking for a
      // smaller extent -- a step down the ladder, not up it.
      "+": () => post({do: "map", zoom: -1}),
      "=": () => post({do: "map", zoom: -1}),
      "-": () => post({do: "map", zoom: 1}),
    };
    if (moves[event.key]) { event.preventDefault(); moves[event.key](); }
  });
}

async function start() {
  setup = await (await fetch("/setup")).json();
  const pad = $("turnPad");
  // Positive is left in this whole repository, and left is the left column, so a
  // button's place on screen matches the way the rover is about to go.
  for (const magnitude of setup.presets_deg) {
    for (const sign of [1, -1]) {
      const button = document.createElement("button");
      button.dataset.move = "preset";
      button.textContent = `${magnitude} ${sign > 0 ? "left" : "right"}`;
      button.onclick = () => turn(sign * magnitude);
      pad.append(button);
    }
  }
  const legend = $("legend");
  for (const [colour, label] of setup.legend) {
    const item = document.createElement("span");
    const swatch = document.createElement("i");
    swatch.style.background = colour;
    item.append(swatch, document.createTextNode(label));
    legend.append(item);
  }
  $("voiceToken").value = voice.token;
  $("voiceTokenSave").onclick = () => {
    voice.token = $("voiceToken").value.trim();
    localStorage.setItem("omniToken", voice.token);
    voice.note(voice.token ? "token remembered" : "the token is empty");
  };
  $("micButton").onclick = () => voice.toggle();
  // A tab that goes away takes the microphone with it, the same rule the rest of
  // this console follows: the session on the rover is closed rather than left
  // holding a conversation with an empty room.
  addEventListener("pagehide", () => { if (voice.wanted()) voice.stop(true); });
  wire();
  wireWorld();
  keys();
  listen();
  setInterval(tick, 1000);
}

start();
