// The drive console's world-state popup: what the rover has seen, drawn.
//
// The browser half of world_state/, and the counterpart to drive_world.py on
// the server side. Read-only apart from the two buttons that act on the rover.
//
// Loaded BEFORE drive_web.js, which is what calls `start()`. Nothing here runs
// at load: it is all declarations, and everything it reaches for -- `$`, `post`,
// `state` -- is looked up when a draw actually happens, by which time
// drive_web.js has run.

// --- the world-state popup --------------------------------------------------
//
// Read-only, apart from the two buttons that act on the rover. The point of it is
// to decide whether any of this is worth believing, and everything in here is
// arranged so that the ways it can be wrong show up rather than being smoothed
// over: choosing a thing shows every look that was decided to be it, each with
// the box on the frame it was read from, so two things wrongly merged into one
// are visible as two different objects in one scroller; an entity last seen under
// a map that has since been cleared is greyed; and a failed inference is a row in
// the diagnostics tab rather than a popup that quietly did nothing.
//
// **Nothing here shows what a thing is called, because nothing measures that any
// more.** The word list that used to supply a name scored every phrase between
// 0.08 and 0.12 whatever the crop held, and it put "a computer monitor" on a
// sofa. What replaced it is the search tab, where the phrase a person types is
// compared with what the rover actually saw, and the pictures in the pane beside
// the entity list.

let worldGen = "", world = {}, worldTab = "entities";

const wTime = (t) => t ? new Date(t * 1000).toLocaleTimeString() : "-";
const wAgo = (t) => {
  if (!t) return "never";
  const s = Math.max(0, Date.now() / 1000 - t);
  if (s < 90) return `${s.toFixed(0)} s ago`;
  if (s < 5400) return `${(s / 60).toFixed(0)} min ago`;
  return `${(s / 3600).toFixed(1)} h ago`;
};
// A colour per entity, from its own name, so the same thing is the same colour on
// the map and in the list without anything having to keep a palette in step.
const wHue = (id) => {
  let hash = 0;
  for (const ch of id) hash = (hash * 31 + ch.charCodeAt(0)) % 360;
  return hash;
};

function drawWorld(w) {
  drawWorldBuilding(w);
  $("worldBack").hidden = !w.open;
  $("world").classList.toggle("on", w.open);
  $("worldCounts").textContent =
      `${w.entities} entit${w.entities === 1 ? "y" : "ies"}, `
      + `${w.observations} observation${w.observations === 1 ? "" : "s"}`
      + (w.backend ? ` -- ${w.backend}` : "");
  $("worldNote").textContent = w.note || "";
  $("worldError").textContent = w.error || "";
  $("worldError").hidden = !w.error;
  // Drawn from the rover's own "an inspection is running", not from a call this
  // console is waiting on: a second browser can start one, and a minute is long
  // enough that two people will.
  $("worldInspect").disabled = w.busy || !state.link.connected;
  $("worldInspect").textContent = w.busy ? "looking..." : "inspect world";
  if (!w.open || !w.gen || w.gen === worldGen) return;
  // Fetched rather than pushed, like the network list: tens of kilobytes that
  // change when somebody presses a button, against a state that goes out ten
  // times a second.
  worldGen = w.gen;
  fetch(`/world.json?gen=${w.gen}`)
    .then((reply) => reply.ok ? reply.json() : null)
    .then((body) => { if (body) { world = body; drawWorldBody(); } })
    .catch(() => {});    // the next state carrying a new tag asks again
}

// Where a thing is, in the words the popup uses for it everywhere. An entity
// without a placement is not a failure and must not read as one: it is a thing
// the rover has pointed at but not yet walked around, which is the ordinary state
// of everything until the rover moves.
function wPlace(entity) {
  const place = entity && entity.placement;
  const line = document.createElement("div");
  line.className = "wmeta";
  if (!place) {
    line.classList.add("wunplaced");
    line.textContent = "no position yet — seen from one place";
    return line;
  }
  const spread = entity.placement_uncertainty_m;
  line.classList.add("wplaced");
  line.textContent = `at (${(+place.x_m).toFixed(2)}, ${(+place.y_m).toFixed(2)}) m`
      + (spread == null ? "" : ` to within ${(+spread).toFixed(2)} m`)
      + (place.baseline_m ? ` · from two looks ${(+place.baseline_m).toFixed(2)} m `
                            + `apart crossing at ${Math.round(place.parallax_deg)}°`
                          : "");
  return line;
}

// The switch below face tracking, which is on whether or not the popup is open.
// What it shows is what the rover last said, never what this console last asked
// for: the voice session, another console or a script can turn it off, and a
// panel showing its own past would be the one place a rover that had quietly
// stopped recording still looked busy.
function drawWorldBuilding(w) {
  const state = $("worldBuilding");
  if (w.available === false) {
    state.textContent = "not on this rover";
    return;
  }
  if (w.building === null || w.building === undefined) {
    state.textContent = "-";
    return;
  }
  // The looks and the placing are on separate clocks, so both are on the line.
  // A rover recording steadily and placing nothing is what sent somebody looking
  // at this panel in the first place, and it has to be readable here.
  const settled = w.settled || {};
  const decided = settled.at
      ? ` · settled ${wAgo(settled.at)}: ${settled.waiting || 0} waiting`
        + (settled.created ? `, ${settled.created} placed` : "")
        + (settled.matched ? `, ${settled.matched} recognised` : "")
      : "";
  state.textContent = w.building
      ? `building — ${w.built_looks} look${w.built_looks === 1 ? "" : "s"}${decided}`
      : "off";
  for (const button of document.querySelectorAll("[data-world-build]")) {
    button.classList.toggle("on",
      (button.dataset.worldBuild === "on") === !!w.building);
  }
}

function drawWorldBody() {
  drawWorldList();
  drawWorldMap();
  drawWorldDetail();
  drawWorldObservations();
  drawWorldSearch();
  drawWorldDiagnostics();
}

function drawWorldList() {
  const list = $("wList");
  list.replaceChildren();
  const entities = world.entities || [];
  const summary = world.summary || {};
  if (!entities.length) {
    const empty = document.createElement("p");
    empty.className = "hint";
    empty.textContent = (summary.observations
        ? `${summary.observations} observations, none placed`
        : summary.inspections
        ? `nothing recorded, after ${summary.inspections} inspections`
        : "nothing recorded yet");
    list.append(empty);
    return;
  }
  const newest = Math.max(...entities.map((e) => e.last_seen_at || 0));

  for (const entity of entities) {
    const row = document.createElement("div");
    row.className = "wrow" + (entity.id === state.world.selected ? " on" : "");
    row.style.borderLeft = `4px solid hsl(${wHue(entity.id)} 70% 45%)`;
    row.onclick = () => post({do: "world", what: "select",
                              id: entity.id === state.world.selected ? "" : entity.id});

    // An identifier and no name. Nothing measures what a thing is called any
    // more, so a row says which thing it is, how often it has been seen and
    // where it is; what it looks like is one click away in the pane beside it.
    const head = document.createElement("div");
    const id = document.createElement("span");
    id.className = "wid mono";
    id.textContent = entity.id;
    head.append(id);
    row.append(head);

    const meta = document.createElement("div");
    meta.className = "wmeta";
    let text = `${entity.observation_count} observation`
             + `${entity.observation_count === 1 ? "" : "s"}`
             + ` · first ${wTime(entity.created_at)}`
             + ` · last ${wAgo(entity.last_seen_at)}`;
    if (entity.last_map_session && summary.map_session
        && entity.last_map_session !== summary.map_session) {
      // Not stale as such -- the sofa is still there -- but everything positional
      // about it belongs to a map that no longer exists, and the popup is the only
      // place that can say so.
      text += ` · last seen under map ${entity.last_map_session}, now `
            + `${summary.map_session}`;
      meta.classList.add("wold");
    }
    if (newest - (entity.last_seen_at || 0) > 300) meta.classList.add("wold");
    meta.append(document.createTextNode(text));
    row.append(meta);
    row.append(wPlace(entity));
    list.append(row);
  }
}

// The exact inverse of the sampling `render` does in lidar_slam/mapimg.py, which
// is where these five lines come from -- `to_px` there. A page that worked the
// geometry out for itself would be a second copy of the map's arithmetic, wrong
// the first time the resolution or the centring changed.
function wPointToPx(x, y, view) {
  const res = 0.05;
  const halfCells = Math.max(8, Math.round(view.half_extent_m / res));
  const pose = view.pose || {};
  const th = view.rover_up ? (pose.heading_deg || 0) * Math.PI / 180 : 0;
  const ac = Math.cos(th), as = Math.sin(th);
  const dgx = x / res - Math.trunc((pose.x_m || 0) / res);
  const dgy = y / res - Math.trunc((pose.y_m || 0) / res);
  const forward = dgx * ac + dgy * as;
  const sideways = -dgx * as + dgy * ac;
  return [(halfCells - sideways) * view.scale, (halfCells - forward) * view.scale];
}

function drawWorldMap() {
  const wrap = $("wMapWrap"), note = $("wMapNote"), svg = $("wRays");
  const view = state.map.view;
  const entities = world.entities || [];
  const selected = state.world.selected;
  const rays = [];
  for (const entity of entities) {
    const drawn = entity.rays || [];
    // One ray each while nothing is chosen, so the picture stays readable; all of
    // a chosen entity's, because whether its own views converge is the question.
    const wanted = entity.id === selected ? drawn : drawn.slice(-1);
    for (const ray of wanted) rays.push(Object.assign({}, ray, {id: entity.id}));
  }
  const placed = entities.filter((one) => one.placement
      && (!(world.summary || {}).map_session
          || one.placement_map_session === world.summary.map_session));
  if (!view || !view.pose || !state.map.gen || (!rays.length && !placed.length)) {
    wrap.hidden = true;
    note.textContent = !state.map.gen ? "no map yet"
        : !rays.length ? "nothing observed from a known pose"
        : "the map did not say where it was drawn from";
    return;
  }
  wrap.hidden = false;
  note.textContent = `${rays.length} ray${rays.length === 1 ? "" : "s"}, `
      + (placed.length ? `${placed.length} placed` : "none placed");
  $("wMapImg").src = `/map.png?gen=${state.map.gen}`;
  const size = state.map.width || 1;
  svg.setAttribute("viewBox", `0 0 ${size} ${size}`);
  svg.replaceChildren();

  for (const ray of rays) {
    const dim = selected && ray.id !== selected;
    const hue = wHue(ray.id);
    const half = (ray.span_deg || 12) / 2;
    const at = (deg) => {
      const t = deg * Math.PI / 180;
      return wPointToPx(ray.x_m + ray.length_m * Math.cos(t),
                        ray.y_m + ray.length_m * Math.sin(t), view);
    };
    const [x0, y0] = wPointToPx(ray.x_m, ray.y_m, view);
    const [xa, ya] = at(ray.bearing_deg - half);
    const [xb, yb] = at(ray.bearing_deg + half);
    const [xt, yt] = at(ray.bearing_deg);

    const wedge = document.createElementNS("http://www.w3.org/2000/svg", "path");
    wedge.setAttribute("d", `M ${x0} ${y0} L ${xa} ${ya} L ${xb} ${yb} Z`);
    wedge.setAttribute("fill", `hsl(${hue} 70% 50%)`);
    wedge.setAttribute("fill-opacity", dim ? "0.06" : "0.16");
    svg.append(wedge);

    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("x1", x0); line.setAttribute("y1", y0);
    line.setAttribute("x2", xt); line.setAttribute("y2", yt);
    line.setAttribute("stroke", `hsl(${hue} 70% 40%)`);
    line.setAttribute("stroke-width", dim ? 1 : 2);
    line.setAttribute("stroke-opacity", dim ? "0.35" : "1");
    svg.append(line);

    if (!dim) {
      const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
      text.setAttribute("x", xt); text.setAttribute("y", yt);
      text.setAttribute("font-size", Math.max(9, size / 40));
      text.setAttribute("fill", `hsl(${hue} 70% 30%)`);
      text.setAttribute("stroke", "rgba(255,255,255,.7)");
      text.setAttribute("stroke-width", "0.6");
      text.setAttribute("paint-order", "stroke");
      text.textContent = ray.id;
      svg.append(text);
    }
  }

  // A placed thing, drawn as big as the fix that placed it is uncertain. The
  // radius is the honest part: two rays crossing at a shallow angle put a thing
  // somewhere along a long smear, and a dot would say the rover knows better
  // than it does.
  const metresToPx = (metres) => {
    const [ax, ay] = wPointToPx(0, 0, view);
    const [bx, by] = wPointToPx(metres, 0, view);
    return Math.hypot(bx - ax, by - ay);
  };
  for (const entity of placed) {
    const dim = selected && entity.id !== selected;
    const hue = wHue(entity.id);
    const [x, y] = wPointToPx(entity.placement.x_m, entity.placement.y_m, view);
    const radius = Math.max(2, metresToPx(entity.placement_uncertainty_m || 0.2));

    const halo = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    halo.setAttribute("cx", x); halo.setAttribute("cy", y);
    halo.setAttribute("r", radius);
    halo.setAttribute("fill", `hsl(${hue} 70% 50%)`);
    halo.setAttribute("fill-opacity", dim ? "0.12" : "0.3");
    halo.setAttribute("stroke", `hsl(${hue} 70% 30%)`);
    halo.setAttribute("stroke-width", dim ? 0.7 : 1.4);
    halo.setAttribute("stroke-opacity", dim ? "0.4" : "1");
    svg.append(halo);

    const dot = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    dot.setAttribute("cx", x); dot.setAttribute("cy", y);
    dot.setAttribute("r", Math.max(1, size / 220));
    dot.setAttribute("fill", `hsl(${hue} 70% 25%)`);
    dot.setAttribute("fill-opacity", dim ? "0.4" : "1");
    svg.append(dot);
  }
}

// The stored frame with the measured box drawn on it. This is the check the
// whole popup exists for, and since nothing names a region any more it is the
// only thing that can settle whether four observations are really one object.
function wShot(observation) {
  if (!observation.frame_id) {
    const none = document.createElement("div");
    none.className = "wmeta";
    none.textContent = "no frame stored";
    return none;
  }
  const shot = document.createElement("div");
  shot.className = "wshot";
  const img = document.createElement("img");
  img.src = `/world_frame.jpg?id=${encodeURIComponent(observation.frame_id)}`;
  // Every observation gets its picture, and the browser decides which of them to
  // ask for: the console fetches a frame off the rover the first time it is
  // wanted, so a stream of several hundred rows costs the ones on screen rather
  // than all of them. Lazily rather than eagerly for that reason alone.
  img.loading = "lazy";
  img.alt = "the picture this observation was read from";
  // A frame the rover no longer has: the row outlives the file, and the popup
  // that exists to show what went wrong must survive that.
  img.onerror = () => {
    img.hidden = true;
    shot.textContent = `frame ${observation.frame_id} is gone`;
    shot.className = "wmeta mono";
  };
  shot.append(img);
  const box = observation.bbox;
  if (Array.isArray(box) && box.length === 4) {
    const rect = document.createElement("div");
    rect.className = "wbox";
    rect.style.left = `${box[0] * 100}%`;
    rect.style.top = `${box[1] * 100}%`;
    rect.style.width = `${(box[2] - box[0]) * 100}%`;
    rect.style.height = `${(box[3] - box[1]) * 100}%`;
    shot.append(rect);
  }
  return shot;
}

function wObservation(observation, options) {
  const block = document.createElement("div");
  block.className = "wobs" + (observation.entity_id ? "" : " wfailed");
  if (observation.entity_id) {
    block.style.borderLeftColor = `hsl(${wHue(observation.entity_id)} 70% 45%)`;
  }
  const head = document.createElement("div");
  head.innerHTML = "";
  const when = document.createElement("span");
  when.className = "mono";
  when.textContent = wTime(observation.observed_at);
  head.append(when);
  if (observation.label) {
    // Only rows the language model wrote carry a name, and there is no language
    // model any more -- so this fires on history alone, in a database that
    // survives deploys. Shown where it exists rather than dropped, because an
    // old row saying what the model said is the record of that experiment.
    const what = document.createElement("strong");
    what.textContent = ` ${observation.label}`;
    head.append(what);
  }
  if (options && options.showEntity) {
    const owner = document.createElement("span");
    owner.className = "mono";
    owner.textContent = observation.entity_id
        ? `  ${observation.entity_id}` : "  no entity";
    if (!observation.entity_id) owner.classList.add("wdup");
    head.append(owner);
  }
  block.append(head);

  if (observation.description) {
    const described = document.createElement("div");
    described.textContent = observation.description;
    block.append(described);
  }
  if (observation.note) {
    // The same field says two opposite things and they must not look alike: for
    // an observation with no entity it is why none was made, which is a warning;
    // for one with an entity it is the resolver's own sentence about why it
    // belongs there, which is the answer to "why did it think that was the same
    // chair" and is the reason this popup exists.
    const note = document.createElement("div");
    note.className = observation.entity_id ? "wbecause" : "wdup";
    note.textContent = observation.entity_id
        ? `why: ${observation.note}` : observation.note;
    block.append(note);
  }

  const pose = observation.pose;
  const meta = document.createElement("div");
  meta.className = "wmeta mono";
  const bits = [`source ${observation.source || "?"}`];
  if (observation.location_hint) bits.push(`hint ${observation.location_hint}`);
  bits.push(`pan ${observation.observer_pan_deg ?? "-"}°`,
            `tilt ${observation.observer_tilt_deg ?? "-"}°`);
  bits.push(pose ? `at (${pose.x_m}, ${pose.y_m}) m facing ${pose.heading_deg}°`
                 : "no rover pose recorded");
  bits.push(`map ${observation.map_session ?? "?"}`);
  if (observation.bbox) {
    bits.push(`box ${observation.bbox.map((n) => (+n).toFixed(2)).join(", ")}`);
  } else {
    bits.push("no usable box");
  }
  bits.push(observation.prompt_version
      ? `${observation.model_id || "?"} / prompt ${observation.prompt_version}`
      : `${observation.model_id || "?"}`);
  meta.textContent = bits.join(" · ");
  block.append(meta);

  block.append(wShot(observation));

  const raw = document.createElement("details");
  const summary = document.createElement("summary");
  summary.className = "wmeta";
  // Two different things behind one field. A language model's inspection is
  // words it chose; a look through the encoders is numbers that were measured,
  // and calling those "what the model said" invites a reader to weigh them as
  // an opinion.
  summary.textContent = observation.prompt_version
      ? "what the model actually said" : "what was measured";
  const body = document.createElement("pre");
  body.className = "wraw mono";
  body.textContent = JSON.stringify(observation.raw, null, 2);
  raw.append(summary, body);
  block.append(raw);
  return block;
}

function drawWorldDetail() {
  const pane = $("wDetail");
  pane.replaceChildren();
  const entity = world.selected;
  if (!entity || !entity.id || entity.id !== state.world.selected) {
    const hint = document.createElement("p");
    hint.className = "hint";
    hint.textContent = "nothing selected";
    pane.append(hint);
    return;
  }
  const head = document.createElement("div");
  const title = document.createElement("h2");
  title.textContent = entity.id;
  head.append(title);
  const meta = document.createElement("div");
  meta.className = "wmeta mono";
  meta.textContent = `kind ${entity.kind} · ${entity.observation_count} `
                   + `observations · created ${wTime(entity.created_at)} · `
                   + `last seen ${wTime(entity.last_seen_at)}`;
  head.append(meta);
  head.append(wPlace(entity));
  if (entity.placement && entity.placement_map_session
      && (world.summary || {}).map_session
      && entity.placement_map_session !== world.summary.map_session) {
    // The coordinates came from a SLAM map that no longer exists, so they point
    // at nowhere in the map now on screen. Said here rather than drawn wrongly.
    const stale = document.createElement("div");
    stale.className = "wmeta wold";
    stale.textContent = `this position was measured under map `
        + `${entity.placement_map_session} and the rover is now on map `
        + `${world.summary.map_session}, so it is not where the map shows`;
    head.append(stale);
  }
  pane.append(head);

  const observations = world.selected_observations || [];
  // Every look that was decided to be this thing, newest first, in a scroller of
  // its own so that the heading above stays put and the entity list beside it
  // does not scroll away. **The pictures are what a person is here to read**:
  // nothing names a region any more, so whether these four crops really are one
  // object is a question only the boxes can answer.
  const scroller = document.createElement("div");
  scroller.className = "wscroll";
  if (!observations.length) {
    const empty = document.createElement("p");
    empty.className = "hint";
    empty.textContent = "no observations";
    scroller.append(empty);
  }
  for (const observation of observations) {
    scroller.append(wObservation(observation));
  }
  pane.append(scroller);
}

function drawWorldObservations() {
  const pane = $("wObsAll");
  pane.replaceChildren();
  const recent = world.recent || [];
  const unmatched = world.unmatched || [];
  const heading = document.createElement("p");
  heading.className = "hint";
  heading.textContent = `${recent.length} shown, newest first`
                      + (unmatched.length
                         ? `, ${unmatched.length} with no entity` : "");
  pane.append(heading);
  if (!recent.length) {
    const empty = document.createElement("p");
    empty.className = "hint";
    empty.textContent = "nothing yet.";
    pane.append(empty);
    return;
  }
  for (const observation of recent) {
    pane.append(wObservation(observation, {showEntity: true}));
  }
}

function drawWorldSearch() {
  const note = $("wSearchNote"), results = $("wSearchResults");
  results.replaceChildren();
  const answer = world.search;
  if (state.world.searching) {
    note.textContent = "asking...";
    return;
  }
  if (!answer) {
    note.textContent = "";
    return;
  }
  note.textContent = `${answer.considered} compared`
      + (answer.skipped ? `, ${answer.skipped} skipped -- other backend` : "");

  // The verdict before the list, because a ranked list always has a top and the
  // question is whether that top means anything.
  const verdict = document.createElement("div");
  verdict.className = "wverdict " + (answer.confident ? "wfound" : "wmissing");
  verdict.textContent = (answer.confident
      ? `found it. ` : `nothing here matches \u201c${answer.query}\u201d. `)
      + (answer.detail || "");
  results.append(verdict);

  const matches = answer.matches || [];
  if (!matches.length) {
    const empty = document.createElement("p");
    empty.className = "hint";
    empty.textContent = "nothing comparable stored";
    results.append(empty);
    return;
  }
  for (const match of matches) {
    const block = document.createElement("div");
    block.className = "wobs";
    if (match.entity_id) {
      block.style.borderLeftColor = `hsl(${wHue(match.entity_id)} 70% 45%)`;
    }
    const head = document.createElement("div");
    const score = document.createElement("span");
    score.className = "mono";
    score.textContent = (+match.score).toFixed(4);
    head.append(score);
    if (match.entity_id) {
      const owner = document.createElement("span");
      owner.className = "mono";
      owner.textContent = `  ${match.entity_id}`;
      head.append(owner);
    }
    block.append(head);

    const meta = document.createElement("div");
    meta.className = "wmeta mono";
    const bits = [`seen ${wTime(match.observed_at)}`];
    if (match.bearing_deg != null) bits.push(`bearing ${match.bearing_deg}\u00b0`);
    bits.push(`map ${match.map_session ?? "?"}`);
    meta.textContent = bits.join(" \u00b7 ");
    block.append(meta);

    // Where it is, which is the answer a person wanted when they typed the
    // phrase. A match on an observation of something never placed can only say
    // which way to look, and says that rather than nothing.
    const place = document.createElement("div");
    place.className = "wmeta";
    if (match.placement) {
      place.classList.add("wplaced");
      place.textContent = `at (${(+match.placement.x_m).toFixed(2)}, `
          + `${(+match.placement.y_m).toFixed(2)}) m`;
    } else {
      place.classList.add("wunplaced");
      place.textContent = match.bearing_deg == null
          ? "no position — no pose recorded"
          : `no position — bearing ${match.bearing_deg}\u00b0`;
    }
    block.append(place);
    // With the box, so the answer points at the thing rather than at the room it
    // was in. `wShot` draws one whenever the observation carries it.
    block.append(wShot({frame_id: match.frame_id, bbox: match.bbox}));
    results.append(block);
  }
}

function drawWorldDiagnostics() {
  const pane = $("wDiag");
  pane.replaceChildren();
  const summary = world.summary || {};
  const lines = document.createElement("p");
  lines.className = "mono";
  lines.textContent =
      `${summary.entities ?? 0} entities · ${summary.observations ?? 0} observations`
      + ` · ${summary.unmatched ?? 0} with no entity`
      + ` · ${summary.inspections ?? 0} inspections`
      + ` · map session ${summary.map_session ?? "?"}`
      + ` · last success ${wAgo(summary.last_ok_at)}`;
  pane.append(lines);
  const table = document.createElement("table");
  table.className = "wtable";
  const header = document.createElement("tr");
  for (const name of ["when", "status", "took", "offered", "stored", "not stored",
                      "why"]) {
    const cell = document.createElement("th");
    cell.textContent = name;
    header.append(cell);
  }
  table.append(header);
  for (const row of world.inferences || []) {
    const line = document.createElement("tr");
    const cells = [
      wTime(row.started_at),
      row.status,
      row.duration_s == null ? "-" : `${(+row.duration_s).toFixed(0)} s`,
      row.returned ?? "-",
      // Older rows, from when the model was still asked which thing it was
      // looking at, have no `stored` and say how many entities they made
      // instead. Both are shown as what they are rather than added together.
      row.stored ?? (row.created == null ? "-" : `${row.created} entities`),
      row.rejected ?? "-",
      row.detail || "",
    ];
    cells.forEach((value, index) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      if (index === 1 && row.status !== "ok") cell.className = "wdup";
      line.append(cell);
    });
    table.append(line);
  }
  pane.append(table);
}

function wireWorld() {
  $("world").onclick = () => post({do: "world", what: state.world.open ? "close" : "open"});
  $("worldClose").onclick = () => post({do: "world", what: "close"});
  $("worldRefresh").onclick = () => post({do: "world", what: "refresh"});
  $("worldInspect").onclick = () => post({do: "world", what: "inspect"});
  // A click on the dimmed page behind the popup closes it, which is what a popup
  // over a page is expected to do. A click inside must not, or every press of a
  // button in it would also shut it.
  $("worldBack").onclick = (event) => {
    if (event.target === $("worldBack")) post({do: "world", what: "close"});
  };
  for (const button of $("worldTabs").querySelectorAll("[data-wtab]")) {
    button.onclick = () => {
      worldTab = button.dataset.wtab;
      for (const other of $("worldTabs").querySelectorAll("[data-wtab]")) {
        other.classList.toggle("on", other === button);
      }
      $("wPaneEntities").hidden = worldTab !== "entities";
      $("wPaneObservations").hidden = worldTab !== "observations";
      $("wPaneSearch").hidden = worldTab !== "search";
      $("wPaneDiagnostics").hidden = worldTab !== "diagnostics";
      if (worldTab === "search") $("wSearchBox").focus();
    };
  }
  $("wSearchForm").onsubmit = (event) => {
    event.preventDefault();
    post({do: "world", what: "search", query: $("wSearchBox").value});
  };
}
