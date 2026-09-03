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
  // Fetched rather than pushed, like the network list: tens of kilobytes against
  // a state that goes out ten times a second. While the popup is open the rover
  // is asked for the world every couple of seconds, so this tag moves on its own
  // as the rover records -- and it moves only when the body really differs, which
  // is what keeps that from being 74 kB of wi-fi every two seconds.
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
  // The error is a cigar and not a disc -- long down the line the crossing was
  // taken along, short across it -- so both axes are here. One number was the
  // long one, which reads as the rover being that unsure in every direction.
  const major = place.error_major_m == null
      ? entity.placement_uncertainty_m : place.error_major_m;
  const minor = place.error_minor_m;
  line.classList.add("wplaced");
  line.textContent = `at (${(+place.x_m).toFixed(2)}, ${(+place.y_m).toFixed(2)}) m`
      + (major == null ? ""
         : minor == null ? ` to within ${(+major).toFixed(2)} m`
         : ` to within ${(+minor).toFixed(2)} m across `
           + `and ${(+major).toFixed(2)} m along the sight line`)
      + (place.extent_m ? ` · ${(+place.extent_m).toFixed(2)} m wide` : "")
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

// Everything below draws in *world* metres and maps each point through
// wPointToPx, rather than working in pixels and rotating. The map can be drawn
// rover-up, which turns every world angle into a different screen angle, and a
// shape built from its own trigonometry would be right only while that switch
// was off. An ellipse becomes a ring of points for the same reason.
const wSvg = (name, attrs) => {
  const node = document.createElementNS("http://www.w3.org/2000/svg", name);
  for (const key in attrs) node.setAttribute(key, attrs[key]);
  return node;
};

// A closed ring in map metres -- a circle when the two radii are equal, and the
// error ellipse a fix actually measured when they are not.
function wRing(x, y, majorM, minorM, tiltDeg, view, steps) {
  const t = (tiltDeg || 0) * Math.PI / 180;
  const ac = Math.cos(t), as = Math.sin(t);
  const points = [];
  for (let i = 0; i < (steps || 40); i++) {
    const a = 2 * Math.PI * i / (steps || 40);
    const ex = majorM * Math.cos(a), ey = minorM * Math.sin(a);
    points.push(wPointToPx(x + ex * ac - ey * as, y + ex * as + ey * ac, view));
  }
  return "M " + points.map(([px, py]) => `${px} ${py}`).join(" L ") + " Z";
}

// Which way a world bearing points once it is on the screen. Read off the map's
// own transform rather than assumed, so it stays right when the map is rover-up.
function wScreenDeg(x, y, bearingDeg, view) {
  const t = bearingDeg * Math.PI / 180;
  const [ax, ay] = wPointToPx(x, y, view);
  const [bx, by] = wPointToPx(x + Math.cos(t), y + Math.sin(t), view);
  return Math.atan2(by - ay, bx - ax) * 180 / Math.PI;
}

// Where the rover stood and which way it was facing, as a small arrowhead. The
// rover's own heading, not the camera's: the gimbal is drawn separately, because
// "standing here, facing there, looking over its shoulder" is three facts and one
// arrow can only carry two of them.
function wObserverMark(svg, ray, view, hue, size, dim) {
  const [px, py] = wPointToPx(ray.x_m, ray.y_m, view);
  const screen = wScreenDeg(ray.x_m, ray.y_m, ray.heading_deg || 0, view)
                 * Math.PI / 180;
  const corner = (deg, r) => {
    const a = screen + deg * Math.PI / 180;
    return `${px + r * Math.cos(a)} ${py + r * Math.sin(a)}`;
  };
  svg.append(wSvg("path", {
    d: `M ${corner(0, size)} L ${corner(140, size * 0.8)} `
       + `L ${corner(220, size * 0.8)} Z`,
    fill: `hsl(${hue} 70% 35%)`,
    "fill-opacity": dim ? "0.35" : "0.95",
    stroke: "rgba(255,255,255,.8)",
    "stroke-width": "0.5",
  }));
}

function drawWorldMap() {
  const wrap = $("wMapWrap"), note = $("wMapNote"), svg = $("wRays");
  const view = state.map.view;
  const entities = world.entities || [];
  const selected = state.world.selected;
  // The chosen entity's own reply carries more of its looks than the list does,
  // and it is the one being examined, so it wins where both have the thing.
  const chosen = world.selected && world.selected.id === selected
      ? world.selected : null;
  const shown = [];
  for (const entity of entities) {
    const mine = entity.id === selected;
    const rays = (mine && chosen && (world.selected_rays || []).length)
        ? world.selected_rays : (entity.rays || []);
    // One sighting each while nothing is chosen, so the picture stays readable;
    // all of a chosen thing's, because whether its own looks converge on the
    // position it settled on is the question the map is here to answer.
    shown.push({entity: entity, mine: mine,
                rays: mine ? rays : rays.slice(-1)});
  }
  const sightings = shown.reduce((n, one) => n + one.rays.length, 0);
  const placed = entities.filter((one) => one.placement
      && (!(world.summary || {}).map_session
          || one.placement_map_session === world.summary.map_session));
  if (!view || !view.pose || !state.map.gen || (!sightings && !placed.length)) {
    wrap.hidden = true;
    note.textContent = !state.map.gen ? "no map yet"
        : !sightings ? "nothing observed from a known pose"
        : "the map did not say where it was drawn from";
    return;
  }
  wrap.hidden = false;
  $("wMapImg").src = `/map.png?gen=${state.map.gen}`;
  const size = state.map.width || 1;
  svg.setAttribute("viewBox", `0 0 ${size} ${size}`);
  svg.replaceChildren();

  const metresToPx = (metres) => {
    const [ax, ay] = wPointToPx(0, 0, view);
    const [bx, by] = wPointToPx(metres, 0, view);
    return Math.hypot(bx - ax, by - ay);
  };
  const placeOf = (entity) => (placed.includes(entity) ? entity.placement : null);
  let agreeing = 0, disagreeing = 0;

  // --- what each look says, and how it stands to where the thing was settled --
  for (const {entity, mine, rays} of shown) {
    const dim = selected && !mine;
    const hue = wHue(entity.id);
    const place = placeOf(entity);
    for (const ray of rays) {
      const relation = ray.relation;
      const agrees = relation ? relation.agrees : null;
      if (agrees === true) agreeing++;
      if (agrees === false) disagreeing++;
      const [x0, y0] = wPointToPx(ray.x_m, ray.y_m, view);

      // The cone the box actually subtends, drawn only as far as the thing is:
      // a wedge running past the settled position claims the rover measured a
      // direction further out than it was looking at anything.
      const reach = place && relation ? Math.max(0.3, relation.range_m)
                                      : ray.length_m;
      const half = (ray.span_deg || 12) / 2;
      const at = (deg, len) => {
        const t = deg * Math.PI / 180;
        return wPointToPx(ray.x_m + len * Math.cos(t),
                          ray.y_m + len * Math.sin(t), view);
      };
      if (!dim) {
        const [xa, ya] = at(ray.bearing_deg - half, reach);
        const [xb, yb] = at(ray.bearing_deg + half, reach);
        svg.append(wSvg("path", {
          d: `M ${x0} ${y0} L ${xa} ${ya} L ${xb} ${yb} Z`,
          fill: `hsl(${hue} 70% 50%)`, "fill-opacity": "0.13",
        }));
      }

      // **The sight line ends at the thing, and that is the change.** It used to
      // be a stub of a fixed 2.5 m, so a look and the position it supports were
      // two unconnected marks on the map and no arrangement of them read as
      // wrong. Drawn to the settled point, a look that disagrees is a fork --
      // the measured bearing going one way, the thing sitting off it -- and how
      // far the two part is the miss in metres the row beside it reports.
      if (place) {
        const [xp, yp] = wPointToPx(place.x_m, place.y_m, view);
        svg.append(wSvg("line", {
          x1: x0, y1: y0, x2: xp, y2: yp,
          stroke: `hsl(${hue} 70% 40%)`,
          "stroke-width": dim ? 0.8 : (agrees ? 1.8 : 1.2),
          "stroke-opacity": dim ? "0.3" : (agrees ? "0.95" : "0.5"),
          "stroke-dasharray": agrees ? "" : `${size / 90} ${size / 90}`,
        }));
      }
      // And the bearing this look actually measured, always: where it agrees it
      // lies under the sight line and adds nothing, and where it does not it is
      // the other half of the fork.
      const [xt, yt] = at(ray.bearing_deg, reach);
      svg.append(wSvg("line", {
        x1: x0, y1: y0, x2: xt, y2: yt,
        stroke: `hsl(${hue} 70% 30%)`,
        "stroke-width": dim ? 0.8 : (agrees === false ? 2 : 1.4),
        "stroke-opacity": dim ? "0.3" : "0.9",
      }));
      // Where the gimbal was pointing is inside the bearing already; what this
      // adds is the rover's own facing, so a standstill that swept the camera
      // and a drive that turned the whole rover are different pictures.
      if (ray.heading_deg !== undefined && ray.heading_deg !== null) {
        wObserverMark(svg, ray, view, hue, Math.max(2.5, size / 90), dim);
      }
    }
  }

  // --- the one position the application has settled on -----------------------
  //
  // Drawn last so it sits on top of every line that argues about it, and drawn
  // as the shape the fix measured rather than as a disc. A crossing taken at a
  // shallow angle is uncertain a long way down its own line of sight and precise
  // across it, and `locate.fix` records that as a major axis, a minor and a
  // direction; a circle of the major radius says the rover is equally unsure in
  // every direction, which is both wrong and flattering in the one direction
  // that matters.
  for (const entity of placed) {
    const dim = selected && entity.id !== selected;
    const hue = wHue(entity.id);
    const place = entity.placement;
    const [x, y] = wPointToPx(place.x_m, place.y_m, view);
    const major = Math.max(0.02, +place.error_major_m
                                 || +entity.placement_uncertainty_m || 0.2);
    const minor = Math.max(0.02, +place.error_minor_m || major);

    // How wide the thing itself is, measured from the crops that placed it. The
    // ellipse is where its centre might be; this is the silhouette a later
    // bearing has to land inside to be counted as pointing at it, so the two
    // are different questions and are drawn as different rings.
    if (place.extent_m) {
      svg.append(wSvg("path", {
        d: wRing(place.x_m, place.y_m, +place.extent_m, +place.extent_m, 0, view),
        fill: "none", stroke: `hsl(${hue} 70% 45%)`,
        "stroke-width": dim ? 0.5 : 0.9,
        "stroke-opacity": dim ? "0.25" : "0.55",
        "stroke-dasharray": `${size / 120} ${size / 120}`,
      }));
    }
    svg.append(wSvg("path", {
      d: wRing(place.x_m, place.y_m, major, minor, place.error_major_deg, view),
      fill: `hsl(${hue} 70% 50%)`, "fill-opacity": dim ? "0.12" : "0.32",
      stroke: `hsl(${hue} 70% 28%)`, "stroke-width": dim ? 0.7 : 1.4,
      "stroke-opacity": dim ? "0.4" : "1",
    }));
    const r = Math.max(1.5, size / 200);
    svg.append(wSvg("circle", {
      cx: x, cy: y, r: r, fill: `hsl(${hue} 70% 20%)`,
      stroke: "rgba(255,255,255,.85)", "stroke-width": "0.7",
      "fill-opacity": dim ? "0.45" : "1",
    }));
    if (!dim) {
      const label = wSvg("text", {
        x: x + r * 1.6, y: y - r * 1.2,
        "font-size": Math.max(9, size / 40),
        fill: `hsl(${hue} 70% 25%)`, stroke: "rgba(255,255,255,.75)",
        "stroke-width": "0.6", "paint-order": "stroke",
      });
      label.textContent = entity.id;
      svg.append(label);
    }
  }
  // An unplaced thing has nowhere to put a label, so its newest sighting carries
  // one instead -- otherwise the only entities named on the map are the ones
  // that already worked, which is the wrong half to show.
  for (const {entity, mine, rays} of shown) {
    if (placeOf(entity) || (selected && !mine) || !rays.length) continue;
    const ray = rays[rays.length - 1];
    const t = ray.bearing_deg * Math.PI / 180;
    const [tx, ty] = wPointToPx(ray.x_m + ray.length_m * Math.cos(t),
                                ray.y_m + ray.length_m * Math.sin(t), view);
    const label = wSvg("text", {
      x: tx, y: ty, "font-size": Math.max(9, size / 40),
      fill: `hsl(${wHue(entity.id)} 70% 30%)`, stroke: "rgba(255,255,255,.7)",
      "stroke-width": "0.6", "paint-order": "stroke",
    });
    label.textContent = entity.id;
    svg.append(label);
  }

  const bits = [`${placed.length} placed`,
                `${sightings} sighting${sightings === 1 ? "" : "s"}`];
  if (agreeing || disagreeing) {
    bits.push(`${agreeing} on it, ${disagreeing} off it`);
  }
  note.textContent = bits.join(" · ");
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
  const off = options && options.relations
      && options.relations[observation.id]
      && !options.relations[observation.id].agrees;
  block.className = "wobs" + (observation.entity_id ? "" : " wfailed")
                  + (off ? " woff" : "");
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
  // Where this look stands to the one position the thing has settled on. It is
  // the same test `resolve` applies when it attaches a look, so a row reading
  // "off it" is a row that would not be attached today.
  const relation = options && options.relations
      ? options.relations[observation.id] : null;
  if (relation) {
    bits.push(`${relation.range_m} m away`,
              `bearing ${relation.off_deg > 0 ? "+" : ""}${relation.off_deg}° `
              + `of it, missing by ${relation.miss_m} m of the `
              + `${relation.tolerance_m} m allowed`,
              relation.agrees ? "on it" : "off it");
  }
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
  // The rays already carry how each look stands to the settled position, worked
  // out on the rover by the resolver's own arithmetic. Joined to the rows by
  // observation rather than recomputed here, for the reason wPointToPx is not
  // reimplemented either: a second copy of the geometry is a copy that can
  // disagree with the one the rover acts on.
  const relations = {};
  for (const ray of world.selected_rays || []) {
    if (ray.id != null && ray.relation) relations[ray.id] = ray.relation;
  }
  for (const observation of observations) {
    scroller.append(wObservation(observation, {relations: relations}));
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
