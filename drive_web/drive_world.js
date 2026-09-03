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
// Whether the pane was drawn mid-search last time round, so that the answer
// -- or the refusal, which brings no new body with it -- puts the pane back.
let worldAsking = false;
// Which observation the tiled stream is showing at full size, by the store's own
// row identifier rather than by the object -- the whole body is replaced every
// time the rover records something, so a held object would be showing a row that
// no longer exists. Beside it, the row as it was last drawn, which is what stops
// a redraw every two seconds from collapsing the details a person just opened.
let worldZoom = null, worldZoomDrawn = "";
// The observation stream, and it is the one thing in this popup that does not
// come out of the fetched body. The body carries the newest forty looks and is
// replaced whole every time the rover records; everything older than that is
// fetched a page at a time as the stream is scrolled, and has to be kept here
// because nothing sends it again. By the store's own row identifier, so that a
// look arriving in the body and the same look in a page are one tile.
let worldStream = new Map();
// The tiles those rows are drawn as, by the same identifier, so that a body
// arriving every second adds the new looks to the top of a grid of several
// hundred rather than rebuilding it.
let worldTiles = new Map();
// The newest row the stream has been given, which is what says whether the next
// body still joins onto what is held; whether the rover may have older ones
// still; whether a page is in flight; and what the last one had to say for
// itself. See `wObsTake` and `wObsFill`.
let worldStreamTop = null, worldStreamMore = true, worldStreamBusy = false;
let worldStreamNote = "";
// How near the end of the drawn stream counts as having scrolled to the bottom
// of it, in pixels. Enough that the next page is on its way before the tiles
// run out, since it is a call to the rover and not a local one.
const STREAM_REACH = 600;

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
//
// **The golden angle at the end is not decoration.** Identifiers here are
// `object:1`, `object:2`, `object:3` and so on, which differ in one character by
// one -- so the hash differed by one, and four things on the map came out four
// shades of the same blue. Since the map draws every thing at once and colour is
// the only thing separating them, that made it unreadable. 137 degrees apart is
// the most two successive numbers can be, and it keeps the fifth and sixth apart
// as well.
const wHue = (id) => {
  let hash = 0;
  for (const ch of id) hash = (hash * 31 + ch.charCodeAt(0)) % 360;
  return (hash * 137) % 360;
};

function drawWorld(w) {
  drawWorldBuilding(w);
  $("worldBack").hidden = !w.open;
  $("world").classList.toggle("on", w.open);
  // Shutting the popup puts the observations tab back to its stream. Otherwise
  // opening it again a day later lands on one frame at full size with no memory
  // of having asked for it.
  if (!w.open && worldZoom !== null) { worldZoom = null; drawWorldZoom(); }
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
  drawWorldAsking(w);
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
                          : "")
      // How many separate places agreed, which is the number that says whether
      // the position was ever tested. Ten looks from one doorway and two from
      // opposite sides of the room are not the same evidence, and the count of
      // observations beside this cannot tell them apart.
      + (place.viewpoints ? ` · agreed from ${place.viewpoints} place`
                            + (place.viewpoints === 1 ? "" : "s")
                          : "")
      + (place.refined_from ? `, fitted over ${place.refined_from} looks` : "");
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

// One look in the tiled stream: the frame with its box on it, and the two things
// that tell one tile from another -- when it was taken, and which thing the
// rover decided it was. Everything else a row used to carry is in the large view
// this opens, because at this size it would not be legible and forty copies of
// it were what made the stream unreadable in the first place.
function wTile(observation) {
  const tile = document.createElement("button");
  tile.type = "button";
  tile.onclick = () => { worldZoom = observation.id; drawWorldZoom(); };
  tile.append(wShot(observation));
  const caption = document.createElement("div");
  tile.append(caption);
  wTileFace(tile, observation);
  return tile;
}

// The part of a tile that can still change after it is drawn: a look with no
// entity gets one when the resolver next settles, and that is the change
// somebody watching this tab is waiting for. Written into the tile that is
// already on the page rather than by building a new one, because the picture
// inside it has been fetched and a replacement would fetch it again.
function wTileFace(tile, observation) {
  tile.className = "wtile" + (observation.entity_id ? "" : " wfailed");
  tile.style.borderLeftColor = observation.entity_id
      ? `hsl(${wHue(observation.entity_id)} 70% 45%)` : "";
  const caption = tile.lastChild;
  caption.className = "wmeta mono" + (observation.entity_id ? "" : " wdup");
  caption.textContent = `${wTime(observation.observed_at)} `
      + (observation.entity_id || "no entity");
}

// The whole stream, newest first: the body's window and every page fetched under
// it, in one order.
function wObsRows() {
  return [...worldStream.values()]
      .sort((a, b) => (b.observed_at - a.observed_at) || (b.id - a.id));
}

// The body's newest looks, folded into the stream the tab is showing.
//
// **The check is whether the two still join up.** The body carries the newest
// forty and arrives again every time the rover records; the pages below it were
// fetched once and are never sent again, so they are kept here. That is only
// sound while the newest row the browser had is still inside the new window: if
// it has fallen out of it, forty or more looks were recorded while nothing was
// drawing -- the popup was shut, or the browser was in the background -- and
// what the browser holds is separated from what has just arrived by a hole it
// cannot see. So the stream starts again from the body, which is also what
// happens when the store is cleared and the window comes back empty.
function wObsTake(recent) {
  if (worldStreamTop !== null
      && !recent.some((row) => row.id === worldStreamTop)) {
    worldStream.clear();
    worldStreamMore = true;
    worldStreamNote = "";
  }
  for (const row of recent) worldStream.set(row.id, row);
  worldStreamTop = recent.length ? recent[0].id : null;
}

function drawWorldObservations() {
  wObsTake(world.recent || []);
  const rows = wObsRows(), summary = world.summary || {};
  const total = summary.observations ?? rows.length;
  $("wObsCount").textContent = !rows.length ? "nothing yet."
      : `${rows.length} of ${total} shown, newest first`
        + (summary.unmatched ? `, ${summary.unmatched} with no entity` : "");
  // Tiles are kept and moved rather than rebuilt. The body arrives again every
  // time the rover records something, which while it is looking is about every
  // second, and rebuilding a grid of several hundred pictures that often threw
  // away both the place somebody had scrolled back to and every frame the
  // browser had already fetched.
  const grid = $("wObsTiles"), kept = new Map();
  let at = grid.firstChild;
  for (const row of rows) {
    let tile = worldTiles.get(row.id);
    if (tile) wTileFace(tile, row);
    else tile = wTile(row);
    kept.set(row.id, tile);
    if (tile === at) at = at.nextSibling;
    else grid.insertBefore(tile, at);
  }
  // Whatever is left below them belongs to a store that has since been cleared.
  while (at) { const next = at.nextSibling; at.remove(); at = next; }
  worldTiles = kept;
  wObsSay(worldStreamNote);
  drawWorldZoom();
  // A page that did not fill the pane leaves the bottom of the stream on screen,
  // and nothing else would ask for the next one.
  wObsFill();
}

// The line under the tiles: what the last page had to say for itself, or that
// one is on its way.
function wObsSay(line) {
  $("wObsNote").textContent = line;
  $("wObsNote").hidden = !line;
}

// The next page of the history, when the tiles have been scrolled near the end
// of what is drawn.
//
// **Asked for by where the stream ends rather than by how far down it we are**,
// so that looks recorded while somebody reads cannot make a page repeat rows
// already on the screen or step over others. Nothing is asked for while the
// stream already holds everything the rover says it has, which is the ordinary
// case on a store smaller than one window.
function wObsFill() {
  const pane = $("wPaneObservations");
  if (pane.hidden || worldStreamBusy || !worldStreamMore) return;
  const rows = wObsRows();
  if (!rows.length || rows.length >= ((world.summary || {}).observations ?? 0)) {
    return;
  }
  if (pane.scrollHeight - pane.scrollTop - pane.clientHeight > STREAM_REACH) return;
  const oldest = rows[rows.length - 1];
  worldStreamBusy = true;
  // Said at once and written straight onto the line rather than through a
  // redraw, because the redraw is what called this. At the bottom of a long
  // stream over the rover's wi-fi, a page is a second in which nothing moves,
  // and nothing moving is what reaching the end of the store looks like.
  wObsSay("fetching older looks...");
  fetch(`/world_observations.json?before_at=${oldest.observed_at}`
        + `&before_id=${oldest.id}`)
    .then((reply) => reply.ok ? reply.json() : null)
    .then((body) => {
      worldStreamBusy = false;
      if (!body || !body.ok) {
        // Said under the tiles rather than swallowed: a stream that stops
        // growing looks exactly like a stream that has reached the bottom.
        worldStreamNote = (body && body.error) || "the rover did not answer";
        drawWorldObservations();
        return;
      }
      for (const row of body.observations || []) worldStream.set(row.id, row);
      worldStreamMore = !!body.more;
      worldStreamNote = "";
      drawWorldObservations();
    })
    .catch(() => {
      worldStreamBusy = false;
      worldStreamNote = "the console did not answer";
      drawWorldObservations();
    });
}

// The clicked look at full size: the same row the stream used to draw, given the
// whole pane. The row it is showing may still be changing under it -- a look with
// no entity gets one when the resolver next settles, and that is exactly the
// change somebody watching this frame is waiting for -- so it is rebuilt when the
// row really differs and left alone when it does not, which is what keeps an
// opened `what was measured` open through a rover that is still recording.
function drawWorldZoom() {
  const layer = $("wZoom"), body = $("wZoomBody");
  const shown = worldZoom === null ? null : worldStream.get(worldZoom);
  if (!shown) {
    // The row has gone: the store was cleared, or the stream started again
    // because what the browser held no longer joined onto the body. Back to the
    // tiles rather than a frozen picture of a look the rover no longer has.
    worldZoom = null;
    worldZoomDrawn = "";
    layer.hidden = true;
    body.replaceChildren();
    return;
  }
  layer.hidden = false;
  const drawn = JSON.stringify(shown);
  if (drawn === worldZoomDrawn) return;
  worldZoomDrawn = drawn;
  body.replaceChildren(wObservation(shown, {showEntity: true}));
}

// A phrase on its way to the rover, drawn from the pushed state rather than from
// the fetched body -- because the body arrives *with* the answer, and the answer
// is several seconds away. Drawn only from the body, the pane sat on the last
// phrase's count for the whole of that wait, so pressing enter changed nothing on
// screen and the count that was there read as the answer to what had just been
// typed. The seconds are the rover's own, so a second browser that joins during a
// search sees how long it has really been running.
function drawWorldAsking(w) {
  const asking = !!w.searching;
  $("wSearchBox").classList.toggle("wasking", asking);
  if (asking) {
    $("wSearchNote").textContent = `asking... ${w.searched_s || 0} s`;
    // The count and the matches below it answer the phrase before this one.
    if (!worldAsking) $("wSearchResults").replaceChildren();
  } else if (worldAsking) {
    // Whatever the rover left: the answer, or nothing at all if it refused --
    // a refusal brings no new body, so no redraw would otherwise happen.
    drawWorldSearch();
  }
  worldAsking = asking;
}

function drawWorldSearch() {
  const note = $("wSearchNote"), results = $("wSearchResults");
  if (state.world.searching) return;   // `drawWorldAsking` has the pane
  results.replaceChildren();
  const answer = world.search;
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
  $("worldInspect").onclick = () => post({do: "world", what: "inspect"});
  // A click on the dimmed page behind the popup closes it, which is what a popup
  // over a page is expected to do. A click inside must not, or every press of a
  // button in it would also shut it.
  $("worldBack").onclick = (event) => {
    if (event.target === $("worldBack")) post({do: "world", what: "close"});
  };
  // Space and Escape are the rover's stop, so the large view is closed by the
  // button or by the room beside the picture -- the same two ways the popup
  // itself closes, and for the same reason.
  $("wZoomClose").onclick = () => { worldZoom = null; drawWorldZoom(); };
  // Scrolling to the bottom of the tiles is what asks the rover for the looks
  // below them, so the stream ends where the store does rather than at the
  // window the body carries.
  $("wPaneObservations").addEventListener("scroll", wObsFill, {passive: true});
  $("wZoom").onclick = (event) => {
    if (event.target === $("wZoom")) { worldZoom = null; drawWorldZoom(); }
  };
  for (const button of $("worldTabs").querySelectorAll("[data-wtab]")) {
    button.onclick = () => {
      worldTab = button.dataset.wtab;
      // The layer covers the whole body, so it would sit over whichever tab was
      // moved to next.
      worldZoom = null;
      drawWorldZoom();
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
    const query = $("wSearchBox").value;
    post({do: "world", what: "search", query: query});
    // Enter is answered here rather than a tenth of a second later, when the
    // state carrying the rover's own flag arrives and takes this over. If the
    // ask never got out, the next state puts the old answer back within a second.
    if (query.trim()) {
      $("wSearchResults").replaceChildren();
      $("wSearchNote").textContent = "asking... 0 s";
      $("wSearchBox").classList.add("wasking");
      worldAsking = true;
    }
  };
}
