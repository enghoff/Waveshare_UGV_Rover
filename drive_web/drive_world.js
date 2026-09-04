// The drive console's world-state popup: what the rover has seen, drawn.
//
// The browser half of world_state/, and the counterpart to drive_world.py on
// the server side. Read-only apart from the three things that act on the rover:
// one inspection, the map's clear, and going to look at a thing.
//
// Loaded BEFORE drive_web.js, which is what calls `start()`. Nothing here runs
// at load: it is all declarations, and everything it reaches for -- `$`, `post`,
// `state` -- is looked up when a draw actually happens, by which time
// drive_web.js has run.

// --- the world-state popup --------------------------------------------------
//
// Read-only, apart from the three things that act on the rover. The point of it is
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
// sofa. What replaced it is the box above the views, where the phrase a person
// types is compared with what the rover actually saw, and the pictures in the
// pane beside the entity list.

let worldGen = "", world = {}, worldTab = "entities";
// Which thing the rover was last said to be on its way to, so that the list is
// redrawn when that moves. It arrives in the pushed state rather than in the
// body the list is otherwise drawn from.
let worldGoing = "";
// Whether the line under the box was drawn mid-search last time round, so that
// the answer -- or the refusal, which brings no new body with it -- puts it
// back.
let worldAsking = false;
// What the box is narrowing the views by, or null while it is empty.
//
// **The search is a filter now and not a pane of its own.** A phrase used to be
// answered in a fourth tab, as a ranked list of crops beside three other views
// of the same store; what a person wanted from it was to find one thing in the
// list, on the map and in the stream, and that meant reading an answer in one
// place and then hunting for it in the others. So the answer narrows those three
// instead, and this holds it in the shape they need: the matching looks by the
// store's own row identifier, and the best score each entity managed.
//
// Everything below asks this rather than reading `world.search` for itself. A
// filter applied to the list and not to the map beside it would be two views of
// the store disagreeing on screen, which is the one thing this popup exists to
// make visible and must therefore never invent.
let worldFilter = null;
// Which observation the tiled stream is showing at full size, by the store's own
// row identifier rather than by the object -- the whole body is replaced every
// time the rover records something, so a held object would be showing a row that
// no longer exists. Beside it, the row as it was last drawn, which is what stops
// a redraw every two seconds from collapsing the details a person just opened.
let worldZoom = null, worldZoomDrawn = "";
// Which look the pointer is resting on in the list of the chosen thing's
// observations, by the store's own row identifier, so that the map can pick out
// the one line it drew for that row. Kept here rather than passed down because a
// redraw arrives every couple of seconds and has to put the highlight back.
let worldHover = null;
// The rows the entity list is drawn as, by the thing each one is for.
//
// **This popup is read while the rover is filling it, and until these were kept
// it could not be.** A body arrives every second or so while the rover is
// recording, and the list used to be built again from nothing each time. That
// destroys the element the browser had chosen to hold the view steady against,
// so its correction is made against nothing: measured on the rover's own store
// of sixty things, the list slid 52 px under the pointer per body and went on
// sliding, which after half a minute of recording is a different part of the
// list entirely. Keeping the row elements and moving them gives the browser
// something to anchor to, and the list stays where it was put.
//
// What is *inside* a row is written again on every draw, because one of the
// things a row says is how long ago the thing was last seen -- and a row held
// unchanged would be a row whose age had quietly stopped counting.
let worldRows = new Map();
// The chosen thing's own pane, which is the one place in this popup where a
// redraw costs more than a flicker: its looks carry the pictures, and reading
// down them with one opened is the whole point of choosing a thing. So the two
// boxes it is drawn in are kept for as long as the same thing is chosen -- the
// lower one being the scroller somebody is inside -- and the heading and each
// look are rebuilt only when what they say has really changed, the way the
// large view of a single look already is. Rebuilding a look throws away both
// the picture the browser had fetched and any raw block opened under it.
let worldDetailFor = "", worldDetailHead = "", worldDetailRows = new Map();
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
  // Which thing the rover is being sent to look at rides in the pushed state and
  // not in the body, so the list has to be told. Without this a press would show
  // nothing until the rover next recorded a look -- which on a rover that has
  // stopped looking is never, and on any rover is exactly the moment somebody is
  // watching for their button to do something.
  if (w.going !== worldGoing) {
    worldGoing = w.going;
    if (w.open && (world.entities || []).length) drawWorldList();
  }
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
      // How high it stands. Above the floor once somebody has measured how high
      // the camera is -- see locate.CAMERA_HEIGHT_M -- and above the camera
      // until then, said as such rather than left to be read as the other one.
      + (place.height_above_floor_m != null
         ? ` · ${(+place.height_above_floor_m).toFixed(2)} m up`
         : place.height_m != null
         ? ` · ${(+place.height_m).toFixed(2)} m above the camera`
         : "")
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

function drawWorldBody() {
  // First, because every view below is drawn through it.
  wReadFilter();
  drawWorldFilter();
  drawWorldList();
  drawWorldMap();
  drawWorldDetail();
  drawWorldObservations();
  drawWorldDiagnostics();
}

// The answer the rover last gave, folded into the two lookups the views need.
// Rebuilt from the body rather than kept across arrivals: the body carries the
// answer for as long as the box holds the phrase, so an emptied box or a cleared
// store takes the filter with it and nothing has to remember to undo it.
function wReadFilter() {
  const answer = world.search;
  if (!answer || !answer.query) { worldFilter = null; return; }
  const looks = new Map(), things = new Map();
  for (const match of answer.matches || []) {
    // The rover sends the whole of a matching look, so a match is an
    // observation row with a score on it and the tiles can draw it as one. The
    // fallback is for a rover that still sends only the ranking's own columns:
    // it filters correctly and the look opens thin, which is better than a
    // filter that quietly matches nothing.
    if (match.id == null) match.id = match.observation_id;
    // Ranked already, so the first score an entity is seen with is its best.
    if (match.id != null) looks.set(match.id, match);
    if (match.entity_id && !things.has(match.entity_id)) {
      things.set(match.entity_id, match.score);
    }
  }
  worldFilter = {answer: answer, looks: looks, things: things,
                 // The bar a match has to clear, measured on the rover and sent
                 // with the answer rather than written down here, so that the
                 // page cannot go on marking rows against a floor that has
                 // since been re-measured.
                 floor: answer.floor};
}

// The things the entity list and the map are showing: all of them, or the ones
// the filter matched with the best-scoring first.
function wShown() {
  const entities = world.entities || [];
  if (!worldFilter) return entities;
  const best = worldFilter.things;
  return entities.filter((one) => best.has(one.id))
      .sort((a, b) => best.get(b.id) - best.get(a.id));
}

// What a row or a tile scored, as the line the two of them share. Dimmed below
// the floor, because those rows are the nearest thing the rover has and are not
// matches -- a list of scores always has a top, and that top meaning nothing is
// the failure this whole feature is designed around.
function wScore(value) {
  const span = document.createElement("span");
  const under = worldFilter.floor != null && value < worldFilter.floor;
  span.className = "wscore mono" + (under ? " wunder" : "");
  span.textContent = (+value).toFixed(3);
  return span;
}

function drawWorldList() {
  const list = $("wList");
  const entities = wShown();
  const summary = world.summary || {};
  if (!entities.length) {
    worldRows = new Map();
    const empty = document.createElement("p");
    empty.className = "hint";
    // Under a filter the list is empty for a different reason, and saying
    // "nothing recorded yet" over a store of four hundred looks would be a lie.
    // The two cases a person has to tell apart are the phrase matching nothing
    // at all and it matching looks the rover has not yet made a thing of --
    // which is the ordinary state of anything seen once, and sends them to the
    // stream rather than to the inspect button.
    empty.textContent = !worldFilter
        ? (summary.observations
           ? `${summary.observations} observations, none placed`
           : summary.inspections
           ? `nothing recorded, after ${summary.inspections} inspections`
           : "nothing recorded yet")
        : worldFilter.looks.size
        ? `${worldFilter.looks.size} matching look`
          + `${worldFilter.looks.size === 1 ? "" : "s"}, none of them a thing yet`
        : "no match";
    list.replaceChildren(empty);
    return;
  }
  const newest = Math.max(...entities.map((e) => e.last_seen_at || 0));

  // Rows are kept and moved rather than built again, exactly as the tiles in the
  // observation stream are and for a sharper version of the same reason: see
  // `worldRows`. Each one is then written afresh, which is what keeps "last 40 s
  // ago" counting on a thing the rover has stopped looking at.
  const kept = new Map();
  let at = list.firstChild;
  for (const entity of entities) {
    let row = worldRows.get(entity.id);
    if (!row) {
      row = document.createElement("div");
      row.style.borderLeft = `4px solid hsl(${wHue(entity.id)} 70% 45%)`;
    }
    wRowFace(row, entity, newest, summary);
    kept.set(entity.id, row);
    if (row === at) at = at.nextSibling;
    else list.insertBefore(row, at);
  }
  // Whatever is left below them is a thing the store no longer has, or one a
  // filter has since narrowed away.
  while (at) { const next = at.nextSibling; at.remove(); at = next; }
  worldRows = kept;
}

// Everything a row in the entity list says, written into the row that is already
// on the page. Nothing in here survives from the draw before it, so the row can
// change hands between a filtered list and a whole one, or stop being the chosen
// thing, without any of that having to be undone.
function wRowFace(row, entity, newest, summary) {
  row.className = "wrow" + (entity.id === state.world.selected ? " on" : "");
  row.onclick = () => post({do: "world", what: "select",
                            id: entity.id === state.world.selected ? "" : entity.id});

  // An identifier and no name. Nothing measures what a thing is called any
  // more, so a row says which thing it is, how often it has been seen and
  // where it is; what it looks like is one click away in the pane beside it.
  const head = document.createElement("div");
  head.className = "whead";
  const id = document.createElement("span");
  id.className = "wid mono";
  id.textContent = entity.id;
  head.append(id);
  // Under a filter, what its best look scored against the phrase -- which is
  // why this row is one of the few left on screen, and the order they are in.
  if (worldFilter) head.append(wScore(worldFilter.things.get(entity.id)));
  // And the one control in this list. Offered only where there is somewhere to
  // go: a thing with no position has none, and a thing placed under a map that
  // has since been cleared has coordinates that are a place in this one only by
  // coincidence. Both of those already say so on the line below.
  if (entity.placement
      && (!summary.map_session
          || entity.placement_map_session === summary.map_session)) {
    head.append(wGoTo(entity));
  }

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
  // In one go, so the row is never briefly empty: the browser holds the list
  // still by watching what is in it, and a row that collapsed and grew back
  // would be a row it had to correct for.
  row.replaceChildren(head, meta, wPlace(entity));
}

// Send the rover to look at this thing.
//
// **Where it actually goes is the rover's answer and not this position.** The
// coordinates in the row are the middle of the thing, which is inside the
// furniture; the rover works out a patch of mapped floor it fits on, with
// nothing between it and the thing, and drives there facing it. All this button
// carries is which thing was asked for.
//
// A click on the map outranks whatever is running, and so does this, for the
// same reason: somebody choosing a destination is saying the rover is going to
// the wrong place. So it is not greyed while the wheels are busy -- only while
// this same thing is the one being driven to.
function wGoTo(entity) {
  const going = state.world.going === entity.id;
  const button = document.createElement("button");
  button.className = "wgo";
  button.textContent = going ? "going" : "go to";
  button.disabled = going || !state.link.connected || !state.link.can_drive;
  button.onclick = (event) => {
    // The row's own click is what chooses a thing to look at. This one must not
    // also do that, or going somewhere would open the pane beside it as well.
    event.stopPropagation();
    post({do: "world", what: "approach", id: entity.id});
  };
  return button;
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
function wObserverMark(into, ray, view, hue, size, pen) {
  const [px, py] = wPointToPx(ray.x_m, ray.y_m, view);
  const screen = wScreenDeg(ray.x_m, ray.y_m, ray.heading_deg || 0, view)
                 * Math.PI / 180;
  const corner = (deg, r) => {
    const a = screen + deg * Math.PI / 180;
    return `${px + r * Math.cos(a)} ${py + r * Math.sin(a)}`;
  };
  into.append(wSvg("path", {
    d: `M ${corner(0, size)} L ${corner(140, size * 0.8)} `
       + `L ${corner(220, size * 0.8)} Z`,
    fill: `hsl(${hue} 70% 35%)`,
    "fill-opacity": "0.95",
    stroke: "rgba(255,255,255,.8)",
    "stroke-width": pen(0.5),
  }));
}

// How far along its own bearing a look's line is drawn: to the thing where the
// thing has a position, and to the length the look itself claims where it does
// not. Asked here by both the drawing and the sizing of the view, so that the
// window cannot be worked out from a different reach than the one on screen.
function wReach(ray, place) {
  return place && ray.relation ? Math.max(0.3, ray.relation.range_m)
                               : ray.length_m;
}

// The part of the map picture the panel shows, as a square in its pixels. It has
// to hold every point given -- where each look was taken from, how far it runs,
// and the ring round the position that was settled on -- with a margin, and it
// is not allowed to close in past `floor`: a thing seen once from a metre away
// would otherwise be blown up to fill the panel at a magnification none of the
// measurements behind it support.
//
// **It is not clamped to the picture.** Half of what the rover has seen was seen
// from outside the six metres the map is drawn across, and a window slid back
// inside the edge would put the chosen thing somewhere off-centre with nothing
// saying why. Outside the picture is empty, and the line under the map says the
// view has left it.
function wWindow(points, floor, ceiling) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const [x, y] of points) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y);
    x1 = Math.max(x1, x); y1 = Math.max(y1, y);
  }
  if (!isFinite(x0)) return null;
  const side = Math.min(ceiling,
      Math.max(floor, (x1 - x0) * 1.2, (y1 - y0) * 1.2));
  return [(x0 + x1) / 2 - side / 2, (y0 + y1) / 2 - side / 2, side];
}

// Which look the map should pick out, or null for none of them. Every look was
// drawn into a group carrying the row it came from, so this is two classes and
// not a redraw -- which matters, because it runs on every pointer move across
// the list and the map is several hundred lines.
//
// A row whose look never reached the map -- no pose, so no bearing -- lights
// nothing and dims nothing, rather than fading the map to say "not this one".
function wHighlight(id) {
  worldHover = id;
  const svg = $("wRays");
  let found = false;
  for (const group of svg.querySelectorAll("g.wray")) {
    const hot = id != null && group.dataset.obs === String(id);
    group.classList.toggle("whot", hot);
    found = found || hot;
  }
  svg.classList.toggle("whover", found);
}

function drawWorldMap() {
  const wrap = $("wMapWrap"), note = $("wMapNote"), svg = $("wRays");
  const view = state.map.view;
  // Whatever the list beside it is showing, and for the same reason: a map still
  // covered in every thing the rover has seen, next to a list narrowed to one of
  // them, is two answers to one question.
  const entities = wShown();
  // A thing chosen before a filter narrowed it away is still in the pane beside
  // the list, but it is not on this map: nothing drawn here is the chosen one,
  // and a map of one missing thing would be a map of nothing.
  const selected = entities.some((one) => one.id === state.world.selected)
      ? state.world.selected : "";
  // **Once a thing is chosen it is the only thing drawn.** Ninety-three things'
  // bearings laid over a map six metres across is a green smear a centimetre
  // deep, and the one question this map answers -- whether a thing's own looks
  // agree about where it is -- cannot be read out of it at all. Every other
  // thing used to be drawn faintly behind the chosen one, which kept the smear
  // and made it grey. Nothing chosen still draws them all, and that is the
  // overview: where the things are, one look each.
  const drawing = selected
      ? entities.filter((one) => one.id === selected) : entities;
  // The chosen entity's own reply carries more of its looks than the list does,
  // and it is the one being examined, so it wins where both have the thing.
  const chosen = world.selected && world.selected.id === selected
      ? world.selected : null;
  const shown = [];
  for (const entity of drawing) {
    const rays = (selected && chosen && (world.selected_rays || []).length)
        ? world.selected_rays : (entity.rays || []);
    // One sighting each while nothing is chosen, so the picture stays readable;
    // all of a chosen thing's, because whether its own looks converge on the
    // position it settled on is the question the map is here to answer.
    shown.push({entity: entity, rays: selected ? rays : rays.slice(-1)});
  }
  const sightings = shown.reduce((n, one) => n + one.rays.length, 0);
  const placed = drawing.filter((one) => one.placement
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
  svg.replaceChildren();

  const metresToPx = (metres) => {
    const [ax, ay] = wPointToPx(0, 0, view);
    const [bx, by] = wPointToPx(metres, 0, view);
    return Math.hypot(bx - ax, by - ay);
  };
  const placeOf = (entity) => (placed.includes(entity) ? entity.placement : null);
  // A point a given distance along a given bearing from where a look was taken.
  const along = (ray, deg, len) => {
    const t = deg * Math.PI / 180;
    return wPointToPx(ray.x_m + len * Math.cos(t),
                      ray.y_m + len * Math.sin(t), view);
  };

  // --- the window, which is the whole map until something is chosen ----------
  //
  // A thing and its looks are a metre or two of a map drawn six metres across,
  // and at full extent the fork between a bearing and the position it argues
  // about is a few pixels wide. So the panel closes in on the chosen thing: the
  // same picture, a smaller piece of it.
  const marks = [];
  for (const {entity, rays} of shown) {
    const place = placeOf(entity);
    for (const ray of rays) {
      const reach = wReach(ray, place);
      const half = (ray.span_deg || 12) / 2;
      marks.push(wPointToPx(ray.x_m, ray.y_m, view),
                 along(ray, ray.bearing_deg, reach),
                 along(ray, ray.bearing_deg - half, reach),
                 along(ray, ray.bearing_deg + half, reach));
    }
    if (place) {
      const reach = Math.max(+place.error_major_m || 0,
                             +place.extent_m || 0, 0.25);
      for (const corner of [[-1, -1], [-1, 1], [1, -1], [1, 1]]) {
        marks.push(wPointToPx(place.x_m + corner[0] * reach,
                              place.y_m + corner[1] * reach, view));
      }
    }
  }
  const closeup = selected
      ? wWindow(marks, metresToPx(2.5), size * 3) : null;
  const [vx, vy, side] = closeup || [0, 0, size];
  const zoom = size / side;
  svg.setAttribute("viewBox", `${vx} ${vy} ${side} ${side}`);
  // The picture under the lines has to move with them. It is one PNG at a fixed
  // resolution, so this is that same window taken out of it, and its cells come
  // out as the squares they are rather than as a blur -- see `.wclose`.
  const picture = $("wMapImg");
  picture.style.transform = closeup
      ? `scale(${zoom}) translate(${-vx / size * 100}%, ${-vy / size * 100}%)`
      : "";
  picture.classList.toggle("wclose", zoom > 1.5);
  // Every width, radius and letter below is in the map's own pixels, so closing
  // in would thicken all of them by the same factor and a magnified view would
  // be drawn in crayon. This is what keeps a line the width it was on screen.
  const pen = (n) => n / zoom;
  // The same unit, where the stylesheet can reach it: what it draws heavier is
  // the one look the pointer is resting on.
  svg.style.setProperty("--wpen", pen(1));
  let agreeing = 0, disagreeing = 0;

  // --- what each look says, and how it stands to where the thing was settled --
  for (const {entity, rays} of shown) {
    const hue = wHue(entity.id);
    const place = placeOf(entity);
    for (const ray of rays) {
      const relation = ray.relation;
      const agrees = relation ? relation.agrees : null;
      if (agrees === true) agreeing++;
      if (agrees === false) disagreeing++;
      const [x0, y0] = wPointToPx(ray.x_m, ray.y_m, view);
      // Everything this one look draws, in a group carrying the row it was read
      // from. That is what lets the pointer resting on a row in the list beside
      // the map pick the look out here -- see `wHighlight` -- without the map
      // having to be drawn again for every pointer move.
      const drawn = wSvg("g", {class: "wray"});
      if (ray.id != null) drawn.setAttribute("data-obs", ray.id);
      svg.append(drawn);

      // The cone the box actually subtends, drawn only as far as the thing is:
      // a wedge running past the settled position claims the rover measured a
      // direction further out than it was looking at anything.
      const reach = wReach(ray, place);
      const half = (ray.span_deg || 12) / 2;
      const at = (deg, len) => along(ray, deg, len);
      {
        const [xa, ya] = at(ray.bearing_deg - half, reach);
        const [xb, yb] = at(ray.bearing_deg + half, reach);
        drawn.append(wSvg("path", {
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
        drawn.append(wSvg("line", {
          x1: x0, y1: y0, x2: xp, y2: yp,
          stroke: `hsl(${hue} 70% 40%)`,
          "stroke-width": pen(agrees ? 1.8 : 1.2),
          "stroke-opacity": agrees ? "0.95" : "0.5",
          "stroke-dasharray": agrees ? "" : `${pen(size / 90)} ${pen(size / 90)}`,
        }));
      }
      // And the bearing this look actually measured, always: where it agrees it
      // lies under the sight line and adds nothing, and where it does not it is
      // the other half of the fork.
      const [xt, yt] = at(ray.bearing_deg, reach);
      drawn.append(wSvg("line", {
        x1: x0, y1: y0, x2: xt, y2: yt,
        stroke: `hsl(${hue} 70% 30%)`,
        "stroke-width": pen(agrees === false ? 2 : 1.4),
        "stroke-opacity": "0.9",
      }));
      // Where the gimbal was pointing is inside the bearing already; what this
      // adds is the rover's own facing, so a standstill that swept the camera
      // and a drive that turned the whole rover are different pictures.
      if (ray.heading_deg !== undefined && ray.heading_deg !== null) {
        wObserverMark(drawn, ray, view, hue, pen(Math.max(2.5, size / 90)), pen);
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
        "stroke-width": pen(0.9), "stroke-opacity": "0.55",
        "stroke-dasharray": `${pen(size / 120)} ${pen(size / 120)}`,
      }));
    }
    svg.append(wSvg("path", {
      d: wRing(place.x_m, place.y_m, major, minor, place.error_major_deg, view),
      fill: `hsl(${hue} 70% 50%)`, "fill-opacity": "0.32",
      stroke: `hsl(${hue} 70% 28%)`, "stroke-width": pen(1.4),
      "stroke-opacity": "1",
    }));
    const r = pen(Math.max(1.5, size / 200));
    svg.append(wSvg("circle", {
      cx: x, cy: y, r: r, fill: `hsl(${hue} 70% 20%)`,
      stroke: "rgba(255,255,255,.85)", "stroke-width": pen(0.7),
      "fill-opacity": "1",
    }));
    const label = wSvg("text", {
      x: x + r * 1.6, y: y - r * 1.2,
      "font-size": pen(Math.max(9, size / 40)),
      fill: `hsl(${hue} 70% 25%)`, stroke: "rgba(255,255,255,.75)",
      "stroke-width": pen(0.6), "paint-order": "stroke",
    });
    label.textContent = entity.id;
    svg.append(label);
  }
  // An unplaced thing has nowhere to put a label, so its newest sighting carries
  // one instead -- otherwise the only entities named on the map are the ones
  // that already worked, which is the wrong half to show.
  for (const {entity, rays} of shown) {
    if (placeOf(entity) || !rays.length) continue;
    const ray = rays[rays.length - 1];
    const [tx, ty] = along(ray, ray.bearing_deg, ray.length_m);
    const label = wSvg("text", {
      x: tx, y: ty, "font-size": pen(Math.max(9, size / 40)),
      fill: `hsl(${wHue(entity.id)} 70% 30%)`, stroke: "rgba(255,255,255,.7)",
      "stroke-width": pen(0.6), "paint-order": "stroke",
    });
    label.textContent = entity.id;
    svg.append(label);
  }
  // Whatever the pointer was resting on before this redraw is still what it is
  // resting on: the list underneath it did not move, and a highlight that fell
  // off every couple of seconds would read as the map losing track.
  wHighlight(worldHover);

  // What is on the screen, which is now one thing's evidence rather than the
  // whole store's.
  const bits = selected
      ? [selected, `${sightings} sighting${sightings === 1 ? "" : "s"}`]
      : [`${placed.length} placed`,
         `${sightings} sighting${sightings === 1 ? "" : "s"}`];
  if (agreeing || disagreeing) {
    bits.push(`${agreeing} on it, ${disagreeing} off it`);
  }
  if (closeup) {
    bits.push(`${(side / metresToPx(1)).toFixed(1)} m across`);
    // Which is where the grey comes from. The map is drawn a few metres around
    // wherever the rover is standing, and most of what it has seen was seen from
    // somewhere else -- so a thing can be perfectly well placed and still have
    // no map under it.
    if (vx < 0 || vy < 0 || vx + side > size || vy + side > size) {
      bits.push("outside the drawn map");
    }
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
  // What this look scored against the phrase, when it is one a filter found.
  // The large view is reached from the filtered grid as well as the whole
  // stream, and there the score is why the tile was on the screen at all.
  if (worldFilter && observation.score != null) {
    head.append(wScore(observation.score), document.createTextNode(" "));
  }
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
  // Which way the thing itself lies from where the rover stood, worked out on
  // the rover when the look was taken. For a look that belongs to no entity it
  // is the only thing on the row that says where to go and find it, and that is
  // the ordinary state of anything a search turns up that has been seen once.
  if (observation.bearing_deg != null) {
    bits.push(`bearing ${(+observation.bearing_deg).toFixed(1)}°`);
  }
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
  const entity = world.selected;
  if (!entity || !entity.id || entity.id !== state.world.selected) {
    worldDetailFor = worldDetailHead = "";
    worldDetailRows = new Map();
    const hint = document.createElement("p");
    hint.className = "hint";
    hint.textContent = "nothing selected";
    pane.replaceChildren(hint);
    return;
  }
  // A different thing is a different pane and starts at the top of itself. The
  // same thing keeps the two boxes it is already drawn in, because the lower one
  // is the scroller somebody is reading down.
  if (worldDetailFor !== entity.id) {
    worldDetailFor = entity.id;
    worldDetailHead = "";
    worldDetailRows = new Map();
    const heading = document.createElement("div");
    // Every look that was decided to be this thing, newest first, in a scroller
    // of its own so that the heading above stays put and the entity list beside
    // it does not scroll away. **The pictures are what a person is here to
    // read**: nothing names a region any more, so whether these four crops
    // really are one object is a question only the boxes can answer.
    const looks = document.createElement("div");
    looks.className = "wscroll";
    pane.replaceChildren(heading, looks);
  }
  const head = pane.firstElementChild, scroller = pane.lastElementChild;
  drawWorldHead(head, entity);
  drawWorldLooks(scroller);
}

// The heading over the chosen thing's looks: what it is, where it is, and the
// two things that make its position unreadable. Written again only when one of
// them has really changed, so that a body arriving every second does not take a
// half-made text selection with it.
function drawWorldHead(head, entity) {
  const key = JSON.stringify([entity, (world.summary || {}).map_session,
                              worldFilter ? worldFilter.things.has(entity.id) : null]);
  if (key === worldDetailHead) return;
  worldDetailHead = key;
  const title = document.createElement("h2");
  title.textContent = entity.id;
  const meta = document.createElement("div");
  meta.className = "wmeta mono";
  meta.textContent = `kind ${entity.kind} · ${entity.observation_count} `
                   + `observations · created ${wTime(entity.created_at)} · `
                   + `last seen ${wTime(entity.last_seen_at)}`;
  const parts = [title, meta, wPlace(entity)];
  if (worldFilter && !worldFilter.things.has(entity.id)) {
    // Chosen before the box narrowed the list, and the list no longer has it.
    const aside = document.createElement("div");
    aside.className = "wmeta wold";
    aside.textContent = "not one of the matches";
    parts.push(aside);
  }
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
    parts.push(stale);
  }
  head.replaceChildren(...parts);
}

// The chosen thing's looks, in the scroller they are already in.
//
// **A look is left exactly as it is unless what it says has changed**, which is
// the whole of what makes this pane readable while the rover records. Each row
// carries a picture the browser has fetched and a raw block somebody may have
// opened, and both of those go with the row: rebuilding all nine of them because
// a tenth arrived is what used to throw a reader back to the top of the list
// with their pictures loading again. Only a look that has genuinely moved --
// most often one the resolver has just attached, or re-measured against a
// position that has settled since -- is drawn again.
function drawWorldLooks(scroller) {
  const observations = world.selected_observations || [];
  if (!observations.length) {
    worldDetailRows = new Map();
    const empty = document.createElement("p");
    empty.className = "hint";
    empty.textContent = "no observations";
    scroller.replaceChildren(empty);
    return;
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
  const kept = new Map();
  let at = scroller.firstChild;
  for (const observation of observations) {
    const relation = relations[observation.id] || null;
    // Everything the row is drawn from, including whether a filter is putting a
    // score on it: the same test the large view of a single look applies.
    const drawn = JSON.stringify([observation, relation, !!worldFilter]);
    let row = worldDetailRows.get(observation.id);
    if (!row || row.drawn !== drawn) {
      const node = wObservation(observation, {relations: relations});
      // Reading down this scroller is reading one look at a time, and the
      // question each one raises -- where was this taken from, and is this the
      // line that disagrees -- is answered by the map beside it. So the pointer
      // resting on a row lights that row's own line and dims the rest, which is
      // the only way to tell which of eight lines belongs to the picture being
      // looked at.
      node.onmouseenter = () => wHighlight(observation.id);
      node.onmouseleave = () => wHighlight(null);
      row = {drawn: drawn, node: node};
    }
    kept.set(observation.id, row);
    if (row.node === at) at = at.nextSibling;
    else scroller.insertBefore(row.node, at);
  }
  // Whatever is left below them is a look this thing no longer owns, or the
  // "no observations" line from when it had none.
  while (at) { const next = at.nextSibling; at.remove(); at = next; }
  worldDetailRows = kept;
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
  // Addressed by class rather than by position, because a filter puts a score
  // line above this one and takes it away again, and a tile is reused across
  // both -- the same tile that was a match a moment ago is an ordinary look in
  // the stream once the box is emptied.
  const caption = document.createElement("div");
  caption.className = "wcaption";
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
  const caption = tile.querySelector(".wcaption");
  const scored = tile.querySelector(".wscore");
  if (scored) scored.remove();
  if (worldFilter && observation.score != null) {
    tile.insertBefore(wScore(observation.score), caption);
  }
  caption.className = "wcaption wmeta mono" + (observation.entity_id ? "" : " wdup");
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
  const summary = world.summary || {};
  // Under a filter the grid is the matches, best first, rather than a window on
  // the stream. **The rover ranked every stored vector it has**, so a look it
  // matched can be far older than anything the tiles had reached, and a grid
  // narrowed to what the browser happened to hold would be missing exactly the
  // look that was asked for. What the stream holds is left untouched while that
  // is on screen, so emptying the box puts the tiles back rather than fetching
  // the history again.
  const rows = worldFilter ? [...worldFilter.looks.values()] : wObsRows();
  const total = summary.observations ?? rows.length;
  if (worldFilter) {
    const compared = worldFilter.answer.considered ?? total;
    $("wObsCount").textContent = rows.length
        ? `${rows.length} matching look${rows.length === 1 ? "" : "s"} `
          + `of ${compared} compared, best first`
        : "no match";
  } else {
    $("wObsCount").textContent = !rows.length ? "nothing yet."
        : `${rows.length} of ${total} shown, newest first`
          + (summary.unmatched ? `, ${summary.unmatched} with no entity` : "");
  }
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
  // Nothing to say about the stream while the grid is not showing it.
  wObsSay(worldFilter ? "" : worldStreamNote);
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
  // Not while a filter is showing: the grid is then the rover's own answer over
  // the whole store, and scrolling to the end of it is not a request for the
  // looks below the ones the stream happens to hold.
  if (worldFilter || pane.hidden || worldStreamBusy || !worldStreamMore) return;
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
  // The filter's own rows first: a match is a whole observation with a score on
  // it, and it may be a look the stream has never fetched.
  const shown = worldZoom === null ? null
      : (worldFilter && worldFilter.looks.get(worldZoom))
        || worldStream.get(worldZoom) || null;
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
// is several seconds away. Drawn only from the body, the line sat on the last
// phrase's verdict for the whole of that wait, so pressing enter changed nothing
// on screen and the verdict that was there read as the answer to what had just
// been typed. The seconds are the rover's own, so a second browser that joins
// during a search sees how long it has really been running.
//
// **The views below are left alone while a phrase is in flight.** They are still
// showing the answer to the phrase before it, which is what they were showing a
// moment ago and is true until this one lands; emptying them would take a
// working screen away for five seconds and put a different one back.
function drawWorldAsking(w) {
  const asking = !!w.searching;
  $("wSearchBox").classList.toggle("wasking", asking);
  if (asking) {
    $("wSearchNote").className = "";
    $("wSearchNote").textContent = `asking... ${w.searched_s || 0} s`;
  } else if (worldAsking) {
    // Whatever the rover left: the answer, or nothing at all if it refused --
    // a refusal brings no new body, so no redraw would otherwise happen, and
    // the filter that was on stays on because it is still what is drawn.
    drawWorldFilter();
  }
  worldAsking = asking;
}

// The line under the box: the verdict, which is the part of a search that
// matters. The list of scores has moved into the views themselves, so this is
// the only place left that can say whether the top of it means anything -- and
// on a filtered screen, whether the things now listed are the thing that was
// asked for or merely the nearest the rover has.
function drawWorldFilter() {
  const note = $("wSearchNote");
  if (state.world.searching) return;   // `drawWorldAsking` has the line
  const answer = worldFilter && worldFilter.answer;
  if (!answer) {
    note.className = "";
    note.textContent = "";
    return;
  }
  note.className = "wverdict " + (answer.confident ? "wfound" : "wmissing");
  note.textContent = (answer.confident
      ? "found it. " : `nothing here matches “${answer.query}”. `)
      + (answer.detail || "")
      + (answer.skipped ? ` -- ${answer.skipped} skipped, other backend` : "");
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
      $("wPaneDiagnostics").hidden = worldTab !== "diagnostics";
      // The box narrows the two views of what the rover has seen. It narrows
      // nothing in the diagnostics table, which is a log of inferences and not
      // a view of the store, so it is not offered there.
      $("wFilter").hidden = worldTab === "diagnostics";
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
      $("wSearchNote").className = "";
      $("wSearchNote").textContent = "asking... 0 s";
      $("wSearchBox").classList.add("wasking");
      worldAsking = true;
    }
  };
  // The box's own clear -- the cross a browser puts in a search field. It takes
  // the filter off at once rather than waiting for an empty phrase to be
  // entered, because clearing a filter is a local thing and nothing has to go
  // to the rover for it. **Not a key listener**: space and Escape stop the
  // rover from anywhere on this page, and nothing in this popup may take them.
  $("wSearchBox").addEventListener("search", () => {
    if (!$("wSearchBox").value.trim()) {
      post({do: "world", what: "search", query: ""});
    }
  });
}
