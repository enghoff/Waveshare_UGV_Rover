// World-state map geometry and drawing. Loaded after drive_world.js.

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

// The window the panel shows, as a square in the picture's own pixels: the map
// itself, centred, with a margin round it.
//
// **Framed on the room and not on the picture.** The picture is centred on
// wherever the rover happens to be standing and is as wide as the widest thing
// the panel has to hold, so on a rover parked in a corner the room sat in one
// half of it and the other half was the grey of a grid nobody has driven
// through. `known_box_m` is the rover's own answer to "where is the map",
// measured off the occupancy grid before the camera cone and the scale bar are
// drawn over it -- so a cone reaching five metres into an unvisited room cannot
// drag the frame off the floor the rover has actually seen.
//
// Square, because the picture is square and the frame stretches the drawing over
// it; kept inside the picture, because past the edge is black and sliding the
// view off the map to keep a box centred shows less map rather than more.
const WMAP_MARGIN = 0.06;

function wMapWindow(view, size, floorPx) {
  const box = view && view.known_box_m;
  if (!Array.isArray(box) || box.length !== 4) return null;
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  // Through `wPointToPx` corner by corner rather than by scaling the metres,
  // because the map can be drawn rover-up and the box is then a diamond.
  for (const [mx, my] of [[box[0], box[1]], [box[2], box[1]],
                          [box[0], box[3]], [box[2], box[3]]]) {
    const [px, py] = wPointToPx(mx, my, view);
    x0 = Math.min(x0, px); y0 = Math.min(y0, py);
    x1 = Math.max(x1, px); y1 = Math.max(y1, py);
  }
  if (!isFinite(x0)) return null;
  const wanted = Math.max(x1 - x0, y1 - y0) * (1 + 2 * WMAP_MARGIN);
  const side = Math.max(floorPx, Math.min(size, wanted));
  const fit = (centre) => Math.max(0, Math.min(size - side, centre - side / 2));
  return [fit((x0 + x1) / 2), fit((y0 + y1) / 2), side, wanted > size];
}

// Choosing a thing by pointing at it. **The map is a locator now, not a canvas
// of evidence**: it carries one mark per thing and nothing else, and the list,
// the pane of looks and the observation stream beside it are what the pointer
// fills in. A click as well would make this the one panel here that has to be
// operated rather than read.
//
// Delayed by `WPICK_MS`, and that is not a nicety. Choosing fetches the thing's
// looks off the rover and moves a payload that every browser on this console
// shares, so a pointer swept across a cluster of forty marks has to buy one of
// those and not forty.
const WPICK_MS = 140;
let worldPickTimer = null;
// Where the marks are, in the units the frame is drawn in, and what the frame is
// showing. Kept because the pointer is answered against the *nearest* mark and
// not against whatever element happens to be under it.
let worldPickMarks = [], worldPickBox = null, worldPickNear = "";

// **Nearest wins, not topmost.** A mark is three pixels across, so it needs a
// target bigger than itself; give each one its own and a room's worth of them
// overlap, at which point the one drawn last is the one the pointer hits and a
// person aiming at a thing chooses its neighbour. Measured against a real store
// of 105 marks that is not an edge case, it is most of the map. So the frame
// answers the pointer itself: whichever centre is closest, if any is close
// enough.
//
// One handler on the frame is also what survives a redraw. The marks are
// replaced every time a body arrives, which is about once a second while the
// rover is looking, and a handler belonging to a mark would go with it.
function wPickOn(svg) {
  svg.onpointermove = (event) => {
    if (!worldPickBox) return;
    const [vx, vy, side] = worldPickBox;
    const rect = svg.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    // The frame is stretched over the panel -- `preserveAspectRatio="none"` --
    // so this is the plain proportion either way, and it stays right when the
    // panel is resized without anything having to be redrawn.
    const ux = vx + (event.clientX - rect.left) / rect.width * side;
    const uy = vy + (event.clientY - rect.top) / rect.height * side;
    const reach = side / 25;
    let best = "", nearest = reach * reach;
    for (const mark of worldPickMarks) {
      const away = (mark.x - ux) ** 2 + (mark.y - uy) ** 2;
      if (away < nearest) { nearest = away; best = mark.id; }
    }
    wPickNear(svg, best);
  };
  // Leaving the map drops a choice that has not been bought yet. It does not
  // undo one that has: the pane beside this is showing that thing's looks, and a
  // panel that emptied itself when the pointer wandered off would be unreadable.
  svg.onpointerleave = () => { wPickNear(svg, ""); };
}

// The thing the pointer is closest to: marked at once, chosen a moment later.
//
// The delay is not a nicety. Choosing fetches the thing's looks off the rover
// and moves a payload every browser on this console shares, so a pointer swept
// across a room's worth of marks must buy one of those and not a hundred. What
// happens immediately is the ring, so that the wait reads as aim rather than as
// a map ignoring the pointer.
function wPickNear(svg, id) {
  if (id !== worldPickNear) {
    worldPickNear = id;
    for (const mark of svg.querySelectorAll("g.wdot")) {
      mark.classList.toggle("wnear", !!id && mark.dataset.entity === id);
    }
  }
  clearTimeout(worldPickTimer);
  if (!id || id === state.world.selected) return;
  worldPickTimer = setTimeout(
      () => post({do: "world", what: "select", id: id}), WPICK_MS);
}

function drawWorldMap() {
  const wrap = $("wMapWrap"), note = $("wMapNote"), svg = $("wRays");
  // **The popup has a map of its own, and this is which one is under it.** The
  // card behind this popup is drawn a few metres around wherever the rover is
  // standing, because that is what driving needs; this panel is about where
  // things are in a whole flat, so a thing placed six metres away sat on black.
  // The console asks the rover for a second picture wide enough to hold both the
  // things and the map -- see `world_map_extent` -- and the driving map is the
  // fallback for the second between the popup opening and the first one landing.
  const picture = state.world.map && state.world.map.gen
      ? {gen: state.world.map.gen, view: state.world.map.view,
         width: state.world.map.width, src: "/world_map.png"}
      : {gen: state.map.gen, view: state.map.view,
         width: state.map.width, src: "/map.png"};
  const view = picture.view;
  // Whatever the list beside it is showing, and for the same reason: a map still
  // covered in every thing the rover has seen, next to a list narrowed to one of
  // them, is two answers to one question.
  const entities = wShown();
  const selected = entities.some((one) => one.id === state.world.selected)
      ? state.world.selected : "";
  // A position measured against a map that has since been cleared is not a
  // position on this one. Those things stay in the list, where their looks and
  // their pictures still mean something; they are not on the map, and the line
  // underneath says how many.
  const session = (world.summary || {}).map_session;
  const placed = entities.filter((one) => one.placement
      && (!session || one.placement_map_session === session));
  if (!view || !view.pose || !picture.gen) {
    wrap.hidden = true;
    note.textContent = !picture.gen ? "no map yet"
        : "the map did not say where it was drawn from";
    return;
  }
  wrap.hidden = false;
  wPickOn(svg);
  $("wMapImg").src = `${picture.src}?gen=${picture.gen}`;
  const size = picture.width || 1;
  svg.replaceChildren();

  const metresToPx = (metres) => {
    const [ax, ay] = wPointToPx(0, 0, view);
    const [bx, by] = wPointToPx(metres, 0, view);
    return Math.hypot(bx - ax, by - ay);
  };

  // --- the window -----------------------------------------------------------
  const window_ = wMapWindow(view, size, metresToPx(2));
  const [vx, vy, side] = window_ || [0, 0, size];
  const zoom = size / side;
  svg.setAttribute("viewBox", `${vx} ${vy} ${side} ${side}`);
  // The picture under the marks has to move with them. It is one PNG at a fixed
  // resolution, so this is that same window taken out of it, and its cells come
  // out as the squares they are rather than as a blur -- see `.wclose`.
  const image = $("wMapImg");
  image.style.transform = window_
      ? `scale(${zoom}) translate(${-vx / size * 100}%, ${-vy / size * 100}%)`
      : "";
  image.classList.toggle("wclose", zoom > 1.5);
  // Every radius and width below is in the map's own pixels, so closing in would
  // thicken all of them by the same factor and a fitted view would be drawn in
  // crayon. This is what keeps a mark the size it was on screen.
  const pen = (n) => n / zoom;
  svg.style.setProperty("--wpen", pen(1));

  // --- one mark per thing, and nothing else ---------------------------------
  //
  // **The bearings, the sight lines and the names are gone from here.** Two
  // hundred things' worth of them over one room was a coloured haze with the
  // map invisible underneath, and the questions they answered -- where was this
  // look taken from, does it agree with the rest -- are answered in numbers on
  // the look's own row in the pane beside this. What a map is good at is *where*,
  // and that is now all it claims.
  const marks = [];
  for (const entity of placed) {
    const hue = wHue(entity.id);
    const place = entity.placement;
    const [x, y] = wPointToPx(place.x_m, place.y_m, view);
    const on = entity.id === selected;
    const mark = wSvg("g", {class: "wdot" + (on ? " won" : "")});
    mark.setAttribute("data-entity", entity.id);

    // How sure the crossing was, for the chosen thing alone. It is the one claim
    // a dot cannot make and the one a person examining a thing needs, and drawn
    // for all of them it is the haze again.
    if (on) {
      const major = Math.max(0.02, +place.error_major_m
                                   || +entity.placement_uncertainty_m || 0.2);
      mark.append(wSvg("path", {
        d: wRing(place.x_m, place.y_m, major,
                 Math.max(0.02, +place.error_minor_m || major),
                 place.error_major_deg, view),
        fill: `hsl(${hue} 70% 50%)`, "fill-opacity": "0.25",
        stroke: `hsl(${hue} 70% 30%)`, "stroke-width": pen(1.2),
      }));
    }
    const r = pen(Math.max(1.5, size / 200)) * (on ? 1.8 : 1);
    mark.append(wSvg("circle", {
      cx: x, cy: y, r: r, fill: `hsl(${hue} 70% ${on ? 30 : 45}%)`,
      stroke: "rgba(255,255,255,.9)", "stroke-width": pen(on ? 1.1 : 0.7),
    }));
    // Where the pointer is answered from -- see `wPickOn`, which measures
    // against these rather than waiting to be hit.
    marks.push({id: entity.id, x: x, y: y});
    svg.append(mark);
  }

  worldPickMarks = marks;
  worldPickBox = [vx, vy, side];
  // Whatever the pointer was nearest before this redraw it is still nearest to:
  // the room did not move, and a ring that fell off every second would read as
  // the map losing the pointer.
  if (worldPickNear) {
    for (const mark of svg.querySelectorAll("g.wdot")) {
      mark.classList.toggle("wnear", mark.dataset.entity === worldPickNear);
    }
  }

  // --- what is on the screen ------------------------------------------------
  const missing = entities.length - placed.length;
  const bits = [];
  if (selected) bits.push(selected);
  bits.push(`${placed.length} on the map`);
  if (missing > 0) {
    // Which is the ordinary state of a thing seen once -- and, after a map has
    // been cleared, of everything. Said as a count rather than left to be read
    // off an empty picture, because an empty picture looks like a broken panel.
    bits.push(`${missing} not placed on it`);
  }
  bits.push(`${(side / metresToPx(1)).toFixed(1)} m across`);
  // The map has outgrown the picture it was drawn into. Rare -- the console asks
  // for a picture that holds it -- and it means the rover's renderer has hit its
  // own ceiling of twelve metres each way, so a flat bigger than that is being
  // shown in part.
  if (window_ && window_[3]) bits.push("wider than the picture");
  note.textContent = bits.join(" · ");
}
