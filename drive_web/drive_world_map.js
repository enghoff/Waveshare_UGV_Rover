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
  // **The popup has a map of its own, and this is which one is under it.** The
  // card behind this popup is drawn a few metres around wherever the rover is
  // standing, because that is what driving needs; this panel draws bearings
  // taken from all over a flat, so a thing placed six metres away sat on black
  // with "outside the drawn map" underneath. The console now asks the rover for
  // a second picture wide enough to hold what is drawn here -- see
  // `world_map_extent` -- and everything below is laid over whichever of the two
  // is in hand. The driving map is the fallback rather than the default: it is
  // what there is until the first of the wider ones lands, which is a second or
  // so after the popup opens.
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
  if (!view || !view.pose || !picture.gen || (!sightings && !placed.length)) {
    wrap.hidden = true;
    note.textContent = !picture.gen ? "no map yet"
        : !sightings ? "nothing observed from a known pose"
        : "the map did not say where it was drawn from";
    return;
  }
  wrap.hidden = false;
  $("wMapImg").src = `${picture.src}?gen=${picture.gen}`;
  const size = picture.width || 1;
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
  const image = $("wMapImg");
  image.style.transform = closeup
      ? `scale(${zoom}) translate(${-vx / size * 100}%, ${-vy / size * 100}%)`
      : "";
  image.classList.toggle("wclose", zoom > 1.5);
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
    // It should not say this any more, and that is why it is still here. The
    // console asks the rover for a picture wide enough to hold what is drawn on
    // it, so a window leaving that picture now means one of two real things: the
    // wider map has not arrived yet and this is still the driving one, or the
    // thing is further from the rover than the renderer will draw -- twelve
    // metres each way, which no longer fits in one picture. Both are worth
    // saying; neither is the ordinary case it used to be.
    if (vx < 0 || vy < 0 || vx + side > size || vy + side > size) {
      bits.push("outside the drawn map");
    }
  }
  note.textContent = bits.join(" Â· ");
}
