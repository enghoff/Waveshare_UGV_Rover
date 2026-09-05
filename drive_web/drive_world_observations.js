// World-state observation stream. Loaded after drive_world.js.

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
