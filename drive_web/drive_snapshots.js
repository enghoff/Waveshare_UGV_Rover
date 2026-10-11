// The drive console's snapshot popup: the rover's map, world state and place on
// the map, kept so they can be put back.
//
// The browser half of drive_snapshots.py. Loaded BEFORE drive_web.js, which is
// what calls `start()`; nothing here runs at load, and `$`, `post` and `state`
// are looked up when a draw happens, by which time drive_web.js has run.
//
// The list is fetched from /snapshots.json when its tag moves, and every press
// posts an action and waits to be told what happened, like the rest of the page.
// Which press is armed comes from the server too, so two browsers agree on what
// the next press of a restore or a delete will do.

"use strict";

let snapGen = "", snapRows = [];

function snapBytes(bytes) {
  if (bytes == null) return "-";
  if (bytes >= 1e9) return `${(bytes / 1e9).toFixed(bytes >= 1e10 ? 0 : 1)} GB`;
  if (bytes >= 1e6) return `${Math.round(bytes / 1e6)} MB`;
  return `${Math.max(1, Math.round(bytes / 1e3))} kB`;
}

function snapTaken(at) {
  if (!at) return "-";
  return new Date(at * 1000).toLocaleString(undefined, {
    month: "short", day: "numeric", hour: "2-digit", minute: "2-digit"});
}

function snapWhere(row) {
  const pose = row.pose;
  if (!pose) return "-";
  return `(${(+pose.x_m).toFixed(2)}, ${(+pose.y_m).toFixed(2)}) `
      + `${Math.round(+pose.heading_deg)}°`
      + (row.settled === false ? " unconfirmed" : "");
}

function drawSnapshots(s) {
  if (!s) return;
  $("snapBack").hidden = !s.open;
  $("snapshots").classList.toggle("on", s.open);
  const doing = s.doing ? `${s.doing}...`
      : s.elsewhere ? `${s.elsewhere} from elsewhere...` : "";
  $("snapCounts").textContent = `${s.count} snapshot${s.count === 1 ? "" : "s"}`
      + (s.free_bytes != null ? ` -- ${snapBytes(s.free_bytes)} free` : "")
      + (doing ? ` -- ${doing}` : "");
  $("snapNote").textContent = s.note || "";
  $("snapError").textContent = s.error || "";
  $("snapError").hidden = !s.error;
  $("snapSave").disabled = !state.link.connected || !!s.doing || !!s.elsewhere;
  $("snapSave").textContent = s.doing === "saving" ? "saving..." : "save now";
  $("snapEmpty").textContent = s.open && !s.count && s.gen ? "no snapshots" : "";
  if (s.open && s.gen && s.gen !== snapGen) {
    snapGen = s.gen;
    fetch(`/snapshots.json?gen=${s.gen}`)
      .then((reply) => reply.ok ? reply.json() : null)
      .then((rows) => { if (rows) { snapRows = rows; drawSnapList(); drawSnapButtons(); } })
      .catch(() => {});    // the next state carrying a new tag asks again
  }
  drawSnapButtons();
}

// The rows, rebuilt only when the list itself changed. A name being typed is put
// back as it was, focus and all: a snapshot an agent takes mid-edit must not
// throw away what somebody was in the middle of writing.
function drawSnapList() {
  const typing = document.activeElement;
  const editing = typing && typing.classList.contains("snap-name")
      ? {id: typing.closest("tr").dataset.id, value: typing.value,
         from: typing.selectionStart, to: typing.selectionEnd} : null;
  const body = $("snapList");
  body.replaceChildren();
  for (const row of snapRows) {
    const line = document.createElement("tr");
    line.dataset.id = row.id;
    const name = document.createElement("input");
    name.type = "text";
    name.className = "snap-name";
    name.maxLength = 80;
    name.value = row.name || "";
    name.setAttribute("aria-label", "snapshot name");
    const cell = document.createElement("td");
    cell.append(name);
    line.append(cell);
    for (const [text, kind] of [
        [snapTaken(row.created_at), ""],
        [row.entities ?? "-", "num"], [row.observations ?? "-", "num"],
        [row.same_map ? "this map" : row.map_id ? `map ${row.map_id.slice(0, 6)}` : "no map", ""],
        [snapWhere(row), "mono"], [snapBytes(row.bytes), "num"]]) {
      const td = document.createElement("td");
      td.textContent = text;
      if (kind) td.className = kind;
      line.append(td);
    }
    const acts = document.createElement("td");
    for (const act of ["restore", "delete"]) {
      const button = document.createElement("button");
      button.dataset.act = act;
      button.textContent = act;
      acts.append(button);
    }
    line.append(acts);
    body.append(line);
    if (editing && editing.id === row.id) {
      name.value = editing.value;
      name.focus();
      name.setSelectionRange(editing.from, editing.to);
    }
  }
}

// What each button says and whether it can be pressed, which moves with the
// rover's state rather than with the list.
function drawSnapButtons() {
  const s = state && state.snapshots;
  if (!s) return;
  const connected = state.link.connected;
  const busy = !!s.doing || !!s.elsewhere;
  const wheelsTaken = state.busy !== null || state.exploring;
  for (const button of $("snapList").querySelectorAll("button[data-act]")) {
    const act = button.dataset.act;
    const armed = s.armed && s.armed.what === act
        && s.armed.id === button.closest("tr").dataset.id;
    const text = armed ? `${act} -- press again` : act;
    // Written only when it changes: this runs on every state, ten a second
    // while the rover moves.
    if (button.textContent !== text) button.textContent = text;
    button.classList.toggle("armed", !!armed);
    const off = !connected || busy || (act === "restore" && wheelsTaken);
    if (button.disabled !== off) button.disabled = off;
  }
}

function wireSnapshots() {
  $("snapshots").onclick = () => post(
      {do: "snapshots", what: state.snapshots.open ? "close" : "open"});
  $("snapClose").onclick = () => post({do: "snapshots", what: "close"});
  $("snapSave").onclick = () => post({do: "snapshots", what: "save"});
  // A click on the dimmed page behind closes it, as the world popup does.
  $("snapBack").onclick = (event) => {
    if (event.target === $("snapBack")) post({do: "snapshots", what: "close"});
  };
  $("snapList").addEventListener("click", (event) => {
    const button = event.target.closest("button[data-act]");
    if (!button) return;
    post({do: "snapshots", what: button.dataset.act,
          id: button.closest("tr").dataset.id});
  });
  // A name is sent when the box is left or Enter is pressed, and only if it
  // changed; an emptied box goes back to the name it had.
  $("snapList").addEventListener("change", (event) => {
    const box = event.target;
    if (!box.classList.contains("snap-name")) return;
    const id = box.closest("tr").dataset.id;
    const row = snapRows.find((one) => one.id === id);
    const name = box.value.trim();
    if (!name || (row && row.name === name)) {
      if (row) box.value = row.name;
      return;
    }
    post({do: "snapshots", what: "rename", id: id, name: name});
  });
  $("snapList").addEventListener("keydown", (event) => {
    if (event.key === "Enter" && event.target.classList.contains("snap-name")) {
      event.target.blur();
    }
  });
}
