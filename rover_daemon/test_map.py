"""Map checks: what the renderer draws and where a tap on it lands.

The map is the one tool whose output a person reads as a picture, so the failure
to guard against is a plausible-looking room that is wrong. A tap has to come
back as the metres it looked like, which is two conversions deep and invisible
when either is off.
"""
from __future__ import annotations

import json
import math

from test_fakes import FakeLink
from test_harness import check

def test_map_png_names_the_clock():
    """`map_png` times itself. A missing `import time` used to reach the page as
    `NameError: name 'time' is not defined` and leave the map blank."""
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")

    class FakeNav:
        class slam:
            class config:
                resolution_m = 0.05
            pose = (0.0, 0.0, 0.0)

        def map_png(self, *args, **kwargs):
            # Signature + IHDR so the handler can read the width at bytes 16:20.
            png = (b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR"
                   + (640).to_bytes(4, "big") + (480).to_bytes(4, "big"))
            return png, "a fake room"

    rover.nav = FakeNav()
    got = rover.call("map_png", {"half_extent_m": 3, "scale": 3})
    check("map_png answers rather than raising", got.get("ok"), True)
    check("...and names the picture size it drew", got.get("pixels"), 640)
    check("...and times the draw", isinstance(got.get("render_s"), (int, float)), True)


def test_show_map_takes_across_and_size():
    """The model can pick how much room is in frame and how big a picture.

    `across_m` is metres of room, not the half-extent `map_png` takes: a model
    told "six metres across" and handed `half_extent_m` would pass 6 and get
    twelve. Leave both out and it is still a room at the console's default
    picture size -- pixels per cell derived, so widening the view does not
    resize the picture.
    """
    import http.server
    import threading

    import rover_daemon

    posted = []
    asked = []

    class Vision(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            posted.append(body)
            reply = json.dumps({"ok": True, "image": f"frame-{len(posted)}"}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)

        def log_message(self, *args):
            pass

    class FakeNav:
        class slam:
            class config:
                resolution_m = 0.05
            pose = (0.0, 0.0, 0.0)

        def map_png(self, half, scale, rover_up=False, camera=None):
            asked.append((half, scale))
            png = (b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR"
                   + (640).to_bytes(4, "big") + (480).to_bytes(4, "big"))
            return png, f"a fake room of {2 * half:.0f} m"

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "a room"}

    res = FakeNav.slam.config.resolution_m
    room = rover_daemon._model_map_view({}, res)
    wide = rover_daemon._model_map_view({"across_m": 24}, res)
    close = rover_daemon._model_map_view({"across_m": 3}, res)
    small = rover_daemon._model_map_view({"pixels": 320}, res)
    large = rover_daemon._model_map_view({"pixels": 800}, res)
    check("nothing asked for is a room", room[0], rover_daemon.MAP_HALF_EXTENT_M)
    check("six metres across is that same room, not twelve",
          rover_daemon._model_map_view({"across_m": 6}, res)[0],
          rover_daemon.MAP_HALF_EXTENT_M)
    check("twenty-four metres across is the drawing ceiling",
          wide[0], rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("a close view is closer than a room", close[0] < room[0], True)
    check("a bigger picture is more pixels per cell, not more room",
          (large[0], large[1] > small[1]), (small[0], True))
    check("widening the view does not raise the magnification",
          wide[1] <= room[1], True)

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Vision)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    address = f"127.0.0.1:{server.server_address[1]}"
    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0",
                               vision=address)
    rover.nav = FakeNav()
    try:
        check("show_map is offered once there is somewhere to send a picture",
              "show_map" in [t["function"]["name"] for t in rover.tools()], True)
        got = rover.call("show_map", {})
        check("nothing asked for still draws", got.get("ok"), True)
        check("...at the room-sized default", asked[-1], room)
        check("...and the picture was posted", len(posted), 1)
        check("a string six means six metres across",
              rover.call("show_map", {"across_m": "6"}).get("ok"), True)
        check("...and is a room, not a floor", asked[-1][0], room[0])
        rover.call("show_map", {"across_m": 24, "pixels": 320})
        check("a floor view at a small picture reaches the renderer as such",
              asked[-1], rover_daemon._model_map_view(
                  {"across_m": 24, "pixels": 320}, res))
        check("the caption still goes to the model, not the picture size",
              "caption" in got, True)
        check("...and the result does not name pixels-per-cell",
              "scale" in got, False)
    finally:
        server.shutdown()


def test_drive_to_takes_a_place_on_the_map():
    """`drive_to` will take a point on the map, and only a console is told so.

    A tap on the console's map has to keep its meaning while the rover is still
    driving: the click stops the move in flight, and the rover carries on until the
    stop lands, so an offset from "where the rover is" is measured from somewhere
    the cursor never was. A point in the map's own frame is not, which is why the
    console sends every tap that way.

    The second half of this is the more important one. The pair is deliberately
    absent from the schema a model is shown, because nothing a model can see says
    where the rover is in that frame -- the room comes back to it as bearings and
    the map as a picture centred on itself -- so a model offered map coordinates
    could only invent them, and an invented pair is a fifteen-metre drive to a
    place nobody chose.
    """
    import rover_daemon
    import tool_schemas

    asked = []

    class FakeNav:
        class Outcome:
            reason = "arrived"

            def asdict(self):
                return {"reason": "arrived", "travelled_m": 1.0}

        def drive_to(self, **kwargs):
            asked.append(kwargs)
            return self.Outcome()

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "a room"}

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    rover.nav = FakeNav()

    got = rover.call("drive_to", {"x_m": 3.0, "y_m": -1.25})
    check("a place on the map is accepted", got.get("ok"), True)
    check("...and reaches the navigator as a place",
          (asked[-1].get("x_m"), asked[-1].get("y_m")), (3.0, -1.25))
    check("...with no offset invented alongside it",
          "ahead_m" in asked[-1], False)

    rover.call("drive_to", {"ahead_m": 1.0, "left_m": -0.4, "speed_ms": 0.15})
    check("an offset still reaches it as an offset",
          (asked[-1].get("ahead_m"), asked[-1].get("left_m")), (1.0, -0.4))
    check("...and the speed goes with it", asked[-1].get("speed_ms"), 0.15)

    # Half a coordinate is not a place, and guessing the other half would drive
    # somewhere nobody named.
    half = rover.call("drive_to", {"x_m": 3.0})
    check("one coordinate on its own is refused", half.get("ok"), False)
    check("...and says what is missing", "y_m" in str(half.get("error")), True)

    schema = next(s for s in tool_schemas.NAV_TOOLS
                  if s["function"]["name"] == "drive_to")
    offered = set(schema["function"]["parameters"]["properties"])
    check("a model is not offered the map's coordinates",
          offered & {"x_m", "y_m"}, set())
    check("...only the offsets it can actually work out",
          offered, {"ahead_m", "left_m", "speed_ms"})


def test_a_point_on_the_map_picture_is_the_place_it_looks_like():
    """A fraction of the map picture means where it appears to on the picture.

    The one thing this tool has to get right, and the one thing nothing else
    would notice if it got wrong. A model is handed a top-down picture and says
    where on it to go; the daemon turns that into a point in the map's own frame
    using the pose the picture was drawn at. Get the axes, the flip or the pose
    wrong and every part still works -- a picture is drawn, a fraction is
    accepted, a route is planned, the rover drives -- to somewhere else in the
    room, and the only symptom is a rover that goes the wrong way.

    So this is read off the picture rather than out of the arithmetic that drew
    it. Three obstacles go onto a synthetic map at known places, the rover's own
    renderer draws them, and their blobs are found in the PNG by colour. Their
    pixel centroids are then handed to the tool as fractions, and what comes back
    has to be the obstacle that was pointed at. Nothing here works a pixel out
    from a world coordinate, which is what would reduce it to the renderer
    agreeing with itself.

    Deliberately at a pose that is neither the origin nor an axis-aligned
    heading, because both of those hide a swapped axis and a dropped rotation.
    """
    import threading

    import mapimg
    import numpy as np

    import rover_daemon

    resolution, cells = 0.05, 800
    pose = (0.6, -0.35, math.radians(40.0))
    obstacles = [(1.6, 0.9), (-0.4, 1.4), (2.2, -1.2)]

    class Synthetic:
        """The three things `mapimg.render` asks a map for."""

        class config:
            resolution_m = resolution
            grid_cells = cells
            occupied_at = 50

        def __init__(self):
            self.lock = threading.Lock()
            self.pose = pose
            self.trail = ()
            grid = np.zeros((cells, cells), dtype=np.int8)
            # Four metres of seen floor around the rover, so that anything left
            # grey in the picture is grey on purpose.
            span = int(4.0 / resolution)
            cx = int(pose[0] / resolution) + cells // 2
            cy = int(pose[1] / resolution) + cells // 2
            grid[cx - span:cx + span, cy - span:cy + span] = -100
            for wx, wy in obstacles:
                ix = int(round(wx / resolution)) + cells // 2
                iy = int(round(wy / resolution)) + cells // 2
                grid[ix - 2:ix + 3, iy - 2:iy + 3] = 100        # 25 cm of solid
            self._grid = grid

        def grid(self):
            return self._grid

    asked = []

    class FakeNav:
        def __init__(self):
            self.slam = Synthetic()

        def map_png(self, half, scale, rover_up=False, camera=None):
            return mapimg.render(self.slam, half, scale, self.slam.trail,
                                 rover_up=rover_up, camera=camera)

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "an invented room"}

        class Outcome:
            reason = "arrived"

            def asdict(self):
                return {"reason": "arrived", "travelled_m": 1.0, "turned_deg": 0.0}

        def drive_to(self, **kwargs):
            asked.append(kwargs)
            return self.Outcome()

    def blobs(mask):
        """Four-connected runs of at least four True cells, as (row, col) lists."""
        seen = np.zeros(mask.shape, dtype=bool)
        found = []
        for start in zip(*np.nonzero(mask)):
            if seen[start]:
                continue
            stack, blob = [start], []
            seen[start] = True
            while stack:
                row, col = stack.pop()
                blob.append((row, col))
                for drow, dcol in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nrow, ncol = row + drow, col + dcol
                    if (0 <= nrow < mask.shape[0] and 0 <= ncol < mask.shape[1]
                            and mask[nrow, ncol] and not seen[nrow, ncol]):
                        seen[nrow, ncol] = True
                        stack.append((nrow, ncol))
            if len(blob) >= 4:
                found.append(blob)
        return found

    # A vision address nothing answers on: the picture cannot be posted, which
    # `show_map` reports and carries on from, and the view is recorded either way.
    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0",
                               vision="127.0.0.1:9")
    # No camera, so no violet cone is washed over the map. The blobs below are
    # found by exact colour and the cone recolours every solid pixel under it.
    rover.device = None
    rover.nav = FakeNav()

    first = rover.call("drive_to_map_point", {"across": 0.5, "down": 0.4})
    check("pointing at a map that has not been taken is refused",
          first.get("ok"), False)
    check("...and says to take one", "show_map" in first.get("error", ""), True)

    shown = rover.call("show_map", {"across_m": 6})
    check("the map is drawn", shown.get("ok"), True)
    check("...and its caption says the picture may be pointed at",
          "drive_to_map_point" in shown.get("caption", ""), True)
    view = rover._map_shown
    png, _caption = rover.nav.map_png(view["half_extent_m"], view["scale"])
    image = mapimg._decode(png)
    check("the picture is the size the remembered view says it is",
          image.shape[0], view["pixels"])

    solid = np.all(image == np.array(mapimg.C_OCCUPIED, dtype=np.uint8), axis=2)
    edge = max(6, view["scale"] * 3)          # the border and the one-metre bar
    solid[:edge, :] = solid[-edge:, :] = solid[:, :edge] = solid[:, -edge:] = False
    pointed = []
    for blob in blobs(solid):
        row = sum(r for r, _ in blob) / len(blob)
        col = sum(c for _, c in blob) / len(blob)
        answer = rover.call("drive_to_map_point",
                            {"across": col / view["pixels"],
                             "down": row / view["pixels"]})
        pointed.append((answer["pointed_at"]["x_m"],
                        answer["pointed_at"]["y_m"], answer))
    check("every obstacle drawn is one blob in the picture",
          len(pointed), len(obstacles))
    for wx, wy in obstacles:
        near = min(pointed, key=lambda p: math.hypot(p[0] - wx, p[1] - wy))
        # A cell and a half: the blob's own centroid is quantised to whole
        # pixels, and the picture is drawn at four pixels to a five-centimetre
        # cell. Anything wrong with the axes or the pose is metres out, not this.
        check(f"the obstacle at {wx:+.1f},{wy:+.1f} is where the picture puts it",
              math.hypot(near[0] - wx, near[1] - wy) < 0.08, True)
        check("...and driving onto it is refused", near[2].get("ok"), False)
        check("...as something solid", "solid" in near[2].get("error", ""), True)

    middle = rover.call("drive_to_map_point",
                        {"across": 0.5, "down": 0.5})["pointed_at"]
    check("the middle of the picture is where the rover is",
          (abs(middle["x_m"] - pose[0]) < 0.08,
           abs(middle["y_m"] - pose[1]) < 0.08), (True, True))
    check("...so pointing at it is no distance away", middle["range_m"] < 0.08, True)

    # Somewhere green, a good way out but not against the edge of what has been
    # seen, so the route is a real one rather than a nudge.
    free = np.all(image == np.array(mapimg.C_REACHABLE, dtype=np.uint8), axis=2)
    rows, cols = np.nonzero(free)
    middle_px = view["pixels"] / 2.0
    out = np.hypot(rows - middle_px, cols - middle_px)
    pick = int(np.argmin(np.abs(out - view["pixels"] * 0.3)))
    answer = rover.call("drive_to_map_point",
                        {"across": cols[pick] / view["pixels"],
                         "down": rows[pick] / view["pixels"], "speed_ms": 0.2})
    check("a green pixel is driven to", answer.get("ok"), True)
    check("...as a place on the map rather than an offset",
          sorted(k for k in asked[-1] if k != "speed_ms"), ["x_m", "y_m"])
    check("...at the point the picture named",
          (round(asked[-1]["x_m"], 2), round(asked[-1]["y_m"], 2)),
          (answer["pointed_at"]["x_m"], answer["pointed_at"]["y_m"]))
    check("...with the speed that was asked for", asked[-1]["speed_ms"], 0.2)
    check("...and says where it went, in the rover's own terms",
          sorted(answer["pointed_at"]),
          ["ahead_m", "left_m", "range_m", "x_m", "y_m"])

    # Grey needs a view wider than the rover has seen for there to be any.
    rover.call("show_map", {"across_m": 12})
    wide = rover._map_shown
    wide_png, _ = rover.nav.map_png(wide["half_extent_m"], wide["scale"])
    grey = np.all(mapimg._decode(wide_png)
                  == np.array(mapimg.C_UNKNOWN, dtype=np.uint8), axis=2)
    rows, cols = np.nonzero(grey)
    unseen = rover.call("drive_to_map_point",
                        {"across": cols[0] / wide["pixels"],
                         "down": rows[0] / wide["pixels"]})
    check("a grey pixel is refused", unseen.get("ok"), False)
    check("...as somewhere never seen rather than as somewhere empty",
          "grey" in unseen.get("error", ""), True)

    off = rover.call("drive_to_map_point", {"across": 1.4, "down": 0.5})
    check("a fraction past the edge of the picture is refused",
          off.get("ok"), False)
    check("...and is not quietly clamped to the edge instead",
          "fraction" in off.get("error", ""), True)

    rover._map_shown["at"] -= rover_daemon.MAP_POINT_MAX_AGE_S + 1
    stale = rover.call("drive_to_map_point", {"across": 0.5, "down": 0.4})
    check("a map the model is no longer looking at is refused",
          stale.get("ok"), False)
    check("...and says to take a fresh one",
          "show_map" in stale.get("error", ""), True)

    schema = next(s for s in [rover_daemon.MAP_POINT_TOOL]
                  if s["function"]["name"] == "drive_to_map_point")
    check("the model is offered the picture and not the metres",
          set(schema["function"]["parameters"]["properties"]),
          {"across", "down", "speed_ms"})
    check("...and both fractions are required",
          schema["function"]["parameters"]["required"], ["across", "down"])


def test_map_view():
    import rover_daemon

    res = 0.05

    def picture(half, pixels):
        """The size the map comes out, and the extent it covers."""
        got_half, scale = rover_daemon._map_view(half, pixels, res)
        return got_half, rover_daemon._map_cells(got_half, res) * scale

    # The whole point of deriving pixels per cell rather than asking for it: zooming
    # changes what is in frame and leaves the picture the size it was. Whole cells at
    # whole pixels cannot hit every size exactly, so this allows a few percent -- but
    # nothing like the five-fold swing you get from fixing the magnification instead.
    wanted = 480
    sizes = [picture(half, wanted)[1]
             for half in (0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)]
    check("every zoom step returns about the size asked for",
          all(abs(size - wanted) <= wanted * 0.06 for size in sizes), True)
    check("...so widening the view does not resize the picture",
          max(sizes) - min(sizes) <= 40, True)

    # Past that a cell is down to one or two whole pixels and the size cannot be held.
    # It still has to degrade rather than break: smaller, never bigger than asked.
    check("wider than the console offers still returns a sane picture",
          all(0 < picture(half, wanted)[1] <= wanted for half in (8.0, 10.0)), True)

    # Asking for a bigger picture is the separate control, and it must actually work.
    small = picture(3.0, 320)[1]
    large = picture(3.0, 800)[1]
    check("a bigger picture was asked for and is bigger", large > small * 1.8, True)
    check("...and covers the same ground",
          picture(3.0, 320)[0], picture(3.0, 800)[0])

    # Nonsense is pulled to the ends rather than refused: these are view settings, and
    # a picture at the nearest sane setting beats an error where a map should be.
    check("a negative extent lands on the floor", rover_daemon._map_view(
        -5.0, wanted, res)[0], 0.5)
    check("a huge extent lands on the ceiling", rover_daemon._map_view(
        500.0, wanted, res)[0], rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("an absurd size is capped",
          picture(3.0, 99999)[1] <= rover_daemon.MAP_MAX_PIXELS, True)
    check("a tiny size still draws something",
          picture(3.0, 1)[1] >= rover_daemon._map_cells(3.0, res), True)

    # Never below one pixel a cell, however wide: a coarse map is still a map, and
    # zero pixels per cell is an empty picture rather than a cheap one.
    for half in sorted({2.0, 6.0, rover_daemon.MAP_MAX_HALF_EXTENT_M}):
        got_half, scale = rover_daemon._map_view(half, wanted, res)
        check(f"{half:g} m across draws at least a pixel a cell", scale >= 1, True)
        check(f"...and {half:g} m is not silently narrowed", got_half, half)


TESTS = (
    test_map_png_names_the_clock,
    test_show_map_takes_across_and_size,
    test_drive_to_takes_a_place_on_the_map,
    test_a_point_on_the_map_picture_is_the_place_it_looks_like,
    test_map_view,
)
