#!/usr/bin/env python3
"""Render the occupancy grid as a PNG, using nothing but the standard library.

There is no image library on the rover's Pi -- no OpenCV, no PIL -- because the
camera hands over MJPEG already encoded and nothing here ever needed one. A PNG is
`zlib.compress` over rows with a filter byte in front of each, which is little
enough code to be worth writing rather than installing.

The rendering matters as much as the encoding. A raw 400x400 occupancy grid shown
to a vision model is a field of grey speckle that invites confident nonsense, so
what goes out is cropped to the few metres that are actually known, scaled up so a
5 cm cell is visible, and marked with the rover, its heading and a scale bar.

The map comes out in colour and the encoder still does greyscale, because the two
have different readers. Occupancy is naturally a lightness ramp from solid to
empty, so what is drawn *over* it -- where the rover is, which way it points,
where it has been -- had nowhere to go but more shades of the same ramp, and the
two that matter most ended up as dark pixels on dark obstacles. Colour gives them
somewhere to live. `png_grey` stays because the encoder self-check and anything
dumping a bare occupancy grid have no overlay to distinguish.
"""
import math
import struct
import zlib

# Greyscale, chosen so the three states are unmistakable even after JPEG-ish
# resampling somewhere downstream: solid black, near-white, and a flat mid grey.
OCCUPIED, FREE, UNKNOWN, DIM = 0, 240, 128, 176
ROVER, TRACK, SCALE = 0, 60, 0

# The same map in colour, for the human watching drive_console. Greyscale asked the
# reader to tell four shades apart, and the two that matter most -- where the rover
# has been and where it is now -- were the two hardest, because both were dark
# pixels drawn over dark obstacles. So hue carries what is drawn on top and
# lightness is left to carry the occupancy underneath: solid to empty keeps its
# black-to-white ramp, and nothing overlaid on it is a shade that ramp contains.
C_OCCUPIED = (24, 24, 28)           # solid, and still the darkest thing here
C_FREE = (247, 246, 242)            # seen to be empty
C_UNKNOWN = (129, 132, 138)         # never seen -- not empty
C_DIM = (196, 186, 164)             # seen, but not enough times to call solid
C_TRACK = (36, 116, 232)            # where it has been
C_ROVER = (222, 46, 46)             # where it is, and which way it points
C_ANCHOR = (250, 236, 120)          # the exact pose, inside the arrow
C_SCALE = (24, 24, 28)              # the one-metre bar
C_BORDER = (150, 150, 156)          # the edge of the crop


def _png(rows, colour):
    """Rows of packed samples -> a complete PNG. `colour` is the PNG colour type:
    0 for one grey byte per pixel, 2 for three RGB bytes."""
    height = len(rows)
    width = len(rows[0]) // (3 if colour == 2 else 1)

    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF))

    # bit depth 8, no interlace.
    ihdr = struct.pack(">IIBBBBB", width, height, 8, colour, 0, 0, 0)
    # Filter type 0 on every row: the data is blocky, so zlib does the work and a
    # cleverer filter would only cost CPU this host does not have.
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def png_grey(rows):
    """A list of equal-length bytes-like rows -> a complete greyscale PNG."""
    return _png(rows, 0)


def png_rgb(rows):
    """Rows of 3*width bytes, R then G then B -> a complete truecolour PNG."""
    return _png(rows, 2)


class Canvas:
    """A mutable bitmap with just enough drawing to annotate a map.

    Greyscale by default, so a value is one byte; pass a 3-tuple as `fill` and it
    becomes RGB and every value is a 3-tuple instead. None of the shapes below
    care which, because they all go through `put`.
    """

    def __init__(self, width, height, fill=UNKNOWN):
        self.w, self.h = width, height
        self.chan = 3 if isinstance(fill, (tuple, list)) else 1
        pixel = bytes(fill) if self.chan == 3 else bytes([fill])
        self.rows = [bytearray(pixel) * width for _ in range(height)]

    def put(self, x, y, value):
        if 0 <= x < self.w and 0 <= y < self.h:
            if self.chan == 1:
                self.rows[y][x] = value
            else:
                self.rows[y][3 * x:3 * x + 3] = bytes(value)

    def disc(self, cx, cy, r, value):
        for y in range(int(cy - r), int(cy + r) + 1):
            for x in range(int(cx - r), int(cx + r) + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    self.put(x, y, value)

    def line(self, x0, y0, x1, y1, value, thickness=1):
        steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for i in range(steps + 1):
            t = i / steps
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            if thickness <= 1:
                self.put(int(x), int(y), value)
            else:
                self.disc(x, y, thickness / 2.0, value)

    def rect(self, x0, y0, x1, y1, value):
        self.line(x0, y0, x1, y0, value)
        self.line(x0, y1, x1, y1, value)
        self.line(x0, y0, x0, y1, value)
        self.line(x1, y0, x1, y1, value)

    def triangle(self, p0, p1, p2, value):
        """A filled triangle, so the rover can be an arrow rather than a dot with a
        whisker off it. Each pixel centre is tested against the three edge
        functions and kept if it is on the same side of all of them, which accepts
        either winding and needs no sorting of the vertices."""
        xs, ys = (p0[0], p1[0], p2[0]), (p0[1], p1[1], p2[1])

        def side(a, b, px, py):
            return (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])

        for y in range(int(math.floor(min(ys))), int(math.ceil(max(ys))) + 1):
            for x in range(int(math.floor(min(xs))), int(math.ceil(max(xs))) + 1):
                cx, cy = x + 0.5, y + 0.5
                d = (side(p0, p1, cx, cy), side(p1, p2, cx, cy), side(p2, p0, cx, cy))
                if all(v >= 0 for v in d) or all(v <= 0 for v in d):
                    self.put(x, y, value)

    def png(self):
        return _png(self.rows, 2 if self.chan == 3 else 0)


def render(slam, half_extent_m=3.0, scale=3, trail=()):
    """The map around the rover as PNG bytes, plus what it shows.

    `half_extent_m` is how far each way to include -- a few metres, deliberately,
    rather than the whole 20 m grid: the pose drifts, so a picture that invites
    global planning is a picture that misleads. `scale` is screen pixels per cell.

    Returns (png_bytes, description) where description says what the picture is,
    because a model shown an unlabelled top-down grid has no way to know the
    orientation, the scale, or that grey means unknown.
    """
    import numpy as np

    with slam.lock:
        grid = slam.grid()
        x, y, th = slam.pose
        res = slam.config.resolution_m
        cells = slam.config.grid_cells
        occupied_at = slam.config.occupied_at
        scan = slam.scan_xy()

    half_cells = max(8, int(half_extent_m / res))
    cx = int(x / res) + cells // 2
    cy = int(y / res) + cells // 2
    x0, x1 = max(0, cx - half_cells), min(cells, cx + half_cells + 1)
    y0, y1 = max(0, cy - half_cells), min(cells, cy + half_cells + 1)
    sub = np.asarray(grid[x0:x1, y0:y1])

    # Grid axes are (ix along start-forward, iy along start-left), so the array's
    # own first axis is already the one that should run up the page. A plan view
    # wants forward up and left to the left, which is therefore two flips and no
    # transpose -- the same arrangement write_pgm uses, and the one to_px below
    # inverts. A transpose here as well would reflect the walls about the diagonal
    # while leaving the rover and its trail alone, which is how they came to
    # disagree.
    shown = np.flipud(np.fliplr(sub))
    nh, nw = shown.shape

    # Occupancy as three RGB planes. The state -> colour step happens here, on the
    # small array, rather than per pixel after the scale-up.
    rgb = np.empty(shown.shape + (3,), dtype=np.uint8)
    rgb[...] = C_UNKNOWN
    rgb[shown < 0] = C_FREE
    rgb[(shown > 0) & (shown < occupied_at)] = C_DIM
    rgb[shown >= occupied_at] = C_OCCUPIED

    big = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    canvas = Canvas(big.shape[1], big.shape[0], C_UNKNOWN)
    canvas.rows = [bytearray(row.tobytes()) for row in big]

    def to_px(wx, wy):
        """World metres -> pixel, matching the flips above."""
        gx, gy = wx / res + cells // 2, wy / res + cells // 2
        col = (y1 - 1 - gy) * scale
        row = (x1 - 1 - gx) * scale
        return col, row

    # Where the rover has been, so "go around it" can be checked afterwards. Drawn
    # thick enough to survive against a busy background, since a one-pixel track
    # over speckle is the thing that was hardest to follow in grey.
    prev = None
    for px, py in trail:
        cur = to_px(px, py)
        if prev is not None:
            canvas.line(prev[0], prev[1], cur[0], cur[1], C_TRACK,
                        thickness=max(1, scale // 2))
        prev = cur

    # Where the rover is and which way it points, as one arrow. A disc with a
    # whisker off it read badly at this scale -- at three pixels per cell the
    # whisker is two pixels wide and its direction has to be guessed -- so the
    # rover is a triangle roughly its own footprint, tip forward. The exact pose is
    # a single bright cell inside it, because the triangle covers about 30 cm and
    # "where is it, to the centimetre" is a different question from "which way".
    forward, side = (math.cos(th), math.sin(th)), (-math.sin(th), math.cos(th))

    def offset(along, across):
        return to_px(x + forward[0] * along + side[0] * across,
                     y + forward[1] * along + side[1] * across)

    canvas.triangle(offset(0.30, 0.0), offset(-0.15, 0.16), offset(-0.15, -0.16),
                    C_ROVER)
    rc, rr = to_px(x, y)
    canvas.disc(rc, rr, max(1.0, scale * 0.5), C_ANCHOR)

    # A one-metre scale bar, bottom left, and a border so the crop is visible.
    bar = int(1.0 / res) * scale
    base_y, base_x = canvas.h - max(4, scale * 2), max(4, scale * 2)
    canvas.line(base_x, base_y, base_x + bar, base_y, C_SCALE, thickness=2)
    canvas.line(base_x, base_y - scale, base_x, base_y + scale, C_SCALE)
    canvas.line(base_x + bar, base_y - scale, base_x + bar, base_y + scale, C_SCALE)
    canvas.rect(0, 0, canvas.w - 1, canvas.h - 1, C_BORDER)

    seen = int((shown != 0).sum())
    solid = int((shown >= occupied_at).sum())
    description = (
        f"A top-down map of roughly {2 * half_extent_m:.0f} by "
        f"{2 * half_extent_m:.0f} metres around the rover, built from its lidar. "
        f"Forward for the rover is up the page and its left is to the left. The red "
        f"triangle is the rover and its tip points the way the rover is facing, with "
        f"a yellow dot at its exact position; the blue line is the path it has "
        f"driven. Black is solid, near-white is space the lidar has seen to be "
        f"empty, sandy beige is seen but not confirmed solid, and flat grey is "
        f"unknown -- not empty. The bar at the bottom "
        f"left is one metre. {solid} cells are solid out of {seen} seen. Distances "
        f"here are good to a few centimetres locally, but the rover's own position "
        f"drifts over a long run, so use this to judge what is nearby rather than to "
        f"navigate back to somewhere it has been."
    )
    return canvas.png(), description


def _decode(png):
    """Our own PNG back to an (h, w, 3) array, so the checks below read the picture
    that shipped rather than the maths that wrote it."""
    import numpy as np

    body, at = b"", 8
    width = height = None
    while at < len(png):
        size = struct.unpack(">I", png[at:at + 4])[0]
        kind = png[at + 4:at + 8]
        if kind == b"IHDR":
            width, height = struct.unpack(">II", png[at + 8:at + 16])
        elif kind == b"IDAT":
            body += png[at + 8:at + 8 + size]
        at += 12 + size
    raw = zlib.decompress(body)
    stride = width * 3 + 1
    rows = [raw[r * stride + 1:(r + 1) * stride] for r in range(height)]
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(height, width, 3)


def _render_probe(heading, wall_axis="ahead"):
    """`render` over a synthetic room: one wall and a track that drove forward."""
    import contextlib

    import numpy as np

    class _Config:
        resolution_m, grid_cells, occupied_at, rover_width_m = 0.05, 400, 20, 0.34

    class _Slam:
        config = _Config()
        lock = contextlib.nullcontext()

        def __init__(self):
            n = _Config.grid_cells
            self._g = np.zeros((n, n), dtype=np.int8)
            if wall_axis == "ahead":
                self._g[n // 2 + 60, :] = 60     # across the map, a metre ahead
            else:
                self._g[:, n // 2 + 60] = 60     # down the map, a metre to the left
            self.pose = (2.0, 0.0, heading)

        def grid(self):
            return self._g

        def scan_xy(self):
            return []

    png, caption = render(_Slam(), half_extent_m=3.0, scale=3,
                          trail=[(cm / 100.0, 0.0) for cm in range(0, 201, 5)])
    return _decode(png), caption


def _mask(img, colour):
    import numpy as np

    return np.all(img == np.array(colour, dtype=np.uint8), axis=-1)


def _check_orientation():
    """Assert the mapped walls and everything drawn over them use one convention.

    The grid arrives indexed [forward, left] and becomes pixels twice over, once as
    an array and once through `to_px`, and for a while those two disagreed by a
    transpose: the walls came out mirrored about the diagonal while the rover and
    its track did not, so a run down a corridor was drawn crossing it. Each half
    looks plausible alone and the mock rover draws both halves with one function of
    its own, so only the real map ever showed it.

    Three claims, then: a wall straight ahead is a horizontal stripe above the
    rover; a track that drove straight forward is one vertical line; and the arrow
    turns the way the heading says, counter-clockwise for a left turn.
    """
    import numpy as np

    img, caption = _render_probe(0.0)
    height, width = img.shape[:2]

    track_rows, track_cols = np.nonzero(_mask(img, C_TRACK))
    assert len(track_cols), "no track was drawn at all"
    # A few pixels of slack: the track is drawn thick, so a straight run is a band
    # rather than a single column.
    assert track_cols.max() - track_cols.min() <= 4, (
        f"the track drove straight forward but spans columns "
        f"{track_cols.min()}..{track_cols.max()}, so forward is not up the page")

    occupied = _mask(img, C_OCCUPIED)
    solid = [r for r in range(height) if occupied[r].sum() > width * 0.9]
    assert solid, "the wall across the map did not come out as a horizontal stripe"
    assert max(solid) < track_rows.min(), (
        f"the wall is at row {max(solid)} but the track starts at row "
        f"{track_rows.min()}, so the wall ahead was not drawn ahead")

    # A wall to the left must be a vertical stripe on the left, which is the half of
    # the convention a square crop and a forward-only track cannot distinguish.
    side_img, _ = _render_probe(0.0, wall_axis="left")
    side = _mask(side_img, C_OCCUPIED)
    columns = [c for c in range(width) if side[:, c].sum() > height * 0.9]
    assert columns, "the wall down the map did not come out as a vertical stripe"
    assert max(columns) < width // 2, (
        f"a wall to the rover's left was drawn at column {max(columns)} of {width}, "
        f"so left is not to the left")

    # The arrow: its tip is the extreme red pixel away from the pose, and turning
    # left must swing it counter-clockwise, which on screen is towards -x.
    def tip(heading):
        got, _ = _render_probe(heading)
        rows, cols = np.nonzero(_mask(got, C_ROVER))
        assert len(rows), f"no arrow was drawn at heading {heading}"
        anchor_r, anchor_c = [v.mean() for v in np.nonzero(_mask(got, C_ANCHOR))]
        far = np.argmax((rows - anchor_r) ** 2 + (cols - anchor_c) ** 2)
        return cols[far] - anchor_c, rows[far] - anchor_r

    ahead, left = tip(0.0), tip(math.pi / 2)
    assert ahead[1] < -2 and abs(ahead[0]) < 3, (
        f"at heading 0 the arrow points {ahead}, not up the page")
    assert left[0] < -2 and abs(left[1]) < 3, (
        f"at heading +90 deg the arrow points {left}, not to the left")

    assert "red triangle" in caption and "blue line" in caption, (
        "the caption no longer names the colours it is explaining")
    print(f"orientation ok: track up column {track_cols[0]}, wall across row "
          f"{solid[0]}, arrow {ahead} ahead and {left} turned left")


if __name__ == "__main__":
    # A synthetic check that needs no rover: a box with a gap, so the geometry and
    # the encoder can be eyeballed without the hardware, and an assertion that the
    # two halves of `render` agree about which way is forward.
    import sys

    _check_orientation()

    c = Canvas(160, 120, UNKNOWN)
    for i in range(20, 140):
        c.put(i, 20, OCCUPIED)
        if not 70 < i < 95:
            c.put(i, 100, OCCUPIED)
    for j in range(20, 101):
        c.put(20, j, OCCUPIED)
        c.put(139, j, OCCUPIED)
    c.disc(80, 60, 4, ROVER)
    c.line(80, 60, 80, 44, ROVER, thickness=2)
    out = sys.argv[1] if len(sys.argv) > 1 else "mapimg-selftest.png"
    with open(out, "wb") as f:
        f.write(c.png())
    print(f"wrote {out} ({len(c.png())} bytes)")
