#!/usr/bin/env python3
"""Render the occupancy grid as a PNG, using nothing but the standard library.

There is no image library on the rover's Pi -- no OpenCV, no PIL -- because the
camera hands over MJPEG already encoded and nothing here ever needed one. A
greyscale PNG is `zlib.compress` over rows with a filter byte in front of each,
which is little enough code to be worth writing rather than installing.

The rendering matters as much as the encoding. A raw 400x400 occupancy grid shown
to a vision model is a field of grey speckle that invites confident nonsense, so
what goes out is cropped to the few metres that are actually known, scaled up so a
5 cm cell is visible, and marked with the rover, its heading and a scale bar.
"""
import math
import struct
import zlib

# Greyscale, chosen so the three states are unmistakable even after JPEG-ish
# resampling somewhere downstream: solid black, near-white, and a flat mid grey.
OCCUPIED, FREE, UNKNOWN, DIM = 0, 240, 128, 176
ROVER, TRACK, SCALE = 0, 60, 0


def png_grey(rows):
    """A list of equal-length bytes-like rows -> a complete greyscale PNG."""
    height = len(rows)
    width = len(rows[0])

    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF))

    # bit depth 8, colour type 0 (greyscale), no interlace.
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    # Filter type 0 on every row: the data is blocky, so zlib does the work and a
    # cleverer filter would only cost CPU this host does not have.
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Canvas:
    """A mutable greyscale bitmap with just enough drawing to annotate a map."""

    def __init__(self, width, height, fill=UNKNOWN):
        self.w, self.h = width, height
        self.rows = [bytearray([fill]) * width for _ in range(height)]

    def put(self, x, y, value):
        if 0 <= x < self.w and 0 <= y < self.h:
            self.rows[y][x] = value

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

    def png(self):
        return png_grey(self.rows)


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

    img = np.full((nh, nw), UNKNOWN, dtype=np.uint8)
    img[shown < 0] = FREE
    img[(shown > 0) & (shown < occupied_at)] = DIM
    img[shown >= occupied_at] = OCCUPIED

    big = np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)
    canvas = Canvas(big.shape[1], big.shape[0])
    canvas.rows = [bytearray(row) for row in big]

    def to_px(wx, wy):
        """World metres -> pixel, matching the transpose and flips above."""
        gx, gy = wx / res + cells // 2, wy / res + cells // 2
        col = (y1 - 1 - gy) * scale
        row = (x1 - 1 - gx) * scale
        return col, row

    # Where the rover has been, so "go around it" can be checked afterwards.
    prev = None
    for px, py in trail:
        cur = to_px(px, py)
        if prev is not None:
            canvas.line(prev[0], prev[1], cur[0], cur[1], TRACK)
        prev = cur

    # The rover, and which way it is facing. Without this the picture has no
    # anchor and no orientation.
    rc, rr = to_px(x, y)
    canvas.disc(rc, rr, max(2.0, scale * 1.2), ROVER)
    nose = to_px(x + 0.45 * math.cos(th), y + 0.45 * math.sin(th))
    canvas.line(rc, rr, nose[0], nose[1], ROVER, thickness=max(1, scale // 2))

    # A one-metre scale bar, bottom left, and a border so the crop is visible.
    bar = int(1.0 / res) * scale
    base_y, base_x = canvas.h - max(4, scale * 2), max(4, scale * 2)
    canvas.line(base_x, base_y, base_x + bar, base_y, SCALE, thickness=2)
    canvas.line(base_x, base_y - scale, base_x, base_y + scale, SCALE)
    canvas.line(base_x + bar, base_y - scale, base_x + bar, base_y + scale, SCALE)
    canvas.rect(0, 0, canvas.w - 1, canvas.h - 1, DIM)

    seen = int((shown != 0).sum())
    solid = int((shown >= occupied_at).sum())
    description = (
        f"A top-down map of roughly {2 * half_extent_m:.0f} by "
        f"{2 * half_extent_m:.0f} metres around the rover, built from its lidar. "
        f"Forward for the rover is up the page and its left is to the left. The "
        f"black dot with the line out of it is the rover, and the line points the "
        f"way it is facing. Black is solid, near-white is space the lidar has seen "
        f"to be empty, and flat grey is unknown -- not empty. The bar at the bottom "
        f"left is one metre. {solid} cells are solid out of {seen} seen. Distances "
        f"here are good to a few centimetres locally, but the rover's own position "
        f"drifts over a long run, so use this to judge what is nearby rather than to "
        f"navigate back to somewhere it has been."
    )
    return canvas.png(), description


def _check_orientation():
    """Assert the mapped walls and the drawn trail use one convention.

    The grid arrives indexed [forward, left] and comes out as pixels twice over,
    once as an array and once through `to_px`, and for a while those two disagreed
    by a transpose: the walls were mirrored about the diagonal while the rover and
    its trail were not. Nothing caught it, because each half looks plausible alone
    and the mock rover draws both halves itself. So: a wall straight ahead and a
    trail that drove straight at it must come out as a vertical track running into
    a horizontal stripe.
    """
    import contextlib

    import numpy as np

    class _Config:
        resolution_m, grid_cells, occupied_at, rover_width_m = 0.05, 400, 20, 0.34

    class _Slam:
        config = _Config()
        lock = contextlib.nullcontext()
        pose = (2.0, 0.0, 0.0)          # 2 m along forward, facing forward

        def __init__(self):
            n = _Config.grid_cells
            self._g = np.zeros((n, n), dtype=np.int8)
            self._g[n // 2 + 60, :] = 60         # a wall across, a metre ahead

        def grid(self):
            return self._g

        def scan_xy(self):
            return []

    png, _ = render(_Slam(), half_extent_m=3.0, scale=3,
                    trail=[(cm / 100.0, 0.0) for cm in range(0, 201, 5)])

    # Decode our own PNG rather than trusting the maths that wrote it.
    body = b""
    at = 8
    while at < len(png):
        size = struct.unpack(">I", png[at:at + 4])[0]
        if png[at + 4:at + 8] == b"IHDR":
            width, height = struct.unpack(">II", png[at + 8:at + 16])
        elif png[at + 4:at + 8] == b"IDAT":
            body += png[at + 8:at + 8 + size]
        at += 12 + size
    raw = zlib.decompress(body)
    rows = [raw[r * (width + 1) + 1:(r + 1) * (width + 1)] for r in range(height)]
    img = np.array([list(r) for r in rows], dtype=np.uint8)

    track_rows, track_cols = np.nonzero(img == TRACK)
    assert len(track_cols), "no trail was drawn at all"
    assert len(set(track_cols.tolist())) <= 2, (
        f"the trail drove straight forward but spans columns "
        f"{track_cols.min()}..{track_cols.max()}, so forward is not up the page")

    # The wall is the one image row that is almost entirely solid.
    solid = [r for r in range(height) if (img[r] == OCCUPIED).sum() > width * 0.9]
    assert solid, "the wall across the map did not come out as a horizontal stripe"
    assert max(solid) < track_rows.min(), (
        f"the wall is at row {max(solid)} but the trail starts at row "
        f"{track_rows.min()}, so the wall ahead was not drawn ahead")
    print(f"orientation ok: trail up column {track_cols[0]}, wall across row "
          f"{solid[0]}, rover at row {track_rows.min()}")


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
