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
    becomes RGB and every value is a 3-tuple instead. None of the shapes below care
    which, because they all go through `_pack` and `span`.

    Written for a Pi 1, where this is the expensive part of making a map -- there is
    no drawing library, so every pixel costs an interpreted bytecode or two. Two
    things follow. Colours are packed to bytes once per call rather than once per
    pixel, because building a `bytes` from a 3-tuple per pixel was costing more than
    the PNG encoder did for the whole image. And every shape reduces to horizontal
    runs, so a run of touching pixels is one slice assignment instead of one call
    each: the border used to cost twice what compressing the picture cost.
    """

    def __init__(self, width, height, fill=UNKNOWN):
        self.w, self.h = width, height
        self.chan = 3 if isinstance(fill, (tuple, list)) else 1
        self.rows = [bytearray(self._pack(fill)) * width for _ in range(height)]

    @classmethod
    def over(cls, rows, channels):
        """A canvas that adopts `rows` as they are, for a background already built
        by something faster than this -- numpy, say. Filling a canvas and then
        throwing the fill away was pure waste on a host this slow."""
        canvas = cls.__new__(cls)
        canvas.rows = rows
        canvas.chan = channels
        canvas.h = len(rows)
        canvas.w = len(rows[0]) // channels
        return canvas

    def _pack(self, value):
        """A colour as the bytes of one pixel. Hoist this out of pixel loops."""
        if self.chan == 1:
            return bytes([value]) if isinstance(value, int) else bytes(value)
        return bytes(value)

    def span(self, y, x0, x1, packed):
        """Pixels x0..x1 inclusive on row y, clipped, in one slice assignment.
        `packed` comes from `_pack`, not a colour tuple."""
        if not 0 <= y < self.h:
            return
        x0, x1 = max(0, int(x0)), min(self.w - 1, int(x1))
        if x1 < x0:
            return
        c = self.chan
        self.rows[y][c * x0:c * (x1 + 1)] = packed * (x1 - x0 + 1)

    def put(self, x, y, value):
        self.span(int(y), int(x), int(x), self._pack(value))

    def disc(self, cx, cy, r, value):
        packed = self._pack(value)
        for y in range(int(math.floor(cy - r)), int(math.ceil(cy + r)) + 1):
            dy = y - cy
            if abs(dy) > r:
                continue
            dx = math.sqrt(max(0.0, r * r - dy * dy))
            self.span(y, math.ceil(cx - dx), math.floor(cx + dx), packed)

    def line(self, x0, y0, x1, y1, value, thickness=1):
        packed = self._pack(value)
        if thickness <= 1 and int(y0) == int(y1):
            # The common case for the furniture: one run, one assignment.
            self.span(int(y0), min(x0, x1), max(x0, x1), packed)
            return
        steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        radius = thickness / 2.0
        for i in range(steps + 1):
            t = i / steps
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            if thickness <= 1:
                self.span(int(y), int(x), int(x), packed)
            else:
                self.disc(x, y, radius, value)

    def rect(self, x0, y0, x1, y1, value):
        packed = self._pack(value)
        self.span(int(y0), x0, x1, packed)
        self.span(int(y1), x0, x1, packed)
        for y in range(int(min(y0, y1)), int(max(y0, y1)) + 1):
            self.span(y, x0, x0, packed)
            self.span(y, x1, x1, packed)

    def triangle(self, p0, p1, p2, value):
        """A filled triangle, so the rover can be an arrow rather than a dot with a
        whisker off it. A triangle is convex, so the inside of each row is one run:
        the row's span is found by intersecting it with the three edges, which is
        both faster and simpler than testing every pixel in the bounding box."""
        packed = self._pack(value)
        edges = ((p0, p1), (p1, p2), (p2, p0))
        ys = (p0[1], p1[1], p2[1])
        for y in range(int(math.floor(min(ys))), int(math.ceil(max(ys))) + 1):
            cy = y + 0.5
            crossings = []
            for (ax, ay), (bx, by) in edges:
                if (ay <= cy < by) or (by <= cy < ay):
                    crossings.append(ax + (bx - ax) * (cy - ay) / (by - ay))
            if len(crossings) >= 2:
                self.span(y, math.ceil(min(crossings) - 0.5),
                          math.floor(max(crossings) - 0.5), packed)

    def png(self):
        return _png(self.rows, 2 if self.chan == 3 else 0)


def _draw_track(image, np, points, scale):
    """Paint the rover's path into an (h, w, 3) numpy image, in one vectorised pass.

    `points` is the whole trail already in pixel coordinates. It is split into runs
    at the edges of the picture -- a rover that left the view and came back must not
    have the two visits joined by a line straight across the middle -- and each run
    is resampled at half-pixel spacing along its own arc length, which is what makes
    the cost depend on how much path is on screen and not on how many poses the
    trail happens to hold.
    """
    height, width = image.shape[:2]
    margin = 4 * scale
    runs, current = [], []
    for col, row in points:
        if -margin <= col <= width + margin and -margin <= row <= height + margin:
            current.append((col, row))
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)

    # A band exactly `thickness` pixels wide, as offsets stamped at each sample and
    # shifted back half its width so it straddles the path. Spreading a radius either
    # side instead drew a track half again as wide as it asked for, which read as a
    # different, fatter line than the rest of the picture.
    thickness = max(1, scale // 2)
    brush = [(dx, dy) for dy in range(thickness) for dx in range(thickness)]
    centre = (thickness - 1) / 2.0
    colour = np.array(C_TRACK, dtype=np.uint8)

    for run in runs:
        if len(run) < 2:
            continue
        cols = np.array([p[0] for p in run], dtype=np.float64)
        rows = np.array([p[1] for p in run], dtype=np.float64)
        along = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(cols),
                                                          np.diff(rows)))))
        if along[-1] <= 0.0:
            continue
        # One pixel between samples along the path, which is the coarsest spacing that
        # cannot leave a gap: whatever direction the line runs in, neither coordinate
        # can move by more than a whole pixel between samples.
        at = np.arange(0.0, along[-1], 1.0)
        cx, cy = np.interp(at, along, cols), np.interp(at, along, rows)
        for dx, dy in brush:
            px = (cx - centre + dx).astype(np.int32)
            py = (cy - centre + dy).astype(np.int32)
            # Masked, not clipped: a point just off the edge must be dropped, and
            # clipping would have smeared it along the border instead.
            inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
            image[py[inside], px[inside]] = colour


def render(slam, half_extent_m=3.0, scale=3, trail=(), rover_up=False):
    """The map around the rover as PNG bytes, plus what it shows.

    `half_extent_m` is how far each way to include -- a few metres, deliberately,
    rather than the whole 20 m grid: the pose drifts, so a picture that invites
    global planning is a picture that misleads. `scale` is screen pixels per cell.

    `rover_up` picks which way is up. False, the default, points the page the way the
    rover was facing when it started, so the room holds still and the arrow turns:
    right for watching where the rover has got to. True points the page the way the
    rover is facing now, so the arrow holds still and the room turns underneath it,
    which is what you want when the question is "can I get through that gap ahead".
    Neither is more correct, and a picture cannot say which it is, so the caption
    does.

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

    # Rounded, not truncated. The resolution comes out of the C config as a float32,
    # so 0.05 is really 0.050000000745 and three metres divided by it is 59.999999 --
    # which truncates to 59 and quietly loses a cell at every end of every map drawn
    # on the rover, while the caption went on claiming the extent that was asked for.
    half_cells = max(8, int(round(half_extent_m / res)))
    span = 2 * half_cells + 1
    cx = int(x / res) + cells // 2
    cy = int(y / res) + cells // 2
    # How the page is turned relative to the grid. The identity is the grid's own
    # frame, which is where the rover started.
    ahead_cos, ahead_sin = (math.cos(th), math.sin(th)) if rover_up else (1.0, 0.0)

    # Sampled rather than sliced, because a rotated view is not a slice of anything.
    # Grid axes are (ix along start-forward, iy along start-left), so with no rotation
    # the array's own first axis is already the one that runs up the page and this
    # reduces to the two flips it used to be -- forward up, left to the left, which is
    # the arrangement write_pgm uses and the one to_px below inverts.
    #
    # Sampling also fixes something slicing got wrong: a crop running off the edge of
    # the 20 m grid used to come back as a smaller picture. Off-grid now reads as
    # never-seen, which it is, and the picture is the size that was asked for.
    ahead = (half_cells - np.arange(span))[:, None]
    left = (half_cells - np.arange(span))[None, :]
    gx = np.rint(cx + ahead * ahead_cos - left * ahead_sin).astype(np.int32)
    gy = np.rint(cy + ahead * ahead_sin + left * ahead_cos).astype(np.int32)
    on_grid = (gx >= 0) & (gx < cells) & (gy >= 0) & (gy < cells)
    shown = np.zeros((span, span), dtype=np.int8)
    shown[on_grid] = np.asarray(grid)[gx[on_grid], gy[on_grid]]

    # Occupancy as three RGB planes. The state -> colour step happens here, on the
    # small array, rather than per pixel after the scale-up.
    rgb = np.empty(shown.shape + (3,), dtype=np.uint8)
    rgb[...] = C_UNKNOWN
    rgb[shown < 0] = C_FREE
    rgb[(shown > 0) & (shown < occupied_at)] = C_DIM
    rgb[shown >= occupied_at] = C_OCCUPIED

    big = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)

    def to_px(wx, wy):
        """World metres -> pixel, the exact inverse of the sampling above."""
        dgx = wx / res + cells // 2 - cx
        dgy = wy / res + cells // 2 - cy
        forward = dgx * ahead_cos + dgy * ahead_sin
        sideways = -dgx * ahead_sin + dgy * ahead_cos
        return (half_cells - sideways) * scale, (half_cells - forward) * scale

    # Where the rover has been, so "go around it" can be checked afterwards. Drawn
    # thick enough to survive against a busy background, since a one-pixel track over
    # speckle is the thing that was hardest to follow in grey.
    #
    # Painted into the numpy array before the canvas exists, which is the one place in
    # here worth departing from drawing through the canvas. The trail holds up to 4000
    # poses 5 cm apart, so a rover that has pottered around one room for an afternoon
    # has 200 m of path to draw; walked a pixel at a time in Python that was 17
    # seconds of a 19-second map, and it grew for as long as the session lasted.
    # Thinning the points does not fix it, because the cost is the length of the line
    # and not the number of corners in it -- thinning only turns curves into chords,
    # and measured, it bought 20%. Resampling each run along its own arc length and
    # writing the pixels in one indexed assignment costs tens of milliseconds, and
    # stops depending on how long the rover has been driving at all.
    _draw_track(big, np, [to_px(*point) for point in trail], scale)

    # From here on the drawing is small and irregular -- an arrow, a bar, a border --
    # which is what the canvas is for.
    canvas = Canvas.over([bytearray(row.tobytes()) for row in big], 3)

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
    # Which way is up has to be said, and said exactly. The old wording claimed the
    # rover's forward was up the page, which was only ever true of the heading it
    # started with -- so a model reading it after any turn was being told the room lay
    # in a direction it did not.
    if rover_up:
        orientation = ("Up the page is the direction the rover is facing right now, "
                       "so straight ahead of it is straight up, its left is to the "
                       "left, and the room turns in the picture as the rover turns.")
    else:
        orientation = ("Up the page is the direction the rover was facing when it "
                       "started, not the way it is facing now -- the room holds still "
                       "and the rover turns within it, so read the arrow to see which "
                       "way it is pointing.")
    description = (
        f"A top-down map of roughly {2 * half_extent_m:.0f} by "
        f"{2 * half_extent_m:.0f} metres around the rover, built from its lidar. "
        f"{orientation} The red triangle is the rover and its tip points the way the "
        f"rover is facing, with a yellow dot at its exact position; the blue line is "
        f"the path it has driven. Black is solid, near-white is space the lidar has "
        f"seen to be empty, sandy beige is seen but not confirmed solid, and flat grey "
        f"is unknown -- not empty. The bar at the bottom "
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


def _render_probe(heading, wall_axis="ahead", rover_up=False):
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
                          trail=[(cm / 100.0, 0.0) for cm in range(0, 201, 5)],
                          rover_up=rover_up)
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

    Then the same again for `rover_up`, where the page turns with the rover instead.
    A rotation is exactly the sort of thing that looks entirely plausible while being
    ninety degrees or a mirror out, so it is checked against a wall whose real
    bearing is known: with the wall dead ahead, turning the rover a quarter turn left
    must swing that wall to the right of the picture and not to the left.
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

    # rover_up: the page turns with the rover. Where the one wall lands says which
    # way, and it has to land somewhere different for each quarter turn.
    def wall_at(heading):
        got, _ = _render_probe(heading, rover_up=True)
        occ = _mask(got, C_OCCUPIED)
        h, w = occ.shape
        rows = [r for r in range(h) if occ[r].sum() > w * 0.7]
        cols = [c for c in range(w) if occ[:, c].sum() > h * 0.7]
        if rows:
            return "above" if rows[0] < h // 2 else "below"
        if cols:
            return "left" if cols[0] < w // 2 else "right"
        return "nowhere"

    for heading, want in ((0.0, "above"), (math.pi / 2, "right"),
                          (math.pi, "below"), (-math.pi / 2, "left")):
        got = wall_at(heading)
        assert got == want, (
            f"with the rover facing {math.degrees(heading):+.0f} deg and rover_up on, "
            f"a wall that is really straight ahead of the start heading was drawn "
            f"{got}, not {want}")

    # And with the page turned, the arrow cannot be: ahead is up by construction.
    turned, _ = _render_probe(math.pi / 2, rover_up=True)
    rows, cols = np.nonzero(_mask(turned, C_ROVER))
    anchor = [v.mean() for v in np.nonzero(_mask(turned, C_ANCHOR))]
    far = np.argmax((rows - anchor[0]) ** 2 + (cols - anchor[1]) ** 2)
    point = (cols[far] - anchor[1], rows[far] - anchor[0])
    assert point[1] < -2 and abs(point[0]) < 3, (
        f"with rover_up on the arrow must always point up the page, but it points "
        f"{point}")

    up_caption = _render_probe(0.0, rover_up=True)[1]
    assert "facing right now" in up_caption and "facing right now" not in caption, (
        "the caption does not distinguish the two orientations, which is the only "
        "way anything reading the picture can tell them apart")
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
