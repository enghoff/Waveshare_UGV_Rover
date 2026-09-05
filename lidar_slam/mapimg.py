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
somewhere to live. Empty floor the rover can actually reach from where it stands
is green rather than cream, so a room behind a wall does not read as a place to
go. `png_grey` stays because the encoder self-check and anything dumping a bare
occupancy grid have no overlay to distinguish.
"""
import math
import struct
import zlib

# Greyscale, chosen so the three states are unmistakable even after JPEG-ish
# resampling somewhere downstream: solid black, near-white, and a flat mid grey.
OCCUPIED, FREE, UNKNOWN, DIM = 0, 240, 128, 176
ROVER, TRACK, SCALE = 0, 60, 0

# The same map in colour, for the human watching the drive console. Greyscale asked the
# reader to tell four shades apart, and the two that matter most -- where the rover
# has been and where it is now -- were the two hardest, because both were dark
# pixels drawn over dark obstacles. So hue carries what is drawn on top and
# lightness is left to carry the occupancy underneath: solid to empty keeps its
# black-to-white ramp, and nothing overlaid on it is a shade that ramp contains.
C_OCCUPIED = (24, 24, 28)           # solid, and still the darkest thing here
C_FREE = (247, 246, 242)            # seen to be empty, but not reachable from here
C_REACHABLE = (58, 162, 90)         # empty, and the rover can get there from here
C_UNKNOWN = (129, 132, 138)         # never seen -- not empty
C_DIM = (196, 186, 164)             # seen, but not enough times to call solid
C_TRACK = (36, 116, 232)            # where it has been
C_ROVER = (222, 46, 46)             # where it is, and which way it points
C_ANCHOR = (250, 236, 120)          # the exact pose, inside the arrow
C_SCALE = (24, 24, 28)              # the one-metre bar
C_BORDER = (150, 150, 156)          # the edge of the crop
C_CAMERA = (150, 80, 210)           # where the camera is looking, and how wide

# The step between two poses in the track that is too far to have been driven, so
# the line is broken rather than drawn through it. The poses are half a second
# apart and this chassis does 0.35 m/s flat out, which is 18 cm; a metre is well
# clear of that and well under the metres a frame moves by when a cleared map
# re-anchors or a loop closure bends the graph.
TRACK_BREAK_M = 1.0

# How much of the crop the camera's cone reaches across, and how finely its far edge
# is drawn. Reaching the edge rather than a fixed number of metres means the cone
# says the same thing at every zoom -- it is a direction and a width, not a range,
# and the camera can see a good deal further than any of these maps are wide.
CAMERA_REACH = 0.95
CAMERA_ARC_DEG = 6.0                # one segment per this much of the arc
# How strongly the cone is washed over the map underneath it. A quarter is enough
# to read as one lit area rather than as three violet lines, and light enough that
# the occupancy under it -- which is the part the cone exists to point at -- is
# still legible through it. The outline is drawn over the wash at full strength, so
# where the cone ends stays exact whatever the fill sits on.
CAMERA_FILL = 0.25


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

    def wash_tables(self, value, alpha):
        """Translation tables for :meth:`wash`: `value` laid over whatever is there,
        at `alpha` from 0 to 1. One per channel, and the caller builds them once for
        a whole shape rather than once per row."""
        return tuple(bytes(round(alpha * component + (1.0 - alpha) * under)
                           for under in range(256))
                     for component in self._pack(value))

    def wash(self, y, x0, x1, tables):
        """Blend a colour into pixels x0..x1 of row y, clipped.

        A translucent fill has to read what is underneath it, so this cannot be
        `span`'s assignment of a constant run. It is still not a per-pixel Python
        loop, which on a Pi 1 is what the difference between a map and a slideshow
        is made of: with the colour and the fraction both fixed, every output byte
        depends only on the byte it replaces, and that is a 256-entry table per
        channel with `translate` applying it in C. The channels interleave, so each
        one is a strided slice of the row.
        """
        if not 0 <= y < self.h:
            return
        x0, x1 = max(0, int(x0)), min(self.w - 1, int(x1))
        if x1 < x0:
            return
        row, c = self.rows[y], self.chan
        lo, hi = c * x0, c * (x1 + 1)
        for i, table in enumerate(tables):
            row[lo + i:hi:c] = row[lo + i:hi:c].translate(table)

    def wash_polygon(self, points, value, alpha):
        """A polygon washed over the map rather than painted on top of it.

        Rows are filled between sorted pairs of edge crossings, which is
        `triangle`'s method generalised: pairing them rather than taking the
        outermost two means a shape that is not convex still fills correctly, and a
        wedge of more than half a turn is not convex. Each row is washed once, so a
        shared edge inside the shape cannot show up as a double-blended seam.
        """
        tables = self.wash_tables(value, alpha)
        ys = [p[1] for p in points]
        edges = list(zip(points, points[1:] + points[:1]))
        for y in range(int(math.floor(min(ys))), int(math.ceil(max(ys))) + 1):
            cy = y + 0.5
            crossings = []
            for (ax, ay), (bx, by) in edges:
                if (ay <= cy < by) or (by <= cy < ay):
                    crossings.append(ax + (bx - ax) * (cy - ay) / (by - ay))
            crossings.sort()
            for i in range(0, len(crossings) - 1, 2):
                self.wash(y, math.ceil(crossings[i] - 0.5),
                          math.floor(crossings[i + 1] - 0.5), tables)

    def png(self):
        return _png(self.rows, 2 if self.chan == 3 else 0)


def _draw_track(image, np, points, scale, break_px=None):
    """Paint the rover's path into an (h, w, 3) numpy image, in one vectorised pass.

    `points` is the whole trail already in pixel coordinates. It is split into runs
    and each run is resampled at half-pixel spacing along its own arc length, which
    is what makes the cost depend on how much path is on screen and not on how many
    poses the trail happens to hold.

    Two things end a run. The edges of the picture: a rover that left the view and
    came back must not have the two visits joined by a line straight across the
    middle. And `break_px`, a step no drive could have made -- because the poses
    are half a second apart on a chassis that does 0.35 m/s, so anything past a
    metre is the coordinates having moved rather than the rover. That happens: a
    cleared map re-anchors the frame, and a loop closure bends it. Drawn through,
    it reads as a drive the rover never made, which on 2026-09-04 was a 5.37 m line
    out of the room and across open grey. Left as a gap, it reads as what it is.
    """
    height, width = image.shape[:2]
    margin = 4 * scale
    runs, current = [], []
    for col, row in points:
        if not (-margin <= col <= width + margin
                and -margin <= row <= height + margin):
            if current:
                runs.append(current)
                current = []
            continue
        if (current and break_px is not None
                and math.hypot(col - current[-1][0],
                               row - current[-1][1]) > break_px):
            runs.append(current)
            current = []
        current.append((col, row))
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


def reachable_free(shown, origin):
    """Free cells 4-connected to the rover, as a boolean mask the same shape as `shown`.

    Free is `shown < 0`, the coding `render` already uses. Occupied, dim and
    unseen do not transmit, so a room the lidar has seen as empty but that sits
    behind a wall stays cream rather than turning green.

    4-connected rather than 8: a one-cell diagonal crack in a wall is a common
    occupancy-grid artefact, and treating it as a doorway would paint the far
    room reachable when nothing the rover's width can fit through is actually
    there. The rover's own cell need not be free -- it is often unseen or dim
    at the start of a run -- so the flood begins at `origin` and walks *onto*
    free cells, rather than refusing to start.
    """
    import numpy as np

    walkable = shown < 0
    height, width = walkable.shape
    reach = np.zeros((height, width), dtype=bool)
    row, col = origin
    if not (0 <= row < height and 0 <= col < width):
        return reach
    if walkable[row, col]:
        reach[row, col] = True
    else:
        for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nrow, ncol = row + drow, col + dcol
            if 0 <= nrow < height and 0 <= ncol < width and walkable[nrow, ncol]:
                reach[nrow, ncol] = True
    if not reach.any():
        return reach
    # Dilate through walkable cells until the front stops moving. Each pass is
    # four boolean ORs on the whole crop, so a 12 m view is tens of milliseconds
    # rather than a Python walk of every cell -- and the iteration count is the
    # longest 4-path in the picture, not the number of free cells.
    for _ in range(height + width):
        grown = reach.copy()
        grown[1:, :] |= reach[:-1, :]
        grown[:-1, :] |= reach[1:, :]
        grown[:, 1:] |= reach[:, :-1]
        grown[:, :-1] |= reach[:, 1:]
        nxt = grown & walkable
        if nxt.sum() == reach.sum():
            return nxt
        reach = nxt
    return reach


def colour_occupancy(shown, occupied_at, origin):
    """The occupancy crop as an (h, w, 3) uint8 picture, reachable floor in green.

    `origin` is the rover's cell in `shown`, row then column -- the centre of the
    crop `render` samples, which is where the rover is. Shared with the mock
    rover so a console looking at an invented room sees the same palette as the
    real map, including which empty cells are green.
    """
    import numpy as np

    rgb = np.empty(shown.shape + (3,), dtype=np.uint8)
    rgb[...] = C_UNKNOWN
    rgb[shown < 0] = C_FREE
    rgb[(shown > 0) & (shown < occupied_at)] = C_DIM
    rgb[shown >= occupied_at] = C_OCCUPIED
    rgb[reachable_free(shown, origin)] = C_REACHABLE
    return rgb


def known_box(slam):
    """Where the map is, as (x0, y0, x1, y1) in map metres, or None for none of it.

    The bounding box of every cell the rover has an opinion about -- free, dim or
    occupied -- with never-seen left out. It is what a reader should frame a view
    on: the grid is forty metres square and a room is a few, so a picture of the
    whole grid is mostly a picture of nothing.

    **Read off the grid and not off the picture, which is the whole point.** The
    camera cone, the track and the scale bar are drawn *over* the occupancy by
    `render`, and the cone in particular reaches several metres into a part of the
    room nobody has been in. A reader that measured the rendered pixels would
    frame on that wedge and push the map itself into a corner.

    Metres in the grid's own frame, which is where the rover started and the frame
    every position in the world state is recorded in -- so a caller converts it
    with the same `to_px` inverse it uses for everything else.
    """
    import numpy as np

    with slam.lock:
        grid = np.asarray(slam.grid())
        res = slam.config.resolution_m
        cells = slam.config.grid_cells
    if grid.size == 0:
        return None
    # Zero is never-seen: `render` codes free negative and occupied positive, and
    # the square is allocated as zeros and painted into. So this is "not zero"
    # rather than a threshold, and it must stay that way -- `>= 0` would count
    # every unseen cell in a forty-metre square as known.
    known = grid != 0
    rows = np.flatnonzero(known.any(axis=1))
    if not rows.size:
        return None
    cols = np.flatnonzero(known.any(axis=0))
    half = cells // 2
    return (float((rows[0] - half) * res), float((cols[0] - half) * res),
            float((rows[-1] + 1 - half) * res), float((cols[-1] + 1 - half) * res))


def camera_caption(bearing_deg, fov_deg):
    """The sentence that tells a reader what the violet wedge is.

    Here rather than inline in `render` because the mock rover draws the same cone
    over its invented room and has to describe it the same way -- a mock whose
    caption differed would be a mock of a different rover, which is the same reason
    it borrows the palette and the drawing.
    """
    side = "left" if bearing_deg > 0 else "right"
    where = ("straight ahead" if abs(bearing_deg) < 1.0
             else f"{abs(bearing_deg):.0f} degrees to the rover's {side}")
    return (f"The violet wedge is where the camera is pointing and how much of the "
            f"room is in shot -- about {fov_deg:.0f} degrees wide, centred {where}. "
            f"Everything the rover can photograph is inside it, and everything "
            f"outside it is known to the lidar only.")


def draw_camera(canvas, to_px, x, y, heading_rad, bearing_deg, fov_deg, reach_m):
    """The gimbal's cone: which way the camera is looking, and how much it takes in.

    The map is what the lidar knows and the camera is the other sensor entirely, so
    without this there is nothing in the picture to say which part of the room the
    photographs are of. The two point in different directions most of the time --
    the gimbal pans a long way either side and sweeps continuously while face
    tracking runs -- and the rover's own arrow says nothing about where the camera
    got to.

    Filled at `CAMERA_FILL` and then outlined at full strength. The fill is what
    makes it read as one lit area rather than as three unrelated violet lines, and
    a quarter is as far as it can go: the interesting part of the map is precisely
    the part inside the cone, so a heavier wash would hide what the cone is there to
    point at. The outline goes on top so that where the shot ends stays exact
    whatever the fill lands on. In a hue outside the black-to-white occupancy ramp
    for the same reason the rover and its track are: anything neutral drawn over the
    map reads as more map.

    `bearing_deg` is in the rover's own frame, counter-clockwise from its nose, and
    the conversion is the caller's. The gimbal counts pan positive to the *right*
    while everything here counts positive to the *left*, so a camera panned to `p`
    is looking along `-p`; a sign error there is invisible, because what comes out
    is a perfectly ordinary cone aimed at the wrong half of the room.
    """
    half = math.radians(fov_deg) / 2.0
    centre = heading_rad + math.radians(bearing_deg)
    origin = to_px(x, y)

    def at(angle):
        return to_px(x + reach_m * math.cos(angle), y + reach_m * math.sin(angle))

    # The apex and the far edge as one polygon, segmented finely enough that the arc
    # does not read as a chord. The same points serve the fill and the outline drawn
    # over it, so the two cannot disagree about where the cone ends.
    steps = max(4, int(fov_deg / CAMERA_ARC_DEG))
    arc = [at(centre - half + 2.0 * half * step / steps) for step in range(steps + 1)]
    canvas.wash_polygon([origin] + arc, C_CAMERA, CAMERA_FILL)

    for edge in (arc[0], arc[-1]):
        canvas.line(origin[0], origin[1], edge[0], edge[1], C_CAMERA)
    # The far edge, as a polyline. It closes the shape, which is what makes it read
    # as a cone rather than as two unrelated lines leaving the rover.
    for previous, point in zip(arc, arc[1:]):
        canvas.line(previous[0], previous[1], point[0], point[1], C_CAMERA)


def render(slam, half_extent_m=3.0, scale=3, trail=(), rover_up=False, camera=None):
    """The map around the rover as PNG bytes, plus what it shows.

    `half_extent_m` is how far each way to include -- a few metres, deliberately,
    rather than the whole 40 m grid: the pose drifts, so a picture that invites
    global planning is a picture that misleads. `scale` is screen pixels per cell.

    `rover_up` picks which way is up. False, the default, points the page the way the
    rover was facing when it started, so the room holds still and the arrow turns:
    right for watching where the rover has got to. True points the page the way the
    rover is facing now, so the arrow holds still and the room turns underneath it,
    which is what you want when the question is "can I get through that gap ahead".
    Neither is more correct, and a picture cannot say which it is, so the caption
    does.

    `camera` is `(bearing_deg, fov_deg)` and draws the gimbal's cone -- where the
    camera is pointed relative to the rover's nose, positive to its left, and how
    much of the room is in shot. Omit it and nothing is drawn and nothing is
    claimed, which is the right answer for a rover with no camera on it.

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
    # the grid used to come back as a smaller picture. Off-grid now reads as
    # never-seen, which it is, and the picture is the size that was asked for.
    ahead = (half_cells - np.arange(span))[:, None]
    left = (half_cells - np.arange(span))[None, :]
    gx = np.rint(cx + ahead * ahead_cos - left * ahead_sin).astype(np.int32)
    gy = np.rint(cy + ahead * ahead_sin + left * ahead_cos).astype(np.int32)
    on_grid = (gx >= 0) & (gx < cells) & (gy >= 0) & (gy < cells)
    shown = np.zeros((span, span), dtype=np.int8)
    shown[on_grid] = np.asarray(grid)[gx[on_grid], gy[on_grid]]

    # Occupancy as three RGB planes. The state -> colour step happens here, on the
    # small array, rather than per pixel after the scale-up. Reachable empty floor
    # is green; empty that sits behind a wall stays cream.
    rgb = colour_occupancy(shown, occupied_at, origin=(half_cells, half_cells))

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
    _draw_track(big, np, [to_px(*point) for point in trail], scale,
                break_px=TRACK_BREAK_M * scale / res)

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

    # Before the arrow, so the arrow is never crossed by it: where the rover is
    # beats where it happens to be looking, and the two share the same origin.
    if camera is not None:
        draw_camera(canvas, to_px, x, y, th, camera[0], camera[1],
                    half_extent_m * CAMERA_REACH)

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
    # Said only when it is drawn. A caption describing a violet cone on a picture
    # that has none is the map's version of the rover saying it turned the lights on.
    cone = "" if camera is None else " " + camera_caption(camera[0], camera[1])
    description = (
        f"A top-down map of roughly {2 * half_extent_m:.0f} by "
        f"{2 * half_extent_m:.0f} metres around the rover, built from its lidar. "
        f"{orientation} The red triangle is the rover and its tip points the way the "
        f"rover is facing, with a yellow dot at its exact position; the blue line is "
        f"the path it has driven.{cone} Black is solid, green is empty space the rover "
        f"can reach from where it is standing, near-white is empty but cut off from "
        f"here by something solid, sandy beige is seen but not confirmed solid, and "
        f"flat grey is unknown -- not empty. The bar at the bottom "
        f"left is one metre. {solid} cells are solid out of {seen} seen. Distances "
        f"here are good to a few centimetres locally, but the rover's own position "
        f"drifts over a long run, so use this to judge what is nearby rather than to "
        f"navigate back to somewhere it has been."
    )
    return canvas.png(), description


def tap_to_relative(col, row, half_extent_m, scale, resolution_m=0.05,
                    rover_up=False, heading_rad=0.0):
    """A click on the map PNG -> (ahead_m, left_m) in the rover's frame.

    Inverse of the sampling in `render`: the rover sits at cell (half_cells,
    half_cells), up the page is `forward`, left is `sideways`. With `rover_up`
    those already are the rover's ahead and left; without, they are offsets in
    the frame the rover started in, and `heading_rad` turns them into the
    rover's current frame so a tap is a place relative to the rover now, which
    is what `drive_to` takes.
    """
    half_cells = max(8, int(round(half_extent_m / resolution_m)))
    forward_m = (half_cells - row / scale) * resolution_m
    left_m = (half_cells - col / scale) * resolution_m
    if rover_up:
        return forward_m, left_m
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    ahead = forward_m * c + left_m * s
    left = -forward_m * s + left_m * c
    return ahead, left


def tap_to_point(col, row, half_extent_m, scale, resolution_m=0.05,
                 rover_up=False, pose=(0.0, 0.0, 0.0)):
    """A click on the map PNG -> (x_m, y_m) in the map's own frame.

    `tap_to_relative` read in the pose the picture was drawn at, which is what a
    caller wants whenever the rover might move before the click is acted on. A
    relative target is measured from wherever the rover has got to by the time the
    call lands; a place on the map is the place that was clicked however late the
    call lands, and a click that interrupts a move is late by definition -- the
    move has to be stopped first, and the rover carries on driving until it is.
    """
    ahead, left = tap_to_relative(col, row, half_extent_m, scale, resolution_m,
                                  rover_up=rover_up, heading_rad=pose[2])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + ahead * c - left * s, pose[1] + ahead * s + left * c


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
