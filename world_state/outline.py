"""A region's own pixels, kept small, and the distance read from the depth under them.

**A box is the wrong thing to range when something stands in front of what is in it.**
The depth service answers a box with its nearest surface, which is right when the thing
is the nearest surface in its box. When it is not, the answer is wrong: on 2026-10-02
the landscape painting behind the dining chairs read 1.22 m against 2.54, because the
nearest surface was a chair, and that one reading placed the painting 0.88 m from where
it hangs. The region finder already says which pixels are the thing -- its outline, the
same mask that blanks the crop for `dino_alone` -- so the depth is read under those
pixels instead, and the box is read only where the outline cannot be.

Replayed from the depth maps the rover kept, with the outlines regenerated from the
stored pictures: at the taped objects 41 readings within 0.25 m against 36, a median
error of 0.19 m against 0.27, and the worst placement among them from 0.87 m to 0.38.
See docs/progress/2026-10-02-ranging-from-the-outline.md. Three things that replay
settled are built in here:

- **The outline is cut to its box.** The finder's mask is not confined to the region:
  one red toolbox's covered every red thing in the picture.
- **It is not trimmed at its edges.** A glass table's only solid pixels are its frame,
  and trimming them read 4.0 m through the glass where the box read 0.97.
- **The box stands in where the outline leaves too few depth pixels**, which is a small
  region or one at the edge of the depth camera's view: 6 to 13% of the regions a box
  can range.

The statistic is the depth service's own -- the nearest surface found by a low
percentile, then the median of the band behind it (`oak_depth/depth_server.py`
`_range_in`) -- copied rather than imported, because the two run in different processes
from different directories. The numbers are the service's and `test_outline` holds this
copy to what they produce.

An outline is kept at half the frame's resolution, packed and compressed, which is a few
hundred bytes a region; `encode` and `decode` are the format.
"""
from __future__ import annotations

import math
import struct
import zlib
from typing import Any

from . import oak, view
from .depth_client import NOTHING_TO_MEASURE, OUTSIDE_VIEW

#: Every STRIDE-th pixel of the outline is kept, both ways. Half resolution is a quarter
#: of the bits, and the depth map the outline is read against is coarser still: one of
#: its pixels covers about two of the fisheye's at the middle of the picture.
STRIDE = 2

#: The depth service's own numbers (`oak_depth/depth_settings.py`), kept in step by hand.
MIN_MM, MAX_MM = 200, 6000
RANGE_PERCENTILE = 20
RANGE_BAND_FRAC = 0.15
RANGE_BAND_M = 0.30
RANGE_MIN_PIXELS = 12
DISPARITY_SIGMA_PX = 0.2
#: The stereo pair's focal length and baseline, as the service falls back to them.
FOCAL_PX = 455.8
BASELINE_M = 0.075

#: x0, y0 of the kept window in frame pixels, its columns and rows, and the stride.
_HEADER = struct.Struct("<HHHHB")

#: Outline, as `Ranged.method` and the store's `range_from` column call it; and the box.
OUTLINE = "outline"
BOX = "box"


# --- the outline as bytes ------------------------------------------------------

def window(bbox, size: tuple[int, int]) -> tuple[int, int, int, int]:
    """A box in fractions of the frame as a pixel window `(x0, y0, x1, y1)`."""
    width, height = size
    left, top, right, bottom = (float(value) for value in bbox)
    x0 = max(0, min(width - 1, int(math.floor(min(left, right) * width))))
    y0 = max(0, min(height - 1, int(math.floor(min(top, bottom) * height))))
    x1 = max(x0 + 1, min(width, int(math.ceil(max(left, right) * width))))
    y1 = max(y0 + 1, min(height, int(math.ceil(max(top, bottom) * height))))
    return x0, y0, x1, y1


def encode(np, mask, bbox) -> bytes | None:
    """One region's outline, cut to its box, as bytes; None if nothing of it is left.

    `mask` is the frame-sized boolean array `perceive._Masks.of` answers.
    """
    if mask is None or bbox is None:
        return None
    height, width = mask.shape[:2]
    x0, y0, x1, y1 = window(bbox, (width, height))
    piece = np.ascontiguousarray(mask[y0:y1:STRIDE, x0:x1:STRIDE], dtype=bool)
    if piece.size == 0 or not piece.any():
        return None
    return (_HEADER.pack(x0, y0, piece.shape[1], piece.shape[0], STRIDE)
            + zlib.compress(np.packbits(piece, axis=None).tobytes()))


def decode(np, blob):
    """`(x0, y0, stride, piece)` from `encode`'s bytes, or None if they do not read."""
    if not blob or len(blob) <= _HEADER.size:
        return None
    try:
        x0, y0, columns, rows, stride = _HEADER.unpack_from(blob)
        bits = np.unpackbits(np.frombuffer(zlib.decompress(blob[_HEADER.size:]),
                                           dtype=np.uint8))
    except (struct.error, zlib.error, ValueError):
        return None
    if columns == 0 or rows == 0 or bits.size < columns * rows or stride < 1:
        return None
    return x0, y0, stride, bits[:columns * rows].reshape(rows, columns).astype(bool)


# --- the depth service's statistic ---------------------------------------------

def surface(np, lengths):
    """`(range_m, spread_m, pixels)` of the nearest surface among these lengths, or None.

    Two steps, as the depth service takes them: a low percentile finds where the front of
    the thing is, and the median of everything within the band behind it is the answer.
    A plain median would blend the thing with whatever else the pixels reach.
    """
    if lengths.size < RANGE_MIN_PIXELS:
        return None
    near = float(np.percentile(lengths, RANGE_PERCENTILE))
    band = max(RANGE_BAND_M, RANGE_BAND_FRAC * near)
    kept = lengths[lengths <= near + band]
    if kept.size < RANGE_MIN_PIXELS:
        kept = lengths
    return float(np.median(kept)), float(np.std(kept)), int(kept.size)


def sigma_m(range_m: float, spread_m: float) -> float:
    """What a reading is worth, as the depth service states it: the stereo model's error
    at this range and half the spread of the surface that produced it, in quadrature."""
    model = range_m * range_m * DISPARITY_SIGMA_PX / (FOCAL_PX * BASELINE_M)
    return round(math.hypot(model, spread_m / 2.0), 3)


# --- where a fisheye pixel looks, and where that lands in the depth map ---------

_DIRECTIONS: dict[tuple[int, int], Any] = {}


def directions(np, size: tuple[int, int]):
    """Every STRIDE-th pixel of a gimbal picture as a direction in that camera's own frame.

    At pan 0 and tilt 0, as `oak.box_for` wants them: the OAK rides the gimbal, so where a
    fisheye pixel lands in its picture does not depend on the servos. Worked out once per
    picture size and kept -- 77 thousand calls into the lens, a fraction of a second.
    """
    size = (int(size[0]), int(size[1]))
    if size not in _DIRECTIONS:
        width, height = size
        grid = np.full((height // STRIDE, width // STRIDE, 3), np.nan)
        for row in range(height // STRIDE):
            for column in range(width // STRIDE):
                found = view.chassis_direction((column * STRIDE + 0.5) / width,
                                               (row * STRIDE + 0.5) / height,
                                               0.0, 0.0, size)
                if found is not None:
                    grid[row, column] = found
        _DIRECTIONS[size] = grid
    return _DIRECTIONS[size]


def into_oak(np, mount=None):
    """`(A, offset)` with `oak._in_oak(d, r) == A @ (d * r - offset)` for every direction.

    The mount's own turn, pitch and roll, taken from `oak`'s functions rather than written
    out again, so that a corrected mount moves this with it. `test_outline` checks the
    two agree.
    """
    mount = mount or oak.MOUNT

    def turned(vector):
        along, left, up = oak._unturn(vector, mount.yaw_deg, mount.pitch_deg)
        right, down = oak._rolled(-left, -up, mount)
        return right, down, along
    matrix = np.array([turned((1.0, 0.0, 0.0)), turned((0.0, 1.0, 0.0)),
                       turned((0.0, 0.0, 1.0))]).T
    return matrix, np.array([mount.forward_m, mount.left_m, mount.up_m])


def turned(np, turn_deg, pan_deg, tilt_deg):
    """The 3x3 matrix taking a direction in the gimbal camera's frame at the shutter to
    the same direction in that frame once the rover has turned `turn_deg` (left
    positive, as headings are), or None for a turn too small to matter.

    **What lets a depth frame taken a little before or after the picture be used rather
    than thrown away.** The rover turns about its own vertical axis, and the camera is
    tilted and panned on the gimbal, so the turn is undone in the chassis frame: out
    through the gimbal's pan and tilt (`oak._turn`), round by the turn, and back. A
    thing dead ahead at the shutter lies to the right of the axis after a left turn.
    """
    if abs(float(turn_deg or 0.0)) < 0.05:
        return None
    pan, tilt = float(pan_deg or 0.0), float(tilt_deg or 0.0)

    def through(vector):
        chassis = oak._turn(vector, pan, tilt)
        rotated = oak._turn(chassis, float(turn_deg), 0.0)
        return oak._unturn(rotated, pan, tilt)
    return np.array([through((1.0, 0.0, 0.0)), through((0.0, 1.0, 0.0)),
                     through((0.0, 0.0, 1.0))]).T


def corners_of(bbox, size):
    """A box on the gimbal camera as four directions in that camera's frame, or None.

    Four corners rather than a centre, because the two lenses do not agree about shape:
    a box near the edge of a 130-degree fisheye maps to a very different rectangle on a
    pinhole. At pan 0 and tilt 0 for the reason `directions` gives.
    """
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    try:
        left, top, right, bottom = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return None
    found = []
    for x_frac, y_frac in ((left, top), (right, top), (left, bottom), (right, bottom)):
        direction = view.chassis_direction(x_frac, y_frac, 0.0, 0.0, size)
        if direction is None:
            return None
        found.append(direction)
    return found


class DepthImage:
    """One depth map, as millimetres, with the colour lens it is aligned to."""

    def __init__(self, np, millimetres, width: int, height: int, lens) -> None:
        self.np = np
        self.mm = np.frombuffer(millimetres, dtype=np.uint16).reshape(height, width)
        self.width, self.height, self.lens = width, height, lens
        self.sx = width / float(lens.width)
        self.sy = height / float(lens.height)

    def lengths(self, columns, rows):
        """The lengths along the OAK's rays at these depth pixels, in metres, where valid.

        A depth map holds how far a surface is in front of the camera's plane, and a range
        is the length of the line to it: they differ by one over the cosine of the angle
        off the axis, 19% at the side of this lens. Per pixel here, where the service takes
        it once at a box's middle.
        """
        np = self.np
        z = self.mm[rows, columns].astype(float)
        good = (z >= MIN_MM) & (z <= MAX_MM)
        lens = self.lens
        x = (columns[good] + 0.5 - lens.cx * self.sx) / (lens.fx * self.sx)
        y = (rows[good] + 0.5 - lens.cy * self.sy) / (lens.fy * self.sy)
        return z[good] / 1000.0 * np.sqrt(1.0 + x * x + y * y), int(z.size)


def _box_once(image: DepthImage, box):
    """The service's reading of one box in the OAK's picture: `(range, spread, pixels,
    valid share)` along the OAK's ray, or None."""
    np = image.np
    width, height = image.width, image.height
    left, top, right, bottom = (float(value) for value in box)
    # Rounded to the nearest pixel, as the service rounds them, so that the same box
    # gives the same answer here and there.
    x0 = max(0, min(width - 1, int(round(min(left, right) * width))))
    x1 = max(x0 + 1, min(width, int(round(max(left, right) * width))))
    y0 = max(0, min(height - 1, int(round(min(top, bottom) * height))))
    y1 = max(y0 + 1, min(height, int(round(max(top, bottom) * height))))
    cell = image.mm[y0:y1, x0:x1].astype(float)
    good = (cell >= MIN_MM) & (cell <= MAX_MM)
    found = surface(np, cell[good] / 1000.0)
    if found is None:
        return None
    lens = image.lens
    # The service's secant, taken once at the box's middle.
    x = ((x0 + x1) / 2.0 - lens.cx * image.sx) / (lens.fx * image.sx)
    y = ((y0 + y1) / 2.0 - lens.cy * image.sy) / (lens.fy * image.sy)
    secant = math.sqrt(1.0 + x * x + y * y)
    return found[0] * secant, found[1], found[2], float(good.mean())


def box_range(image: DepthImage, bbox, size, mount=None, turn=None):
    """The box read the way the depth service reads it, as a length along the gimbal
    camera's ray: a dict, or one with `absent` saying which silence it was. `turn` is
    `turned`'s matrix, when the depth frame was taken after the rover turned."""
    corners = corners_of(bbox, size)
    if corners is None:
        return {"absent": NOTHING_TO_MEASURE}
    if turn is not None:
        corners = [tuple(turn @ image.np.array(corner)) for corner in corners]
    box = oak.box_for(corners, image.lens, mount=mount)
    if box is None:
        return {"absent": OUTSIDE_VIEW}
    got = _box_once(image, box)
    if got is not None and abs(got[0] - oak.GUESS_RANGE_M) > 0.40 * oak.GUESS_RANGE_M:
        # Drawn again from where the answer says the thing is, as the inspection's
        # server path does: the two lenses are centimetres apart.
        again = oak.box_for(corners, image.lens, got[0], mount=mount)
        second = None if again is None else _box_once(image, again)
        if second is not None:
            got = second
    if got is None:
        return {"absent": NOTHING_TO_MEASURE}
    ranged = oak.range_from_gimbal(corners, got[0], mount=mount)
    if ranged is None or ranged <= 0.0:
        return {"absent": NOTHING_TO_MEASURE}
    return {"range_m": round(ranged, 3), "sigma_m": sigma_m(ranged, got[1]),
            "pixels": got[2], "valid": round(got[3], 3), "method": BOX}


def outline_range(image: DepthImage, blob, size, guess=None, mount=None, turn=None):
    """The depth under one region's own outline, as a length along the gimbal camera's
    ray, or None when the outline leaves too few depth pixels to say."""
    np = image.np
    decoded = decode(np, blob)
    if decoded is None:
        return None
    x0, y0, stride, piece = decoded
    rows, columns = np.nonzero(piece)
    grid = directions(np, size)
    at_rows = np.clip((y0 + rows * stride) // STRIDE, 0, grid.shape[0] - 1)
    at_columns = np.clip((x0 + columns * stride) // STRIDE, 0, grid.shape[1] - 1)
    pointing = grid[at_rows, at_columns]
    pointing = pointing[~np.isnan(pointing).any(axis=1)]
    if len(pointing) < RANGE_MIN_PIXELS:
        return None
    if turn is not None:
        pointing = pointing @ turn.T
    middle = pointing.mean(axis=0)
    middle = tuple(middle / (np.linalg.norm(middle) or 1.0))
    matrix, offset = into_oak(np, mount)
    lens = image.lens
    answer = None
    assumed = float(guess or oak.GUESS_RANGE_M)
    # Drawn at a guess, then again at the answer: the two lenses are a few centimetres
    # apart, so where an outline lands in the depth map depends on how far away it is.
    for _ in range(2):
        point = (pointing * assumed - offset) @ matrix.T
        ahead = point[:, 2] > 1e-6
        u = (lens.cx + lens.fx * point[ahead, 0] / point[ahead, 2]) * image.sx
        v = (lens.cy + lens.fy * point[ahead, 1] / point[ahead, 2]) * image.sy
        inside = (u >= 0) & (u < image.width) & (v >= 0) & (v < image.height)
        if int(inside.sum()) < RANGE_MIN_PIXELS:
            return None
        keys = np.unique(v[inside].astype(int) * image.width + u[inside].astype(int))
        lengths, asked = image.lengths(keys % image.width, keys // image.width)
        found = surface(np, lengths)
        if found is None:
            return None
        ranged = oak.range_from_gimbal([middle], found[0], mount=mount)
        if ranged is None or ranged <= 0.0:
            return None
        answer = {"range_m": round(ranged, 3), "sigma_m": sigma_m(ranged, found[1]),
                  "pixels": found[2], "valid": round(lengths.size / max(1, asked), 3),
                  "method": OUTLINE}
        assumed = ranged
    return answer


def read(np, millimetres, width, height, lens, regions, size, mount=None,
         turn_deg=0.0, pan_deg=0.0, tilt_deg=0.0):
    """One answer per region: `(bbox, outline)` pairs, read under the outline where it can
    be and as the box otherwise. Each answer is a dict with `range_m`, `sigma_m`, `pixels`,
    `valid` and `method`, or one with `absent` alone. `turn_deg` is how far the rover
    turned between the picture and the depth frame, left positive; see `turned`."""
    image = DepthImage(np, millimetres, width, height, lens)
    turn = turned(np, turn_deg, pan_deg, tilt_deg)
    answers = []
    for bbox, blob in regions:
        boxed = box_range(image, bbox, size, mount, turn)
        found = None
        if blob and not boxed.get("absent") == OUTSIDE_VIEW:
            found = outline_range(image, blob, size, boxed.get("range_m"), mount, turn)
        answers.append(found or boxed)
    return answers
