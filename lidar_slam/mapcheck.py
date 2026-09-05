#!/usr/bin/env python3
"""Synthetic checks for the map renderer: no rover, no lidar, no image library.

    python3 mapcheck.py                 # the checks, then a picture to eyeball
    python3 mapcheck.py /tmp/map.png    # ...written where you say

A box with a gap in it is enough to prove the things that go wrong silently. The
two halves of `render` have to agree about which way is forward -- a map drawn
the other way up is still a plausible room -- and a tap has to come back as the
metres it looked like. The camera cone and the reachable-floor colouring are
checked the same way, by rendering a known scene and reading the pixels back.

Kept out of mapimg.py because they are four hundred lines of scaffolding around a
six-hundred-line renderer, and nothing that runs on the rover imports them.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import struct
import zlib

from mapimg import (
    C_ANCHOR, C_CAMERA, C_FREE, C_OCCUPIED, C_REACHABLE, C_ROVER, C_TRACK,
    Canvas, OCCUPIED, ROVER, UNKNOWN, camera_caption, colour_occupancy,
    _decode, known_box, reachable_free, render, tap_to_point, tap_to_relative,
)


def _render_probe(heading, wall_axis="ahead", rover_up=False, camera=None):
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

    png, caption = render(_Slam(), half_extent_m=3.0, scale=3,
                          trail=[(cm / 100.0, 0.0) for cm in range(0, 201, 5)],
                          rover_up=rover_up, camera=camera)
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
    assert "green" in caption, (
        "the caption does not name the reachable-floor colour")

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


def _check_tap():
    """A click on the rover is here, a click above it is ahead, a click left is left.

    `rover_up` and a heading change must not change what a tap on the picture means
    in the rover's own frame, because the tool that consumes it is relative to the
    rover, not to the page.
    """
    res, half, scale = 0.05, 3.0, 4
    half_cells = max(8, int(round(half / res)))
    rover_col = rover_row = half_cells * scale

    ahead, left = tap_to_relative(rover_col, rover_row, half, scale)
    assert abs(ahead) < res and abs(left) < res, (ahead, left)

    ahead, left = tap_to_relative(rover_col, rover_row - 20 * scale, half, scale)
    assert abs(ahead - 1.0) < res and abs(left) < res, (ahead, left)

    ahead, left = tap_to_relative(rover_col - 20 * scale, rover_row, half, scale)
    assert abs(ahead) < res and abs(left - 1.0) < res, (ahead, left)

    # Same tap, page turned with the rover: still ahead/left of the rover.
    heading = math.pi / 2
    ahead, left = tap_to_relative(rover_col, rover_row - 20 * scale, half, scale,
                                  rover_up=True, heading_rad=heading)
    assert abs(ahead - 1.0) < res and abs(left) < res, (ahead, left)

    # Without rover_up, up the page is the start heading. A rover that has turned
    # left 90 deg sees that tap as to its right.
    ahead, left = tap_to_relative(rover_col, rover_row - 20 * scale, half, scale,
                                  rover_up=False, heading_rad=heading)
    assert abs(ahead) < res and abs(left + 1.0) < res, (ahead, left)
    print("tap ok: rover is 0,0; up is ahead; left is left")

    # The same click read as a place on the map, which is what a caller wants when
    # the rover may move before the click is acted on. A tap on the rover itself is
    # the rover's own position, however the page is turned.
    pose = (2.0, -1.0, math.pi / 2)          # at (2, -1), facing +y
    x, y = tap_to_point(rover_col, rover_row, half, scale, pose=pose)
    assert abs(x - pose[0]) < res and abs(y - pose[1]) < res, (x, y)
    x, y = tap_to_point(rover_col, rover_row, half, scale, rover_up=True, pose=pose)
    assert abs(x - pose[0]) < res and abs(y - pose[1]) < res, (x, y)

    # Page not turned: up the page is +x on the map, whatever the rover faces.
    x, y = tap_to_point(rover_col, rover_row - 20 * scale, half, scale, pose=pose)
    assert abs(x - 3.0) < res and abs(y + 1.0) < res, (x, y)

    # Page turned with the rover: up the page is a metre ahead of a rover facing
    # +y, so it is +y on the map and not +x.
    x, y = tap_to_point(rover_col, rover_row - 20 * scale, half, scale,
                        rover_up=True, pose=pose)
    assert abs(x - 2.0) < res and abs(y) < res, (x, y)

    # And the whole reason this exists. A click that interrupts a move is acted on
    # only after the move has been stopped, by which time the rover has driven on.
    # Read as a place it still means the pixel that was clicked; sent as an offset
    # and applied from where the rover ended up, it lands a whole metre away --
    # exactly the metre driven in between.
    col, row = rover_col - 20 * scale, rover_row - 20 * scale
    fixed = tap_to_point(col, row, half, scale, pose=pose)
    later = (2.0, 0.0, math.pi / 2)          # a metre further along that heading
    ahead, left = tap_to_relative(col, row, half, scale, heading_rad=pose[2])
    cos_th, sin_th = math.cos(later[2]), math.sin(later[2])
    landed = (later[0] + ahead * cos_th - left * sin_th,
              later[1] + ahead * sin_th + left * cos_th)
    drifted = math.hypot(landed[0] - fixed[0], landed[1] - fixed[1])
    assert abs(drifted - 1.0) < res, (fixed, landed, drifted)
    print("tap-to-point ok: a click is a place on the map, not an offset that "
          "drifts with the rover")


def _check_camera():
    """The cone points where the camera points, on both turns of the page.

    It goes through `to_px` like everything else drawn over the map, so it is
    subject to the same transpose that once mirrored the walls out from under their
    own track -- and a mirrored cone is worse than a mirrored wall, because a wall
    only claims the room is a shape it is not, while this claims the photographs are
    of the opposite side of it.
    """
    import numpy as np

    def bearing_of(img):
        """Which way the drawn cone leaves the rover, in degrees ccw from up."""
        rows, cols = np.nonzero(_mask(img, C_CAMERA))
        assert len(rows), "the camera cone was not drawn at all"
        centre = img.shape[0] / 2.0
        # The mean of the wedge, taken from the rover at the middle of the picture.
        return math.degrees(math.atan2(centre - cols.mean(), centre - rows.mean()))

    straight, caption = _render_probe(0.0, camera=(0.0, 65.0))
    assert abs(bearing_of(straight)) < 6.0, bearing_of(straight)
    # Positive is the rover's left, which is to the left of the page when the rover
    # faces up it. This is the sign the whole thing rests on.
    left = bearing_of(_render_probe(0.0, camera=(50.0, 65.0))[0])
    assert 35.0 < left < 65.0, left
    right = bearing_of(_render_probe(0.0, camera=(-50.0, 65.0))[0])
    assert -65.0 < right < -35.0, right

    # A wider field of view is a wider wedge, and a narrow one does not simply
    # vanish -- both of which a single fixed-width cone would pass.
    def spread(fov):
        img = _render_probe(0.0, camera=(0.0, fov))[0]
        rows, cols = np.nonzero(_mask(img, C_CAMERA))
        return cols.max() - cols.min()

    assert spread(30.0) < spread(65.0) < spread(120.0),         (spread(30.0), spread(65.0), spread(120.0))

    # Filled, not merely outlined. Everything above finds the cone by its exact
    # colour, which is the outline alone, so a fill that silently stopped being
    # drawn would pass every one of those checks: this one asks instead how much of
    # the picture the cone changed without becoming the cone's own colour, which is
    # the wash and nothing else.
    plain = _render_probe(0.0)[0]
    outline = _mask(straight, C_CAMERA)
    washed = np.any(straight != plain, axis=-1) & ~outline
    assert washed.sum() > outline.sum(), (
        f"the cone is an outline with nothing inside it: {washed.sum()} washed "
        f"pixels against {outline.sum()} of outline")

    # Turned rover, page held still: the cone turns with the rover, because the
    # bearing it is given is relative to the nose and not to the page.
    turned = bearing_of(_render_probe(math.pi / 2, camera=(0.0, 65.0))[0])
    assert 80.0 < turned < 100.0, turned
    # ...and with the page turned too, it comes back to straight up.
    both = bearing_of(_render_probe(math.pi / 2, rover_up=True, camera=(0.0, 65.0))[0])
    assert abs(both) < 6.0, both

    # Said only when drawn, and said with the side the right way round.
    assert "violet" in caption.lower() and "straight ahead" in caption
    assert "violet" not in _render_probe(0.0)[1].lower(),         "the caption describes a cone on a map that has none"
    assert "50 degrees to the rover's left" in _render_probe(0.0, camera=(50.0, 65.0))[1]
    assert "50 degrees to the rover's right" in _render_probe(0.0, camera=(-50.0, 65.0))[1]
    assert "straight ahead" in camera_caption(0.0, 65.0)
    print("camera ok: cone aims where the gimbal does, widens with the field of view")


def _check_reachable():
    """Green is the empty floor connected to the rover; cream is empty behind a wall.

    The whole point of a second empty colour is that distinction, so a picture that
    painted every free cell green -- or that leaked through a diagonal crack in a
    wall -- would look right at a glance and send the rover at a room it cannot
    enter. Checked on the occupancy array first, then on a rendered PNG, because
    the PNG is what a person actually sees and the array is what a one-cell leak
    is easiest to write down on.
    """
    import numpy as np

    # A 7x7 crop: rover in the middle, a wall across the top half, free both sides.
    shown = np.full((7, 7), -1, dtype=np.int8)
    shown[2, :] = 60
    origin = (5, 3)
    reach = reachable_free(shown, origin)
    assert reach[5, 3] and reach[6, 3], "the rover's own side of the wall is not reachable"
    assert not reach[0, 3] and not reach[1, 3], "empty behind the wall was marked reachable"
    assert not reach[2, 3], "the wall itself was marked reachable"

    rgb = colour_occupancy(shown, occupied_at=20, origin=origin)
    assert tuple(rgb[6, 3]) == C_REACHABLE
    assert tuple(rgb[0, 3]) == C_FREE
    assert tuple(rgb[2, 3]) == C_OCCUPIED

    # A one-cell gap in the wall *is* a doorway: both rooms turn green.
    gapped = shown.copy()
    gapped[2, 3] = -1
    through = reachable_free(gapped, origin)
    assert through[0, 3] and through[6, 3], "a gap in the wall did not connect the two rooms"

    # A checkerboard seam spans the width: 8-connected would walk the diagonals
    # into the far room, 4-connected must not.
    seam = np.full((6, 5), -1, dtype=np.int8)
    seam[2, 0::2] = 60
    seam[3, 1::2] = 60
    leaked = reachable_free(seam, (4, 2))
    assert leaked[4, 2] and leaked[5, 2]
    assert not leaked[0, 2] and not leaked[1, 2], (
        "a diagonal crack in a wall leaked reachability into the far room")

    # Unseen under the rover still lets the flood walk onto neighbouring free cells.
    unseen = np.full((5, 5), -1, dtype=np.int8)
    unseen[2, 2] = 0
    from_unknown = reachable_free(unseen, (2, 2))
    assert not from_unknown[2, 2], "unseen under the rover was painted reachable"
    assert from_unknown[2, 3] and from_unknown[1, 2]

    # And the same claims on a picture `render` actually drew, so a regression that
    # only hit the scale-up or the sampling would still fail here.
    class _Config:
        resolution_m, grid_cells, occupied_at, rover_width_m = 0.05, 400, 20, 0.34

    class _Slam:
        config = _Config()
        lock = __import__("contextlib").nullcontext()
        pose = (2.0, 0.0, 0.0)

        def __init__(self, gap=False):
            n = _Config.grid_cells
            self._g = np.full((n, n), -1, dtype=np.int8)
            self._g[n // 2 + 60, :] = 60          # a metre ahead of the start, full width
            if gap:
                self._g[n // 2 + 60, n // 2 - 1:n // 2 + 2] = -1

        def grid(self):
            return self._g

    def cell(img, ahead_cells, left_cells, scale=3):
        """The colour of one occupancy cell, sampled at its centre pixel."""
        half = max(8, int(round(3.0 / 0.05)))
        row = (half - ahead_cells) * scale + scale // 2
        col = (half - left_cells) * scale + scale // 2
        return tuple(img[row, col])

    blocked, caption = render(_Slam(), half_extent_m=3.0, scale=3)
    blocked = _decode(blocked)
    # Wall is 1 m = 20 cells ahead of a rover at x=2 m. Sample to the side of
    # the arrow: the triangle covers the pose itself, and a cell behind the
    # rover is still red.
    assert cell(blocked, 8, 8) == C_REACHABLE, cell(blocked, 8, 8)
    assert cell(blocked, 20, 8) == C_OCCUPIED, cell(blocked, 20, 8)
    assert cell(blocked, 24, 8) == C_FREE, cell(blocked, 24, 8)
    assert "green" in caption and "cut off" in caption

    opened, _ = render(_Slam(gap=True), half_extent_m=3.0, scale=3)
    opened = _decode(opened)
    assert cell(opened, 24, 8) == C_REACHABLE, cell(opened, 24, 8)
    print("reachable ok: green is connected empty floor, cream is empty behind a wall")


def _check_known_box():
    """`known_box` measures the map, not the picture drawn over it.

    Two ways to get this wrong and both are silent. Counting never-seen cells as
    known returns the whole forty-metre square, so a panel that frames on it
    shows a room the size of a postage stamp in a field of grey. Measuring the
    rendered pixels instead picks up the camera cone, which reaches metres into
    a part of the room nobody has been in and drags the frame off the map.
    """
    import math
    import threading

    import numpy as np

    cells, res = 800, 0.05

    class Fake:
        lock = threading.Lock()

        class config:
            resolution_m = res
            grid_cells = cells
            occupied_at = 20

        def __init__(self):
            self.pose = (0.0, 0.0, 0.0)
            self.grid_ = np.zeros((cells, cells), dtype=np.int8)
            self.trail = ()

        def grid(self):
            return self.grid_

    slam = Fake()
    assert known_box(slam) is None, "an untouched grid claims to hold a map"

    # A room from -1 m to +2 m ahead and -0.5 m to +1 m left, and nothing else.
    half = cells // 2
    lo_x, hi_x = half + int(-1.0 / res), half + int(2.0 / res)
    lo_y, hi_y = half + int(-0.5 / res), half + int(1.0 / res)
    slam.grid_[lo_x:hi_x, lo_y:hi_y] = -1          # seen, and empty
    slam.grid_[hi_x - 1, lo_y:hi_y] = 60           # a wall along the far end
    box = known_box(slam)
    for got, want in zip(box, (-1.0, -0.5, 2.0, 1.0)):
        assert math.isclose(got, want, abs_tol=res), f"{box} is not the room"

    # And the cone does not move it. `render` draws it over the occupancy after
    # this is measured, several metres past anything the rover has seen.
    render(slam, 6.0, 1, camera=(0.0, 60.0))
    assert known_box(slam) == box, "the camera cone moved the map"
    print("known box ok: the map is where the cells are, not where the cone points")


def main():
    _check_orientation()
    _check_tap()
    _check_camera()
    _check_reachable()
    _check_known_box()

    # And a picture to eyeball: a box with a gap, the geometry and the encoder
    # both visible without any hardware being involved.
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
