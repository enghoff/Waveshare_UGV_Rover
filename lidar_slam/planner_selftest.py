"""Offline checks for planner.py. Run via `python3 planner.py`."""
import math

import numpy as np

from planner import (
    CROP_MARGIN_FRACTION, CROP_MARGIN_M, CROP_MARGIN_MIN_KEEPOUTS,
    CROP_MARGIN_WORTH, KINK_CREDIT_M, KINK_CREDIT_MAX_M, _CROP_PROOF, _astar, _crop_margins,
    _inflate, _straighten, _turn_after, carrot_at, length, plan, point_at,
    project, segment_end_s,
)


def selftest():
    import numpy as np

    res, n, occ = 0.05, 80, 20
    grid = np.full((n, n), -10, dtype=np.int8)  # known free
    origin = n // 2

    def at(x, y):
        return origin + int(round(x / res)), origin + int(round(y / res))

    # A wall with a gap to the left, so the straight line is blocked and the
    # route has to go around.
    wx, _ = at(1.0, 0.0)
    grid[wx, :] = occ
    gap_y0 = origin + int(round(0.6 / res))
    gap_y1 = origin + int(round(1.1 / res))
    grid[wx, gap_y0:gap_y1 + 1] = -10

    path, why = plan(grid, res, occ, (0.0, 0.0), (1.8, 0.0), inflate_m=0.20)
    assert path is not None, why
    assert length(path) > 1.8, f"took the blocked straight line, length {length(path)}"
    assert any(y > 0.4 for _, y in path), f"did not detour left through the gap: {path}"
    # Thinned: a cell path would be ~40 points; a route is a handful of corners.
    assert len(path) <= 12, f"not thinned, {len(path)} waypoints: {path}"

    # Occupied target is refused rather than planned onto.
    path, why = plan(grid, res, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.20)
    assert path is None and "solid" in why, why

    # A rover that has ended up inside the wall's inflation ring can still plan
    # its way out -- this is the wedged case, and refusing it strands the rover.
    path, why = plan(grid, res, occ, (0.85, 0.0), (-0.5, 0.0), inflate_m=0.20)
    assert path is not None, f"wedged start was refused: {why}"
    assert path[0] == (0.85, 0.0), f"route does not start at the rover: {path[0]}"

    # Unseen target is refused.
    grid[:, :] = 0
    path, why = plan(grid, res, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.20)
    assert path is None and "seen" in why, why

    # Corners are given room when there is room to give. A wall ends at
    # (1.0, 0.25); the route from (0,0) to (2,0) has to round that end. The
    # preferred keep-out is a wall, not a toll: if a 0.45 m ring fits, the
    # route must stay that far out. A soft comfort still helps a little on a
    # single ring, but it is not what keeps the follower off the brake.
    def corner_distance(pts, cx, cy):
        best, s = 1e9, 0.0
        while s <= length(pts) + 1e-6:
            px, py = point_at(pts, s)
            best = min(best, math.hypot(px - cx, py - cy))
            s += 0.02
        return best

    big = np.full((120, 120), -10, dtype=np.int8)   # 6 m of known-free floor
    o = 60
    wx = o + int(round(1.0 / res))
    big[wx, :o + int(round(0.25 / res)) + 1] = occ  # wall along x=1.0, y <= 0.25
    hug, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0), inflate_m=0.25)
    assert hug is not None, why
    wide, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, comfort_m=0.55)
    assert wide is not None, why
    d_hug = corner_distance(hug, 1.0, 0.25)
    d_wide = corner_distance(wide, 1.0, 0.25)
    assert d_wide >= 0.35, f"still hugging the corner at {d_wide:.2f} m"
    assert d_wide > d_hug + 0.04, (
        f"comfort changed nothing: {d_hug:.2f} -> {d_wide:.2f}")
    assert length(wide) < length(hug) + 1.0, (
        f"the wide route ballooned: {length(hug):.2f} -> {length(wide):.2f}")

    pref, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, preferred_m=0.45)
    assert pref is not None, why
    d_pref = corner_distance(pref, 1.0, 0.25)
    assert d_pref >= 0.40, f"preferred still grazed the corner at {d_pref:.2f} m"
    assert d_pref > d_hug + 0.10, (
        f"preferred changed nothing: {d_hug:.2f} -> {d_pref:.2f}")

    # ...but a narrow gap is still taken when it is the only way through. A
    # 0.6 m opening leaves one free cell after the 0.25 m keep-out; preferred
    # 0.45 m seals it, so this is the fallback, not a refusal.
    big[wx, :] = occ
    gap0 = o - int(round(0.30 / res))
    gap1 = o + int(round(0.30 / res))
    big[wx, gap0:gap1] = -10
    path, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, preferred_m=0.45, comfort_m=0.55)
    assert path is not None, f"refused a gap the chassis fits through: {why}"
    assert all(abs(y) < 0.31 for x, y in
               [point_at(path, s * 0.05) for s in range(int(length(path) / 0.05))]
               if 0.9 < x < 1.1), f"did not go through the gap: {path}"

    # Facing a wall, goal on the other side: the nose is blocked and turning is
    # free, so the first hop must not be into the keep-out.
    faced = np.full((120, 120), -10, dtype=np.int8)
    wx = o + int(round(0.50 / res))
    faced[wx, :o + int(round(0.80 / res)) + 1] = occ
    path, why = plan(faced, res, occ, (0.0, 0.0), (1.2, 0.0),
                     inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert path is not None, f"refused a turn-first route: {why}"
    assert len(path) >= 2, path
    hop = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    assert abs(hop) > math.radians(50), (
        f"first hop into the wall: {path[:4]} heading {math.degrees(hop):.0f} deg")

    # And a clear run ahead is still a clear run: yaw must not invent a turn.
    clear = np.full((120, 120), -10, dtype=np.int8)
    path, why = plan(clear, res, occ, (0.0, 0.0), (1.5, 0.0),
                     inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert path is not None, why
    assert all(abs(y) < 0.15 for _, y in path), f"yaw forced a detour: {path}"

    # Straightening. On empty floor a route to anywhere is one segment: every
    # monotone staircase A* could have returned costs the same, and the corner it
    # picked between them is the grid talking. This is the case the whole thing
    # exists for -- before it, this route came back as a 4.25 m run straight ahead
    # and then a 25 degree kink.
    empty = np.full((160, 160), -10, dtype=np.int8)
    for goal in ((3.0, 0.8), (3.0, 2.0), (2.0, 3.0), (3.0, -1.7)):
        line, why = plan(empty, res, occ, (-3.0, 0.0), goal, inflate_m=0.25,
                         preferred_m=0.45, comfort_m=0.55)
        assert line is not None, why
        assert len(line) == 2, f"empty floor to {goal} came back with {line}"
        direct = math.hypot(goal[0] + 3.0, goal[1])
        assert abs(length(line) - direct) < 0.02, (
            f"straight route is {length(line):.2f} m, direct is {direct:.2f} m")

    # But it is a shortcut only where there is nothing in the way. The same skew
    # trip with a wall across the middle keeps the corner that gets round it, and
    # keeps its distance from the end of that wall.
    walled = np.full((160, 160), -10, dtype=np.int8)
    o2 = 80
    wcol = o2 + int(round(0.0 / res))
    walled[wcol, :o2 + int(round(1.5 / res))] = occ   # wall along x=0, y < 1.5
    bent, why = plan(walled, res, occ, (-3.0, 0.0), (3.0, 2.0), inflate_m=0.25,
                     preferred_m=0.45, comfort_m=0.55)
    assert bent is not None, why
    assert len(bent) >= 3, f"straightened through a wall: {bent}"
    assert corner_distance(bent, 0.0, 1.5) >= 0.40, (
        f"straightening grazed the wall end at "
        f"{corner_distance(bent, 0.0, 1.5):.2f} m")

    # And it never crosses the keep-out anywhere, which is the promise that makes
    # it safe to shorten a route at all.
    def min_clearance(pts, grid_, from_m=0.0, to_m=None):
        cells = grid_.shape[0]
        og = cells // 2
        solid = np.argwhere(grid_ >= occ)
        xs = (solid[:, 0] - og) * res
        ys = (solid[:, 1] - og) * res
        best, s2 = 1e9, from_m
        stop = length(pts) if to_m is None else to_m
        while s2 <= stop + 1e-9:
            px, py = point_at(pts, s2)
            best = min(best, float(np.min(np.hypot(xs - px, ys - py))))
            s2 += 0.02
        return best

    assert min_clearance(bent, walled) >= 0.44, (
        f"straightened route runs {min_clearance(bent, walled):.3f} m from a wall, "
        f"inside the 0.45 m ring it was planned with")

    # The turn-off-a-wall hop is not a corner to be shortcut away. Straightening
    # the route from the pose onto that first free cell is exactly the chord
    # through the wall that the hop exists to prevent.
    hopped, why = plan(faced, res, occ, (0.0, 0.0), (1.2, 0.0),
                       inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert hopped is not None, why
    first_hop = math.atan2(hopped[1][1] - hopped[0][1],
                           hopped[1][0] - hopped[0][0])
    assert abs(first_hop) > math.radians(50), (
        f"straightening ate the turn-first hop: {hopped[:3]}")

    # carrot_at. Far from a vertex it is the plain lookahead...
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]      # a right angle
    bend = [(0.0, 0.0), (2.0, 0.0), (3.0, 0.5)]        # 27 degrees, driven through
    cx, cy = carrot_at(square, 0.5, 0.80, 0.40, 40.0)
    assert abs(cx - 1.3) < 1e-6 and abs(cy) < 1e-6, (cx, cy)
    # ...and it never aims onto the next leg, which is what drives the chord.
    cx, cy = carrot_at(square, 1.6, 0.80, 0.0, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy) < 1e-6, (cx, cy)
    # At a gentle corner it runs on along this leg's own line rather than
    # collapsing onto a vertex 5 cm away -- and stays on that line, not the next.
    cx, cy = carrot_at(bend, 1.95, 0.80, 0.40, 40.0)
    assert abs(cy) < 1e-6, f"the carrot bent onto the next leg: {(cx, cy)}"
    assert abs(cx - 2.35) < 1e-6, (cx, cy)
    assert math.hypot(cx - 1.95, cy) >= 0.40 - 1e-9, "carrot still inside the minimum"
    # A corner the rover is going to stop and spin at keeps the carrot it had.
    # Running on past a right angle arrives at the corner still under power, and
    # a corner is where the route has the least room to spare.
    cx, cy = carrot_at(square, 1.95, 0.80, 0.40, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy) < 1e-6, (
        f"the carrot ran on past a right angle: {(cx, cy)}")
    # And the last waypoint is left alone, or a rover a little to one side of the
    # goal would be steered past it instead of round to it.
    cx, cy = carrot_at(square, 3.9, 0.80, 0.40, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy - 2.0) < 1e-6, (
        f"the carrot ran on past the goal: {(cx, cy)}")
    assert abs(_turn_after(square, 2.0) - 90.0) < 1e-6, _turn_after(square, 2.0)
    assert abs(_turn_after(bend, 2.0) - 26.565) < 1e-3, _turn_after(bend, 2.0)
    assert _turn_after(square, length(square)) == 0.0, "a last vertex turns nowhere"

    # The two things straightening promises that a route through a clean room
    # does not exercise, checked on the function itself: that it will not shortcut
    # across the turn-off-a-wall hop, and that the corner credit is what decides a
    # shortcut which is straighter but runs nearer the toll.
    free = np.zeros((60, 60), dtype=bool)
    toll_field = np.zeros((60, 60), dtype=np.float32)
    toll_field[25:36, 28:33] = 0.8          # something to give a wide berth to

    def cellwise(x, y):
        return x / res + 30, y / res + 30

    dogleg = [(-1.0, 0.0), (0.0, -0.5), (1.0, 0.0)]
    tight = _straighten(dogleg, free, toll_field, cellwise, credit_cells=0.0,
                        credit_cap_cells=20.0)
    assert tight == dogleg, (
        f"took a more tolled line for nothing: {tight}")
    paid = _straighten(dogleg, free, toll_field, cellwise,
                       credit_cells=KINK_CREDIT_M / res,
                       credit_cap_cells=KINK_CREDIT_MAX_M / res)
    assert paid == [dogleg[0], dogleg[-1]], (
        f"the corner was worth more than the detour and was kept anyway: {paid}")

    # `first` holds the hop even where the line past it is perfectly clear.
    hop = [(0.0, 0.0), (0.0, 0.5), (1.0, 0.5)]
    assert _straighten(hop, free, None, cellwise, first=0) == [hop[0], hop[-1]], (
        "nothing was shortcut on an empty floor")
    assert _straighten(hop, free, None, cellwise, first=1) == hop, (
        "the hop that turns the rover off a wall was shortcut away")

    # Progress along a path does not rewind for a small sideways weave.
    line = [(0.0, 0.0), (2.0, 0.0)]
    s, d = project(line, (1.0, 0.12), 1.0, slack_m=0.35)
    assert abs(s - 1.0) < 0.05 and 0.10 < d < 0.15, (s, d)
    s, d = project(line, (0.7, 0.0), 1.0, slack_m=0.35)
    assert 0.65 < s < 0.75, f"should be allowed to slide back 30 cm, got s={s}"
    s, d = project(line, (0.4, 0.0), 1.0, slack_m=0.35)
    assert s >= 0.64, f"rewound past the slack to s={s}"

    corner = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert abs(segment_end_s(corner, 0.0) - 1.0) < 1e-9
    assert abs(segment_end_s(corner, 0.4) - 1.0) < 1e-9
    assert abs(segment_end_s(corner, 1.0) - 2.0) < 1e-9
    assert abs(segment_end_s(corner, 1.5) - 2.0) < 1e-9

    # --- the window A* looks in, and how it is grown -------------------------
    #
    # A first window that fails has opened every cell it could reach, so trying a
    # small one before the full one is a bet: cheap when it pays, and a whole
    # wasted search when it does not.
    far_cells = int(math.ceil(CROP_MARGIN_M / res))
    keep = 0.45
    near, far = _crop_margins((0.0, 0.0), (0.2, 0.0), res, keep)
    assert far == far_cells, (far, far_cells)
    assert near < far, "a 20 cm hop still searched the full window"
    # Two keep-out radii, in metres, whatever the constants are called: a window
    # narrower than that is all keep-out once inflated and cannot hold a way round.
    assert near * res >= 2.0 * keep - 1e-9, (
        f"the first window is {near * res:.2f} m either side of a {keep:.2f} m "
        f"keep-out, so there is nowhere in it for a route to go")

    same = _crop_margins((0.0, 0.0), (5.0, 0.0), res, keep)
    assert same == (far_cells, far_cells), (
        "a long route was given two windows to search when the smaller one is "
        "barely smaller -- that is the case that cost 40% and gained nothing")
    # ...and the same refusal on the marginal case, which is the one the ratio is
    # actually for: 4 m apart the smaller window is still 94% of the area.
    assert _crop_margins((0.0, 0.0), (4.0, 0.0), res, keep) == (far_cells, far_cells), (
        "took a first window that saves six percent and risks a whole extra search")

    for span in (0.05, 0.5, 1.0, 2.0, 3.0, 4.0, 8.0):
        a, b = _crop_margins((0.0, 0.0), (span, span * 0.3), res, keep)
        assert a <= b == far_cells, (span, a, b)
        dx, dy = span / res, span * 0.3 / res
        if a < b:
            assert ((dx + 2 * a + 1) * (dy + 2 * a + 1)
                    <= CROP_MARGIN_WORTH * (dx + 2 * b + 1) * (dy + 2 * b + 1)
                    + 1e-9), f"tried a first window that saves too little at {span} m"

    # A detour wider than the first window must still be found. The wall here is
    # 1.4 m long with the way round it well outside a short hop's window, so a
    # planner that gave up when the small search failed would refuse a route that
    # is plainly there.
    res2, n2 = 0.05, 120
    room = np.full((n2, n2), -10, dtype=np.int8)
    o2 = n2 // 2

    def at2(x, y):
        return o2 + int(round(x / res2)), o2 + int(round(y / res2))

    bx, _ = at2(0.5, 0.0)
    y_lo, y_hi = at2(0.0, -1.4)[1], at2(0.0, 1.4)[1]
    room[bx:bx + 3, y_lo:y_hi] = occ
    path, why = plan(room, res2, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.25,
                     preferred_m=0.45)
    assert path is not None, f"the route round the wall was lost: {why}"
    assert max(abs(y) for _x, y in path) > 1.2, (
        f"claimed a route that goes through the wall: {path}")

    # And it must still be the *preferred* clearance. A first window too small to
    # hold the way round does not lose the route -- the pinch pass underneath finds
    # one at the smaller keep-out -- so the damage is silent: the rover squeezes
    # past the end of the wall instead of going round it with room to spare.
    solid_xy = [((ix - o2) * res2, (iy - o2) * res2)
                for ix, iy in np.argwhere(room >= occ)]
    closest = min(
        min(math.dist((ax + (bx - ax) * k / 40.0, ay + (by - ay) * k / 40.0), s)
            for s in solid_xy for k in range(41))
        for (ax, ay), (bx, by) in zip(path, path[1:]))
    # 0.40 m planned at the preferred keep-out against 0.22 at the pinch one --
    # measured to cell centres, so a 0.45 m ring reads a little under.
    assert closest > 0.32, (
        f"the route passes {closest:.2f} m from the wall, so it was planned at the "
        f"pinch keep-out rather than the preferred one -- widening the window was "
        f"skipped and clearance was given up instead")

    # --- the route A* returns is the cheapest one, not merely one that works ---
    #
    # Guarding the heuristic. It may underestimate the remaining cost as much as
    # it likes and the answer stays optimal; the moment it overestimates, A* goes
    # greedy and returns whatever it stumbled into. Nothing that checks only that
    # a route exists would notice, so both halves here are about its cost. On an
    # empty grid the cheapest 8-connected route costs the octile distance, which
    # is a number this can be checked against without a second implementation.
    def route_cost(cells):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(cells, cells[1:]))

    open_grid = np.zeros((40, 40), dtype=bool)
    for s, g in (((5, 5), (25, 17)), ((2, 30), (30, 2)), ((10, 10), (10, 31)),
                 ((0, 0), (39, 39))):
        cells = _astar(open_grid, s, g)
        assert cells is not None, (s, g)
        dx, dy = abs(g[0] - s[0]), abs(g[1] - s[1])
        octile = max(dx, dy) + (math.sqrt(2) - 1.0) * min(dx, dy)
        assert abs(route_cost(cells) - octile) < 1e-6, (
            f"A* from {s} to {g} across an empty grid cost {route_cost(cells):.3f} "
            f"where the cheapest 8-connected route costs {octile:.3f}")

    # An empty grid cannot catch a heuristic that overestimates, because there is
    # nothing to go the wrong way around: greedy and optimal walk the same line.
    # A search that is optimal is symmetric, though -- the cheapest way there is
    # the cheapest way back -- and a greedy one is not. Cluttered both ways round
    # is where that shows.
    seed = np.random.default_rng(20260821)
    checked = 0
    for _ in range(30):
        clutter = seed.random((40, 40)) < 0.22
        a, b = (1, 1), (38, 38)
        clutter[a] = clutter[b] = False
        there, back = _astar(clutter, a, b), _astar(clutter, b, a)
        if there is None or back is None:
            continue
        checked += 1
        assert abs(route_cost(there) - route_cost(back)) < 1e-6, (
            f"the way there costs {route_cost(there):.2f} and the way back "
            f"{route_cost(back):.2f} -- the search is not finding the cheapest "
            f"route, so the heuristic is overestimating somewhere")
    assert checked >= 10, f"only {checked} of the cluttered grids had a route at all"

    # --- the inflation is the same disc, faster ------------------------------
    def disc_by_offsets(blocked, r):
        """The offset-by-offset dilation _inflate replaces, kept as the oracle."""
        if r <= 0:
            return blocked.copy()
        out = blocked.copy()
        h, w = blocked.shape
        r2 = r * r
        for du in range(-r, r + 1):
            for dv in range(-r, r + 1):
                if (du or dv) and du * du + dv * dv <= r2:
                    out[max(0, du):h + min(0, du), max(0, dv):w + min(0, dv)] |= \
                        blocked[max(0, -du):h + min(0, -du),
                                max(0, -dv):w + min(0, -dv)]
        return out

    rng = np.random.default_rng(20260821)
    for _ in range(6):
        h = int(rng.integers(14, 46))
        w = int(rng.integers(14, 46))
        mask = rng.random((h, w)) < float(rng.choice([0.004, 0.05, 0.25]))
        for r in range(0, 11):
            assert np.array_equal(disc_by_offsets(mask, r), _inflate(mask, r)), (
                f"the fast inflation is not the same disc at radius {r} "
                f"on a {h}x{w} grid")
    # A single cell gives the disc itself, which pins the shape rather than just
    # agreeing with the other implementation about it.
    dot = np.zeros((21, 21), dtype=bool)
    dot[10, 10] = True
    ring = _inflate(dot, 5)
    assert ring[10, 15] and ring[10, 5] and ring[15, 10], "the disc is not 5 wide"
    assert not ring[10, 16], "the disc reaches further than its radius"
    assert ring[13, 14] and not ring[14, 14], (
        "the corners are not Euclidean -- 3,4 is inside a radius of 5 and 4,4 is not")

    print("planner: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(selftest())
