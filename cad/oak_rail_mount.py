"""OAK-D-Lite to Picatinny rail adapter for the gimbal.

Two printed parts that carry the OAK-D-Lite on a MIL-STD-1913 rail:

  body   the clamp roof, the fixed jaw, the recoil lug and the vertical plate
         the camera bolts to
  wedge  the moving jaw, drawn up under the roof by two M4 screws so its
         45 degree face pulls the rail against the fixed jaw

The camera hangs off its own two M4 holes in the back of the case, which is
the only mount on it that is symmetric about the lens axis: they are 75.00 mm
apart, on the same vertical line as the two mono cameras, and 18.48 mm above
the bottom edge of the case.  Every camera number here was measured off
Luxonis's published enclosure model (DM9095_enclosure.STL), not off the
datasheet drawing.

Frame: X across the rail (= along the camera), Y along the rail with +Y the
direction the camera looks, Z up from the top surface of the rail.  The plate
face the camera bolts to is Y = 0, so the camera occupies Y = 0 .. 17.45 and
everything printed sits at Y <= 0.

Run it to write STEP and STL for both parts, a STEP of the two assembled,
and to print the fit checks:

    python cad/oak_rail_mount.py
"""

from pathlib import Path

from build123d import (
    Box,
    Compound,
    Cylinder,
    GeomType,
    Plane,
    Polygon,
    Pos,
    Rot,
    ShapeList,
    Sphere,
    chamfer,
    export_step,
    export_stl,
    extrude,
)

# --------------------------------------------------------------------------
# MIL-STD-1913 rail, at maximum material condition (Figure 1, inches -> mm)
# --------------------------------------------------------------------------
RAIL_W_MAX = 21.20  # .835 across the shoulders
RAIL_W_BOT = 15.67  # .617 below the undercut
RAIL_TOP_CHAMFER = 1.42  # rise of the upper 45 deg chamfer off the top face
RAIL_LAND = 2.74  # .108 vertical land at full width
SLOT_W = 5.23  # .206 recoil slot
SLOT_DEPTH = 3.00  # .118
SLOT_PITCH = 10.01  # .394

RAIL_BEVEL = (RAIL_W_MAX - RAIL_W_BOT) / 2  # 45 deg, so rise == run
Z_LAND_TOP = -RAIL_TOP_CHAMFER  # top of the full-width land
Z_LAND_BOT = Z_LAND_TOP - RAIL_LAND  # where the undercut starts
Z_UNDERCUT = Z_LAND_BOT - RAIL_BEVEL  # bottom of the undercut, -6.925

# --------------------------------------------------------------------------
# OAK-D-Lite, measured from the Luxonis enclosure STEP/STL
# --------------------------------------------------------------------------
CAM_LEN = 91.0
CAM_HEIGHT = 28.0
CAM_DEPTH = 17.45
CAM_HOLE_PITCH = 75.00  # M4 holes, == the stereo baseline
CAM_HOLE_UP = 18.48  # above the bottom edge of the case
CAM_HOLE_DEPTH = 6.45  # tapped depth in the case, all the screw has to bite on
CAM_LENS_UP = 19.06  # optical axis above the bottom edge
CAM_BACK_FLAT_LO = 10.3  # the flat band on the back the pads bear on
CAM_BACK_FLAT_HI = 24.7

# --------------------------------------------------------------------------
# Adapter
# --------------------------------------------------------------------------
FIT = 0.25  # clearance on the rail's clamping faces
JAW_GAP = 0.80  # how far the wedge must rise before it touches

CAM_BOTTOM_Z = 10.0  # camera's bottom edge above the rail top surface.
# 10 mm leaves room for a right-angle USB-C plug, which points
# straight down out of the bottom of the case.  Drop it to ~3 if
# the rail ends before the camera and the cable can hang free.

PLATE_T = 6.0
PLATE_HALF_W = 44.5
PLATE_TOP = CAM_BOTTOM_Z + CAM_HOLE_UP + 7.0
PAD_W, PAD_H, PAD_PROUD = 14.0, 13.0, 2.0  # bearing pads, so the fins breathe

ROOF_T = 5.0
BODY_LEN = 34.0  # along the rail, behind the plate
SHELF_LEN = 10.0  # full-width buttress under the plate

JAW_INNER_X = RAIL_W_BOT / 2 + FIT  # fixed jaw hooks in to here
JAW_BOTTOM_Z = -9.0

WEDGE_TOE_X = RAIL_W_BOT / 2 + JAW_GAP
WEDGE_TOP_Z = -2.0  # travel available before it fouls the roof

# --------------------------------------------------------------------------
# Fasteners: four M4 x 10 screws and two 20-series M4 T-nuts, and nothing else
# --------------------------------------------------------------------------
# Ten millimetres is short for both jobs, so both pairs of heads are sunk into
# the part.  What the screw cannot reach comes out of the plastic rather than
# out of the thread, which is the half of the joint that carries the load.
SCREW_LEN = 10.0
HEAD_D = 7.6  # ISO 7380 button head; a DIN 912 cap head is 7.0 and also drops in
M4_CLEAR_D = 4.5
CBORE_D = HEAD_D + 0.8
CBORE_ROOF = 2.0  # into the 5 mm roof, so 3 mm of it is left under the head
CBORE_PLATE = 3.0  # into the 6 mm plate, so 5 mm of screw is left for the camera

# The T-nut, measured off the ones in hand: a 10 x 6 x 4 mm drop-in nut, 3 mm of
# that being flange and the last 1 mm the boss.  No vendor publishes a drawing of
# these, so the numbers are calipers rather than a datasheet.  The boss is the
# one to watch -- it is the whole of the thread standing above the seat, and at
# 1 mm it is a millimetre less than a T-nut of this width usually carries.
TNUT_HEAD_W = 10.0  # across the flanges, so across the slot when it is fitted
TNUT_NECK_W = 6.0  # the boss, which is the 20-series slot opening
TNUT_BOSS_H = 1.0  # flange top to boss top: the thread standing above the seat
TNUT_FLANGE_T = 3.0  # flange top down to the bottom of the nut
TNUT_LEN = 6.0  # along the channel
TNUT_THREAD = TNUT_BOSS_H + TNUT_FLANGE_T  # all there is for a screw to find
TNUT_SLOP = 0.4  # clearance across the channel

CHANNEL_W = TNUT_HEAD_W + TNUT_SLOP
NECK_W = TNUT_NECK_W + TNUT_SLOP
LEDGE_T = 3.0  # wedge left above the nut, which is what the clamp load pulls on
NUT_SEAT_Z = WEDGE_TOP_Z - LEDGE_T  # where the flanges bear
CHANNEL_WALL = 2.0  # wedge left either side of the channel
TOP_FIN = 2.0  # top face left inboard of the neck, so a chamfer still fits on it

# The slot has a floor for the same reason the extrusion it imitates does: it is
# the only thing tying the two sides of the slot together.  Cut the chamber out
# of the underside instead and the wedge stops being one part -- the neck takes
# the top of it apart and the open bottom takes the rest, and what is left is
# two loose rails with the nuts between them.
CHANNEL_FLOOR = NUT_SEAT_Z - TNUT_FLANGE_T - 0.4  # 0.4 clear under a seated nut
FLOOR_T = 2.0
WEDGE_BOTTOM_Z = CHANNEL_FLOOR - FLOOR_T


def ramp_x(z):
    """Where the wedge's 45 degree face stands at height z, with it at rest."""
    return WEDGE_TOE_X + (z - Z_UNDERCUT)


# The clamp screws stand as far inboard as the nut allows, because a millimetre
# outboard widens the whole clamp by two.  Two things hold them out: the nut
# wants a wall between it and the ramp, and the slot its boss rides in must
# leave a strip of the wedge's top face that a chamfer can still be cut on.
SCREW_X = max(
    ramp_x(NUT_SEAT_Z) + CHANNEL_WALL + CHANNEL_W / 2,
    ramp_x(WEDGE_TOP_Z) + TOP_FIN + NECK_W / 2,
)
SCREW_Y = (-14.0, -32.0)

# The neck is left filled between the two nuts, which closes the top of the
# wedge over its middle and stops the two sides of the slot working against each
# other.  It can only go there: a nut reaches its station by sliding in from the
# near end of the wedge, and its boss needs an open neck the whole way, so
# filling either end would leave nowhere for the nuts to come in.
CAP_Y = sum(SCREW_Y) / 2
CAP_LEN = abs(SCREW_Y[0] - SCREW_Y[1]) - TNUT_LEN - 2.0  # 1 mm clear of each nut

WEDGE_OUTER_X = SCREW_X + CHANNEL_W / 2 + CHANNEL_WALL
BODY_HALF_W = WEDGE_OUTER_X + 1.0

LUG_Y = -(PLATE_T + BODY_LEN / 2)  # recoil lug, centred in the clamp
LUG_LEN = SLOT_W - 0.50
LUG_DEPTH = SLOT_DEPTH - 0.25
LUG_HALF_W = 8.0

GUSSET_X, GUSSET_T = 30.0, 5.0  # outboard of the clamp screws, so a key can reach them
GUSSET_TOP = 22.0
# The gussets stand on the buttress, but stop a millimetre short of its back
# edge.  Run them all the way and the acute tip lands exactly on that edge,
# which is a vertex no chamfer can be built on -- and one such vertex fails the
# whole edge-breaking pass, not just itself.
GUSSET_LEN = SHELF_LEN - 1.0

KEY_CLEAR_D = 9.0  # room above each clamp screw for the head and a key

CHAMFER = 0.6  # every convex edge on both parts, bar the clamping faces


def _convex(part, edge, probe=0.35):
    """True when less than half a small ball on the edge lies inside the solid.

    Which is what tells an outside corner from an inside one, and only the
    outside ones want breaking -- chamfering an inside corner just cuts a
    notch into the joint you were relying on.
    """
    ball = Pos(*tuple(edge.center())) * Sphere(probe)
    return (ball & part).volume < 0.5 * ball.volume


def _along_rail(edge):
    """A straight edge running the length of the clamp rather than across it."""
    if edge.geom_type != GeomType.LINE:
        return False
    return abs((edge.end_point() - edge.start_point()).Y) > 0.9 * edge.length


def _clamping_body(edge):
    """The fixed jaw's inner profile -- the vertical land and the 45 deg hook."""
    c = edge.center()
    return _along_rail(edge) and edge.length > 20 and -11.5 < c.X < -7.5 and -8.5 < c.Z < 0.5


def _clamping_wedge(edge):
    """The wedge's toe, where its ramp meets the rail's undercut."""
    c = edge.center()
    return _along_rail(edge) and c.X < 10.0 and -7.5 < c.Z < -6.0


def _edge_key(edge):
    c = edge.center()
    return (round(c.X, 2), round(c.Y, 2), round(c.Z, 2), round(edge.length, 2))


def _unbroken(part, raw, clamping):
    """Edges of the raw solid that wanted breaking and came through untouched."""
    survived = {_edge_key(e) for e in part.edges()}
    return [
        e
        for e in raw.edges()
        if _convex(raw, e) and not clamping(e) and _edge_key(e) in survived
    ]


def break_edges(part, clamping):
    """Chamfer every outside edge except the ones that hold the rail.

    A sharp edge on a printed part is what catches a finger, what the first
    layer squashes out of square, and what stops a nut sitting flat, so the
    default is to break all of them and name the exceptions rather than the
    other way round -- which is how edges got missed the first time.
    """
    cut = ShapeList(
        [e for e in part.edges() if _convex(part, e) and not clamping(e)]
    )
    return chamfer(cut, length=CHAMFER)


def _yz(points, x_start, thickness):
    """Extrude a closed YZ profile sideways, for the gussets."""
    return Pos(x_start, 0, 0) * extrude(Plane.YZ * Polygon(*points, align=None), thickness)


def _xz(points, y_start, length):
    """Extrude a closed XZ profile from y_start backwards along the rail."""
    return Pos(0, y_start, 0) * extrude(Plane.XZ * Polygon(*points, align=None), -length)


def rail(length=140.0, slot_phase=0.0):
    """A reference length of rail at maximum material, for the fit checks."""
    half_top = RAIL_W_MAX / 2 - RAIL_TOP_CHAMFER
    profile = [
        (-half_top, 0),
        (half_top, 0),
        (RAIL_W_MAX / 2, Z_LAND_TOP),
        (RAIL_W_MAX / 2, Z_LAND_BOT),
        (RAIL_W_BOT / 2, Z_UNDERCUT),
        (RAIL_W_BOT / 2, -20.0),
        (-RAIL_W_BOT / 2, -20.0),
        (-RAIL_W_BOT / 2, Z_UNDERCUT),
        (-RAIL_W_MAX / 2, Z_LAND_BOT),
        (-RAIL_W_MAX / 2, Z_LAND_TOP),
    ]
    solid = _xz(profile, length / 2, length)
    for i in range(-12, 13):
        y = slot_phase + i * SLOT_PITCH
        if abs(y) < length / 2 - SLOT_W:
            solid -= Pos(0, y, -SLOT_DEPTH / 2) * Box(
                RAIL_W_MAX + 2, SLOT_W, SLOT_DEPTH
            )
    return solid


def body(chamfered=True):
    """Clamp roof, fixed jaw, recoil lug, camera plate."""
    # Roof over the rail, plus the full-width buttress under the plate.
    part = Pos(0, -(PLATE_T + BODY_LEN / 2), ROOF_T / 2) * Box(
        2 * BODY_HALF_W, BODY_LEN, ROOF_T
    )
    part += Pos(0, -(PLATE_T + SHELF_LEN / 2), ROOF_T / 2) * Box(
        2 * PLATE_HALF_W, SHELF_LEN, ROOF_T
    )

    # Fixed jaw: hangs down on -X and hooks under the rail's undercut.
    jaw = [
        (-BODY_HALF_W, 0),
        (-(RAIL_W_MAX / 2 + FIT), 0),
        (-(RAIL_W_MAX / 2 + FIT), Z_LAND_BOT),
        (-JAW_INNER_X, Z_UNDERCUT),
        (-JAW_INNER_X, JAW_BOTTOM_Z),
        (-BODY_HALF_W, JAW_BOTTOM_Z),
    ]
    part += _xz(jaw, -PLATE_T, BODY_LEN)

    # Recoil lug, dropping into one cross slot.
    part += Pos(0, LUG_Y, -LUG_DEPTH / 2) * Box(2 * LUG_HALF_W, LUG_LEN, LUG_DEPTH)

    # Two gussets tying the plate back to the roof, because the joint
    # between them is the one the camera's weight tries to peel open.
    gusset = [(-PLATE_T, ROOF_T), (-PLATE_T, GUSSET_TOP), (-PLATE_T - GUSSET_LEN, ROOF_T)]
    for sx in (-1, 1):
        part += _yz(gusset, sx * GUSSET_X - GUSSET_T / 2, GUSSET_T)

    # Camera plate, with raised pads so the heatsink fins stay open.
    part += Pos(0, -PLATE_T / 2, PLATE_TOP / 2) * Box(
        2 * PLATE_HALF_W, PLATE_T, PLATE_TOP
    )
    for sx in (-1, 1):
        part += Pos(
            sx * CAM_HOLE_PITCH / 2,
            PAD_PROUD / 2,
            CAM_BOTTOM_Z + (CAM_BACK_FLAT_LO + CAM_BACK_FLAT_HI) / 2,
        ) * Box(PAD_W, PAD_PROUD, PAD_H)

    # Camera screw holes, through the plate and its pads, each head sunk into
    # the back of the plate far enough that an M4 x 10 still finds the thread.
    for sx in (-1, 1):
        part -= (
            Pos(sx * CAM_HOLE_PITCH / 2, 0, CAM_BOTTOM_Z + CAM_HOLE_UP)
            * Rot(90, 0, 0)
            * Cylinder(M4_CLEAR_D / 2, 40)
        )
        part -= (
            Pos(
                sx * CAM_HOLE_PITCH / 2,
                -PLATE_T - 1.0 + (CBORE_PLATE + 1.0) / 2,
                CAM_BOTTOM_Z + CAM_HOLE_UP,
            )
            * Rot(90, 0, 0)
            * Cylinder(CBORE_D / 2, CBORE_PLATE + 1.0)
        )

    # Clamp screw holes, vertical through the roof, counterbored for the same
    # reason: an M4 x 10 starting at the top of a 5 mm roof reaches nothing.
    for sy in SCREW_Y:
        part -= Pos(SCREW_X, sy, 0) * Cylinder(M4_CLEAR_D / 2, 60)
        part -= Pos(
            SCREW_X, sy, ROOF_T - CBORE_ROOF + (CBORE_ROOF + 1.0) / 2
        ) * Cylinder(CBORE_D / 2, CBORE_ROOF + 1.0)

    # Break the edges last, so the pass sees the finished shape.
    return break_edges(part, _clamping_body) if chamfered else part


def _wedge_profile():
    toe_top_x = WEDGE_TOE_X + (WEDGE_TOP_Z - Z_UNDERCUT)
    return [
        (WEDGE_TOE_X, Z_UNDERCUT),
        (toe_top_x, WEDGE_TOP_Z),
        (WEDGE_OUTER_X, WEDGE_TOP_Z),
        (WEDGE_OUTER_X, WEDGE_BOTTOM_Z),
        (WEDGE_TOE_X, WEDGE_BOTTOM_Z),
    ]


def channel():
    """The T-slot down the length of the wedge, which is what holds the nuts.

    A nut in a blind pocket has to be held in place while the screw hunts for
    it, and a 20-series T-nut is too wide to bury in a wedge this size anyway.
    Cutting the slot the nut was made for answers both: it is open at both ends
    so the nuts slide in, its walls stop one turning, and the ledge the flanges
    pull up on runs the whole length of the wedge instead of one nut's worth.
    """
    y_mid = -(PLATE_T + BODY_LEN / 2)
    run = BODY_LEN + 4.0  # straight out of both ends of the wedge
    chamber = Pos(SCREW_X, y_mid, (CHANNEL_FLOOR + NUT_SEAT_Z) / 2) * Box(
        CHANNEL_W, run, NUT_SEAT_Z - CHANNEL_FLOOR
    )
    neck = Pos(SCREW_X, y_mid, (NUT_SEAT_Z + WEDGE_TOP_Z + 1.0) / 2) * Box(
        NECK_W, run, (WEDGE_TOP_Z + 1.0) - NUT_SEAT_Z
    )
    cap = Pos(SCREW_X, CAP_Y, (NUT_SEAT_Z + WEDGE_TOP_Z + 2.0) / 2) * Box(
        NECK_W + 2.0, CAP_LEN, (WEDGE_TOP_Z + 2.0) - NUT_SEAT_Z
    )
    return chamber + (neck - cap)


def tnut(y):
    """One T-nut where the screw pulls it: flanges up under the ledge."""
    flange = Pos(SCREW_X, y, NUT_SEAT_Z - TNUT_FLANGE_T / 2) * Box(
        TNUT_HEAD_W, TNUT_LEN, TNUT_FLANGE_T
    )
    boss = Pos(SCREW_X, y, NUT_SEAT_Z + TNUT_BOSS_H / 2) * Box(
        TNUT_NECK_W, TNUT_LEN, TNUT_BOSS_H
    )
    return flange + boss


def wedge(chamfered=True):
    """The moving jaw: a 45 degree wedge pulled up by the two clamp screws."""
    part = _xz(_wedge_profile(), -(PLATE_T + 0.4), BODY_LEN - 0.8) - channel()
    return break_edges(part, _clamping_wedge) if chamfered else part


def checks():
    """Assemble against a reference rail and report the fit.

    Nothing here is a substitute for offering the printed part up to the real
    rail, but it does catch a jaw that cuts into the rail, a wedge that binds
    before it grips, and a lug that will not enter a slot.
    """
    b, w, r = body(), wedge(), rail(slot_phase=LUG_Y)
    ok = True

    def clash(name, a, c, want_clear=True):
        nonlocal ok
        v = (a & c).volume
        clear = v < 1e-6
        ok &= clear == want_clear
        verb = "clear of" if clear else f"cuts into ({v:.2f} mm3)"
        print(f"  {'ok ' if clear == want_clear else 'BAD'} {name} {verb} the rail")

    # Ask first whether there is still a part at all.  A pocket deep enough to
    # meet another one cuts a part in two without failing anything else here:
    # it still clears the rail, still takes the nut, still exports, and the STL
    # carries both halves quite happily.
    for label, part in (("body", b), ("wedge", w)):
        n = len(part.solids())
        ok &= n == 1
        print(f"  {'ok ' if n == 1 else 'BAD'} {label} is {n} solid piece(s)")

    print("fit against a max-material MIL-STD-1913 rail:")
    clash("body", b, r)
    clash("wedge at rest", w, r)
    clash("body vs wedge", b, w)

    # The wedge must reach the rail's 45 deg face before it hits the roof.
    touch = None
    for lift in [i * 0.05 for i in range(1, 60)]:
        if (Pos(0, 0, lift) * wedge() & r).volume > 1e-6:
            touch = lift
            break
    # What is left of that travel is the gap under the roof once the clamp is
    # tight, and it is meant to still be there: a wedge that reached the roof
    # would be resting on the body instead of pulling on the rail.
    travel = -WEDGE_TOP_Z
    print(
        f"  wedge grips after {touch:.2f} mm of lift, {travel:.2f} mm available, "
        f"so {travel - touch:.2f} mm of gap is left under the roof"
    )
    ok &= touch is not None and touch < travel

    # Each T-nut has to lie in the channel without touching the wedge, and the
    # ledge over it has to overlap its flanges by enough to pull on.
    for sy in SCREW_Y:
        v = (tnut(sy) & w).volume
        ok &= v < 1e-6
        print(f"  {'ok ' if v < 1e-6 else 'BAD'} T-nut at y={sy:.0f} lies in the channel")
    bearing = (TNUT_HEAD_W - NECK_W) / 2
    ok &= bearing > 1.0
    print(
        f"  {'ok ' if bearing > 1.0 else 'BAD'} flanges pull on {bearing:.2f} mm of "
        f"ledge a side, {2 * bearing * TNUT_LEN:.0f} mm2 in all, through "
        f"{LEDGE_T:.1f} mm of wedge"
    )
    apart = abs(SCREW_Y[0] - SCREW_Y[1]) - TNUT_LEN
    ok &= apart > 2.0
    print(f"  {'ok ' if apart > 2.0 else 'BAD'} the two nuts miss each other by {apart:.1f} mm")

    # The fill between the nuts has to clear both of them, or a nut cannot reach
    # its station: it slides in from the near end of the wedge, boss in the neck.
    run_in = abs(SCREW_Y[0] - CAP_Y) - TNUT_LEN / 2 - CAP_LEN / 2
    ok &= run_in >= 0.5
    print(
        f"  {'ok ' if run_in >= 0.5 else 'BAD'} the {CAP_LEN:.0f} mm fill between the "
        f"nuts clears each by {run_in:.1f} mm"
    )

    # What is left of the wedge around that channel.  The inboard wall is the
    # tight one, because the ramp leans out over it as it rises.
    for label, got, want in (
        ("wall inboard of the channel", (SCREW_X - CHANNEL_W / 2) - ramp_x(NUT_SEAT_Z), CHANNEL_WALL),
        ("wall outboard of the channel", WEDGE_OUTER_X - (SCREW_X + CHANNEL_W / 2), CHANNEL_WALL),
        ("of top face inboard of the neck", (SCREW_X - NECK_W / 2) - ramp_x(WEDGE_TOP_Z), TOP_FIN),
    ):
        ok &= got >= want - 0.01
        print(f"  {'ok ' if got >= want - 0.01 else 'BAD'} {got:.2f} mm {label}")

    # A clamp screw that fouls the rail cannot be fitted at all, and one
    # buried under a gusset cannot be turned.
    for sy in SCREW_Y:
        v = (Pos(SCREW_X, sy, 0) * Cylinder(CBORE_D / 2, 60) & r).volume
        ok &= v < 1e-6
        print(f"  {'ok ' if v < 1e-6 else 'BAD'} clamp screw at y={sy:.0f} misses the rail")
        key = Pos(SCREW_X, sy, ROOF_T + 30) * Cylinder(KEY_CLEAR_D / 2, 60)
        v = (key & b).volume
        ok &= v < 1e-6
        print(f"  {'ok ' if v < 1e-6 else 'BAD'} a key reaches the screw at y={sy:.0f}")

    # Both counterbores have to leave the part enough to bear on, and both
    # screws have to come out the far side with thread to spare.
    for label, left in (
        ("roof under the clamp screw heads", ROOF_T - CBORE_ROOF),
        ("plate behind the camera screw heads", PLATE_T - CBORE_PLATE),
    ):
        ok &= left >= 2.5
        print(f"  {'ok ' if left >= 2.5 else 'BAD'} {left:.1f} mm of {label}")

    tip_z = ROOF_T - CBORE_ROOF - SCREW_LEN
    into_nut = min((NUT_SEAT_Z + TNUT_BOSS_H) - tip_z, TNUT_THREAD)
    # Three millimetres is the bar rather than four because there are only four
    # to have.  What that buys is not the weak end of this joint anyway: 3 mm of
    # M4 in a steel nut will outlast the screw, while the plastic under the head
    # and the ledge under the flanges both give up around a kilonewton.
    good = into_nut >= 3.0 and tip_z > CHANNEL_FLOOR
    print(
        f"  {'ok ' if good else 'BAD'} clamp screw takes {into_nut:.1f} of the "
        f"T-nut's {TNUT_THREAD:.0f} mm of thread, tip "
        f"{tip_z - CHANNEL_FLOOR:.1f} mm above the slot floor"
    )
    ok &= good

    # A counterbore is wide enough to break out of a part that has grown a hole
    # or shrunk around it, and it would do it quietly.
    for label, margin in (
        ("roof outboard of its counterbore", BODY_HALF_W - SCREW_X - CBORE_D / 2),
        ("plate outboard of its counterbore", PLATE_HALF_W - CAM_HOLE_PITCH / 2 - CBORE_D / 2),
        ("plate above its counterbore", PLATE_TOP - CAM_BOTTOM_Z - CAM_HOLE_UP - CBORE_D / 2),
    ):
        ok &= margin >= 1.5
        print(f"  {'ok ' if margin >= 1.5 else 'BAD'} {margin:.2f} mm of {label}")

    into_case = SCREW_LEN - (PLATE_T + PAD_PROUD - CBORE_PLATE)
    room = CAM_HOLE_DEPTH - into_case
    ok &= into_case >= 4.0 and room >= 0.5
    print(
        f"  {'ok ' if into_case >= 4.0 and room >= 0.5 else 'BAD'} camera screw takes "
        f"{into_case:.1f} mm of the case's {CAM_HOLE_DEPTH:.2f} mm thread, "
        f"{room:.2f} mm short of the bottom"
    )

    # Every outside edge should have been broken.  Missing some is exactly the
    # sort of thing that is invisible until the part is in your hand.
    for label, part, raw, clamping in (
        ("body", b, body(chamfered=False), _clamping_body),
        ("wedge", w, wedge(chamfered=False), _clamping_wedge),
    ):
        left = _unbroken(part, raw, clamping)
        kept = [e for e in raw.edges() if clamping(e)]
        ok &= not left
        if left:
            print(f"  BAD {label}: {len(left)} outside edge(s) left sharp")
            for e in left:
                c = e.center()
                print(f"        ({c.X:7.2f},{c.Y:7.2f},{c.Z:7.2f}) len {e.length:.2f}")
        else:
            n = len(raw.edges()) - len(kept)
            print(f"  ok  {label}: every outside edge broken, {len(kept)} clamping edge(s) left sharp")

    lug_side = (SLOT_W - LUG_LEN) / 2
    print(f"  recoil lug {LUG_LEN:.2f} long in a {SLOT_W:.2f} slot ({lug_side:.2f} mm a side)")
    print(f"  optical axis {CAM_BOTTOM_Z + CAM_LENS_UP:.1f} mm above the rail top surface")
    print(f"  clamp {2 * BODY_HALF_W:.1f} mm across, screws {2 * SCREW_X:.1f} mm apart")
    print(f"  all four screws M4 x {SCREW_LEN:.0f}, both nuts 20-series M4 T-nuts")
    return ok


def camera():
    """A block the size of the camera, so the assembly can be looked at."""
    return Pos(0, CAM_DEPTH / 2, CAM_BOTTOM_Z + CAM_HEIGHT / 2) * Box(
        CAM_LEN, CAM_DEPTH, CAM_HEIGHT
    )


def show(section_at=None):
    """Throw the assembly at the OCP CAD Viewer panel in VS Code.

    Open the viewer first (command palette, "OCP CAD Viewer: Open viewer").
    Pass section_at to slice everything at that station along the rail, which
    is the only way to see what the jaws are really doing.
    """
    import os

    from ocp_vscode import Camera
    from ocp_vscode import show as ocp_show

    port = int(os.environ.get("OCP_PORT", 3939))
    nuts = tnut(SCREW_Y[0]) + tnut(SCREW_Y[1])
    shown = [
        ("body", "#3a7bc8", 1.0, body()),
        ("wedge", "#e08c2a", 1.0, wedge()),
        ("nuts", "#d0d4d8", 1.0, nuts),
        ("rail", "#8a9099", 1.0, rail(length=80, slot_phase=LUG_Y)),
        ("camera", "#c8c8cc", 0.35, camera()),
    ]
    if section_at is not None:
        slab = Pos(0, section_at, 0) * Box(400, 1.0, 400)
        # A station behind the plate misses the camera entirely, and an empty
        # solid is not something the viewer can be handed.
        shown = [(n, c, a, p & slab) for n, c, a, p in shown]
        shown = [row for row in shown if row[3].volume > 1e-9]

    ocp_show(
        *[p for _, _, _, p in shown],
        names=[n for n, _, _, _ in shown],
        colors=[c for _, c, _, _ in shown],
        alphas=[a for _, _, a, _ in shown],
        transparent=True,
        port=port,
        reset_camera=Camera.RESET,
    )


if __name__ == "__main__":
    import sys

    if "--show" in sys.argv:
        at = None
        if "--section" in sys.argv:
            at = float(sys.argv[sys.argv.index("--section") + 1])
        show(section_at=at)
        raise SystemExit(0)

    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    b, w = body(), wedge()
    for name, part in (("oak_rail_body", b), ("oak_rail_wedge", w)):
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"))
        bb = part.bounding_box()
        print(
            f"{name}: {part.volume/1000:.1f} cm3   "
            f"{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm"
        )

    # And one file with both parts where they actually sit, named, for anything
    # that wants the assembly rather than two things to print.  The wedge is at
    # rest in it, so the gap under the roof is the clamp's unused travel.
    b.label, w.label = "body", "wedge"
    asm = Compound(label="oak_rail_mount", children=[b, w])
    export_step(asm, str(out / "oak_rail_mount.step"))
    abb = asm.bounding_box()
    print(
        f"oak_rail_mount: both parts assembled, {asm.volume/1000:.1f} cm3   "
        f"{abb.size.X:.1f} x {abb.size.Y:.1f} x {abb.size.Z:.1f} mm"
    )
    print()
    raise SystemExit(0 if checks() else 1)
