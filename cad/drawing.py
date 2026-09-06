"""Printable orthographic drawings of the rail mount, one part per A4 page.

Each page carries the three axis-aligned views -- looking at the front of the
part, down onto it, and in from its right -- laid out in third angle, plus an
isometric to say what you are looking at.  The views are at full size, so a
printed part can be held against the paper, and every sheet carries a 100 mm
rule to catch a printer that has quietly scaled the page.

Hidden detail is drawn as fine grey dashes: the nut pockets in the wedge and
the camera screw holes in the body are only visible that way.

    python cad/drawing.py            writes cad/out/oak_rail_mount.pdf
    python cad/drawing.py --svg      the same sheets as SVG, to look at

Views come from build123d's hidden line removal, which projects the real
solid rather than a mesh of it, so what is on the paper is the part.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date
from math import atan2, cos, degrees, hypot, radians, sin
from pathlib import Path

from build123d import GeomType, Vector

from oak_rail_mount import (
    CAM_BOTTOM_Z,
    CAM_HOLE_PITCH,
    CAM_HOLE_UP,
    CAM_LENS_UP,
    JAW_BOTTOM_Z,
    LEDGE_T,
    NUT_SEAT_Z,
    PLATE_T,
    ROOF_T,
    SCREW_LEN,
    SCREW_X,
    SCREW_Y,
    WEDGE_OUTER_X,
    WEDGE_TOE_X,
    WEDGE_TOP_Z,
    body,
    wedge,
)
from sheet import Sheet, Text, read_back, text_width, write_pdf, write_svg

# --------------------------------------------------------------------------
# Sheet furniture.  A4 landscape, because the body is 89 mm across and wants
# its three views side by side at full size.
# --------------------------------------------------------------------------
SHEET_W, SHEET_H = 297.0, 210.0
FRAME = 8.0  # border, in from the paper edge
TITLE_H = 32.0  # strip along the bottom, inside the border
VIEW_GAP = 18.0  # between neighbouring views
DIM_PAD = 25.0  # room left of and below the block, for dimensions

W_OUTLINE = 0.45  # line weights, mm
W_HIDDEN = 0.18
W_THIN = 0.13
DASH = (1.4, 0.9)
GREY_HIDDEN = 0.45
TEXT = 2.5  # nominal text height, mm

# Which way is right and which is up, on the page, for each view.  Front looks
# at the plate the camera bolts to -- from where the camera itself is, so the
# rail's +X runs to the left -- and the other two follow from it in third
# angle: the top view goes above the front, the right view to its right.
FRONT = ((-1, 0, 0), (0, 0, 1))
TOP = ((-1, 0, 0), (0, -1, 0))
RIGHT = ((0, -1, 0), (0, 0, 1))


# --------------------------------------------------------------------------
# Projection
# --------------------------------------------------------------------------
@dataclass
class View:
    """One projection, in millimetres of paper, plus where it sits on it."""

    label: str = ""
    visible: list = field(default_factory=list)
    hidden: list = field(default_factory=list)
    show_hidden: bool = True
    scale: float = 1.0
    ox: float = 0.0  # page point that the part's origin projects to
    oy: float = 0.0
    below: float = 0.0  # how far under the view its dimensions reach

    def px(self, sx: float) -> float:
        return self.ox + sx * self.scale

    def py(self, sy: float) -> float:
        return self.oy + sy * self.scale

    def bounds(self) -> tuple[float, float, float, float]:
        pts = [p for line in self.visible for p in line]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return min(xs), min(ys), max(xs), max(ys)

    def size(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.bounds()
        return x1 - x0, y1 - y0


def _flatten(edge, chord=0.15):
    """An edge as a run of points, straight lines kept as their two ends."""
    if edge.length < 1e-6:
        return []
    steps = 1 if edge.geom_type == GeomType.LINE else max(8, int(edge.length / chord))
    return [((edge @ (i / steps)).X, (edge @ (i / steps)).Y) for i in range(steps + 1)]


def project(solid, right, up, label="") -> View:
    """Look at `solid` with `right` running right and `up` running up the page.

    The eye goes on the axis right x up, far enough away that the projection
    is parallel, and the part's origin lands at (0, 0) in the returned
    coordinates -- which is what lets two views share an axis on the sheet.
    """
    right_v, up_v = Vector(right).normalized(), Vector(up).normalized()
    eye = right_v.cross(up_v)  # build123d takes the direction to the viewer
    visible, hidden = solid.project_to_viewport(eye * 1000.0, up_v, look_at=(0, 0, 0))
    return View(
        label=label,
        visible=[line for e in visible if (line := _flatten(e))],
        hidden=[line for e in hidden if (line := _flatten(e))],
    )


def isometric(solid, eye, label="isometric") -> View:
    """A pictorial view down `eye`, with the part's Z still going up the page.

    Hidden lines are left off: on a pictorial they are noise, and the three
    orthographic views next to it show everything they would have.
    """
    eye_v = Vector(eye).normalized()
    up_v = (Vector(0, 0, 1) - eye_v * Vector(0, 0, 1).dot(eye_v)).normalized()
    view = project(solid, up_v.cross(eye_v), up_v, label)
    view.show_hidden = False
    return view


# --------------------------------------------------------------------------
# Putting a view on the sheet
# --------------------------------------------------------------------------
def draw(sheet: Sheet, view: View) -> None:
    if view.show_hidden:
        for line in view.hidden:
            sheet.polyline(
                [(view.px(x), view.py(y)) for x, y in line],
                weight=W_HIDDEN,
                grey=GREY_HIDDEN,
                dash=DASH,
            )
    for line in view.visible:
        sheet.polyline([(view.px(x), view.py(y)) for x, y in line], weight=W_OUTLINE)


def caption(sheet: Sheet, view: View, text=None) -> None:
    """Name the view, clear of whatever dimensions hang below it."""
    x0, y0, x1, _ = view.bounds()
    sheet.text(
        view.px((x0 + x1) / 2),
        view.py(y0) - view.below - 6.5,
        text or view.label,
        size=TEXT,
        grey=0.25,
        anchor="middle",
    )


def _number(value: float) -> str:
    return f"{value:.2f}".rstrip("0").rstrip(".")


def _arrow(sheet: Sheet, tip, towards, length=2.6, half=0.45):
    """A solid arrowhead at `tip`, pointing away from `towards`."""
    angle = atan2(tip[1] - towards[1], tip[0] - towards[0])
    back = (tip[0] - length * cos(angle), tip[1] - length * sin(angle))
    across = (half * -sin(angle), half * cos(angle))
    sheet.fill(
        [
            tip,
            (back[0] + across[0], back[1] + across[1]),
            (back[0] - across[0], back[1] - across[1]),
        ]
    )


def _dimension(sheet: Sheet, a, b, line_a, line_b, text: str):
    """A dimension between two page points, its line running a->b, offset out."""
    for start, end in ((a, line_a), (b, line_b)):
        run = hypot(end[0] - start[0], end[1] - start[1])
        if run < 0.1:
            continue
        ux, uy = (end[0] - start[0]) / run, (end[1] - start[1]) / run
        sheet.line(
            start[0] + ux,
            start[1] + uy,
            end[0] + ux * 1.6,
            end[1] + uy * 1.6,
            weight=W_THIN,
            grey=0.2,
        )
    sheet.line(*line_a, *line_b, weight=W_THIN, grey=0.2)
    _arrow(sheet, line_a, line_b)
    _arrow(sheet, line_b, line_a)

    angle = degrees(atan2(line_b[1] - line_a[1], line_b[0] - line_a[0]))
    if angle > 90 or angle <= -90:
        angle += 180  # so nothing ever reads upside down
    mid = ((line_a[0] + line_b[0]) / 2, (line_a[1] + line_b[1]) / 2)
    off = radians(angle + 90)
    sheet.text(
        mid[0] + cos(off),
        mid[1] + sin(off),
        text,
        size=TEXT,
        anchor="middle",
        angle=angle,
    )


def dim_h(sheet, view, sx1, sx2, sy, offset, text=None):
    """Across the page: `offset` is measured out from the bottom of the view.

    The extension lines start at whatever feature `sy` names -- a hole's
    centre as readily as an outside edge -- and run past the dimension line.
    """
    y = view.py(view.bounds()[1]) - offset
    view.below = max(view.below, offset)
    _dimension(
        sheet,
        (view.px(sx1), view.py(sy)),
        (view.px(sx2), view.py(sy)),
        (view.px(sx1), y),
        (view.px(sx2), y),
        text or _number(abs(sx2 - sx1)),
    )


def dim_v(sheet, view, sy1, sy2, sx, offset, text=None, right=False):
    """Up the page: `offset` out from the left edge of the view, or the right."""
    edge = view.bounds()[2] if right else view.bounds()[0]
    x = view.px(edge) + (offset if right else -offset)
    _dimension(
        sheet,
        (view.px(sx), view.py(sy1)),
        (view.px(sx), view.py(sy2)),
        (x, view.py(sy1)),
        (x, view.py(sy2)),
        text or _number(abs(sy2 - sy1)),
    )


def centre_mark(sheet, view, sx, sy, reach=4.5):
    """The cross through a hole, so a dimension has something to point at."""
    for dx, dy in ((reach, 0.0), (0.0, reach)):
        sheet.line(
            view.px(sx) - dx,
            view.py(sy) - dy,
            view.px(sx) + dx,
            view.py(sy) + dy,
            weight=W_THIN,
            grey=0.3,
            dash=(2.0, 0.8),
        )


# --------------------------------------------------------------------------
# Layout
# --------------------------------------------------------------------------
def lay_out(front: View, top: View, right: View, area) -> tuple[float, ...]:
    """Third angle: top above front, right to the right, axes lined up.

    All three views put the part's origin at their own (0, 0), so lining them
    up is a matter of sharing one number: front and top share the page x that
    the origin lands on, front and right share the page y.
    """
    ax0, ay0, _, _ = area
    fx0, fy0, fx1, fy1 = front.bounds()
    tx0, ty0, tx1, ty1 = top.bounds()
    rx0, ry0, rx1, ry1 = right.bounds()

    front.ox = top.ox = ax0 - min(fx0, tx0)
    front.oy = right.oy = ay0 - min(fy0, ry0)
    right.ox = front.px(max(fx1, tx1)) + VIEW_GAP - rx0
    top.oy = front.py(max(fy1, ry1)) + VIEW_GAP - ty0

    return (
        ax0,
        ay0,
        max(right.px(rx1), front.px(max(fx1, tx1))),
        max(top.py(ty1), front.py(max(fy1, ry1))),
    )


def fit_scale(view: View, box_w: float, box_h: float) -> float:
    """The largest of the usual ratios that still fits the view in the box."""
    width, height = view.size()
    for scale in (5.0, 4.0, 2.0, 1.0, 1 / 2, 1 / 2.5, 1 / 4, 1 / 5, 1 / 10):
        if width * scale <= box_w and height * scale <= box_h:
            return scale
    return 1 / 10


def scale_label(scale: float) -> str:
    if scale == 1:
        return "1:1"
    return f"{_number(scale)}:1" if scale > 1 else f"1:{_number(1 / scale)}"


def place(view: View, box, fixed=None) -> None:
    """Centre a view in a page rectangle, at a fixed ratio or the best fit."""
    x0, y0, x1, y1 = box
    view.scale = fixed if fixed and fit_scale(view, x1 - x0, y1 - y0) >= fixed else 0.0
    view.scale = view.scale or fit_scale(view, x1 - x0, y1 - y0)
    vx0, vy0, vx1, vy1 = view.bounds()
    width, height = (vx1 - vx0) * view.scale, (vy1 - vy0) * view.scale
    view.ox = x0 + (x1 - x0 - width) / 2 - vx0 * view.scale
    view.oy = y0 + (y1 - y0 - height) / 2 - vy0 * view.scale


def frame_and_title(sheet: Sheet, title: str, notes: list[str]) -> None:
    """Border, title strip and the rule that catches a rescaled printout."""
    sheet.rect(FRAME, FRAME, SHEET_W - 2 * FRAME, SHEET_H - 2 * FRAME, weight=0.35)
    strip = FRAME + TITLE_H
    sheet.line(FRAME, strip, SHEET_W - FRAME, strip, weight=0.35)

    sheet.text(FRAME + 4, strip - 7.5, title, size=4.6)
    for i, note in enumerate(notes):
        sheet.text(FRAME + 4, strip - 13.5 - 4.2 * i, note, size=2.4, grey=0.2)

    sheet.text(
        SHEET_W - FRAME - 4, strip - 7.5, "third angle projection", size=2.6, grey=0.2,
        anchor="end",
    )
    sheet.text(
        SHEET_W - FRAME - 4,
        strip - 12.5,
        "print at 100% -- do not fit to page",
        size=2.6,
        grey=0.2,
        anchor="end",
    )
    # A rule, because the one thing that ruins a full size drawing is a print
    # dialogue quietly fitting it to the page.
    x0, y = SHEET_W - FRAME - 104.0, FRAME + 6.0
    sheet.line(x0, y, x0 + 100, y, weight=0.35)
    for mm in range(0, 101, 10):
        sheet.line(x0 + mm, y, x0 + mm, y + (3.0 if mm % 50 == 0 else 1.8), weight=0.25)
    for mm, text in ((0, "0"), (50, "50"), (100, "100 mm")):
        sheet.text(x0 + mm, y + 4.4, text, size=2.2, grey=0.2, anchor="middle")


def part_sheet(solid, title, notes, extras, annotate=None) -> Sheet:
    """One page: the three axis views, then the extras down the right of them."""
    sheet = Sheet(SHEET_W, SHEET_H)
    front = project(solid, *FRONT, "FRONT")
    top = project(solid, *TOP, "TOP")
    right = project(solid, *RIGHT, "RIGHT")

    # A view whose projection is not the size of the part is not at scale,
    # which is the one thing this drawing is for.
    box = solid.bounding_box()
    for view, want in (
        (front, (box.size.X, box.size.Z)),
        (top, (box.size.X, box.size.Y)),
        (right, (box.size.Y, box.size.Z)),
    ):
        got = view.size()
        if abs(got[0] - want[0]) > 0.02 or abs(got[1] - want[1]) > 0.02:
            raise ValueError(
                f"{view.label} came out {got[0]:.2f} x {got[1]:.2f} mm, "
                f"but the part is {want[0]:.2f} x {want[1]:.2f} mm"
            )

    area = (
        FRAME + 3 + DIM_PAD,
        FRAME + TITLE_H + 3 + DIM_PAD,
        SHEET_W - FRAME - 3,
        SHEET_H - FRAME - 3,
    )
    block = lay_out(front, top, right, area)

    # Centre the three views in whatever the block did not need, so a small
    # part does not end up huddled in one corner of the sheet.
    spare_x = max(0.0, (area[2] - block[2]) * 0.3)
    spare_y = max(0.0, (area[3] - block[3]) / 2)
    for view in (front, top, right):
        view.ox += spare_x
        view.oy += spare_y

    for view in (front, top, right):
        draw(sheet, view)
    if annotate:
        annotate(sheet, front, top, right)
    for view in (front, top, right):
        caption(sheet, view)

    # The extras share the column left over to the right of the block.
    column = (block[2] + spare_x + VIEW_GAP, area[1], area[2], area[3])
    slot = (column[3] - column[1]) / max(1, len(extras))
    for i, (view, fixed) in enumerate(extras):
        top_of_slot = column[3] - i * slot
        place(view, (column[0], top_of_slot - slot + 10, column[2], top_of_slot), fixed)
        draw(sheet, view)
        caption(sheet, view, f"{view.label} -- {scale_label(view.scale)}")

    frame_and_title(sheet, title, notes)
    return sheet


# --------------------------------------------------------------------------
# The two parts
# --------------------------------------------------------------------------
def _envelope(solid) -> str:
    box = solid.bounding_box()
    return (
        f"{box.size.X:.1f} x {box.size.Y:.1f} x {box.size.Z:.1f} mm envelope, "
        f"{solid.volume / 1000:.1f} cm3 of plastic"
    )


def body_dimensions(sheet, front, top, right):
    hole_x, hole_z = CAM_HOLE_PITCH / 2, CAM_BOTTOM_Z + CAM_HOLE_UP
    for sign in (-1, 1):
        centre_mark(sheet, front, sign * hole_x, hole_z)
    # Front: the camera screws first, the overall width outside them.
    dim_h(sheet, front, -hole_x, hole_x, hole_z, 9.0)
    dim_h(sheet, front, -44.5, 44.5, -9.0, 18.0)
    # Front: heights, both read from Z = 0, which is the rail's top face.
    dim_v(sheet, front, 0.0, hole_z, -hole_x, 9.0)
    dim_v(sheet, front, -9.0, 35.48, -44.5, 18.0)
    # Top: the clamp screws along the rail, then the overall depth.
    for y in SCREW_Y:
        centre_mark(sheet, top, -SCREW_X, -y)
    dim_h(sheet, top, -44.5, 44.5, -2.0, 9.0)
    dim_v(sheet, top, -SCREW_Y[0], -SCREW_Y[1], -SCREW_X, 9.0)
    dim_v(sheet, top, -2.0, 40.0, -44.5, 18.0)
    dim_v(sheet, top, 0.0, -SCREW_Y[0], -SCREW_X, 9.0, right=True)
    # Right: how the depth splits between plate and clamp, and the roof.
    dim_h(sheet, right, 0.0, PLATE_T, 0.0, 9.0)
    dim_h(sheet, right, -2.0, 40.0, -9.0, 18.0)
    dim_v(sheet, right, 0.0, ROOF_T, 40.0, 9.0, right=True)


def wedge_dimensions(sheet, front, top, right):
    for y in SCREW_Y:
        centre_mark(sheet, top, -SCREW_X, -y, reach=4.0)
    dim_h(sheet, front, -WEDGE_OUTER_X, -WEDGE_TOE_X, JAW_BOTTOM_Z, 9.0)
    dim_v(sheet, front, JAW_BOTTOM_Z, WEDGE_TOP_Z, -WEDGE_OUTER_X, 9.0)
    dim_v(sheet, front, NUT_SEAT_Z, WEDGE_TOP_Z, -SCREW_X, 4.0, right=True)
    dim_v(sheet, top, -SCREW_Y[0], -SCREW_Y[1], -SCREW_X, 9.0)
    dim_v(sheet, top, 6.4, 39.6, -WEDGE_OUTER_X, 18.0)
    dim_v(sheet, top, 6.4, -SCREW_Y[0], -SCREW_X, 9.0, right=True)
    dim_h(sheet, right, 6.4, 39.6, -9.0, 9.0)


def sheets() -> list[Sheet]:
    solid_body, solid_wedge = body(), wedge()
    stamp = f"drawn from cad/oak_rail_mount.py, {date.today().isoformat()}"
    return [
        part_sheet(
            solid_body,
            "OAK-D-Lite rail mount  --  BODY  (1 of 2)",
            [
                _envelope(solid_body),
                "PETG or ABS, plate face down on the bed, 4 perimeters, 40% infill, "
                "supports under the jaws",
                f"4 x M4 x {SCREW_LEN:.0f} throughout: two into the case at "
                f"{CAM_HOLE_PITCH:.0f} mm centres, two down to the T-nuts in the "
                f"wedge.  Both pairs of heads sink into their counterbores",
                "FRONT looks at the plate the camera bolts to.  Z = 0 is the top "
                f"face of the rail, and the optical axis lands "
                f"{CAM_BOTTOM_Z + CAM_LENS_UP:.0f} mm above it",
                stamp,
            ],
            extras=[(isometric(solid_body, (1, 1, 1)), None)],
            annotate=body_dimensions,
        ),
        part_sheet(
            solid_wedge,
            "OAK-D-Lite rail mount  --  WEDGE  (2 of 2)",
            [
                _envelope(solid_wedge),
                "PETG or ABS, underside on the bed, no supports; the ramp is a 45 "
                "degree overhang and the slot's roof bridges the chamber",
                "2 x 20-series M4 T-nuts slide into either end of the T-slot; "
                f"their flanges pull up on the {LEDGE_T:.0f} mm ledge over them",
                "sits under the body's roof; FRONT is the profile that does the "
                "work, and the T-slot is the notch out of its top",
                stamp,
            ],
            extras=[
                (isometric(solid_wedge, (-1, 1, -1), "isometric, from below"), None),
                (project(solid_wedge, *FRONT, "FRONT, enlarged"), 4.0),
            ],
            annotate=wedge_dimensions,
        ),
    ]


def check(path, pages: list[Sheet]) -> bool:
    """Read the written PDF back and see that it says what was drawn.

    The sheets are only ever seen as SVG while this is being worked on, so
    the PDF itself has to be opened again and its own drawing operators
    replayed -- otherwise a page could be a millimetre out, or half missing,
    and nothing here would notice.
    """
    ok = True
    for number, (drawn, (points, strings)) in enumerate(
        zip(pages, read_back(path)), start=1
    ):
        wanted = [p for item in drawn.items for p in getattr(item, "points", [])]
        said = [item.string for item in drawn.items if isinstance(item, Text)]
        same = len(wanted) == len(points) and all(
            abs(a[0] - b[0]) < 0.01 and abs(a[1] - b[1]) < 0.01
            for a, b in zip(wanted, points)
        )
        spilled = [p for p in points if not _on_the_page(p[0], p[1])] + [
            item.string
            for item in drawn.items
            if isinstance(item, Text) and not _text_on_the_page(item)
        ]
        ok &= same and said == strings and not spilled
        print(
            f"  page {number}: {len(points)} points and {len(strings)} labels read "
            f"back, {'the ones drawn' if same and said == strings else 'NOT THE ONES DRAWN'}"
            + (f", {len(spilled)} OFF THE PAGE" if spilled else ", all inside the border")
        )
    return ok


def _on_the_page(x: float, y: float) -> bool:
    return FRAME <= x <= SHEET_W - FRAME and FRAME <= y <= SHEET_H - FRAME


def _text_on_the_page(item: Text) -> bool:
    """Text is placed by one corner, so its width has to be walked out."""
    width = text_width(item.string, item.size)
    start = {"start": 0.0, "middle": -width / 2, "end": -width}[item.anchor]
    ends = [
        (
            item.x + (start + at) * cos(radians(item.angle)),
            item.y + (start + at) * sin(radians(item.angle)),
        )
        for at in (0.0, width)
    ]
    return all(_on_the_page(x, y) for x, y in ends)


if __name__ == "__main__":
    import sys

    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    pages = sheets()
    pdf = out / "oak_rail_mount.pdf"
    write_pdf(pdf, pages, "OAK-D-Lite rail mount")
    print(f"cad/out/oak_rail_mount.pdf  {len(pages)} pages, A4 landscape, views at 1:1")
    if "--svg" in sys.argv:
        for name, page in zip(("body", "wedge"), pages):
            write_svg(out / f"drawing_{name}.svg", page)
            print(f"cad/out/drawing_{name}.svg")
    raise SystemExit(0 if check(pdf, pages) else 1)
