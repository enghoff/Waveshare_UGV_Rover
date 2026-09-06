"""A sheet of paper you can draw lines and text on, in millimetres.

Everything here is in millimetres measured from the bottom left of the sheet,
because that is the unit the parts are in and a drawing that is not at true
scale is not worth printing.  PDF's own unit is 1/72 inch, so a page starts
with the one transform that turns millimetres into it and nothing else ever
converts anything.

The same sheet writes out as SVG as well.  That is not for anybody to use --
it is so the drawing can be looked at in a browser while it is being written,
which a PDF cannot be.

No third-party library: a line-art PDF is a few hundred lines of text and an
offset table, and build123d is already a big enough thing to have to install.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, radians, sin

MM_TO_PT = 72.0 / 25.4

# Adobe's widths for Helvetica, in 1/1000 em, for space through '~'.  Needed
# to centre a dimension's text on its line: PDF will not measure a string.
_HELVETICA_W = [
    278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278,
    556, 556, 556, 556, 556, 556, 556, 556, 556, 556, 278, 278, 584, 584, 584, 556,
    1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833, 722, 778,
    667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556,
    333, 556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556,
    556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334, 260, 334, 584,
]


def text_width(string: str, size: float) -> float:
    """How wide `string` will come out, in the same units as `size`."""
    total = sum(
        _HELVETICA_W[ord(c) - 32] if 32 <= ord(c) <= 126 else 556 for c in string
    )
    return size * total / 1000.0


@dataclass
class Stroke:
    points: list[tuple[float, float]]
    weight: float = 0.25  # mm
    grey: float = 0.0  # 0 black .. 1 white
    dash: tuple[float, float] | None = None


@dataclass
class Fill:
    points: list[tuple[float, float]]
    grey: float = 0.0


@dataclass
class Text:
    x: float
    y: float
    string: str
    size: float = 2.5  # mm, cap-to-descender nominal
    grey: float = 0.0
    anchor: str = "start"  # start | middle | end
    angle: float = 0.0  # degrees, anticlockwise


@dataclass
class Sheet:
    """One page.  Draw onto it, then hand it to write_pdf."""

    width: float
    height: float
    items: list = field(default_factory=list)

    def polyline(self, points, weight=0.25, grey=0.0, dash=None, close=False):
        pts = list(points)
        if close:
            pts.append(pts[0])
        if len(pts) > 1:
            self.items.append(Stroke(pts, weight, grey, dash))

    def line(self, x1, y1, x2, y2, weight=0.25, grey=0.0, dash=None):
        self.polyline([(x1, y1), (x2, y2)], weight, grey, dash)

    def rect(self, x, y, w, h, weight=0.25, grey=0.0):
        self.polyline(
            [(x, y), (x + w, y), (x + w, y + h), (x, y + h)], weight, grey, close=True
        )

    def fill(self, points, grey=0.0):
        self.items.append(Fill(list(points), grey))

    def text(self, x, y, string, size=2.5, grey=0.0, anchor="start", angle=0.0):
        self.items.append(Text(x, y, string, size, grey, anchor, angle))


def _anchor_shift(item: Text) -> tuple[float, float]:
    """Where the baseline actually starts, once the anchor is honoured."""
    if item.anchor == "start":
        return item.x, item.y
    width = text_width(item.string, item.size)
    back = width if item.anchor == "end" else width / 2
    a = radians(item.angle)
    return item.x - back * cos(a), item.y - back * sin(a)


# --------------------------------------------------------------------------
# PDF
# --------------------------------------------------------------------------
def _escape(string: str) -> str:
    out = string.replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)")
    return "".join(c if 32 <= ord(c) <= 126 else "?" for c in out)


def _content(sheet: Sheet) -> bytes:
    """The page's drawing commands, with millimetres as the user unit."""
    ops = [f"{MM_TO_PT:.6f} 0 0 {MM_TO_PT:.6f} 0 0 cm", "1 J 1 j"]  # round caps/joins
    for item in sheet.items:
        if isinstance(item, Stroke):
            ops.append(f"{item.grey:.3f} G {item.weight:.3f} w")
            ops.append(
                f"[{item.dash[0]:.2f} {item.dash[1]:.2f}] 0 d" if item.dash else "[] 0 d"
            )
            x, y = item.points[0]
            ops.append(f"{x:.3f} {y:.3f} m")
            ops += [f"{px:.3f} {py:.3f} l" for px, py in item.points[1:]]
            ops.append("S")
        elif isinstance(item, Fill):
            ops.append(f"{item.grey:.3f} g")
            x, y = item.points[0]
            ops.append(f"{x:.3f} {y:.3f} m")
            ops += [f"{px:.3f} {py:.3f} l" for px, py in item.points[1:]]
            ops.append("h f")
        else:
            x, y = _anchor_shift(item)
            a = radians(item.angle)
            ops.append(f"BT /F1 {item.size:.3f} Tf {item.grey:.3f} g")
            ops.append(
                f"{cos(a):.5f} {sin(a):.5f} {-sin(a):.5f} {cos(a):.5f} "
                f"{x:.3f} {y:.3f} Tm ({_escape(item.string)}) Tj ET"
            )
    return "\n".join(ops).encode("ascii")


def write_pdf(path, sheets: list[Sheet], title: str = "") -> None:
    """Write the sheets out as one PDF, one page each."""
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets: dict[int, int] = {}

    def add(number: int, body: bytes) -> None:
        offsets[number] = len(out)
        out.extend(f"{number} 0 obj\n".encode("ascii") + body + b"\nendobj\n")

    first_page = 4  # 1 catalog, 2 page tree, 3 font, then page/content pairs
    kids = " ".join(f"{first_page + 2 * i} 0 R" for i in range(len(sheets)))
    add(1, b"<< /Type /Catalog /Pages 2 0 R >>")
    add(2, f"<< /Type /Pages /Kids [{kids}] /Count {len(sheets)} >>".encode("ascii"))
    add(3, b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica "
           b"/Encoding /WinAnsiEncoding >>")

    for i, sheet in enumerate(sheets):
        page, content = first_page + 2 * i, first_page + 2 * i + 1
        add(
            page,
            (
                f"<< /Type /Page /Parent 2 0 R /MediaBox "
                f"[0 0 {sheet.width * MM_TO_PT:.3f} {sheet.height * MM_TO_PT:.3f}] "
                f"/Resources << /Font << /F1 3 0 R >> >> /Contents {content} 0 R >>"
            ).encode("ascii"),
        )
        stream = _content(sheet)
        add(
            content,
            f"<< /Length {len(stream)} >>\nstream\n".encode("ascii")
            + stream
            + b"\nendstream",
        )

    info = f"<< /Title ({_escape(title)}) /Producer (cad/sheet.py) >>"
    info_num = first_page + 2 * len(sheets)
    add(info_num, info.encode("ascii"))

    count = info_num + 1
    start = len(out)
    out.extend(f"xref\n0 {count}\n".encode("ascii"))
    out.extend(b"0000000000 65535 f \n")
    for number in range(1, count):
        out.extend(f"{offsets[number]:010d} 00000 n \n".encode("ascii"))
    out.extend(
        f"trailer\n<< /Size {count} /Root 1 0 R /Info {info_num} 0 R >>\n"
        f"startxref\n{start}\n%%EOF\n".encode("ascii")
    )

    with open(path, "wb") as handle:
        handle.write(bytes(out))


# --------------------------------------------------------------------------
# SVG, for looking at while the drawing is being written
# --------------------------------------------------------------------------
def write_svg(path, sheet: Sheet) -> None:
    """One sheet as SVG.  Y is flipped, because SVG counts down the page."""
    flip = sheet.height
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{sheet.width}mm" '
        f'height="{sheet.height}mm" viewBox="0 0 {sheet.width} {sheet.height}">',
        f'<rect width="{sheet.width}" height="{sheet.height}" fill="white"/>',
    ]
    for item in sheet.items:
        if isinstance(item, (Stroke, Fill)):
            pts = " ".join(f"{x:.3f},{flip - y:.3f}" for x, y in item.points)
            shade = int(round(item.grey * 255))
            colour = f"rgb({shade},{shade},{shade})"
            if isinstance(item, Fill):
                parts.append(f'<polygon points="{pts}" fill="{colour}"/>')
            else:
                dash = (
                    f' stroke-dasharray="{item.dash[0]} {item.dash[1]}"'
                    if item.dash
                    else ""
                )
                parts.append(
                    f'<polyline points="{pts}" fill="none" stroke="{colour}" '
                    f'stroke-width="{item.weight}" stroke-linecap="round"{dash}/>'
                )
        else:
            shade = int(round(item.grey * 255))
            anchor = {"start": "start", "middle": "middle", "end": "end"}[item.anchor]
            transform = (
                f' transform="rotate({-item.angle} {item.x:.3f} {flip - item.y:.3f})"'
                if item.angle
                else ""
            )
            parts.append(
                f'<text x="{item.x:.3f}" y="{flip - item.y:.3f}" '
                f'font-family="Helvetica, Arial, sans-serif" font-size="{item.size}" '
                f'fill="rgb({shade},{shade},{shade})" text-anchor="{anchor}"'
                f'{transform}>{_xml(item.string)}</text>'
            )
    parts.append("</svg>")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts))


def _xml(string: str) -> str:
    return string.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# --------------------------------------------------------------------------
# Reading a written PDF back
# --------------------------------------------------------------------------
def read_back(path) -> list[tuple[list[tuple[float, float]], list[str]]]:
    """What a written PDF actually draws: its points, in mm, and its strings.

    Replaying the file's own operators is the only way to be sure the sheet
    that was drawn is the sheet that was written, and that a millimetre on
    the page really is a millimetre: the page transform is checked here, and
    without it the numbers in the stream would be points, not millimetres.
    """
    import re

    with open(path, "rb") as handle:
        data = handle.read()

    pages = []
    for stream in re.findall(rb"stream\r?\n(.*?)\r?\nendstream", data, re.S):
        points: list[tuple[float, float]] = []
        strings: list[str] = []
        for line in stream.decode("ascii").splitlines():
            word = line.split()
            if not word:
                continue
            if word[-1] == "cm":
                if abs(float(word[0]) - MM_TO_PT) > 1e-6:
                    raise ValueError(f"{path} is not drawn in millimetres")
            elif word[-1] in ("m", "l") and len(word) == 3:
                points.append((float(word[0]), float(word[1])))
            elif "Tm" in word:
                shown = re.search(r"Tm \((.*)\) Tj", line)
                text = shown.group(1) if shown else ""
                for was, now in ((r"\(", "("), (r"\)", ")"), (r"\\\\", "\\")):
                    text = text.replace(was, now)
                strings.append(text)
        pages.append((points, strings))
    return pages
