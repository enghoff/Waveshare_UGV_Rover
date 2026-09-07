#!/usr/bin/env python3
"""Create the A4 ChArUco target used by the bounded P0 gimbal calibration.

Run from the repository root with the local PDF environment:

    .venv/Scripts/python usb_cameras/make_gimbal_calibration_target.py

The generated PDF is deliberately versioned.  Its printed dimensions, dictionary
and board geometry are inputs to the measurement, not presentation details.
"""

from __future__ import annotations

import io
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.utils import ImageReader


SQUARES_X = 10
SQUARES_Y = 7
SQUARE_MM = 24.0
MARKER_MM = 17.0
DICTIONARY_NAME = "DICT_4X4_50"
RASTER_DPI = 600
OUTPUT = Path("output/pdf/p0-gimbal-charuco-a4.pdf")
MIN_CONTENT_MARGIN_MM = 20.0


def board_png() -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, DICTIONARY_NAME)
    )
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_MM / 1000.0,
        MARKER_MM / 1000.0,
        dictionary,
    )
    width = round(SQUARES_X * SQUARE_MM / 25.4 * RASTER_DPI)
    height = round(SQUARES_Y * SQUARE_MM / 25.4 * RASTER_DPI)
    pixels = board.generateImage((width, height), marginSize=0, borderBits=1)
    image = Image.fromarray(np.asarray(pixels, dtype=np.uint8)).convert("1")
    stream = io.BytesIO()
    image.save(stream, format="PNG", optimize=True)
    return stream.getvalue()


def make_pdf(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    page_w, page_h = landscape(A4)
    board_w = SQUARES_X * SQUARE_MM * mm
    board_h = SQUARES_Y * SQUARE_MM * mm
    left = (page_w - board_w) / 2
    bottom = (page_h - board_h) / 2
    assert min(left, bottom, page_w - left - board_w, page_h - bottom - board_h) \
        >= MIN_CONTENT_MARGIN_MM * mm

    canvas = Canvas(
        str(path), pagesize=(page_w, page_h), pageCompression=1, invariant=1
    )
    canvas.setTitle("P0 gimbal calibration ChArUco target - A4")
    canvas.setSubject(
        "10 x 7 ChArUco board; 24.0 mm squares; 17.0 mm markers; "
        "OpenCV DICT_4X4_50"
    )
    canvas.drawImage(
        ImageReader(io.BytesIO(board_png())),
        left,
        bottom,
        width=board_w,
        height=board_h,
        preserveAspectRatio=True,
        mask="auto",
    )

    # Keep every mark beyond 20 mm from every page edge.  Canon specifies a
    # 16.7 mm trailing margin for the MG2577S, and a landscape driver may rotate
    # which PDF edge becomes the trailing edge.  The board dimensions are the
    # primary scale check; this independent bar catches print-dialog scaling.
    bar_x = 22.0 * mm
    bar_y = (page_h - 100.0 * mm) / 2
    assert bar_x - 1.1 * mm >= MIN_CONTENT_MARGIN_MM * mm
    canvas.setLineWidth(0.35)
    canvas.line(bar_x, bar_y, bar_x, bar_y + 100.0 * mm)
    for step in range(0, 101, 10):
        tick = 2.2 * mm if step in (0, 100) else 1.3 * mm
        y = bar_y + step * mm
        canvas.line(bar_x - tick / 2, y, bar_x + tick / 2, y)
    canvas.setFont("Helvetica", 6.5)
    canvas.saveState()
    canvas.translate(25.5 * mm, page_h / 2)
    canvas.rotate(90)
    canvas.drawCentredString(
        0,
        0,
        "100 mm check | P0 10x7 | square 24 mm | marker 17 mm | 4X4_50 | print 100%",
    )
    canvas.restoreState()
    canvas.showPage()
    canvas.save()


if __name__ == "__main__":
    make_pdf(OUTPUT)
    print(OUTPUT.resolve())
