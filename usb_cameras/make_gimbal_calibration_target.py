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
SQUARE_MM = 25.0
MARKER_MM = 18.0
DICTIONARY_NAME = "DICT_4X4_50"
RASTER_DPI = 600
OUTPUT = Path("output/pdf/p0-gimbal-charuco-a4.pdf")


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

    canvas = Canvas(
        str(path), pagesize=(page_w, page_h), pageCompression=1, invariant=1
    )
    canvas.setTitle("P0 gimbal calibration ChArUco target - A4")
    canvas.drawImage(
        ImageReader(io.BytesIO(board_png())),
        left,
        bottom,
        width=board_w,
        height=board_h,
        preserveAspectRatio=True,
        mask="auto",
    )

    # The board dimensions are the primary scale check.  This independent bar
    # catches print-dialog scaling before the target is mounted.
    bar_y = page_h - 6.0 * mm
    bar_x = (page_w - 100.0 * mm) / 2
    canvas.setLineWidth(0.35)
    canvas.line(bar_x, bar_y, bar_x + 100.0 * mm, bar_y)
    for step in range(0, 101, 10):
        tick = 2.2 * mm if step in (0, 100) else 1.3 * mm
        x = bar_x + step * mm
        canvas.line(x, bar_y - tick / 2, x, bar_y + tick / 2)
    canvas.setFont("Helvetica", 6.5)
    canvas.drawCentredString(
        page_w / 2,
        page_h - 11.0 * mm,
        "100 mm check - print A4 landscape at Actual size / 100% (no Fit or Shrink)",
    )
    canvas.setFont("Helvetica", 6.0)
    canvas.drawCentredString(
        page_w / 2,
        5.0 * mm,
        "10 x 7 squares; 25.0 mm square; 18.0 mm marker; OpenCV DICT_4X4_50",
    )
    canvas.showPage()
    canvas.save()


if __name__ == "__main__":
    make_pdf(OUTPUT)
    print(OUTPUT.resolve())
