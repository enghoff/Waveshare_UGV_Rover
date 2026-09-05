"""Depth preview colours shared by the workstation OAK viewers."""
import cv2
import numpy as np

DEPTH_RANGE_MM = (200, 4000)

def colourise(depth: np.ndarray) -> np.ndarray:
    """uint16 millimetres -> BGR, near = red, far = blue, invalid = black."""
    lo, hi = DEPTH_RANGE_MM
    scaled = ((np.clip(depth, lo, hi) - lo) / (hi - lo) * 255).astype(np.uint8)
    colour = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
    colour[depth == 0] = 0
    return colour
