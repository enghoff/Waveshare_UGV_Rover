"""The full-resolution picture a still look keeps beside the one it reads.

The gimbal camera delivers 2592x1944 and the rover has always recorded 640x480,
so 94% of its pixels were never kept. A look now keeps both, and they are two
grabs, not one picture and a reduction of it.

**The models are still shown the camera's own 640x480 picture**, because a
reduction is not the same picture to them. Measured on the rover on 2026-10-12,
on three views each grabbed both ways seconds apart: DINOv2's vectors for the
same regions agreed at a median cosine of 0.89 between the camera's 640x480 and
a 2592x1944 frame reduced to it, against 0.98 between two of the camera's own
640x480 grabs; SigLIP2's at 0.93 against 0.98. Every vector the store holds
came off the camera's 640x480 mode, and a look's appearance is compared with
them, so feeding the models a reduction would make every new look a little
less like everything already seen. See
docs/progress/2026-10-12-full-resolution-evidence.md.

**So the full picture is evidence, taken right after the look and kept only
when it shows the same view.** It is grabbed once the look's own shutter
bracket has closed, so it costs that bracket nothing, and it is kept only when
the rover has neither moved nor turned between the two and the gimbal is where
it was: then the look's pose, pan and tilt are the full picture's too. A look
taken while driving, or while face tracking holds the camera, keeps none.

**The two sizes do not see quite the same window.** The 640x480 mode (and the
1280x960 one, which matches it) reads a 2580 by 1934 window of the sensor that
starts about one and a half pixels in, so the full frame holds about eleven
more columns at the right and eight more rows at the bottom. Fitted on
2026-10-12 from 236-296 matched features per pair over three pairs, each to a
quarter of a pixel; a reduction through this window lands on the camera's own
640x480 picture to within 0.2 pixels, which is 0.04 degrees.
`face_tracking/lens.py` carries the full mode's lens, derived from the same
window, so a pixel of either picture gives the same direction.
"""
from __future__ import annotations

from typing import Any

#: The size the gimbal camera's still evidence is grabbed at: the sensor's own.
FULL_SIZE = (2592, 1944)

#: Where the camera's 640x480 picture sits inside the 2592x1944 one, as pixel
#: edges (left, top, right, bottom): pixel 0 of the small picture starts at 1.45
#: in the large one and pixel 640 ends at 2581.45. Measured 2026-10-12.
NATIVE_WINDOW = (1.45, 2.00, 2581.45, 1936.12)

#: How far the rover may have gone, and turned, between the look and its full
#: picture for the two to count as one view. Two centimetres at two metres is
#: about half a degree, the same as the turn allowed; both are well inside the
#: degree and a half a still look's bearing is worth.
STILL_MOVED_M = 0.02
STILL_TURNED_DEG = 0.5
#: How far the gimbal may read differently. It is not commanded between the two
#: grabs, so any difference at all means something moved it.
SAME_GIMBAL_DEG = 0.05


def to_native(x: float, y: float) -> tuple[float, float]:
    """A pixel of the full picture, as the same point in the 640x480 one."""
    left, top, right, bottom = NATIVE_WINDOW
    return ((x - left) * 640.0 / (right - left),
            (y - top) * 480.0 / (bottom - top))


def from_native(x: float, y: float) -> tuple[float, float]:
    """A pixel of the 640x480 picture, as the same point in the full one."""
    left, top, right, bottom = NATIVE_WINDOW
    return (left + x * (right - left) / 640.0,
            top + y * (bottom - top) / 480.0)


def _heading_turn(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def same_view(look: dict[str, Any], pose_then: dict[str, Any] | None,
              full: dict[str, Any], pose_now: dict[str, Any] | None) -> str:
    """Why the full picture does not show the look's view, or "" if it does.

    `look` and `full` are the two captures (each with its pan and tilt), and the
    poses are the navigator's either side of the full grab's: the one the
    look's bracket closed on, and one read after the full picture was in hand.
    """
    if not full.get("ok") or not full.get("jpeg"):
        return str(full.get("error") or "the camera gave no full picture")
    if (int(full.get("width") or 0), int(full.get("height") or 0)) != FULL_SIZE:
        return "the full picture came back at another size"
    for axis in ("pan", "tilt"):
        a, b = look.get(axis), full.get(axis)
        if a is None or b is None or abs(float(a) - float(b)) > SAME_GIMBAL_DEG:
            return f"the gimbal's {axis} changed between the two pictures"
    if pose_then is None or pose_now is None:
        return "no pose either side, so nothing can say the view was the same"
    moved = ((pose_now["x_m"] - pose_then["x_m"]) ** 2
             + (pose_now["y_m"] - pose_then["y_m"]) ** 2) ** 0.5
    if moved > STILL_MOVED_M:
        return f"the rover moved {moved:.2f} m between the two pictures"
    turned = _heading_turn(pose_now["heading_deg"], pose_then["heading_deg"])
    if turned > STILL_TURNED_DEG:
        return f"the rover turned {turned:.1f} degrees between the two pictures"
    return ""
