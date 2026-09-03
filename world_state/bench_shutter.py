#!/usr/bin/env python3
"""How well this camera says when it took a picture.

    ssh orin 'cd ~/ugv/world_state && python3 bench_shutter.py'

`Inspector._where` interpolates the rover's heading to the moment the shutter
opened, and what is left over is that moment's own uncertainty multiplied by the
turn rate -- `FRAME_TIME_SIGMA_S`, which is the one number the recovery of a
turning look's bearing rests on and which was estimated rather than measured.

**Two things are measured here and only the second is the number itself.**

The frame's stamp is taken in userspace as the buffer arrives, so it lags the
exposure by the driver's own buffering. A *constant* lag does not matter: it
applies to every frame alike and washes out of a bearing, because the heading is
interpolated across a bracket rather than read at an absolute time. What matters
is the jitter around it, and that shows up as spread in the interval between
consecutive frames of a steady stream -- a camera delivering at a fixed rate whose
stamps wobble is a camera whose stamps are wobbling, since the sensor is not.

So: grab bursts, look at the spread of the gaps between frames, and report what
that spread costs a bearing at the turn rates this rover actually reaches. It
needs the camera to itself and takes a few seconds.

**What it cannot separate** is stamping jitter from the camera genuinely
delivering unevenly. Both hurt a bearing by exactly as much, so the number is the
right one to charge either way -- but a large answer here is worth chasing into
the driver rather than accepting.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for path in (ROOT, os.path.join(ROOT, "face_tracking")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

#: The turn rates this rover was measured at on the drive of 2026-09-03, in
#: degrees a second: the median it manages while driving, and the worst it
#: reached. What a timing error costs a bearing is one of these multiplied by it.
TURN_RATES = {"median driving": 29.1, "worst on that drive": 94.8}


def gaps(device: str, size: tuple[int, int], bursts: int,
         frames: int) -> list[float]:
    """The intervals between consecutively delivered frames, in seconds."""
    from track_face_pi import snapshot

    apart: list[float] = []
    for _burst in range(bursts):
        got = snapshot(device, size, frames=frames)
        stamps = [at for _jpeg, at in got]
        apart += [later - earlier
                  for earlier, later in zip(stamps, stamps[1:])]
    return apart


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--device",
                        default="/dev/v4l/by-id/usb-Xitech_USB_Camera_"
                                "20250606105-video-index0")
    parser.add_argument("--size", default="640x480")
    parser.add_argument("--bursts", type=int, default=8)
    parser.add_argument("--frames", type=int, default=8)
    args = parser.parse_args()
    width, height = (int(part) for part in args.size.lower().split("x"))

    apart = gaps(args.device, (width, height), args.bursts, args.frames)
    if len(apart) < 4:
        print("the camera delivered too few frames to say anything",
              file=sys.stderr)
        return 1

    apart.sort()
    middle = statistics.median(apart)
    spread = statistics.pstdev(apart)
    print(f"{len(apart) + args.bursts} frames in {args.bursts} bursts, "
          f"{len(apart)} gaps between them")
    print(f"  frame rate      {1.0 / middle:5.1f} /s "
          f"({middle * 1000:.1f} ms between frames)")
    print(f"  gap spread      {spread * 1000:5.1f} ms "
          f"(worst {max(apart) * 1000:.1f}, best {min(apart) * 1000:.1f})")

    # Half the spread, because the stamp is as likely early as late; this is the
    # figure `FRAME_TIME_SIGMA_S` is meant to hold.
    sigma_s = spread / 2.0
    print(f"\n  so the moment of a frame is known to about "
          f"+-{sigma_s * 1000:.0f} ms")
    print("  which costs a bearing:")
    for name, rate in TURN_RATES.items():
        print(f"    {name:22} {rate:5.1f} deg/s -> "
              f"{rate * sigma_s:5.2f} deg")

    from world_state.inspector import FRAME_TIME_SIGMA_S

    print(f"\n  FRAME_TIME_SIGMA_S is {FRAME_TIME_SIGMA_S * 1000:.0f} ms")
    if sigma_s > FRAME_TIME_SIGMA_S:
        print("  MEASURED WORSE THAN ASSUMED -- raise it, and note that it only "
              "ever widens a bearing, so this costs precision rather than "
              "correctness")
    else:
        print("  measured no worse than assumed, so every bearing is being "
              "charged at least what it is worth")
    return 0


if __name__ == "__main__":
    sys.exit(main())
