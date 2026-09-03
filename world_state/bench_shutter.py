#!/usr/bin/env python3
"""How well this camera says when it took a picture.

    ssh orin 'cd ~/ugv/world_state && python3 bench_shutter.py'

`Inspector._where` interpolates the rover's heading to the moment the shutter
opened, and what is left over is that moment's own uncertainty multiplied by the
turn rate. That uncertainty is `FRAME_TIME_SIGMA_S`, and it is the one number the
recovery of a turning look's bearing rests on.

**What the stamp on a one-shot grab actually is**, from `uvc_camera.snapshot`'s
own docstring: one clock reading as v4l2-ctl exits, less an estimate of the
camera's pipeline lag -- not the per-buffer V4L2 timestamps the tracking feed
pairs up. Every frame in a burst therefore carries the *same* stamp, so the gaps
between frames are zero and say nothing at all. That docstring also warns that
aiming a gimbal from a stamp this rough would not be safe. The world state aims
nothing, but it does now work a bearing out from one, which is why this exists.

So what is measured is where the stamp sits relative to the grab the caller
timed, and how much that varies:

  * **jitter** -- how much the offset from the start of the call moves. This is
    the part that can be measured from here, and on this rover it is small,
    because v4l2-ctl's own runtime is consistent and the camera streams at a
    fixed rate.
  * **bias** -- the constant gap between that stamp and the true exposure. This
    is *not* measurable from here and does not wash out: a stamp systematically
    late swings every bearing taken while turning the same way. Its natural
    scale is one frame interval, since the newest frame was exposed within one of
    the stamp, and that is what `FRAME_TIME_SIGMA_S` is really sized for.

It needs the camera, and a burst taken while the rover's own looking loop is
grabbing can lose the race -- see `rover_camera._snapshot` for that collision.
Lost bursts are counted and retried rather than being an error.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for path in (ROOT, os.path.join(ROOT, "face_tracking")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

#: The turn rates this rover was measured at on the drive of 2026-09-03, in
#: degrees a second: the median it managed while driving, and the worst it
#: reached. What a timing error costs a bearing is one of these multiplied by it.
TURN_RATES = (("median driving", 29.1), ("worst on that drive", 94.8))


def bursts(device: str, size: tuple[int, int], count: int, frames: int):
    """For each burst that got the camera, how the stamp sat inside the call.

    `(offset, tail, within, length)` per burst, in seconds: from the call
    starting to the newest stamp, from that stamp to the call returning, the
    spread of stamps inside the burst, and how long the whole call took.

    `snapshot` answers `(frames, why)` and each frame is `(jpeg, at)`. Worth
    saying because getting it wrong reads exactly like the collision above --
    every burst empty -- while the camera is free the whole time.
    """
    from track_face_pi import snapshot

    got_bursts = []
    lost = 0
    for _burst in range(count):
        began = time.monotonic()
        try:
            frames_got, _why = snapshot(device, size, frames=frames)
        except Exception:                                      # noqa: BLE001
            frames_got = []
        ended = time.monotonic()
        stamps = [pair[1] for pair in (frames_got or [])
                  if isinstance(pair, (tuple, list)) and len(pair) == 2]
        if not stamps:
            lost += 1
            time.sleep(0.35)
            continue
        newest = stamps[-1]
        got_bursts.append((newest - began, ended - newest,
                           max(stamps) - min(stamps), ended - began))
        time.sleep(0.35)
    return got_bursts, lost


def _line(name: str, values: list[float]) -> float:
    spread = statistics.pstdev(values) if len(values) > 1 else 0.0
    print(f"  {name:30} median {statistics.median(values) * 1000:7.1f} ms  "
          f"spread {spread * 1000:6.1f}  worst {max(values) * 1000:7.1f}")
    return spread


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--device",
                        default="/dev/v4l/by-id/usb-Xitech_USB_Camera_"
                                "20250606105-video-index0")
    parser.add_argument("--size", default="640x480")
    parser.add_argument("--bursts", type=int, default=18)
    parser.add_argument("--frames", type=int, default=8)
    args = parser.parse_args()
    width, height = (int(part) for part in args.size.lower().split("x"))

    got, lost = bursts(args.device, (width, height), args.bursts, args.frames)
    if len(got) < 4:
        print(f"the camera gave nothing usable: {lost} of {args.bursts} bursts "
              f"came back empty, which is the rover's own looking loop holding "
              f"the camera. Try again, or with more --bursts.", file=sys.stderr)
        return 1

    print(f"{len(got)} of {args.bursts} bursts got the camera "
          f"({lost} lost to the looking loop), {args.frames} frames each")
    _line("grab length", [one[3] for one in got])
    _line("stamp spread within a burst", [one[2] for one in got])
    jitter = _line("call start -> newest stamp", [one[0] for one in got])
    _line("newest stamp -> call return", [one[1] for one in got])

    where = statistics.median([one[0] for one in got]) / \
        statistics.median([one[3] for one in got])
    print(f"\n  the stamp lands {where:.0%} of the way through the call, so the "
          f"midpoint this replaced was wrong by {abs(where - 0.5):.0%} of "
          f"whatever the rover turned")
    print(f"  the jitter in that offset is {jitter * 1000:.0f} ms, which costs "
          f"a bearing:")
    for name, rate in TURN_RATES:
        print(f"    {name:22} {rate:5.1f} deg/s -> {rate * jitter:5.2f} deg")

    from world_state.inspector import FRAME_TIME_SIGMA_S

    print(f"\n  FRAME_TIME_SIGMA_S is {FRAME_TIME_SIGMA_S * 1000:.0f} ms")
    if jitter > FRAME_TIME_SIGMA_S:
        print("  MEASURED JITTER ALONE EXCEEDS IT -- raise it. It only ever "
              "widens a bearing, so being low costs correctness where being "
              "high costs precision.")
    else:
        print("  which covers the measured jitter with room for the bias this "
              "cannot see. One frame interval is the scale of that bias, and is "
              "what the number is sized for.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
