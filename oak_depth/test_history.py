#!/usr/bin/env python3
"""The depth frame taken nearest a moment, without a camera.

    python3 oak_depth/test_history.py

`Depth.at` is the one part of the service whose answer is arithmetic rather than
hardware, so it is checked here on frames made by hand. The rest of the service is
proved on the device by `selftest.py`.
"""
import collections
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import depth_server  # noqa: E402

FAILED = []


def check(label, got, want):
    ok = got == want
    print(("  ok    " if ok else "  FAIL  ") + label + ("" if ok else f": got {got!r}, want {want!r}"))
    if not ok:
        FAILED.append(label)


def a_service(stamps):
    """A `Depth` holding one frame per stamp, made without opening anything."""
    depth = object.__new__(depth_server.Depth)
    depth._wanted = True
    depth._lock = threading.Lock()
    depth.history = collections.deque(maxlen=45)
    depth._periods = []
    now = time.monotonic()
    for index, stamp in enumerate(stamps):
        frame = [index]                    # stands in for a uint16 array
        depth.history.append((now + stamp if stamp is not None else 0.0, now, frame))
        depth.frame, depth.frame_at = frame, now
    depth.jpeg, depth.jpeg_at, depth.jpeg_stamp = b"", 0.0, 0.0
    depth.frame_stamp, depth.rate = 0.0, 0.0
    return depth, now


def main() -> int:
    # Frames 67 ms apart, as at 15 fps, the newest 0.1 s ago.
    depth, now = a_service([-0.3, -0.233, -0.167, -0.1])
    offset = time.time() - time.monotonic()
    frame, _age, taken, off = depth.at(now - 0.18 + offset)
    check("the frame nearest the moment asked for", frame, [2])
    check("...said to be 13 ms after it", round(off, 3), 0.013)
    check("...taken when it was, on the caller's clock",
          round(taken - (now - 0.167 + offset), 3), 0.0)
    frame, _age, _taken, off = depth.at(now - 5.0 + offset)
    check("a moment older than the history gets its oldest frame, and says how far",
          (frame, round(off, 1)), ([0], 4.7))

    unstamped, now = a_service([None, None])
    frame, _age, taken, off = unstamped.at(now + offset)
    check("a device that stamps nothing gets the newest frame, with no time",
          (frame, taken, off), ([1], None, None))

    depth._wanted = False
    check("a camera switched off has nothing", depth.at(now + offset)[0], None)
    depth._wanted = True
    depth.forget()
    check("and forgetting the frames forgets the history", len(depth.history), 0)

    print(f"\n{len(FAILED)} failed" if FAILED else "\nall passed")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
