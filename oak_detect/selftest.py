#!/usr/bin/env python3
"""Prove the stack on the machine that runs it, from the library up.

    ssh rpi 'python3 ~/ugv/oak_detect/selftest.py'
    ssh rpi 'python3 ~/ugv/oak_detect/selftest.py --jpeg /tmp/face.jpg'

Each stage is one more thing that can be wrong, in the order they fail: the
library loads, the device is on the bus and can be opened at all, the firmware
uploads, the graph uploads, and then it runs. The first line that fails is the
one that says what to fix, which is the whole point of doing it in this order
rather than starting the server and reading a stack trace.

Without --jpeg this never decodes anything, so it says nothing about the camera
or about JPEG. It measures the device: boot, graph upload, and inference on a
buffer of noise. Give it a photograph to exercise the decoder and the boxes.
"""

import argparse
import ctypes
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak import Oak, OakError  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FAILURES = []


def check(label, fn):
    """Run one stage, print its line, and keep going unless it was fatal."""
    started = time.perf_counter()
    try:
        detail = fn()
    except Exception as error:
        FAILURES.append(label)
        print("  FAIL  %-22s %s" % (label, error))
        return None
    print("  ok    %-22s %s" % (label, detail if detail is not None else ""))
    return detail if detail is not None else True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--jpeg", help="a photograph to decode and detect in")
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--blob", default=os.path.join(
        HERE, "face-detection-retail-0004-640x480.blob"))
    args = parser.parse_args()

    print("oak_detect selftest")

    device = [None]

    def load():
        device[0] = Oak(args.blob)
        return os.path.basename(device[0]._lib._name)

    def boot():
        started = time.perf_counter()
        device[0].open()
        return "%.1f s, %dx%d input" % (time.perf_counter() - started,
                                        device[0].input_shape[2],
                                        device[0].input_shape[1])

    if check("library loads", load) is None:
        return report()
    if check("VPU boots, graph loads", boot) is None:
        print("\n  the device is not there, is already open in another process,")
        print("  or /dev/bus/usb is not writable -- see README, the udev rule.")
        return report()

    oak = device[0]
    buffer = ctypes.create_string_buffer(oak.input_bytes)

    def inference():
        # Noise rather than zeros: a black frame is a degenerate input and this
        # is timing the device, which should not be measured on a special case.
        noise = bytes((i * 37 + 11) & 0xff for i in range(oak.input_bytes))
        ctypes.memmove(buffer, noise, oak.input_bytes)
        times = []
        for _ in range(args.runs):
            started = time.perf_counter()
            oak.infer(buffer)
            times.append((time.perf_counter() - started) * 1e3)
        times.sort()
        return "median %.1f ms, min %.1f, max %.1f over %d" % (
            times[len(times) // 2], times[0], times[-1], len(times))

    check("inference", inference)

    if args.jpeg:
        def decode():
            with open(args.jpeg, "rb") as handle:
                jpeg = handle.read()
            times, sizes = [], None
            for _ in range(10):
                started = time.perf_counter()
                sizes = oak.jpeg_to_input(jpeg, buffer)
                times.append((time.perf_counter() - started) * 1e3)
            if sizes is None:
                raise ValueError("%s is not a decodable image" % args.jpeg)
            times.sort()
            self_w, self_h, dec_w, dec_h = sizes
            return "%dx%d decoded at %dx%d, median %.1f ms" % (
                self_w, self_h, dec_w, dec_h, times[len(times) // 2])

        def faces():
            import struct

            raw = oak.infer(buffer)
            rows = len(raw) // 14
            values = struct.unpack("<%de" % (rows * 7), raw[:rows * 14])
            found = []
            for row in range(rows):
                image_id, _, confidence, xmin, ymin, xmax, ymax = values[row * 7:row * 7 + 7]
                if image_id < 0:
                    break
                if confidence >= 0.6:
                    found.append("%.2f at (%.2f,%.2f)-(%.2f,%.2f)"
                                 % (confidence, xmin, ymin, xmax, ymax))
            if not found:
                raise ValueError("no face above 0.6 -- is there one in the picture?")
            return "; ".join(found)

        if check("jpeg decode", decode) is not None:
            check("faces found", faces)

    oak.close()
    return report()


def report():
    if FAILURES:
        print("\nFAILED: %s" % ", ".join(FAILURES))
        return 1
    print("\nall stages passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
