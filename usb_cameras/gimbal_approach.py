#!/usr/bin/env python3
"""How small an approach undershoot still seats the pan servo's backlash.

Every gimbal move in `rover_daemon/rover.py` first drops APPROACH_UNDERSHOOT_DEG
below its target and comes back up, so the servo arrives from the side the pan
calibration was measured from. The backlash it takes up is about 1.5 degrees
(2026-09-07); the undershoot is 30 because the calibration bench used 30, not
because 30 was shown to be needed. This finds the smallest undershoot whose
landing cannot be told from the 30-degree one.

Run on the rover, parked, with nothing moving in the middle of the picture. Each
trial starts from +30 so the servo begins seated on the far side of its play --
the worst case -- then drops to target - U, comes up to the target, settles and
photographs. U = 0 comes straight down from +30: the landing from the other side,
which measures the backlash itself in the same sitting. Where the camera landed is
read as the horizontal shift of the middle of the picture against the first
30-degree landing (phase correlation), and turned into degrees by two 30-degree
landings a commanded few degrees apart.

PASS_RULE was written before the first trial was run. A smaller undershoot passes
when every one of its landings is within TOLERANCE_DEG of the mean 30-degree
landing. The smallest undershoot that passes, with every larger one also passing,
is the measured answer; the deployed value is that doubled, for margin against
another position and another sitting, and never more than 30.

    python3 gimbal_approach.py OUTPUT_DIR            # on the rover, port 8769
    python3 gimbal_approach.py OUTPUT_DIR --analyse  # again, from saved frames
"""
from __future__ import annotations

import argparse
import base64
import json
import random
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

UNDERSHOOTS = (30, 15, 8, 5, 3, 2, 1, 0)
REPEATS = 3
#: Positions besides rest, at fewer undershoots: an aimed look pans to the
#: calibrated limit and approaches it the same way.
EDGES = ((-20, (30, 5, 3)), (20, (30, 5, 3)))
EDGE_REPEATS = 2
#: The commanded step that turns pixels into degrees, landed with the 30.
SCALE_STEP_DEG = 4
START_DEG = 30
TILT_DEG = 20
MOVE_WAIT_S = 1.0
SETTLE_S = 1.5
#: The middle of a 640x480 frame, clear of the frame edges where the fisheye
#: compresses and away from the left side where a person may sit and move.
CROP = (200, 60, 600, 320)
TOLERANCE_DEG = 0.2
PASS_RULE = ("an undershoot passes when all its landings are within %.1f deg of the "
             "mean 30-degree landing at the same position; the answer is the smallest "
             "that passes with every larger one passing; deploy twice it, at most 30"
             % TOLERANCE_DEG)


class Daemon:
    def __init__(self, port=8769):
        self.f = socket.create_connection(("127.0.0.1", port), 30).makefile("rwb")

    def call(self, name, **arguments):
        self.f.write(json.dumps({"call": name, "arguments": arguments}).encode() + b"\n")
        self.f.flush()
        answer = json.loads(self.f.readline())
        if answer.get("ok") is False:
            raise RuntimeError("%s refused: %s" % (name, answer.get("error")))
        return answer


def land(daemon, target, undershoot):
    """From +30, down to target - undershoot, up to target; a picture there."""
    daemon.call("look_at", pan=START_DEG, tilt=TILT_DEG)
    time.sleep(MOVE_WAIT_S)
    if undershoot:
        daemon.call("look_at", pan=target - undershoot, tilt=TILT_DEG)
        time.sleep(MOVE_WAIT_S)
    daemon.call("look_at", pan=target, tilt=TILT_DEG)
    time.sleep(SETTLE_S)
    return base64.b64decode(daemon.call("camera_jpeg")["jpeg_base64"])


def plan(seed=5):
    rng = random.Random(seed)
    trials = []
    for repeat in range(REPEATS):
        order = [u for u in UNDERSHOOTS if u != 30]
        rng.shuffle(order)
        trials.append((0, 30))                      # a reference every round
        trials.extend((0, u) for u in order)
    trials.append((0, 30))
    for step in (SCALE_STEP_DEG, SCALE_STEP_DEG):
        trials.append((step, 30))
        trials.append((0, 30))
    for target, values in EDGES:
        for _ in range(EDGE_REPEATS):
            order = list(values)
            rng.shuffle(order)
            trials.extend((target, u) for u in order)
    return trials


def run(out):
    out.mkdir(parents=True, exist_ok=False)
    daemon = Daemon()
    manifest = {"pass_rule": PASS_RULE, "crop": CROP, "trials": []}
    try:
        for index, (target, undershoot) in enumerate(plan()):
            jpeg = land(daemon, target, undershoot)
            name = "%02d_t%+d_u%d.jpg" % (index, target, undershoot)
            (out / name).write_bytes(jpeg)
            manifest["trials"].append({"index": index, "target": target,
                                       "undershoot": undershoot, "frame": name,
                                       "at": time.time()})
            print(name, flush=True)
    finally:
        daemon.call("center_camera")
        (out / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")


def gray(path):
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_GRAYSCALE)
    x0, y0, x1, y1 = CROP
    return np.float32(image[y0:y1, x0:x1])


def analyse(out):
    manifest = json.loads((out / "manifest.json").read_text())
    trials = manifest["trials"]
    window = None
    references = {}
    for one in trials:
        if one["undershoot"] == 30 and one["target"] not in references:
            references[one["target"]] = gray(out / one["frame"])
    for one in trials:
        image = gray(out / one["frame"])
        if window is None:
            window = cv2.createHanningWindow(image.shape[::-1], cv2.CV_32F)
        reference = references[one["target"] if one["target"] in references else 0]
        (dx, _dy), response = cv2.phaseCorrelate(reference, image, window)
        one["dx_px"], one["response"] = round(dx, 3), round(response, 3)
    # Pixels per degree from the commanded step, both landed with the 30.
    zero = trials[0]["frame"]
    stepped = []
    for one in trials:
        if one["target"] == SCALE_STEP_DEG:
            (dx, _), _r = cv2.phaseCorrelate(gray(out / zero), gray(out / one["frame"]), window)
            stepped.append(dx)
    px_per_deg = abs(float(np.mean(stepped))) / SCALE_STEP_DEG
    summary = {"pass_rule": manifest["pass_rule"], "px_per_deg": round(px_per_deg, 3),
               "positions": {}}
    for target in sorted({t["target"] for t in trials if t["target"] != SCALE_STEP_DEG}):
        here = [t for t in trials if t["target"] == target]
        base = float(np.mean([t["dx_px"] for t in here if t["undershoot"] == 30]))
        rows = {}
        for u in sorted({t["undershoot"] for t in here}, reverse=True):
            off = [(t["dx_px"] - base) / px_per_deg for t in here if t["undershoot"] == u]
            rows[u] = {"landings_deg": [round(v, 3) for v in off],
                       "worst_deg": round(max(abs(v) for v in off), 3),
                       "passes": bool(u == 30 or max(abs(v) for v in off) <= TOLERANCE_DEG),
                       "low_response": min(t["response"] for t in here
                                           if t["undershoot"] == u) < 0.2}
        passing = 30
        for u in sorted(rows, reverse=True):
            if u == 0 or not rows[u]["passes"]:
                break
            passing = u
        summary["positions"][str(target)] = {"undershoots": rows, "smallest_passing": passing}
    rest_answer = summary["positions"]["0"]["smallest_passing"]
    summary["backlash_deg"] = summary["positions"]["0"]["undershoots"].get(0, {}).get("landings_deg")
    summary["measured"] = rest_answer
    summary["deploy"] = min(30, 2 * rest_answer)
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    return summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("output", type=Path)
    p.add_argument("--analyse", action="store_true")
    a = p.parse_args()
    if not a.analyse:
        run(a.output)
    print(json.dumps(analyse(a.output), indent=1))
    sys.exit(0)
