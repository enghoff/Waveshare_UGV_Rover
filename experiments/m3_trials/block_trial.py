"""A person in the rover's way, on a goal the person knows in advance.

Blocking an autonomous run is guesswork for whoever does the blocking: nothing
says where the rover is going next. So the two cases still owed are driven here
as a person's drives along the trials' own straight leg in the charger room
(`m3_trials.START` to `m3_trials.FAR`, 4.4 m, body-fit), and the owner is told
where to stand before it sets off:

    block_trial.py start    drive to the charger end of the leg, facing along it
    block_trial.py far      drive the whole leg, 4.4 m, as one goal: longer than
                            a near goal, so it is the route watch of 2026-10-08
                            (`nav_moves.route_watch`) that should stop it for
                            somebody standing on the line about 2 m along
    block_trial.py near     drive 1.2 m straight ahead, arriving facing the way
                            it faces now: a near goal, which should wait for
                            somebody standing 0.7 m in front of it for more than
                            5 s and then go round them or hand the goal back

Each waits `COUNTDOWN_S` before it moves, and writes the rover's pose and what
navigation says it is doing twice a second to `/tmp/m3trials/block-<mode>-<time>.jsonl`.
Run on the rover, with the owner present and the rover untethered.
"""
import json
import math
import os
import socket
import sys
import threading
import time

START = (-17.034, -16.013)
FAR = (-17.05, -11.60)
COUNTDOWN_S = 10.0
NEAR_M = 1.2
OUT = "/tmp/m3trials"


def connect():
    s = socket.create_connection(("127.0.0.1", 8769), 10)
    s.settimeout(300)
    return s.makefile("rwb")


def call(f, name, arguments=None):
    f.write(json.dumps({"call": name, "arguments": arguments or {}}).encode() + b"\n")
    f.flush()
    return json.loads(f.readline())


def watch(path, stop):
    """Pose and navigation's own account of the move, twice a second."""
    f = connect()
    with open(path, "w") as out:
        while not stop.is_set():
            n = call(f, "nav_status")
            move = n.get("move") or {}
            out.write(json.dumps({"t": round(time.time(), 2), "pose": n.get("pose"),
                                  "driving": n.get("driving"), "phase": move.get("phase"),
                                  "why": move.get("why"), "detail": move.get("detail")}) + "\n")
            out.flush()
            time.sleep(0.5)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    f = connect()
    status = call(f, "nav_status")
    here = status.get("pose") or {}
    if not here:
        raise SystemExit("navigation has no pose: nothing driven")
    # The rule autonomy's permission applies (R-SAFE-17), which a person's drive
    # does not go through: a bias of exactly zero or beyond 5 deg/s is the
    # board's motion sensor frozen, and a rover that cannot feel itself turn
    # must not be driven. Seen again at a power-up on 2026-10-08.
    bias = status.get("gyro_bias_dps")
    if bias is not None and (bias == 0 or abs(float(bias)) > 5.0):
        raise SystemExit("the gyro bias reads %s deg/s: the motion sensor is frozen; "
                         "power-cycle the rover, nothing driven" % bias)
    along = math.degrees(math.atan2(FAR[1] - START[1], FAR[0] - START[0]))
    if mode == "start":
        args = {"x_m": START[0], "y_m": START[1], "heading_deg": along}
        told = "to the charger end of the leg, facing up the room"
    elif mode == "far":
        args = {"x_m": FAR[0], "y_m": FAR[1]}
        told = ("the whole leg, %.1f m up the room; stand on its line about 2 m along"
                % math.hypot(FAR[0] - here["x_m"], FAR[1] - here["y_m"]))
    elif mode == "near":
        heading = float(here["heading_deg"])
        args = {"ahead_m": NEAR_M, "left_m": 0.0, "heading_deg": heading}
        told = "%.1f m straight ahead; stand 0.7 m in front of it and stay" % NEAR_M
    else:
        raise SystemExit(__doc__)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "block-%s-%s.jsonl" % (mode, time.strftime("%H%M%S")))
    print("%s: driving %s in %.0f s" % (time.strftime("%H:%M:%S"), told, COUNTDOWN_S), flush=True)
    time.sleep(COUNTDOWN_S)
    stop = threading.Event()
    log = threading.Thread(target=watch, args=(path, stop), daemon=True)
    log.start()
    began = time.time()
    got = call(f, "drive_to", args)
    stop.set()
    log.join(timeout=2)
    print("%s after %.0f s: %s, travelled %s m, turned %s deg -- %s" % (
        time.strftime("%H:%M:%S"), time.time() - began, got.get("reason"),
        got.get("travelled_m"), got.get("turned_deg"), got.get("detail") or got.get("error")),
        flush=True)
    print("log:", path)


if __name__ == "__main__":
    main()
