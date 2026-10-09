"""Open an M4 trial run from wherever the rover stands; the executive starts with it.

    python3 start_trial.py truth.json "purpose" [flat | min_x,max_x,min_y,max_y]

The run may look only at the records `truth.json` names for its taped things
(`score_attempts.py` reads the same file), copies the world store before each
attempt and takes a re-look from where the rover stands before driving to the
viewpoint it chose (docs/plans/autonomous-curiosity.md, M4). Run on the rover,
with the owner present and the rover untethered. The battery floor and the
fences are `experiments/m3_trials/start_run.py`'s.
"""
import json
import socket
import sys

truth_path, purpose = sys.argv[1], sys.argv[2]
area = sys.argv[3] if len(sys.argv) > 3 else "flat"
truth = json.load(open(truth_path))
targets = sorted({record for thing in truth["things"].values()
                  for record in thing.get("records", [])})
if not targets:
    raise SystemExit(f"{truth_path} names no records: nothing to look at")
if area == "flat":
    fence = None
else:
    x0, x1, y0, y1 = (float(v) for v in area.split(","))
    fence = {"min_x_m": x0, "max_x_m": x1, "min_y_m": y0, "max_y_m": y1}
s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")


def call(name, arguments=None):
    f.write(json.dumps({"call": name, "arguments": arguments or {}}).encode() + b"\n")
    f.flush()
    return json.loads(f.readline())


nav = call("nav_status")
percent = call("battery").get("percent")
print("pre", {k: nav.get(k) for k in ("pose", "position_trusted", "map_settled", "map_id",
                                     "gyro_bias_dps")},
      percent, "%", "fence", fence, "targets", len(targets))
if percent is not None and percent <= 10:
    raise SystemExit("the battery reads %s%%, at the 10%% floor: no run opened; drive home"
                     % percent)
budget = {"seconds": 900, "travel_m": None, "actions": None}
if fence:
    budget["geofence"] = fence
answer = call("autonomy_start", {
    "purpose": purpose, "budget": budget,
    "trial": {"targets": targets, "relook": True, "snapshot": True,
              "name": truth.get("name") or truth_path}})
print(json.dumps({k: answer.get(k) for k in ("ok", "error", "note", "trial")}),
      (answer.get("run") or {}).get("id"))
