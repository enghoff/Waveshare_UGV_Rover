import json, socket, sys
# Opens an M3 run from wherever the rover stands; the executive starts with it.
#   start_run.py "purpose"                      the charger room, fenced
#   start_run.py "purpose" flat                 no fence: the whole flat, ending on the battery
#   start_run.py "purpose" -20,-15,-19,-10      a fence min_x,max_x,min_y,max_y in metres
# A fence must reach 0.6 m beyond where the rover stands, or the run is refused.
purpose = sys.argv[1]
area = sys.argv[2] if len(sys.argv) > 2 else "charger"
# The room as fenced for sessions 3-5, 0.1 m wider on every side: the margin grew
# from 0.5 to 0.6 m, and at the old edges session 6's route home along the top
# wall was refused.
CHARGER_ROOM = {"min_x_m": -21.1, "max_x_m": -15.2, "min_y_m": -19.0, "max_y_m": -10.2}
if area == "charger":
    fence = CHARGER_ROOM
elif area == "flat":
    fence = None
else:
    x0, x1, y0, y1 = (float(v) for v in area.split(","))
    fence = {"min_x_m": x0, "max_x_m": x1, "min_y_m": y0, "max_y_m": y1}
s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")
def call(c, a=None):
    f.write(json.dumps({"call": c, "arguments": a or {}}).encode() + b"\n"); f.flush()
    return json.loads(f.readline())
nav = call("nav_status")
percent = call("battery").get("percent")
print("pre", {k: nav.get(k) for k in ("pose", "position_trusted", "map_settled", "map_id",
                                     "gyro_bias_dps")},
      percent, "%", "fence", fence)
# The owner's floor is 5% at rest (10% until 2026-10-10); standing still to start
# a run is at rest. On 2026-10-08 a run was opened straight after a reading at the
# then 10% floor, because a wait for set-asides to lapse had drained the last 5%
# while the rover idled.
if percent is not None and percent <= 5:
    raise SystemExit("the battery reads %s%%, at the 5%% floor: no run opened; drive home" % percent)
budget = {"seconds": 900, "travel_m": None, "actions": None}
if fence:
    budget["geofence"] = fence
r = call("autonomy_start", {"purpose": purpose, "budget": budget})
print(json.dumps({k: r.get(k) for k in ("ok", "error", "note")}),
      (r.get("run") or {}).get("id"), (r.get("run") or {}).get("start"))
