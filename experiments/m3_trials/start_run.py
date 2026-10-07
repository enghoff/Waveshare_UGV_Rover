import json, socket, sys
# Opens a fenced M3 run from wherever the rover stands; the executive starts with it.
purpose = sys.argv[1]
s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")
def call(c, a=None):
    f.write(json.dumps({"call": c, "arguments": a or {}}).encode() + b"\n"); f.flush()
    return json.loads(f.readline())
nav = call("nav_status")
print("pre", {k: nav.get(k) for k in ("pose", "position_trusted", "map_settled", "map_id")},
      call("battery").get("percent"), "%")
r = call("autonomy_start", {
    "purpose": purpose,
    "budget": {"seconds": 900, "travel_m": None, "actions": None,
               # The room as fenced for sessions 3-5, 0.1 m wider on every side: the
               # margin grew from 0.5 to 0.6 m, and at the old edges session 6's
               # route home along the top wall was refused.
               "geofence": {"min_x_m": -21.1, "max_x_m": -15.2,
                            "min_y_m": -19.0, "max_y_m": -10.2}}})
print(json.dumps({k: r.get(k) for k in ("ok", "error", "note")}),
      (r.get("run") or {}).get("id"), (r.get("run") or {}).get("start"))
