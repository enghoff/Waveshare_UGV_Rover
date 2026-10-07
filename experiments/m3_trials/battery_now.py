import json, socket, sys, time
# What the battery is now, and what kind of reading that is. Run it before saying
# what the charge is: on 2026-10-07 the charge was misstated four times, by a
# number from an hour before while it charged, by a reading taken seconds after a
# drive while the voltage was still recovering, and by single readings caught in
# a turn's sag. Read-only.
#
#   battery_now.py [seconds=10]
#
# Reads every second and reports the median, the voltage's trend over the
# window, and how long ago the wheels last moved. Near empty the curve is steep:
# 11.04 V is 10%, 11.22 V 20%, 11.37 V 40%, so a few tenths of recovery move
# the number by tens of points.
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
RECOVERING_S = 120.0   # the pack was still climbing two minutes after a drive

s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")


def call(c, a=None):
    f.write(json.dumps({"call": c, "arguments": a or {}}).encode() + b"\n"); f.flush()
    return json.loads(f.readline())


rows = []
t0 = time.time()
while time.time() - t0 < SECONDS or len(rows) < 3:
    b = call("battery")
    n = call("nav_status")
    rows.append((time.time() - t0, b.get("volts"), b.get("percent"), n.get("driving"),
                 (n.get("move") or {}).get("age_s"), (n.get("move") or {}).get("phase")))
    time.sleep(1.0)
volts = sorted(r[1] for r in rows if r[1] is not None)
pcts = sorted(r[2] for r in rows if r[2] is not None)
if not volts:
    print("no battery reading"); raise SystemExit(1)
v = volts[len(volts) // 2]
p = pcts[len(pcts) // 2] if pcts else None
first = [r[1] for r in rows[:3] if r[1] is not None]
last = [r[1] for r in rows[-3:] if r[1] is not None]
trend = (sum(last) / len(last) - sum(first) / len(first)) / max(rows[-1][0] - rows[0][0], 1.0) * 60.0
driving = any(r[3] for r in rows)
# "idle" is navigation with no move since it started: after a reboot or a
# restart, which read as driving here until 2026-10-07 (90% on the charger).
phase = rows[-1][5]
moved_ago = rows[-1][4] if phase == "ended" else (None if phase in (None, "idle") else 0.0)
if driving or phase not in (None, "ended", "idle"):
    kind = "UNDER LOAD (driving): reads low, not the charge"
elif trend > 0.03 and (moved_ago is None or moved_ago > RECOVERING_S):
    kind = "CHARGING (rising %.2f V/min with the wheels still): reads high" % trend
elif moved_ago is not None and moved_ago < RECOVERING_S:
    kind = "RECOVERING (wheels stopped %.0f s ago, rising %.2f V/min): will read higher" % (moved_ago, trend)
else:
    kind = "AT REST (%s, %+.2f V/min)" % (
        "wheels still %.0f s" % moved_ago if moved_ago is not None
        else "no move since navigation started", trend)
print("%s  %s%%  %.2f V  %s" % (time.strftime("%H:%M:%S"), p, v, kind))
