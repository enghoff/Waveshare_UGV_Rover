"""Live alerts for a supervised session: one line per event worth acting on.

Polls every 2 s. Prints when a drive has gone 20 s without moving 0.3 m, when
the run ends, when the battery reads 10% or less three times standing still,
and a one-line summary every two minutes. Read-only.
"""
import json, math, socket, time

FLOOR = 10
s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")


def call(c, a=None):
    f.write(json.dumps({"call": c, "arguments": a or {}}).encode() + b"\n"); f.flush()
    return json.loads(f.readline())


seen_run = False
low_since = None
drive_id, drive_from, drive_at, warned = None, None, 0.0, False
last_summary = 0.0
t0 = time.time()
while time.time() - t0 < 3600:
    st = call("autonomy_status")
    nav = call("nav_status")
    bat = call("battery")
    pct, volts = bat.get("percent"), bat.get("volts")
    run = st.get("run") or {}
    if run:
        seen_run = True
    p = nav.get("pose") or {}
    now = time.time()
    doing = st.get("doing") or {}
    if nav.get("driving"):
        key = doing.get("id") or "manual"
        if key != drive_id:
            drive_id, drive_from, drive_at, warned = key, (p.get("x_m"), p.get("y_m")), now, False
        moved = math.hypot((p.get("x_m") or 0) - (drive_from[0] or 0),
                           (p.get("y_m") or 0) - (drive_from[1] or 0))
        if moved >= 0.3:
            drive_from, drive_at = (p.get("x_m"), p.get("y_m")), now
        elif now - drive_at > 20 and not warned:
            warned = True
            print(time.strftime("%H:%M:%S"), "STUCK? driving %.0f s, moved %.2f m, heading %s, %s"
                  % (now - drive_at, moved, p.get("heading_deg"),
                     json.dumps(doing.get("params"))[:160]), flush=True)
    else:
        drive_id = None
    if seen_run and (not st.get("enabled") or run.get("ended")):
        print(time.strftime("%H:%M:%S"), "RUN ENDED", st.get("why"),
              json.dumps(run.get("spent")), "battery", pct, flush=True)
        break
    # At the floor only when every reading over 12 s of standing says so: a turn
    # sags the voltage for a moment, and three 2-s readings caught one on
    # 2026-10-07 reading 10% at a battery that stood at 40%.
    if nav.get("driving") or pct is None or pct > FLOOR:
        low_since = None
    elif low_since is None:
        low_since = now
    if low_since is not None and now - low_since >= 12:
        print(time.strftime("%H:%M:%S"), "BATTERY AT FLOOR", pct, volts, flush=True)
        break
    if now - last_summary > 120:
        last_summary = now
        print(time.strftime("%H:%M:%S"), "summary: battery", pct, "spent",
              json.dumps(run.get("spent")), flush=True)
    time.sleep(2)
