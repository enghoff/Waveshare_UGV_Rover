import json, socket, time
# Polls the run every 5 s. Exits when the run ends, when the battery reads 25%
# or less three times standing still (the supervisor then stops it and drives it
# back), or after about 17 minutes.
s = socket.create_connection(("127.0.0.1", 8769), 10)
f = s.makefile("rwb")
def call(c, a=None):
    f.write(json.dumps({"call": c, "arguments": a or {}}).encode() + b"\n"); f.flush()
    return json.loads(f.readline())
t0 = time.time()
seen_run = False
low = 0
while time.time() - t0 < 1000:
    st = call("autonomy_status")
    nav = call("nav_status")
    pct = call("battery").get("percent")
    run = st.get("run") or {}
    if run:
        seen_run = True
    p = nav.get("pose") or {}
    print(time.strftime("%H:%M:%S"), "en", st.get("enabled"), "bat", pct,
          "pose", p.get("x_m"), p.get("y_m"), p.get("heading_deg"),
          "spent", json.dumps(run.get("spent")), "doing", json.dumps(st.get("doing"))[:160],
          "why", st.get("why"), flush=True)
    if seen_run and (not st.get("enabled") or run.get("ended")):
        print("RUN ENDED", json.dumps(st)[:1500], flush=True)
        break
    # Judged only standing still: a reading while driving sags by up to 30 points.
    if not nav.get("driving"):
        low = low + 1 if (pct is not None and pct <= 25) else 0
    if low >= 3:
        print("BATTERY LOW", pct, flush=True)
        break
    time.sleep(5)
