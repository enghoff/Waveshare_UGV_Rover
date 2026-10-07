#!/usr/bin/env python3
"""M3 hardware trials, 2026-10-06: a repeated request, a hung executive, a lost
connection, and the console's stop at speed. Run on the rover, outside the
deploy tree.

    python3 m3_trials.py dup      R-AUT-11: the connection drops mid-leg; the
                                  stand-in reconnects and asks again with the
                                  same action id; it must be refused, twice
    python3 m3_trials.py hang     criterion 9: the stand-in is frozen (SIGSTOP)
                                  mid-leg; the permit must stop the rover
    python3 m3_trials.py drop     criterion 9: the stand-in's connection is cut
                                  mid-leg and not restored
    python3 m3_trials.py console  criterion 10: the owner presses the console's
                                  stop while the rover drives a long leg
    python3 m3_trials.py takeover criterion 12: a person's drive is sent mid-leg the
                                  way the console sends one; the run must end and
                                  the person's drive must be carried out
    python3 m3_trials.py voice    (deferred, 2026-10-07) the owner tells the voice model,
                                  through the console, to turn while the rover
                                  drives a long leg; the person must win

Each opens its own run (the owner is present and has handed the rover over),
drives one leg between the two points below, and writes what it saw to
/tmp/m3trials/<mode>-<time>.json. The observer samples the rover at 8 Hz on a
connection of its own. Nothing here moves the rover except through a run's
permit, and `stop_driving` as a fallback.
"""
import json, math, os, signal, socket, subprocess, sys, threading, time

HOST, PORT, RELAY_PORT = "127.0.0.1", 8769, 18769
START = (-17.034, -16.013)          # where session 3 began, by the charger
FAR = (-17.05, -11.60)              # 4.4 m along the room, body-fit, straight
FENCE = {"min_x_m": -21.0, "max_x_m": -15.3, "min_y_m": -18.9, "max_y_m": -10.3}
OUT = "/tmp/m3trials"


class Rover:
    def __init__(self, port=PORT, timeout=10.0):
        self.sock = socket.create_connection((HOST, port), timeout)
        self.f = self.sock.makefile("rwb")

    def call(self, name, args=None):
        self.f.write(json.dumps({"call": name, "arguments": args or {}}).encode() + b"\n")
        self.f.flush()
        line = self.f.readline()
        if not line:
            raise ConnectionError("the daemon closed the connection")
        return json.loads(line)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def stamp():
    return round(time.time(), 3)


# --- the relay: a connection that can be cut ---------------------------------

def relay():
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, RELAY_PORT))
    server.listen(4)
    print("relay listening", flush=True)
    while True:
        a, _ = server.accept()
        b = socket.create_connection((HOST, PORT))

        def pipe(src, dst):
            try:
                while True:
                    data = src.recv(65536)
                    if not data:
                        break
                    dst.sendall(data)
            except OSError:
                pass
            for s in (src, dst):
                try:
                    s.close()
                except OSError:
                    pass
        threading.Thread(target=pipe, args=(a, b), daemon=True).start()
        threading.Thread(target=pipe, args=(b, a), daemon=True).start()


# --- the stand-in executive ----------------------------------------------------

def standin(run_id, x, y, heading, action_id, port, reconnect):
    log = lambda *a: print(stamp(), *a, flush=True)
    rover = Rover(port)
    permit = rover.call("autonomy_permit", {"run": run_id})
    log("permit", json.dumps(permit)[:300])
    if not permit.get("ok"):
        return 2
    permit = permit["permit"]
    map_id = rover.call("nav_status").get("map_id")
    params = {"x_m": x, "y_m": y, "heading_deg": heading, "map_id": map_id,
              "said": "M3 trial leg"}
    episode = action_id.split("#")[0]
    act = {"permit": permit, "action": "drive_to", "action_id": action_id,
           "episode": episode, "params": params}
    first = rover.call("autonomy_act", act)
    log("act", json.dumps(first)[:400])
    asked_again = 0
    while True:
        try:
            time.sleep(1.0)
            renewed = rover.call("autonomy_permit", {"run": run_id})
            log("renew", renewed.get("ok"), str(renewed.get("error") or "")[:160])
            if not renewed.get("ok"):
                return 0
            act["permit"] = renewed["permit"]
            status = rover.call("autonomy_status")
            doing = status.get("doing") or {}
            if doing.get("id") == action_id and doing.get("ok") is not None:
                log("done", json.dumps(doing)[:300])
                if reconnect and asked_again < 2:
                    again = rover.call("autonomy_act", act)
                    log("asked again after it finished", json.dumps(again)[:400])
                rover.call("autonomy_release", {"run": run_id, "why": "trial leg done"})
                log("released")
                return 0
        except (ConnectionError, OSError) as error:
            log("connection lost:", error)
            if not reconnect:
                log("not reconnecting")
                while True:
                    time.sleep(1.0)
            rover.close()
            rover = Rover(PORT)
            renewed = rover.call("autonomy_permit", {"run": run_id})
            log("reconnected; renew", renewed.get("ok"), str(renewed.get("error") or "")[:160])
            if not renewed.get("ok"):
                return 0
            act["permit"] = renewed["permit"]
            again = rover.call("autonomy_act", act)
            asked_again += 1
            log("asked again after the drop", json.dumps(again)[:400])


# --- the observer ----------------------------------------------------------------

def observe(mode):
    os.makedirs(OUT, exist_ok=True)
    rover = Rover()
    nav = rover.call("nav_status")
    here = nav["pose"]
    bat = rover.call("battery")
    if not (nav.get("position_trusted") and nav.get("map_settled")):
        print("refused: position not trusted or map not settled", flush=True)
        return 2
    near_start = math.hypot(here["x_m"] - START[0], here["y_m"] - START[1])
    near_far = math.hypot(here["x_m"] - FAR[0], here["y_m"] - FAR[1])
    to = FAR if near_start <= near_far else START
    heading = math.degrees(math.atan2(to[1] - here["y_m"], to[0] - here["x_m"]))
    ttl = 3.0 if mode in ("hang", "drop") else 15.0
    opened = rover.call("autonomy_enable", {
        "purpose": f"M3 trial ({mode}), supervised; the owner is present",
        "budget": {"seconds": 120, "travel_m": 12, "actions": 6,
                   "permit_ttl_s": ttl, "geofence": FENCE}})
    if not opened.get("ok"):
        print("refused to open:", json.dumps(opened)[:400], flush=True)
        return 2
    run_id = opened["run"]["id"]
    action_id = f"trial/{mode}/{int(time.time())}#1"
    print(stamp(), "opened", run_id, "ttl", ttl, "leg to", to, "battery", bat.get("percent"), flush=True)

    procs, relay_proc = [], None
    port = PORT
    if mode in ("dup", "drop"):
        relay_proc = subprocess.Popen([sys.executable, __file__, "relay"],
                                      stdout=subprocess.PIPE, text=True)
        relay_proc.stdout.readline()
        port = RELAY_PORT
    log_path = f"{OUT}/{mode}-standin.log"
    stand = subprocess.Popen([sys.executable, __file__, "standin", run_id,
                              str(to[0]), str(to[1]), str(round(heading, 1)),
                              action_id, str(port), "1" if mode == "dup" else "0"],
                             stdout=open(log_path, "w"), stderr=subprocess.STDOUT)
    procs.append(stand)

    samples, events = [], []
    moving_since = None
    disrupted = None
    rest_since = None
    t0 = time.time()
    while time.time() - t0 < 60:
        nav = rover.call("nav_status")
        st = rover.call("autonomy_status")
        p = nav.get("pose") or {}
        run = st.get("run") or {}
        sample = {"t": stamp(), "x": p.get("x_m"), "y": p.get("y_m"), "h": p.get("heading_deg"),
                  "v": nav.get("speed_ms"), "driving": nav.get("driving"),
                  "nav2": nav.get("nav2_ready"), "stop_seq": nav.get("stop_seq"),
                  "enabled": st.get("enabled"), "latched": st.get("latched"),
                  "ended": run.get("ended") or "", "spent": run.get("spent")}
        samples.append(sample)
        v = abs(sample["v"] or 0.0)
        if v >= 0.25 and moving_since is None:
            moving_since = sample["t"]
        if disrupted is None and moving_since and sample["t"] - moving_since >= 1.0:
            if mode == "hang":
                os.kill(stand.pid, signal.SIGSTOP)
                disrupted = stamp(); events.append(("SIGSTOP the stand-in", disrupted))
            elif mode in ("dup", "drop"):
                relay_proc.kill()
                disrupted = stamp(); events.append(("cut the stand-in's connection", disrupted))
            elif mode in ("console", "voice"):
                disrupted = stamp(); events.append(("moving at speed; waiting for the console's stop", disrupted))
                print(stamp(), "PRESS STOP NOW", flush=True)
            elif mode == "takeover":
                # A person's drive, 0.3 m ahead, sent the way the console sends
                # one -- an ordinary call, not autonomy_act -- on a connection
                # of its own, so the sampling goes on while it runs. Trial S3 of
                # 2026-10-02 sent the same and was answered "busy".
                def person():
                    began = stamp()
                    answer = Rover(timeout=30).call("drive_to", {"ahead_m": 0.3, "left_m": 0.0})
                    events.append(("the person's drive answered", stamp(), round(stamp() - began, 2),
                                    {k: answer.get(k) for k in ("ok", "reason", "travelled_m", "detail", "error")}))
                disrupted = stamp(); events.append(("a person's drive sent mid-leg", disrupted))
                threading.Thread(target=person, daemon=True).start()
        # The owner presses whenever they see it moving; the leg is left to run
        # its course if they do not, and only a leg still going after 30 s is
        # stopped from here.
        if mode in ("console", "voice") and disrupted and not sample["latched"] and sample["t"] - disrupted > 30:
            events.append(("no console stop in 30 s; stop_driving sent", stamp()))
            rover.call("stop_driving")
        settled = v < 0.02 and not sample["driving"]
        rest_since = (rest_since or sample["t"]) if settled else None
        done = sample["ended"] or not sample["enabled"]
        answered = mode != "takeover" or any(e[0] == "the person's drive answered" for e in events)
        if disrupted and done and answered and rest_since and sample["t"] - rest_since > 2.0:
            break
        if mode == "dup" and done and rest_since and sample["t"] - rest_since > 2.0:
            break
        time.sleep(0.12)

    if disrupted is None and mode != "dup":
        # Never reached speed, so the trial did not happen: end the run rather
        # than leave a stand-in holding it.
        events.append(("never reached 0.25 m/s; the run was stopped", stamp()))
        rover.call("autonomy_stop", {"why": "trial did not reach speed"})

    if mode == "hang":
        os.kill(stand.pid, signal.SIGCONT)
        events.append(("SIGCONT the stand-in", stamp()))
        try:
            stand.wait(10)
        except subprocess.TimeoutExpired:
            stand.kill()
        fresh = subprocess.run([sys.executable, __file__, "standin", run_id,
                                str(to[0]), str(to[1]), "0", action_id + "x", str(PORT), "0"],
                               capture_output=True, text=True, timeout=30)
        events.append(("a fresh stand-in on the same run", fresh.stdout.strip()[:400]))
    else:
        try:
            stand.wait(15)
        except subprocess.TimeoutExpired:
            stand.kill()
    if relay_proc and relay_proc.poll() is None:
        relay_proc.kill()

    final = rover.call("autonomy_status")
    report = {"mode": mode, "run": run_id, "ttl_s": ttl, "leg_to": to, "action_id": action_id,
              "events": events, "samples": samples, "final_autonomy": final,
              "standin_log": open(log_path).read().splitlines()}
    path = f"{OUT}/{mode}-{int(time.time())}.json"
    json.dump(report, open(path, "w"), indent=1)
    summarise(report)
    print("written", path, flush=True)
    return 0


def summarise(r):
    s = r["samples"]
    print("events:", r["events"], flush=True)
    for line in r["standin_log"]:
        print("  stand-in:", line[:300], flush=True)
    if not s:
        return
    moved = [x for x in s if abs(x["v"] or 0) >= 0.25]
    vmax = max((abs(x["v"] or 0) for x in s), default=0)
    print("top speed %.2f m/s" % vmax, flush=True)
    marks = [e[1] for e in r["events"] if isinstance(e[1], float)]
    latch = (r.get("final_autonomy") or {}).get("latch") or {}
    if r["mode"] in ("console", "voice", "takeover"):
        # Timed from the moment the daemon took the press, by its own clock.
        marks = [float(latch["at"])] if latch.get("at") else []
        print("the stop:", latch.get("by"), "-", latch.get("why"), flush=True)
        if marks:
            near = [x for x in s if x["t"] <= marks[0]][-3:]
            print("speed by pose over the 0.4 s before it: %.2f m/s" % (
                math.hypot(near[-1]["x"] - near[0]["x"], near[-1]["y"] - near[0]["y"])
                / max(1e-3, near[-1]["t"] - near[0]["t"])), flush=True)
    if marks:
        m = marks[0]
        before = [x for x in s if x["t"] <= m]
        at = before[-1] if before else s[0]
        rest = None
        for i, x in enumerate(s):
            if x["t"] > m and abs(x["v"] or 0) < 0.02 and not x["driving"]:
                rest = x
                break
        if rest:
            d = math.hypot(rest["x"] - at["x"], rest["y"] - at["y"])
            print("from the disruption: at rest %.2f s and %.2f m later" % (rest["t"] - m, d), flush=True)
        ended = next((x for x in s if x["t"] > m and (x["ended"] or not x["enabled"])), None)
        if ended:
            print("run ended %.2f s after the disruption: %s" % (ended["t"] - m, ended["ended"][:160]), flush=True)
        print("nav2 up throughout:", all(x["nav2"] for x in s), flush=True)
    final = r["final_autonomy"]
    print("final: enabled", final.get("enabled"), "latched", final.get("latched"),
          "spent", json.dumps((final.get("run") or {}).get("spent")), flush=True)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "relay":
        relay()
    elif what == "standin":
        run_id, x, y, h, aid, port, rc = sys.argv[2:9]
        sys.exit(standin(run_id, float(x), float(y), float(h), aid, int(port), rc == "1"))
    else:
        sys.exit(observe(what))
