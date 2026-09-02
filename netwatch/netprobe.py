#!/usr/bin/env python3
"""The other half of netwatch: watching the rover from a machine that stays up.

    python3 netprobe.py                        # watch 192.168.1.139, print changes
    python3 netprobe.py --host jetson-orin.local
    python3 netprobe.py --log probe.log        # ...and keep a record
    python3 netprobe.py --report probe.log     # read one back

The rover's own log stops the instant the rover does, and it cannot record the
failure that looks worst from a desk: a board that is up, associated and pinging
its gateway while nothing here can reach it. So this runs somewhere that is not
the rover and asks four questions a second apart, from cheapest to most telling:

* **ICMP** -- is anything at that address at all.
* **TCP 22** -- does the kernel accept a connection. A board whose userspace has
  wedged still does this.
* **the ssh banner** -- does `sshd` actually say `SSH-2.0-...`. A connection that
  is accepted and then silent is the failure this rover has shown twice, and it
  is invisible to ping.
* **TCP 8769** -- is the rover daemon listening, which is the only one of the
  four that says the rover is *working* rather than merely reachable.

Nothing here is deployed. It runs on the workstation or on MEDIA, and its log is
meant to be read beside the rover's own -- the pair is what separates "the board
went down" from "the board was fine and the network was not".
"""

import argparse
import os
import platform
import re
import socket
import subprocess
import sys
import time

DEFAULT_HOST = "192.168.1.139"
DAEMON_PORT = 8769
SSH_PORT = 22


def ping(host, timeout=1):
    """Milliseconds, or None. Windows and Linux take different flags."""
    windows = platform.system() == "Windows"
    if windows:
        cmd = ["ping", "-n", "1", "-w", str(int(timeout * 1000)), host]
    else:
        cmd = ["ping", "-n", "-c", "1", "-W", str(int(timeout)), host]
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout + 2, text=True)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    m = re.search(r"time[=<]([\d.]+)\s*ms", out.stdout)
    return round(float(m.group(1)), 1) if m else 0.0


def tcp(host, port, timeout=3):
    """'open', 'refused' or 'timeout'. Refused is a live kernel with nothing
    listening, which is a different fault from a board that has gone."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return "open"
    except socket.timeout:
        return "timeout"
    except ConnectionRefusedError:
        return "refused"
    except OSError:
        return "unreachable"


def ssh_banner(host, port=SSH_PORT, timeout=4):
    """Whether sshd introduces itself, which is the test that catches a stall.

    A wedged board accepts the connection -- that is the kernel -- and then never
    gets far enough to write a banner, so `ssh` hangs where a down board would
    have failed in milliseconds.
    """
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
    except socket.timeout:
        return "timeout"
    except ConnectionRefusedError:
        return "refused"
    except OSError:
        return "unreachable"
    try:
        sock.settimeout(timeout)
        data = sock.recv(64)
        return "ok" if data.startswith(b"SSH-") else "silent"
    except socket.timeout:
        return "silent"
    except OSError:
        return "reset"
    finally:
        sock.close()


def verdict(rtt, banner, daemon):
    """One word for what the rover looks like from here."""
    if rtt is None and banner in ("timeout", "unreachable"):
        return "gone"
    if rtt is not None and banner in ("silent", "timeout"):
        return "stalled"      # answers ping, accepts TCP, says nothing
    if banner == "ok" and daemon != "open":
        return "no-daemon"
    if banner == "ok":
        return "ok"
    return "degraded"


def line(state):
    return " ".join([
        "t=" + time.strftime("%Y-%m-%dT%H:%M:%S"),
        "kind=probe",
        "verdict=" + state["verdict"],
        "rtt=" + ("none" if state["rtt"] is None else str(state["rtt"])),
        "ssh=" + state["banner"],
        "daemon=" + state["daemon"],
    ])


def watch(args):
    log = open(args.log, "a", buffering=1) if args.log else None
    previous = None
    since = time.monotonic()
    print("watching %s -- ping, ssh banner and daemon port, every %gs"
          % (args.host, args.interval))
    try:
        while True:
            started = time.monotonic()
            rtt = ping(args.host, args.timeout)
            banner = ssh_banner(args.host, timeout=args.timeout + 2)
            daemon = tcp(args.host, args.port, args.timeout)
            state = {"rtt": rtt, "banner": banner, "daemon": daemon}
            state["verdict"] = verdict(rtt, banner, daemon)
            record = line(state)
            if log:
                log.write(record + "\n")
                os.fsync(log.fileno())
            if state["verdict"] != previous:
                held = round(time.monotonic() - since)
                if previous is not None:
                    print("  ... %s held for %s" % (previous, _human(held)))
                print(record)
                previous = state["verdict"]
                since = time.monotonic()
            time.sleep(max(0, args.interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        held = round(time.monotonic() - since)
        print("\n  ... %s held for %s at the end" % (previous, _human(held)))
    finally:
        if log:
            log.close()
    return 0


def _human(seconds):
    if seconds < 90:
        return "%ds" % seconds
    if seconds < 3600:
        return "%dm%02ds" % (seconds // 60, seconds % 60)
    return "%dh%02dm" % (seconds // 3600, (seconds % 3600) // 60)


def report(path, interval):
    """Every stretch where the rover was not simply `ok`, and how long it lasted."""
    runs = []
    with open(path, errors="replace") as fh:
        for raw in fh:
            rec = dict(tok.split("=", 1) for tok in raw.split() if "=" in tok)
            if rec.get("kind") != "probe":
                continue
            v = rec.get("verdict", "?")
            if runs and runs[-1]["verdict"] == v:
                runs[-1]["n"] += 1
                runs[-1]["to"] = rec.get("t")
            else:
                runs.append({"verdict": v, "n": 1, "from": rec.get("t"),
                             "to": rec.get("t"), "ssh": rec.get("ssh"),
                             "rtt": rec.get("rtt")})
    total = sum(r["n"] for r in runs)
    if not total:
        print("no probe records in %s" % path)
        return 1
    print("FROM THE DESK  (%s, %d probes)" % (path, total))
    for run in runs:
        if run["verdict"] == "ok":
            continue
        print("  %s  %-9s for %s  (ssh=%s rtt=%s)"
              % (run["from"], run["verdict"], _human(run["n"] * interval),
                 run["ssh"], run["rtt"]))
    ok = sum(r["n"] for r in runs if r["verdict"] == "ok")
    print("  reachable and serving for %.1f%% of %s"
          % (100.0 * ok / total, _human(total * interval)))
    for name in ("gone", "stalled", "no-daemon", "degraded"):
        n = sum(r["n"] for r in runs if r["verdict"] == name)
        if n:
            print("  %-9s %s" % (name, _human(n * interval)))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DAEMON_PORT)
    p.add_argument("--interval", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--log", help="append every probe here")
    p.add_argument("--report", metavar="FILE", help="read a probe log back")
    args = p.parse_args(argv)
    if args.report:
        return report(args.report, args.interval)
    return watch(args)


if __name__ == "__main__":
    sys.exit(main())
