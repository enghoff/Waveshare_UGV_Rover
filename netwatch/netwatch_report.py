#!/usr/bin/env python3
"""Read a netwatch log back as boots and outages, rather than as 40,000 lines.

    netwatch_report.py                       # on the rover, the default log
    netwatch_report.py netwatch.log          # or a copy pulled back to a desk
    netwatch_report.py --episodes            # just the outages
    netwatch_report.py --window 3            # samples either side of each one

The log is a flat record of everything; this is the part that answers the two
questions somebody actually has. **Which boots ended badly** -- a boot whose
predecessor left no `stop` record went down without being asked, so the count of
those is the count of times the board fell over rather than the count of times
somebody rebooted it. And **when was the rover unreachable** -- every stretch
where the association, the address, the route or the gateway ping was missing,
with what the supplicant and the kernel said inside it and what the board was
doing either side.

An outage here is deliberately judged from the *rover's* point of view, and a
rover cannot see the one failure mode that matters most to a desk: a board that
is up, associated, addressed, pinging its gateway and still not answering ssh.
`netprobe.py` is the other half of that, run from a machine that stays up, and
the two logs are meant to be read together.
"""

import argparse
import os
import sys

HEALTHY_STATES = ("up",)


def parse(line):
    """One `key=value key=value` record into a dict, or None if it is not one."""
    if "kind=" not in line:
        return None
    out = {}
    for tok in line.strip().split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        out[k] = v
    return out if "kind" in out else None


def load(paths):
    """Every record in file order. Rotated files first, so time runs forwards."""
    records = []
    for path in paths:
        try:
            with open(path, "r", errors="replace") as fh:
                for line in fh:
                    rec = parse(line)
                    if rec:
                        rec["_file"] = os.path.basename(path)
                        records.append(rec)
        except OSError as exc:
            print("skipping %s: %s" % (path, exc), file=sys.stderr)
    return records


def default_paths(directory):
    """The rotated logs oldest first, so a report spans them in order."""
    base = os.path.join(directory, "netwatch.log")
    found = []
    for n in (2, 1):
        candidate = "%s.%d" % (base, n)
        if os.path.exists(candidate):
            found.append(candidate)
    if os.path.exists(base):
        found.append(base)
    return found


def healthy(rec):
    """Was the rover on the network at this sample?

    Four separate facts, and all four have to hold. The interesting failures are
    the ones where three of them do: associated with no address is DHCP, an
    address with no gateway answer is a link that carries nothing, and a `wpa`
    state that is not COMPLETED with everything else intact is a supplicant that
    is about to take the link away.
    """
    if rec.get("kind") not in ("sample", "change"):
        return None
    if rec.get("state") not in HEALTHY_STATES:
        return False
    if not rec.get("ip") or not rec.get("gw"):
        return False
    if rec.get("rtt", "none") == "none":
        return False
    return True


def boots(records):
    """Split the log into boots, and say how each of them ended."""
    out = []
    current = None
    for rec in records:
        if rec["kind"] == "boot":
            if current:
                out.append(current)
            current = {"boot": rec.get("boot", "?"), "start": rec.get("t", "?"),
                       "prev": rec.get("prev", "?"), "end": None, "ended": "running",
                       "up_end": None, "records": [], "lastwords": None}
        if current is None:
            # A log that starts mid-boot -- the first rotation, usually.
            current = {"boot": "?", "start": rec.get("t", "?"), "prev": "?",
                       "end": None, "ended": "running", "up_end": None,
                       "records": [], "lastwords": None}
        current["records"].append(rec)
        if rec["kind"] == "lastwords":
            current["lastwords"] = rec.get("line", "").replace("|", " ")
        if rec.get("up"):
            current["up_end"] = rec["up"]
        current["end"] = rec.get("t", current["end"])
        if rec["kind"] == "stop":
            current["ended"] = "clean"
    if current:
        out.append(current)
    # A boot ends badly when the *next* boot says so, which is the only place
    # that fact exists: the board that died did not get to write anything.
    for i, b in enumerate(out[:-1]):
        if b["ended"] != "clean":
            b["ended"] = "hard" if out[i + 1]["prev"] == "hard" else "unknown"
    return out


def episodes(records, window=2):
    """Contiguous stretches of unhealthy samples, with their surroundings.

    Events are kept from the last healthy sample onwards rather than from the
    first unhealthy one, because the record that explains an outage almost always
    lands just *before* it: the supplicant says `CTRL-EVENT-DISCONNECTED
    reason=4` the instant it happens and the next sample is up to ten seconds
    later. Attaching events only once the samples turn bad would file the cause
    under the previous healthy stretch, where nobody would look for it.
    """
    out = []
    current = None
    recent = []
    pending = []
    for rec in records:
        state = healthy(rec)
        if state is None:
            (current["events"] if current is not None else pending).append(rec)
            continue
        if state is False:
            if current is None:
                current = {"from": rec.get("t"), "to": rec.get("t"),
                           "before": list(recent[-window:]), "samples": [],
                           "events": pending, "after": []}
                pending = []
            current["to"] = rec.get("t")
            current["samples"].append(rec)
        else:
            if current is not None:
                current["after"].append(rec)
                if len(current["after"]) >= window:
                    out.append(current)
                    current = None
            pending = []
            recent.append(rec)
            recent = recent[-window:]
    if current is not None:
        out.append(current)
    return out


def secs(a, b):
    """Seconds between two `up=` readings, which is the only clock that is safe.

    Wall time on this board comes from fake-hwclock until NTP answers and then
    jumps; uptime never does.
    """
    try:
        return round(float(b) - float(a))
    except (TypeError, ValueError):
        return None


def human(seconds):
    if seconds is None:
        return "?"
    if seconds < 90:
        return "%ds" % seconds
    if seconds < 3600:
        return "%dm%02ds" % (seconds // 60, seconds % 60)
    return "%dh%02dm" % (seconds // 3600, (seconds % 3600) // 60)


def why(rec):
    """The one field of a sample that explains why it counted as unhealthy."""
    if rec.get("state") not in HEALTHY_STATES:
        return "not associated (state=%s)" % rec.get("state", "?")
    if not rec.get("ip"):
        return "no address"
    if not rec.get("gw"):
        return "no default route"
    if rec.get("rtt", "none") == "none":
        return "gateway did not answer"
    return "?"


def print_boots(bs):
    """One row per run of the service, and the kernel's boot id is what says
    whether a new row means the *board* restarted.

    The distinction is the whole point of the file, so it must not be blurred
    here: reinstalling netwatch restarts it and opens a new row, and reading that
    as a reboot would invent exactly the fault this is meant to be measuring.
    Same id as the row above means the board never went anywhere.
    """
    print("RUNS OF THE RECORDER  (a new boot id means the board itself restarted)")
    print("  %-10s %-20s %-9s %-10s %s"
          % ("boot id", "started", "board up", "ended", "note"))
    hard = 0
    reboots = 0
    previous = None
    for b in bs:
        length = human(secs(0, b["up_end"])) if b["up_end"] else "?"
        same = previous is not None and b["boot"] == previous
        if not same:
            reboots += 1
        ended = b["ended"]
        note = ""
        if ended == "hard":
            hard += 1
            note = "no shutdown recorded -- the board went down unasked"
        elif same:
            note = "same boot: the service was restarted, the board was not"
        print("  %-10s %-20s %-9s %-10s %s"
              % (b["boot"], b["start"], length, ended, note))
        if b["lastwords"]:
            print("      last words: %s" % b["lastwords"])
        previous = b["boot"]
    print("  %d run(s) of the recorder across %d board boot(s); "
          "%d ended without a shutdown" % (len(bs), reboots, hard))


def print_episodes(eps, verbose):
    print()
    print("OUTAGES  (as the rover saw them)")
    if not eps:
        print("  none: every sample had an association, an address, a route and a"
              " gateway that answered")
        return
    for ep in eps:
        first, last = ep["samples"][0], ep["samples"][-1]
        length = secs(first.get("up"), last.get("up"))
        print()
        print("  %s  for %s  -- %s" % (ep["from"], human(length), why(first)))
        before = ep["before"][-1] if ep["before"] else None
        if before:
            print("      before: sig=%s ssid=%s load=%s temp=%s cpu=%s"
                  % (before.get("sig", "?"), before.get("ssid", "?"),
                     before.get("load", "?"), before.get("temp", "?"),
                     before.get("cpu", "?")))
        print("      during: sig=%s wpa=%s usbn=%s load=%s temp=%s"
              % (first.get("sig", "?"), first.get("wpa", "?"),
                 first.get("usbn", "?"), first.get("load", "?"),
                 first.get("temp", "?")))
        for ev in ep["events"]:
            if ev["kind"] == "wpa":
                print("      wpa:  %s" % ev.get("ev", "").replace("_", " "))
            elif ev["kind"] == "kmsg":
                print("      kern: %s" % ev.get("msg", "").replace("_", " "))
            elif ev["kind"] == "boot":
                print("      >>> the board rebooted here (prev=%s)" % ev.get("prev"))
        if ep["after"]:
            back = ep["after"][-1]
            print("      after:  sig=%s ssid=%s bssid=%s rtt=%s"
                  % (back.get("sig", "?"), back.get("ssid", "?"),
                     back.get("bssid", "?"), back.get("rtt", "?")))
        elif verbose:
            print("      never recovered inside this log")


def print_summary(records, eps):
    samples = [r for r in records if r["kind"] in ("sample", "change")]
    good = [r for r in samples if healthy(r)]
    print()
    print("SUMMARY")
    if samples:
        print("  %d samples, %d healthy (%.1f%%)"
              % (len(samples), len(good), 100.0 * len(good) / len(samples)))
    reasons = {}
    for rec in records:
        if rec["kind"] == "wpa":
            ev = rec.get("ev", "")
            if "DISCONNECTED" in ev:
                reason = "reason=?"
                for tok in ev.split("_"):
                    if tok.startswith("reason="):
                        reason = tok
                reasons[reason] = reasons.get(reason, 0) + 1
    if reasons:
        print("  disconnections by reason: "
              + ", ".join("%s x%d" % (k, v) for k, v in sorted(reasons.items())))
    signals = [int(r["sig"]) for r in samples if r.get("sig", "").lstrip("-").isdigit()]
    if signals:
        print("  signal: best %d dBm, worst %d dBm, median %d dBm"
              % (max(signals), min(signals), sorted(signals)[len(signals) // 2]))
    ssids = {}
    for r in samples:
        if r.get("ssid"):
            ssids[r["ssid"]] = ssids.get(r["ssid"], 0) + 1
    if ssids:
        print("  time on each network: "
              + ", ".join("%s %.0f%%" % (k, 100.0 * v / len(samples))
                          for k, v in sorted(ssids.items(), key=lambda kv: -kv[1])))
    temps = [float(r["temp"]) for r in samples if r.get("temp", "").replace(".", "").isdigit()]
    if temps:
        print("  temperature: peak %.1f C, median %.1f C"
              % (max(temps), sorted(temps)[len(temps) // 2]))
    print("  %d outage(s)" % len(eps))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("files", nargs="*", help="log files (default: the rover's own)")
    p.add_argument("--dir", default="/var/lib/netwatch", help="where the log lives")
    p.add_argument("--episodes", action="store_true", help="outages only")
    p.add_argument("--window", type=int, default=2,
                   help="samples of context either side of an outage")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    paths = args.files or default_paths(args.dir)
    if not paths:
        print("no log found in %s -- is netwatch running?" % args.dir, file=sys.stderr)
        return 1
    records = load(paths)
    if not records:
        print("no records in %s" % ", ".join(paths), file=sys.stderr)
        return 1

    eps = episodes(records, args.window)
    if not args.episodes:
        print_boots(boots(records))
    print_episodes(eps, args.verbose)
    if not args.episodes:
        print_summary(records, eps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
