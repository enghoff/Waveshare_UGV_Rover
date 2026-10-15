"""Two pose_timeline.py outputs side by side, between two clock times.

    python compare_poses.py LIVE.txt REPLAY.txt HH:MM:SS HH:MM:SS

Flags a row where either pose's best neighbour fits the map 0.1 better than it.
"""
import re, sys
pat = re.compile(r"^(\S+)\s+\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+) m\s+([-+\d.]+) deg")
def load(p):
    out = {}
    for line in open(p):
        m = pat.match(line)
        if m:
            g = m.groups(); out[g[0]] = tuple(float(v) for v in g[1:])
    return out
live, rep = load(sys.argv[1]), load(sys.argv[2])
lo, hi = sys.argv[3], sys.argv[4]
print("%-9s | %-36s | %-36s" % ("time", "live: pose  fit best off", "replay: pose  fit best off"))
for t in sorted(set(live) & set(rep)):
    if lo <= t <= hi:
        l, r = live[t], rep[t]
        flag = "  <-" if (l[4] - l[3] > 0.1 or r[4] - r[3] > 0.1) else ""
        print("%-9s | %6.2f %6.2f %5.0f %4.2f %4.2f %3.1f %+5.1f | %6.2f %6.2f %5.0f %4.2f %4.2f %3.1f %+5.1f%s" % ((t,) + l + r + (flag,)))
