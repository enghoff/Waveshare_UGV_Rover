"""pose_timeline.py outputs summed up over a stretch of the session.

    python summarise.py HH:MM:SS HH:MM:SS NAME=TIMELINE.txt ...

Per timeline: the scored rows in the stretch, the share whose believed pose is
within 0.15 m and 3 degrees of the best-fitting pose nearby (the reference the
first-principles test uses where no independent pose exists), the median and
worst miss, and the stretch's worst minute.
"""
import re
import sys

import numpy as np

pat = re.compile(r"^(\S+)\s+\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+) m\s+([-+\d.]+) deg")


def load(p):
    out = []
    for line in open(p):
        m = pat.match(line)
        if m:
            g = m.groups()
            out.append((g[0], float(g[4]), float(g[5]), float(g[6]), float(g[7])))
    return out


def main():
    lo, hi = sys.argv[1], sys.argv[2]
    for arg in sys.argv[3:]:
        name, path = arg.split("=", 1)
        rows = [r for r in load(path) if lo <= r[0] <= hi and r[1] > 0]
        off = np.array([r[3] for r in rows])
        deg = np.abs(np.array([r[4] for r in rows]))
        good = (off <= 0.15) & (deg <= 3.0)
        worst = max(rows, key=lambda r: r[3]) if rows else None
        print("%-12s rows %3d  within 0.15 m and 3 deg %.0f%%  miss median %.2f m, worst %.2f m at %s" % (
            name, len(rows), 100 * good.mean() if len(rows) else 0, np.median(off) if len(off) else 0,
            worst[3] if worst else 0, worst[0] if worst else "-"))


if __name__ == "__main__":
    main()
