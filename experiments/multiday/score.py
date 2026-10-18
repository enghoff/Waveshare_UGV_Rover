"""The multi-day benchmark scored by the rules set before it ran
(docs/decisions/the-multi-day-benchmark-is-judged-so.md).

    python score.py [DAYS_DIR]

DAYS_DIR (default days/ beside this file) holds, per day N:
- dayN-answers.json: answer.py's output, copied from the rover;
- dayN-truth.json: where the owner put each thing, {"as_asked": true} or
  {"as_asked": false, "put": {"stool": "P4" or [x, y], ...}, "note": "..."},
  written from the owner's own message before the answers are read;
- day2-restart.json: day.py restart's record, copied from the rover.

Each question is "where is X?", put to three states of the world model --
the morning's (`base`), and the morning's with only one arm's pictures added
(`A` autonomy, `B` the fixed loop). An answer within `correct_m` of where X is
is right; one elsewhere is wrong, a claim that would send someone to the wrong
place; none, or one from a record seen gone or set aside, is unresolved. From day 2, "has X moved since yesterday?" for each
staged thing: an arm says it has when its answer is more than `moved_m` from the
morning's.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = json.load(open(os.path.join(HERE, "benchmark.json")))
MIN_GAIN = 3


def place(spot_or_xy):
    if isinstance(spot_or_xy, str):
        return BENCH["spots"][spot_or_xy]["at"]
    return spot_or_xy


def truth_of(day, truth):
    put = dict(BENCH["days"][str(day)]["put"])
    if not truth.get("as_asked", True):
        put.update(truth.get("put") or {})
    out = {item: place(where) for item, where in put.items()}
    out.update({name: c["at"] for name, c in BENCH["controls"].items()})
    return out, put


def judge(answer, at):
    """Right or wrong only from a record still standing: one seen gone, or set
    aside as not found again, is the rover saying it does not know where the
    thing is now, which find_thing says in so many words."""
    if answer is None or answer.get("gone") or answer.get("set_aside"):
        return "unresolved"
    return "right" if math.dist(answer["at"], at) <= BENCH["correct_m"] else "wrong"


def main():
    days_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "days")
    total = {s: {"right": 0, "wrong": 0, "unresolved": 0, "moved_right": 0, "moved_asked": 0,
                 "minutes": 0.0, "battery_used": 0} for s in ("base", "A", "B")}
    days, prev_put, stale, stops = [], None, [], []
    for day in (1, 2, 3):
        path = os.path.join(days_dir, "day%d-answers.json" % day)
        if not os.path.exists(path):
            break
        ans = json.load(open(path))
        truth, put = truth_of(day, json.load(open(os.path.join(days_dir, "day%d-truth.json" % day))))
        row = {"day": day}
        for state in ("base", "A", "B"):
            got = ans["states"].get(state, {}).get("answers") or {}
            verdicts = {q: judge(got.get(q), at) for q, at in truth.items()}
            for v in verdicts.values():
                total[state][v] += 1
            row[state] = verdicts
            if prev_put is not None and state != "base":
                base = ans["states"]["base"]["answers"]
                for item in BENCH["items"]:
                    moved = place(put[item]) != place(prev_put[item])
                    a, b = got.get(item), base.get(item)
                    said = a is not None and (b is None or math.dist(a["at"], b["at"]) > BENCH["moved_m"])
                    total[state]["moved_asked"] += 1
                    total[state]["moved_right"] += int(said == moved)
            if state in ans.get("cost", {}):
                c = ans["cost"][state]
                total[state]["minutes"] += c.get("minutes") or 0.0
                total[state]["battery_used"] += c.get("battery_used") or 0
        stale += (ans.get("autonomy_goals") or {}).get("at_gone_or_set_aside") or []
        stops += ans.get("stops") or []
        days.append(row)
        prev_put = put
    for row in days:
        print("day %d" % row["day"])
        for q in row["A"]:
            print("  %-30s base %-10s A %-10s B %s" % (q, row["base"][q], row["A"][q], row["B"][q]))
    print()
    for s in ("base", "A", "B"):
        t = total[s]
        print("%-4s right %2d  wrong %2d  unresolved %2d  moved right %d of %d  minutes %.1f  battery %d%%" % (
            s, t["right"], t["wrong"], t["unresolved"], t["moved_right"], t["moved_asked"], t["minutes"],
            t["battery_used"]))
    restart = os.path.join(days_dir, "day2-restart.json")
    same = None
    if os.path.exists(restart):
        entries = json.load(open(restart))["entries"]
        same = next((e.get("same") for e in entries if "same" in e), None)
    print("\nanswers the same across the day-2 restart:", same)
    print("autonomy goals at records seen gone or set aside:", stale or "none")
    print("stops and interventions:", stops or "none")
    if len(days) == 3:
        a, b = total["A"], total["B"]
        passed = (same is True and not stale and a["right"] >= b["right"] + MIN_GAIN
                  and a["wrong"] <= b["wrong"])
        print("\nverdict: %s (autonomy %d right against the loop's %d, %d wrong against %d; needs %d more right and no more wrong)"
              % ("PASS" if passed else "FAIL", a["right"], b["right"], a["wrong"], b["wrong"], MIN_GAIN))


if __name__ == "__main__":
    main()
