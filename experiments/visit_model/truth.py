"""What the benchmark knows to be true: the sessions, what was staged in each,
and which regions show which physical object.

Labels come from two kinds of file. The staged sessions' own (M5 criterion 4's,
experiments/m5_changes/labels-*.json, and the shakedown's from branch
what-where) name each staged thing as the session called it; OBJECTS says
which physical object that is, as count_all.py --across does. `labels/stayed-
put.json` names objects that were never staged, per session.
"""
import json
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))


def _main_checkout():
    """The main checkout, whose tmp/ holds the sessions' records and stores."""
    common = subprocess.run(["git", "rev-parse", "--git-common-dir"], cwd=HERE,
                            capture_output=True, text=True).stdout.strip()
    return os.path.dirname(os.path.abspath(os.path.join(HERE, common)))


MAIN = _main_checkout()
TMP = os.path.join(MAIN, "tmp")
STAGED = os.path.join(REPO, "experiments", "m5_changes", "staged")
M5 = os.path.join(REPO, "experiments", "m5_changes")

#: name, session record, staged file, labels; development or test.
SESSIONS = [
    ("shakedown", os.path.join(TMP, "ww-recordings", "shakedown-1012.json"),
     os.path.join(STAGED, "shakedown-1012.json"), os.path.join(HERE, "labels", "shakedown-1012.json"),
     "development"),
    ("session 1", os.path.join(TMP, "session1-1014", "session1-1014.json"),
     os.path.join(STAGED, "session1-1014.json"), os.path.join(M5, "labels-session1-1014.json"),
     "development"),
    ("session 2", os.path.join(TMP, "session2-1014", "session2-1014.json"),
     os.path.join(STAGED, "session2-1014.json"), os.path.join(M5, "labels-session2-1014.json"),
     "development"),
    ("session 3", os.path.join(TMP, "session3-1015", "session3-1015.json"),
     os.path.join(STAGED, "session3-1015.json"), os.path.join(M5, "labels-session3-1015.json"),
     "test"),
    ("session 4", os.path.join(TMP, "session4-1015", "session4-1015.json"),
     os.path.join(STAGED, "session4-1015.json"), os.path.join(HERE, "labels", "session4-1015.json"),
     "test"),
    # Version 2's test (docs/decisions/the-world-model-is-pass-fail-on-honesty-alone.md).
    ("session 5", os.path.join(TMP, "session5-1016", "session5-1016.json"),
     os.path.join(MAIN, "experiments", "m5_changes", "staged", "session5-1016.json"),
     os.path.join(HERE, "labels", "session5-1016.json"), "session 5"),
    # Version 3's test (docs/plans/a-world-model-kept-per-visit.md, step 6).
    ("session 6", os.path.join(TMP, "session6-1016", "session6-1016.json"),
     os.path.join(MAIN, "experiments", "m5_changes", "staged", "session6-1016.json"),
     os.path.join(HERE, "labels", "session6-1016.json"), "session 6"),
    # Version 4's test (docs/plans/a-world-model-kept-per-visit.md, step 6).
    ("session 7", os.path.join(TMP, "session7-1017", "session7-1017.json"),
     os.path.join(MAIN, "experiments", "m5_changes", "staged", "session7-1017.json"),
     os.path.join(HERE, "labels", "session7-1017.json"), "session 7"),
    # Ordinary drives, nothing staged: the survey loop three times running
    # (2026-10-17), for comparing version 4 with the deployed store on
    # pictures neither was tuned on.
    ("drives 1", os.path.join(TMP, "drives1-1017", "drives1-1017.json"),
     os.path.join(MAIN, "experiments", "m5_changes", "staged", "drives1-1017.json"),
     os.path.join(HERE, "labels", "drives1-1017.json"), "drives"),
]
#: Labels for a session made later than its own file, for things a later
#: session carried in again (session 5's box, stool and step ladder were
#: session 2's).
EXTRA_LABELS = {"session 2": [os.path.join(HERE, "labels", "session2-1014-extra.json")]}
STAYED_PUT = os.path.join(HERE, "labels", "stayed-put.json")

#: Which physical object each session's staged thing is (count_all.OBJECTS, and
#: session 4's): the charger armchair of the shakedown is the one with the
#: throw, told by its photographs.
OBJECTS = {
    "black armchair by the charger": "throw armchair", "black armchair by the balcony": "plain armchair",
    "red bean bag": "red bean bag", "black bean bag": "black bean bag",
    "black armchair, to the balcony door": "plain armchair",
    "black armchair, to the black cabinet": "throw armchair",
    "plain black armchair, toward wall A": "plain armchair", "black cabinet, along wall A": "cabinet",
    "black armchair with the throw (not moved in session 2)": "throw armchair",
    "black armchair with the throw": "throw armchair", "black cabinet": "cabinet",
    "plain black armchair (not moved in session 3)": "plain armchair",
    "plain black armchair (no throw)": "plain armchair",
    "red bean bag, carried in from a room the rover never enters": "red bean bag",
    "box": "box", "stool": "stool", "step ladder": "step ladder",
}


#: Bad-pose windows the staged files do not carry. The shakedown's pose was
#: right until 08:00:58 and went 1.33 m wrong until the owner's refit at 08:11
#: (docs/progress/2026-10-12-m5-shakedown.md).
EXTRA_BAD = {"shakedown": ["08:00:58-08:12:00"]}


def _clock(day, hhmmss):
    return time.mktime(day[:3] + tuple(int(v) for v in hhmmss.split(":")) + (0, 0, -1))


class Session:
    def __init__(self, name, record, staged, labels, role):
        self.name, self.role = name, role
        entries = json.load(open(record))["entries"]
        self.start, self.end = entries[0]["at"], entries[-1]["at"]
        self.record_start = self.start
        # A session of ordinary drives has no wait and nothing staged.
        self.staged_at = next((e["at"] for e in entries if e["step"] == "wait" and e.get("end")), self.end)
        # Session 5 waited twice: things carried in during the first wait and
        # moved during the second. "moved_at_step": "wait N" is the end of the
        # Nth wait.
        self.wait_ends = [e["at"] for e in entries if e["step"] == "wait" and e.get("end")]
        self.staged = json.load(open(staged))
        day = time.localtime(self.start)
        # Session 7 began, in effect, with a drive someone made through the
        # staged rooms before its record ("began": the clock time): its
        # pictures belong to the session.
        if self.staged.get("began"):
            self.start = _clock(day, self.staged["began"])
        bad = self.staged.get("bad", []) + EXTRA_BAD.get(name, [])
        self.bad = [tuple(_clock(day, s) for s in w.split("-")) for w in bad]
        # A thing moved during the session rather than in the staging moves at
        # the start of the step named (identity_count.py's rule).
        self.moved_at = {}
        for m in self.staged.get("also_moved", []):
            self.moved_at[OBJECTS[m["what"]]] = next(
                (e["at"] for e in entries if e["step"] == m.get("before") and e.get("begin")),
                self.staged_at)
        self.labels = json.load(open(labels)) if os.path.exists(labels) else {"things": {}}
        for extra in EXTRA_LABELS.get(name, []):
            if os.path.exists(extra):
                for thing, ids in json.load(open(extra))["things"].items():
                    self.labels["things"].setdefault(thing, []).extend(ids)

    def is_bad(self, t):
        return any(lo <= t <= hi for lo, hi in self.bad)

    def movers(self):
        """Physical objects staged in this session: moved, moved later, or
        carried in from elsewhere (a trial only if seen in an earlier session)."""
        # Things no label names (a dining chair, one of six alike) are not trials.
        out = [OBJECTS[m["what"]] for m in self.staged.get("moves", []) if m["what"] in OBJECTS]
        out += [OBJECTS[m["what"]] for m in self.staged.get("also_moved", []) if m["what"] in OBJECTS]
        out += [OBJECTS[a["what"]] for a in self.staged.get("added", []) if a["what"] in OBJECTS]
        return list(dict.fromkeys(out))

    def move_time(self, obj):
        return self.moved_at.get(obj, self.staged_at)

    def _at_step(self, item):
        step = item.get("moved_at_step")
        # Session 6's things were set out with the rover switched off, before
        # the session's record began.
        if step == "start":
            return self.start
        # Put down after an earlier drive, before the session's record began.
        if step == "record":
            return self.record_start
        if step and step.startswith("wait "):
            return self.wait_ends[int(step.split()[1]) - 1]
        return self.move_time(OBJECTS[item["what"]])

    def events(self):
        """(object, when it was staged, when its next staging in the session
        was or None): one trial each, so a thing carried in and later moved in
        one session is two trials, the first judged on the pictures between."""
        items = [i for key in ("added", "moves", "also_moved") for i in self.staged.get(key, [])
                 if i["what"] in OBJECTS]
        times = {}
        for i in items:
            times.setdefault(OBJECTS[i["what"]], set()).add(self._at_step(i))
        out = []
        for obj, ts in times.items():
            ts = sorted(ts)
            for k, t in enumerate(ts):
                out.append((obj, t, ts[k + 1] if k + 1 < len(ts) else None))
        return out


def sessions():
    return [Session(*s) for s in SESSIONS]


#: A region belongs to the session whose record began no more than this
#: before it and ended no more than this after it. Later sessions' label files
#: carry earlier sessions' pictures as history (session 2's carries 22 of the
#: cabinet from session 1), so a region's session is read from its time.
SESSION_MARGIN_S = 1800.0


def session_of(t, ss):
    for s in ss:
        if s.start - SESSION_MARGIN_S <= t <= s.end + SESSION_MARGIN_S:
            return s.name
    return None


def regions_by_object(t_of, ss=None):
    """{physical object: {region id: session name}} over every label file,
    each region in the session its time falls in."""
    ss = ss or sessions()
    out = {}
    for s in ss:
        for thing, ids in s.labels["things"].items():
            for rid in ids:
                t = t_of.get(int(rid))
                name = session_of(t, ss) if t is not None else None
                if name is not None:
                    out.setdefault(OBJECTS[thing], {})[int(rid)] = name
    if os.path.exists(STAYED_PUT):
        for obj, by_session in json.load(open(STAYED_PUT))["objects"].items():
            for name, ids in by_session.items():
                for rid in ids:
                    out.setdefault(obj, {})[int(rid)] = name
    return out


def stayed_put():
    if not os.path.exists(STAYED_PUT):
        return []
    return list(json.load(open(STAYED_PUT))["objects"])
