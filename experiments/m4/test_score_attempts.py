"""Checks for M4's scoring, on a room drawn by hand: one painting, two old looks
at it, and an attempt whose chosen look ranged it and whose re-look did not.

    python experiments/m4/test_score_attempts.py
"""
from __future__ import annotations

import json
from contextlib import closing
import math
import os
import sqlite3
import struct
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "autonomy"))
import score_attempts as sa  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

FAILED: list[str] = []
PLAIN = 384
#: Where the painting really is, and where the rover has it.
TRUTH = (3.05, 0.0)
PLACED = (3.0, 0.25)


def check(what, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + what + ("" if ok else f"\n         got:  {got}\n         want: {want}"))
    if not ok:
        FAILED.append(what)


def _vector():
    values = [0.0] * PLAIN
    values[0] = 1.0
    return struct.pack(f"<{PLAIN}f", *values)


def _look(store, frame, at, toward, entity=None, range_m=None, seconds_ago=50.0):
    bearing = math.degrees(math.atan2(toward[1] - at[1], toward[0] - at[0]))
    store.db.execute(
        "INSERT INTO observations(entity_id, inference_id, observed_at, source, frame_id,"
        " bbox_json, observer_pose_json, map_session, bearing_deg, span_deg, dino_blob,"
        " range_m, range_sigma_m) VALUES(?,?,?,?,?,?,?,1,?,5.0,?,?,?)",
        (entity, abs(hash(frame)) % 100000, time.time() - seconds_ago, "test", frame,
         "[0.4, 0.4, 0.6, 0.6]", json.dumps({"x_m": at[0], "y_m": at[1], "heading_deg": 0.0}),
         bearing, _vector(), range_m, None if range_m is None else 0.05))
    store.db.execute("INSERT OR IGNORE INTO frames(id, path, taken_at) VALUES(?,?,?)",
                     (frame, f"{frame}.jpg", time.time() - seconds_ago))
    store.db.commit()


def the_room(directory):
    """The snapshot: the painting placed from two old looks. Returns its path."""
    store = WorldStore(directory)
    store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', '1')")
    born = time.time() - 100.0
    placement = {"x_m": PLACED[0], "y_m": PLACED[1], "uncertainty_m": 0.4,
                 "error_major_m": 0.4, "error_minor_m": 0.2, "error_major_deg": 0.0,
                 "extent_m": 0.3, "viewpoints": 2, "baseline_m": 2.0}
    store.db.execute(
        "INSERT INTO entities(id, kind, label, canonical_description, created_at,"
        " last_seen_at, observation_count, placement_json, placement_uncertainty_m,"
        " placement_map_session, placement_updated_at, exemplars)"
        " VALUES('object:1', 'object', '', '', ?, ?, 2, ?, 0.4, 1, ?, ?)",
        (born, born, json.dumps(placement), born, _vector() * 3))
    store.db.commit()
    _look(store, "old-1", (0.0, -1.0), PLACED, entity="object:1", seconds_ago=90)
    _look(store, "old-2", (0.0, 2.0), PLACED, entity="object:1", seconds_ago=80)
    path = store.path
    store.close()
    return path


def after_the_run(snapshot, directory):
    """The store after the attempt: the snapshot's room plus the two new looks."""
    store = sa.open_copy(snapshot, directory)
    chosen_from = (1.6, 1.4)
    _look(store, "relook", (0.0, -1.0), PLACED, seconds_ago=20)
    _look(store, "chosen", chosen_from, TRUTH,
          range_m=math.hypot(TRUTH[0] - chosen_from[0], TRUTH[1] - chosen_from[1]),
          seconds_ago=10)
    path = store.path
    store.close()
    return path


def test_the_score():
    print("the score")
    truth = (0.0, 0.0)
    near, far = sa.score({"x_m": 0.1, "y_m": 0.0, "uncertainty_m": 0.3, "viewpoints": 2,
                          "baseline_m": 1.0}, truth), \
        sa.score({"x_m": 0.5, "y_m": 0.0, "uncertainty_m": 0.3, "viewpoints": 2,
                  "baseline_m": 1.0}, truth)
    check("closer to the tape scores higher", near["score"] > far["score"], True)
    honest = sa.score({"x_m": 0.1, "y_m": 0.0, "uncertainty_m": 0.15, "viewpoints": 2,
                       "baseline_m": 1.0}, truth)
    check("...and so does a claim that narrows and still holds the tape",
          honest["score"] > near["score"], True)
    past = sa.score({"x_m": 0.5, "y_m": 0.0, "uncertainty_m": 0.05, "viewpoints": 2,
                     "baseline_m": 1.0}, truth)
    check("...while one that narrows past it scores lower", past["score"] < far["score"], True)
    check("...an overconfident claim says so", (past["inside"], past["overconfident"]),
          (False, True))
    wild = sa.score({"x_m": 9.0, "y_m": 0.0, "uncertainty_m": 0.05, "viewpoints": 2,
                     "baseline_m": 1.0}, truth)
    check("a wild claim scores the floor and no lower", wild["score"], round(sa.FLOOR, 4))
    check("an unplaced thing scores the floor", sa.score(None, truth)["score"], sa.FLOOR)


def test_a_look_is_filed_into_its_own_copy():
    print("filing one look into a copy of the snapshot")
    with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
        snapshot = the_room(a)
        after = after_the_run(snapshot, b)
        chosen = sa.gain_of(sa.file_looks(snapshot, after, "object:1", ["chosen"]), TRUTH)
        check("the chosen look, which ranged the painting, is filed to it",
              (chosen["filed"], chosen["ranged"]), (1, True))
        check("...and moves the placement towards the tape",
              chosen["after"]["off_m"] < chosen["before"]["off_m"], True)
        check("...a gain", chosen["gain"] > 0, True)
        again = sa.gain_of(sa.file_looks(snapshot, after, "object:1", ["relook"]), TRUTH)
        check("the re-look from an old viewpoint gains less",
              again["gain"] < chosen["gain"], True)
        nothing = sa.gain_of(sa.file_looks(snapshot, after, "object:1", []), TRUTH)
        check("no look at all gains exactly nothing", nothing["gain"], 0.0)
        with closing(sqlite3.connect(snapshot)) as kept:
            rows = kept.execute("select count(*) from observations").fetchone()[0]
        check("the snapshot itself is left as it was", rows, 2)


def _episodes(directory, snapshot_path):
    import events
    import store as store_mod
    store = store_mod.EpisodeStore(directory)
    ref = store.open_episode("the rover chose what to do next and did it")
    goal = "improve_geometry:object:1@1.60,1.40"
    store.append(ref, events.candidate(goal, "placed loosely", score=0.4, params={
        "type": "improve_geometry", "gain_value": 0.2,
        "constraints": {"in_depth_view": True, "look_at": {"x_m": 3.0, "y_m": 0.25}}}))
    store.append(ref, events.decision(goal, "it was the best", "sha256:" + "0" * 64))
    store.append(ref, events.call("world_snapshot", {"name": ref}, ok=True,
                                  result={"path": snapshot_path, "took_s": 0.6}))
    store.append(ref, events.call("drive_to", {"x_m": 0.0, "y_m": -1.0}, ok=True,
                                  result={"role": "relook"}))
    store.append(ref, events.call("world_inspect", {"target": "object:1"}, ok=True,
                                  result={"role": "relook", "frame_id": "relook"}))
    store.append(ref, events.call("drive_to", {"x_m": 1.6, "y_m": 1.4}, ok=True, result={}))
    store.append(ref, events.call("world_inspect", {"target": "object:1"}, ok=True,
                                  result={"frame_id": "chosen"}))
    store.append(ref, events.measured("the attempt", placement_improved_m=0.15,
                                      with_relook=True))
    store.close_episode(ref, "succeeded")
    other = store.open_episode("the rover chose what to do next and did it")
    store.close_episode(other, "succeeded")
    store.close()
    return ref, os.path.join(directory, "episodes.db")


def test_an_attempt_is_read_from_its_episode_and_scored():
    print("an attempt, from its episode to its score")
    with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b, \
            tempfile.TemporaryDirectory() as c, tempfile.TemporaryDirectory() as d:
        snapshot = the_room(a)
        after = after_the_run(snapshot, b)
        rover_path = "/home/jetson/.ugv/world/snapshots/au_1_episode_1.db"
        ref, episodes = _episodes(c, rover_path)
        with closing(sqlite3.connect(snapshot)) as source, \
                closing(sqlite3.connect(os.path.join(d, "au_1_episode_1.db"))) as copy:
            source.backup(copy)
        attempts = sa.attempts_in(episodes)
        check("only the episode that took a snapshot is an attempt", len(attempts), 1)
        one = attempts[0]
        check("...its target, re-look and chosen look are read off its calls",
              (one["target"], one["relook_frames"], one["chosen_frames"]),
              ("object:1", ["relook"], ["chosen"]))
        check("...and the rover's own account of it",
              (one["own_gain_m"], one["constraints"].get("in_depth_view")), (0.15, True))
        truth = {"things": {"P1": {"x_m": TRUTH[0], "y_m": TRUTH[1], "records": ["object:1"],
                                   "where": "the test room"}}}
        scored = sa.score_attempts(attempts, truth, after, d)
        check("it is scored", scored[0]["scorable"], True)
        check("...the chosen look beat the re-look", scored[0]["difference"] > 0, True)
        check("...the chosen look's own claim fell, and the tape agrees",
              (scored[0]["chosen"]["own_change_m"] > 0, scored[0]["chosen"]["own_agrees"]),
              (True, True))
        check("...as the executive's record of the whole attempt does",
              scored[0]["recorded_agrees"], True)
        summary = sa.summary(scored)
        check("the summary counts it", (summary["attempts"], summary["scorable"]), (1, 1))
        check("...and says how often the chosen look's own account agreed",
              (summary["criterion_8"]["chosen"]["n"],
               summary["criterion_8"]["chosen"]["mean"]), (1, 1.0))
        os.remove(os.path.join(d, "au_1_episode_1.db"))
        gone = sa.score_attempts(attempts, truth, after, d)
        check("a snapshot that is not here makes an attempt unscorable, not dropped",
              (len(gone), gone[0]["scorable"], gone[0]["why"]),
              (1, False, "the snapshot is not here"))
        elsewhere = {"things": {"P2": {"x_m": 0, "y_m": 0, "records": ["object:9"]}}}
        with closing(sqlite3.connect(snapshot)) as source, \
                closing(sqlite3.connect(os.path.join(d, "au_1_episode_1.db"))) as copy:
            source.backup(copy)
        check("...and so does a target that is none of the taped things",
              sa.score_attempts(attempts, elsewhere, after, d)[0]["why"],
              "its target is none of the taped things")


def test_the_plan_figures():
    print("how many pairs")
    check("a re-look that never gains: about two dozen pairs",
          sa.pairs_needed(0.17, 0.0), 23)
    check("...one that gains half as often: over two hundred",
          sa.pairs_needed(0.17, 0.085) > 200, True)
    check("...and one that gains as often cannot be told apart",
          sa.pairs_needed(0.17, 0.17), None)


def test_the_own_account():
    print("the rover's own account against the tape")
    check("both say better", sa.agrees(0.1, 0.5), True)
    check("the rover says better and the tape says worse", sa.agrees(0.1, -0.5), False)
    check("a claim that did not change is not counted as agreeing with no change",
          sa.agrees(0.001, 0.0), None)
    check("...nor is no account at all", sa.agrees(None, 0.5), None)
    rows = [{"episode": f"e{n}", "thing": f"T{n}", "own": own, "tape": tape}
            for n, (own, tape) in enumerate([(0.1, 0.4), (0.2, 0.3), (-0.1, 0.2),
                                              (0.0, 0.0), (0.0, 0.5), (None, 0.1)])]
    got = sa.own_agreement(rows, lambda one: one["own"], lambda one: one["tape"])
    check("the rate is over the looks whose claim changed: two of three",
          (got["n"], round(got["mean"], 3)), (3, 0.667))
    check("...with the unchanged ones, and those the tape still moved, apart",
          (got["claim_unchanged"], got["claim_unchanged_but_tape_moved"], got["no_account"]),
          (2, 1, 1))


if __name__ == "__main__":
    for test in (test_the_score, test_a_look_is_filed_into_its_own_copy,
                 test_an_attempt_is_read_from_its_episode_and_scored,
                 test_the_plan_figures, test_the_own_account):
        test()
    print(f"\n{'all passed' if not FAILED else f'{len(FAILED)} failed'}")
    sys.exit(1 if FAILED else 0)
