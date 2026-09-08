#!/usr/bin/env python3
"""Watch the rover do its ordinary work and write down what happened.

    ssh orin 'cd ~/ugv/autonomy && python3 recorder.py --seconds 1800'

**This is a shadow run: it observes and records, and it decides nothing.** There
is no executive yet, so the episodes it writes contain no decision and no call --
`replay` reports them as "decided nothing", which is the honest reading. What
they prove is that the record works against a real rover, which is what has to be
true before anything is allowed to write a decision into it.

It reaches the rover only through `client.ReadOnly`, which refuses every call
that could move anything. That is the structural half of "the autonomy component
has no movement-capable path": not a recorder that never calls `drive`, but one
that cannot.

## Why it reads the history rather than watching for events

The daemon has no event stream to subscribe to, so something has to poll. Polling
for *state* would drop whatever happened between two polls, and a recording with
silent gaps is worse than no recording. So the looks are taken from the world
state's own history, which is numbered: the recorder remembers the last row it
recorded and walks back from the newest until it meets it. Nothing between two
polls can be missed, and a recorder that was stopped for an hour catches up when
it starts again.

Navigation has no history of *moves*, but the driving loop keeps the last
thirty-two sentences it said and will hand back everything said since a sequence
number the caller names. So the recorder names the last sentence it recorded and
gets the ones in between -- which matters, because a replan lasts about a fifth
of a second and is the one phase of a move worth knowing about. Only a gap long
enough to overrun that history loses anything, and the recorder counts what it
lost rather than leaving the reader to assume the rover sat still.

## What one look costs to record

A frame per look, copied. That is the number the retention policy has to be
written against, and `--no-frames` exists so the cost can be measured with and
without.
"""
from __future__ import annotations

import argparse
import base64
import sys
import time
from typing import Any

import builds as builds_mod
import client as client_mod
import events
import refs
import retention as retention_mod
import store as store_mod
import summary as summary_mod

#: Where the recorder keeps its place in the world state's history, in the
#: driving loop's running commentary, in a move it has opened and not yet seen
#: the end of, and in which build of the rover was producing all of it.
LOOK_MARK = "last_inference"
BUILD_MARK = "builds"
WORLD_MARK = "world_generation"
MOVE_MARK = "last_move_seq"
OPEN_MOVE = "open_move_episode"
OPEN_MOVE_ID = "open_move_identity"

#: How a move's own word for how it ended becomes an episode outcome. The
#: vocabulary is the navigator's -- see `ros_navigator`, which is where these
#: come from -- and anything not in it closes `failed`, because a move that
#: ended for a reason nothing here recognises is not one to record as fine.
ENDINGS = {
    "arrived": "succeeded", "finished": "succeeded",
    "stopped": "interrupted", "busy": "interrupted", "refused": "interrupted",
    "blocked": "failed", "failed": "failed", "lost": "failed",
}

#: How often, in seconds, a running recorder brings the record back within its
#: retention policy. **The recorder is the only thing that ever runs retention**,
#: which is deliberate: the record only grows while something is recording, so
#: the thing doing the growing is the right thing to do the pruning, and there is
#: no timer to install and forget on a rover nobody is watching.
RETAIN_EVERY_S = 300.0

#: How much of the world to snapshot beside a look. The whole entity listing is
#: the honest answer and it is also the expensive one, so it is snapshotted at
#: the summary level per look and in full only when the listing has changed --
#: content addressing means an unchanged world costs one row however often it is
#: recorded, so "in full" is cheaper than it sounds.
SNAPSHOT_EVERY_S = 60.0


class Recorder:
    """One shadow run.

    Holds no state that is not in the store: the last look recorded and the last
    move seen are marks, so stopping the process and starting it again continues
    the same recording rather than beginning a new one.
    """

    def __init__(self, store: store_mod.EpisodeStore,
                 client: client_mod.ReadOnly, *, keep_frames: bool = True,
                 catch_up: int = 200,
                 policy: retention_mod.Policy | None = retention_mod.DEFAULT
                 ) -> None:
        self.store = store
        self.client = client
        self.keep_frames = keep_frames
        self.catch_up = catch_up
        #: None turns retention off, which is for measuring what a run would
        #: cost if nothing pruned it. Anything else is enforced as it records.
        self.policy = policy
        self.recorded = {"looks": 0, "moves": 0, "frames": 0, "polls": 0,
                         "missed_moves": 0, "unreachable": 0,
                         "evidence_removed": 0, "bytes_freed": 0,
                         "redeploys": 0, "world_cleared": 0}
        self._entities_at = 0.0
        self._entities_digest = ""
        self._retained_at = 0.0
        self._builds = builds_mod.builds()

    # --- one pass -------------------------------------------------------------

    def poll(self) -> dict[str, Any]:
        """Record everything that has happened since the last pass."""
        self.recorded["polls"] += 1
        try:
            live = self._summary()
        except client_mod.Unreachable as exc:
            # Not a failure of the recording. The daemon restarts under it --
            # a deploy does exactly that -- and the next poll carries on from
            # the mark, so nothing is lost.
            self.recorded["unreachable"] += 1
            return {"ok": False, "why": str(exc)}
        redeployed = self._check_builds()
        cleared = self._check_world(live)
        looks = self._record_looks(live)
        moves = self._record_move(live)
        return {"ok": True, "looks": looks, "moves": moves,
                "redeployed": redeployed, "cleared": cleared}

    def _check_world(self, live: dict[str, Any]) -> str:
        """Notice the semantic world being emptied under the recording.

        The same kind of event as a deploy and worth marking for the same
        reason: every identifier the episodes before it hold has been handed to
        something else, so the two halves of a run either side of a clear are
        talking about different rooms. Each episode already carries the
        generation it belongs to; the mark is what makes the moment findable.
        """
        now = refs.generation_of(live)
        if now == refs.UNKNOWN:
            return ""
        was = self.store.marked(WORLD_MARK)
        if now == was:
            return ""
        self.store.mark(WORLD_MARK, now)
        if not was:
            return ""
        self.recorded["world_cleared"] += 1
        return now

    def _check_builds(self) -> list[str]:
        """Notice a deploy landing under the recording, and write it down.

        A deploy does not only restart the daemon; it can change the rules that
        decide what a look means. When that happens mid-run, the episodes either
        side of it are not the same experiment, and the mark is what lets
        somebody find the moment afterwards.
        """
        now = builds_mod.builds()
        if not now or now == self._builds:
            return []
        moved = builds_mod.changed(self._builds, now)
        self._builds = now
        self.store.mark(BUILD_MARK, builds_mod.describe(now))
        self.recorded["redeploys"] += 1
        return moved

    def run(self, *, seconds: float, every_s: float = 2.0,
            report: Any = None) -> dict[str, Any]:
        """Poll until the time is up, then say what was recorded.

        Retention runs as part of this rather than on a timer somewhere else.
        A record that only grows while something is recording should be pruned
        by the thing doing the growing -- otherwise the honest claim is not
        "the record cannot fill the disk" but "it cannot, as long as somebody
        remembers to run the other program".
        """
        started = time.time()
        deadline = started + seconds
        self._retained_at = started
        while time.time() < deadline:
            got = self.poll()
            if report and (got.get("looks") or got.get("moves")
                           or got.get("redeployed") or got.get("cleared")
                           or not got.get("ok")):
                report(got)
            if (self.policy is not None
                    and time.time() - self._retained_at >= RETAIN_EVERY_S):
                self.retain(report=report)
            time.sleep(max(0.1, min(every_s, deadline - time.time())))
        if self.policy is not None:
            self.retain(report=report)
        return {**self.recorded, "seconds": round(time.time() - started, 1)}

    def retain(self, *, report: Any = None) -> dict[str, Any]:
        """Bring the record back within its policy, and count what went."""
        self._retained_at = time.time()
        got = retention_mod.apply(self.store, self.policy)
        self.recorded["evidence_removed"] += got["removed"]
        self.recorded["bytes_freed"] += got["freed"]
        if report and got["removed"]:
            report({"ok": True, "retained": got})
        return got

    # --- looks ----------------------------------------------------------------

    def _record_looks(self, live: dict[str, Any]) -> int:
        """One episode per inspection, oldest first.

        Oldest first so that the mark only ever moves forward over episodes that
        are already written: a recorder killed halfway through a catch-up
        repeats at most the one it was in the middle of, rather than skipping
        everything it had not got to.
        """
        since = _int(self.store.marked(LOOK_MARK), 0)
        fresh = self._observations_since(since)
        if not fresh:
            return 0
        by_look: dict[Any, list[dict]] = {}
        for row in fresh:
            by_look.setdefault(row.get("inference_id"), []).append(row)

        generation = refs.generation_of(live)
        world = self._world_snapshot(live)
        written = 0
        for inference_id in sorted(by_look, key=lambda one: (one is None, one)):
            rows = sorted(by_look[inference_id], key=lambda r: r.get("id") or 0)
            self._one_look(inference_id, rows, generation, world, live)
            self.store.mark(LOOK_MARK, max(r.get("id") or 0 for r in rows))
            written += 1
        self.recorded["looks"] += written
        return written

    def _one_look(self, inference_id: Any, rows: list[dict], generation: str,
                  world: str, live: dict[str, Any]) -> str:
        first = rows[0]
        episode = self.store.open_episode(
            "the rover looked",
            world_generation=generation if generation != refs.UNKNOWN else None,
            map_session=first.get("map_session"),
            detail={"inference_id": inference_id,
                    "frame_id": first.get("frame_id"),
                    "regions": len(rows),
                    "pan_deg": first.get("observer_pan_deg"),
                    "tilt_deg": first.get("observer_tilt_deg"),
                    "camera": first.get("camera"),
                    "builds": self._builds},
            note="recorded by a shadow run; the rover decided nothing here")

        kept = self._keep_frame(first, generation)
        named = [refs.world(generation if generation != refs.UNKNOWN else None,
                            str(row["entity_id"]))
                 for row in rows if row.get("entity_id")]
        ranged = [row for row in rows if row.get("range_m") is not None]

        self.store.append(episode, events.world_change(
            _what_the_look_did(rows),
            matched=named,
            refs=named,
            evidence=kept))
        self.store.append(episode, events.measured(
            "the look",
            regions=len(rows),
            attached=len(named),
            ranged=len(ranged),
            bearing_sigma_deg=first.get("bearing_sigma_deg"),
            pose=first.get("pose"),
            battery_v=live.get("battery_v"),
            world_at=world))
        self.store.close_episode(
            episode, "succeeded" if named else "abandoned",
            detail=("" if named else
                    "every region in this look is still unattached, which is "
                    "the ordinary state until two bearings cross"))
        return episode

    def _keep_frame(self, row: dict, generation: str) -> list[str]:
        frame_id = row.get("frame_id")
        if not self.keep_frames or not frame_id:
            return []
        try:
            got = self.client.call("world_state_frame", {"frame_id": frame_id})
        except client_mod.Unreachable:
            return []
        if not got.get("ok") or not got.get("jpeg_base64"):
            return []
        try:
            jpeg = base64.b64decode(got["jpeg_base64"])
        except (ValueError, TypeError):
            return []
        digest = self.store.keep_evidence("frame", jpeg, source={
            "frame_id": frame_id,
            "world": refs.world(generation if generation != refs.UNKNOWN
                                else None, "observation:%d" % (row.get("id") or 0))
            if row.get("id") else None,
        })
        self.recorded["frames"] += 1
        return [digest]

    def _observations_since(self, since: int) -> list[dict]:
        """Walk the history back from the newest until the mark is met.

        `catch_up` bounds the walk. A recorder started against a store with a
        month of history in it should record the recent past and say it did not
        go further, rather than spend an hour copying frames nobody asked for.
        """
        out: list[dict] = []
        looks: set[Any] = set()
        before: tuple[float, int] | None = None
        while True:
            arguments: dict[str, Any] = {"limit": 50}
            if before is not None:
                arguments["before_at"], arguments["before_id"] = before
            got = self.client.call("world_state_observations", arguments)
            rows = got.get("observations") or []
            if not rows:
                break
            for row in rows:
                if (row.get("id") or 0) <= since:
                    return out
                look = row.get("inference_id")
                if look not in looks:
                    # The bound is counted in looks and cut between them. Cutting
                    # inside one would record an inspection with some of its
                    # regions missing, which reads exactly like an inspection
                    # that found fewer things.
                    if len(looks) >= self.catch_up:
                        return out
                    looks.add(look)
                out.append(row)
            if not got.get("more"):
                break
            last = rows[-1]
            before = (last.get("observed_at") or 0.0, last.get("id") or 0)
        return out

    # --- moves ----------------------------------------------------------------

    def _record_move(self, live: dict[str, Any]) -> int:
        """One episode per move, built from the driving loop's own sentences.

        A move is not one reading. The loop says something each time the move
        turns a corner -- choosing, turning, driving, replanning, ended -- and
        each sentence carries its own sequence number, so what arrives here is a
        run of them belonging to one move. The episode opens on the first and
        closes on the sentence that says how it ended.
        """
        move = dict(live.get("move") or {})
        missed = move.pop("missed", None) or []
        last = _int(self.store.marked(MOVE_MARK), 0)
        said = [one for one in [*missed, move]
                if isinstance(one.get("seq"), int) and one["seq"] > last]
        if not said:
            return 0
        # The loop keeps thirty-two sentences. A recorder away for longer than
        # that really has lost some, and saying how many is the whole difference
        # between a gap and a rover that sat still.
        if last and said[0]["seq"] > last + 1:
            self.recorded["missed_moves"] += said[0]["seq"] - last - 1
        opened = 0
        for one in said:
            # **Only the last sentence was seen; the rest are being read back.**
            # The pose and the battery come from one reading of the rover taken
            # now, so attaching them to a sentence the loop said before we
            # looked would be recording a measurement nobody made. The sentence
            # keeps its own fields either way.
            opened += self._one_sentence(one, live, watched=one is said[-1])
            self.store.mark(MOVE_MARK, one["seq"])
        return opened

    def _one_sentence(self, said: dict[str, Any], live: dict[str, Any], *,
                      watched: bool) -> int:
        """Fold one sentence into the move it belongs to. Returns 1 if it began one."""
        phase = said.get("phase")
        open_ref = self.store.marked(OPEN_MOVE)
        identity = _dumps(said.get("kind"), said.get("asked"))

        if phase == "idle":
            # The loop is saying nothing is happening. Anything still open ended
            # without the recorder seeing it end.
            if open_ref:
                self._close_move(open_ref, None,
                                 "the loop went idle without this move being "
                                 "seen to end")
            return 0

        if open_ref and self.store.marked(OPEN_MOVE_ID) != identity:
            self._close_move(open_ref, None,
                             "a new move began before this one was seen to end")
            open_ref = ""

        began = 0
        if not open_ref:
            open_ref = self.store.open_episode(
                "the rover moved",
                world_generation=(live.get("world_generation")
                                  if refs.generation_of(live) != refs.UNKNOWN
                                  else None),
                map_session=live.get("map_session"),
                detail={"kind": said.get("kind"), "asked": said.get("asked"),
                        "first_seq": said.get("seq"),
                        "builds": self._builds},
                note="recorded by a shadow run; the rover was driven by "
                     "somebody else")
            self.store.mark(OPEN_MOVE, open_ref)
            self.store.mark(OPEN_MOVE_ID, identity)
            self.recorded["moves"] += 1
            began = 1

        self.store.append(open_ref, events.measured(
            f"the move said {phase!r}",
            seq=said.get("seq"), phase=phase, why=said.get("why"),
            route_m=said.get("route_m"), waypoints=said.get("waypoints"),
            replans=said.get("replans"), reason=said.get("reason"),
            frontiers_left=said.get("frontiers_left"),
            # Where the rover was and what its battery read, on the one sentence
            # that was actually watched. Absent on the rest rather than filled
            # in from this reading -- see `_record_move`.
            watched=True if watched else None,
            pose=live.get("pose") if watched else None,
            battery_v=live.get("battery_v") if watched else None,
            map_settled=live.get("map_settled") if watched else None))

        if phase == "ended":
            self._close_move(open_ref, said.get("reason"), said.get("why") or "")
        return began

    def _close_move(self, episode_ref: str, reason: str | None,
                    detail: str) -> None:
        outcome = ENDINGS.get(str(reason or ""), "failed" if reason else
                              "interrupted")
        self.store.close_episode(
            episode_ref, outcome,
            detail=(f"{reason}: {detail}" if reason and detail else
                    str(reason or detail)))
        self.store.mark(OPEN_MOVE, "")
        self.store.mark(OPEN_MOVE_ID, "")

    # --- what the rover was like -----------------------------------------------

    def _summary(self) -> dict[str, Any]:
        """One reading of the rover, assembled from the reads it is allowed."""
        world = self.client.call("world_state_summary")
        live: dict[str, Any] = dict(world.get("summary") or {})
        live["backend"] = world.get("backend")
        try:
            # Named so the loop hands back the sentences said since -- a replan
            # lasts about a fifth of a second and would otherwise be gone.
            # Zero on a fresh recorder rather than nothing, so that a move
            # already under way when it started is picked up from the beginning
            # of what the loop still remembers, the way the looks are.
            nav = self.client.call("nav_status", {
                "since_seq": _int(self.store.marked(MOVE_MARK), 0)})
            for key in ("move", "pose", "driving", "exploring", "estop",
                        "map_settled", "map_kept", "position_trusted",
                        "map_id", "match_score"):
                live[key] = nav.get(key)
            battery = self.client.call("battery")
            live["battery_v"] = battery.get("volts")
        except client_mod.Unreachable:
            # The world state answered and something else did not. Recording the
            # look with less around it beats not recording it.
            pass
        return live

    def _world_snapshot(self, live: dict[str, Any]) -> str:
        """A snapshot of the things the rover holds, taken at intervals.

        Content addressing means an unchanged world costs one row however often
        it is snapshotted, so the interval is about the cost of *fetching* the
        listing rather than of storing it.
        """
        now = time.time()
        if self._entities_digest and now - self._entities_at < SNAPSHOT_EVERY_S:
            return self._entities_digest
        try:
            listing = self.client.call("world_state_entities")
        except client_mod.Unreachable:
            return self._entities_digest
        self._entities_at = now
        self._entities_digest = self.store.snapshot("world_state", {
            "summary": {k: v for k, v in live.items()
                        if k not in ("move", "pose")},
            "entities": listing.get("entities") or [],
        })
        return self._entities_digest


def _what_the_look_did(rows: list[dict]) -> str:
    attached = sum(1 for row in rows if row.get("entity_id"))
    ranged = sum(1 for row in rows if row.get("range_m") is not None)
    return (f"a look found {len(rows)} region{'' if len(rows) == 1 else 's'}, "
            f"{attached} of them attached to a thing the rover already knows, "
            f"{ranged} with a measured distance")


def _dumps(*values: Any) -> str:
    """A stable name for a move's identity, so that a new one is recognisable."""
    import json
    return json.dumps(values, sort_keys=True, default=str)


def _int(text: str, fallback: int) -> int:
    try:
        return int(text)
    except (TypeError, ValueError):
        return fallback


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Record what the rover does, deciding nothing. Read-only.")
    parser.add_argument("--seconds", type=float, default=1800.0,
                        help="how long to watch for (default 1800)")
    parser.add_argument("--every", type=float, default=2.0,
                        help="seconds between polls (default 2)")
    parser.add_argument("--no-frames", action="store_true",
                        help="record looks without copying their pictures")
    parser.add_argument("--catch-up", type=int, default=200,
                        help="most looks to record from before it started")
    parser.add_argument("--dir", default=None,
                        help="where to keep the record (default ~/.ugv/autonomy)")
    parser.add_argument("--no-retention", action="store_true",
                        help="do not prune the record while recording, which is "
                             "how the unpruned cost of a run gets measured")
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    policy = None if args.no_retention else retention_mod.DEFAULT
    recorder = Recorder(store, client_mod.ReadOnly(),
                        keep_frames=not args.no_frames,
                        catch_up=args.catch_up, policy=policy)
    before = store.summary()
    print(f"watching for {args.seconds:.0f}s, polling every {args.every:.0f}s, "
          f"frames {'off' if args.no_frames else 'on'}")
    print(f"the record holds {before['episodes']} episodes and "
          f"{before['evidence_bytes']} bytes of evidence to begin with")
    print("retention: " + (policy.describe() if policy else
                           "off -- the record will not be pruned"))
    print("running: " + builds_mod.describe(recorder._builds))

    def say(got: dict[str, Any]) -> None:
        if not got.get("ok"):
            print(f"  the daemon did not answer: {got.get('why')}")
            return
        if got.get("cleared"):
            print(f"  {time.strftime('%H:%M:%S')} the semantic world was "
                  f"emptied -- it is now {got['cleared']}, and every thing the "
                  f"episodes above name has been handed to something else")
        if got.get("redeployed"):
            print(f"  {time.strftime('%H:%M:%S')} redeployed under this run: "
                  f"{', '.join(got['redeployed'])} -- episodes either side of "
                  f"this are not the same experiment")
        if got.get("retained"):
            kept = got["retained"]
            print(f"  {time.strftime('%H:%M:%S')} retention removed "
                  f"{kept['removed']} piece(s) of evidence, "
                  f"{kept['freed']} bytes")
            return
        parts = []
        if got.get("looks"):
            parts.append(f"{got['looks']} look(s)")
        if got.get("moves"):
            parts.append(f"{got['moves']} move(s)")
        print(f"  {time.strftime('%H:%M:%S')} recorded " + ", ".join(parts))

    got = recorder.run(seconds=args.seconds, every_s=args.every, report=say)
    after = store.summary()
    print()
    print(f"recorded {got['looks']} looks and {got['moves']} moves in "
          f"{got['seconds']}s over {got['polls']} polls")
    if got["missed_moves"]:
        print(f"  {got['missed_moves']} move(s) began and ended between polls "
              f"and were not recorded")
    if got["unreachable"]:
        print(f"  the daemon did not answer on {got['unreachable']} poll(s)")
    if got["world_cleared"]:
        print(f"  the semantic world was emptied {got['world_cleared']} time(s) "
              f"during this run, so the things named before and after are not "
              f"the same things")
    if got["redeploys"]:
        print(f"  the rover was redeployed {got['redeploys']} time(s) during "
              f"this run, so it is not one experiment throughout")
    if got["evidence_removed"]:
        print(f"  retention removed {got['evidence_removed']} piece(s) of "
              f"evidence, freeing {got['bytes_freed']} bytes")
    print(f"  {after['episodes'] - before['episodes']} episodes, "
          f"{after['events'] - before['events']} events, "
          f"{got['frames']} frames, "
          f"{after['evidence_bytes'] - before['evidence_bytes']} bytes of evidence")
    print()
    print(summary_mod.recent(store, limit=5))
    store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
