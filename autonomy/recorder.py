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

Navigation cannot be read that way -- there is no history, only what the driving
loop is doing now -- so a move is polled and diffed on its sequence number. A move
that started and finished inside one poll interval is therefore missed, and the
recorder says so in its report rather than leaving the reader to assume the rover
sat still.

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

import client as client_mod
import events
import refs
import store as store_mod
import summary as summary_mod

#: Where the recorder keeps its place in the world state's history.
LOOK_MARK = "last_inference"
MOVE_MARK = "last_move_seq"

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
                 catch_up: int = 200) -> None:
        self.store = store
        self.client = client
        self.keep_frames = keep_frames
        self.catch_up = catch_up
        self.recorded = {"looks": 0, "moves": 0, "frames": 0, "polls": 0,
                         "missed_moves": 0, "unreachable": 0}
        self._entities_at = 0.0
        self._entities_digest = ""

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
        looks = self._record_looks(live)
        moves = self._record_move(live)
        return {"ok": True, "looks": looks, "moves": moves}

    def run(self, *, seconds: float, every_s: float = 2.0,
            report: Any = None) -> dict[str, Any]:
        """Poll until the time is up, then say what was recorded."""
        started = time.time()
        deadline = started + seconds
        while time.time() < deadline:
            got = self.poll()
            if report and (got.get("looks") or got.get("moves")
                           or not got.get("ok")):
                report(got)
            time.sleep(max(0.1, min(every_s, deadline - time.time())))
        return {**self.recorded, "seconds": round(time.time() - started, 1)}

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
                    "camera": first.get("camera")},
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
        """One episode per move the driving loop reports, on its sequence number.

        Polled rather than replayed from a history, because there is no history
        of moves to read -- so a move that began and ended between two polls is
        counted as missed and reported, never silently absent.
        """
        move = live.get("move") or {}
        seq = move.get("seq")
        if not isinstance(seq, int) or seq <= 0:
            return 0
        last = _int(self.store.marked(MOVE_MARK), 0)
        if seq <= last:
            return 0
        if seq > last + 1 and last:
            self.recorded["missed_moves"] += seq - last - 1
        generation = refs.generation_of(live)
        episode = self.store.open_episode(
            "the rover moved",
            world_generation=generation if generation != refs.UNKNOWN else None,
            map_session=live.get("map_session"),
            detail={"seq": seq, "kind": move.get("kind"),
                    "asked": move.get("asked"), "why": move.get("why")},
            note="recorded by a shadow run; the rover was driven by somebody else")
        self.store.append(episode, events.measured(
            "the move as the recorder found it",
            phase=move.get("phase"), route_m=move.get("route_m"),
            waypoints=move.get("waypoints"), replans=move.get("replans"),
            reason=move.get("reason"), age_s=move.get("age_s"),
            pose=live.get("pose"), battery_v=live.get("battery_v"),
            map_settled=live.get("map_settled")))
        # Closed straight away rather than held open across polls. The recorder
        # sees a move's phase, not its beginning and end, so an episode left
        # open would be waiting for a transition it cannot rely on seeing; what
        # it can honestly record is the state the move was in when it was found.
        self.store.close_episode(
            episode,
            "succeeded" if move.get("phase") in ("arrived", "done") else
            "interrupted" if move.get("phase") in ("stopped", "failed") else
            "abandoned",
            detail=f"seen in phase {move.get('phase')!r} by a recorder that "
                   f"polls; the phases before it were not observed")
        self.store.mark(MOVE_MARK, seq)
        self.recorded["moves"] += 1
        return 1

    # --- what the rover was like -----------------------------------------------

    def _summary(self) -> dict[str, Any]:
        """One reading of the rover, assembled from the reads it is allowed."""
        world = self.client.call("world_state_summary")
        live: dict[str, Any] = dict(world.get("summary") or {})
        live["backend"] = world.get("backend")
        try:
            nav = self.client.call("nav_status")
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
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    recorder = Recorder(store, client_mod.ReadOnly(),
                        keep_frames=not args.no_frames,
                        catch_up=args.catch_up)
    before = store.summary()
    print(f"watching for {args.seconds:.0f}s, polling every {args.every:.0f}s, "
          f"frames {'off' if args.no_frames else 'on'}")
    print(f"the record holds {before['episodes']} episodes and "
          f"{before['evidence_bytes']} bytes of evidence to begin with")

    def say(got: dict[str, Any]) -> None:
        if not got.get("ok"):
            print(f"  the daemon did not answer: {got.get('why')}")
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
