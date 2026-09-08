"""Retention: that it frees space, admits what it removed, and spares what is pinned.

The dangerous mistake here is not a store that fills up. It is a store that
quietly deletes the evidence behind the acceptance recording somebody is arguing
from, and then goes on summarising those episodes as though they could still be
checked. So what is tested is the refusal and the honesty as much as the freeing.
"""
from __future__ import annotations

import tempfile
import time

import refs
import replay
import retention
import summary
from store import DELETED, HELD
from test_fakes import PICTURE, WORLD, a_store, an_episode
from test_harness import check

DAY = 86400.0
NOW = 1757320000.0


def _evidence(store, count, *, first_at=NOW - 30 * DAY, every=DAY,
              size=1024):
    """`count` pieces of evidence, oldest first, at a day apart."""
    kept = []
    for n in range(count):
        data = b"x" * size + str(n).encode()
        kept.append(store.keep_evidence("frame", data,
                                        at=first_at + n * every))
    return kept


def test_evidence_older_than_the_limit_goes() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        old, new = _evidence(store, 2, first_at=NOW - 30 * DAY,
                             every=29 * DAY)
        got = retention.apply(store, retention.Policy(14.0, 10 ** 9), now=NOW)
        check("one removed", got["removed"], 1)
        check("...the old one", store.evidence_state(old)["state"], DELETED)
        check("...and the recent one kept",
              store.evidence_state(new)["state"], HELD)
        check("...with a reason a person can read",
              "older than 14 days" in store.evidence_state(old)["why"], True)
        store.close()


def test_the_oldest_go_first_until_the_store_is_under_its_limit() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        kept = _evidence(store, 5, first_at=NOW - 4 * DAY, size=1000)
        held = store.summary()["evidence_bytes"]
        # Room for about three of the five.
        got = retention.apply(store, retention.Policy(365.0, int(held * 0.6)),
                              now=NOW)
        check("two removed", got["removed"], 2)
        check("...the two oldest",
              [store.evidence_state(one)["state"] for one in kept],
              [DELETED, DELETED, HELD, HELD, HELD])
        check("...and it is under the limit now", got["over_limit_still"], False)
        store.close()


def test_a_pinned_episode_keeps_its_evidence_whatever_the_limit() -> None:
    """The failure this prevents is deleting the recording somebody is arguing
    from. A full disk with a loud reason is the better outcome."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.pin(episode, "the acceptance recording of 2026-09-08")
        loose = _evidence(store, 2, first_at=NOW - 40 * DAY)

        got = retention.apply(store, retention.Policy(1.0, 1), now=NOW)
        check("the loose evidence goes",
              [store.evidence_state(one)["state"] for one in loose],
              [DELETED, DELETED])
        check("...the pinned episode's does not",
              store.evidence_state(refs.digest(PICTURE))["state"], HELD)
        check("...and it says it kept some", got["pinned_kept"], 2)
        check("...while admitting it could not get under the limit",
              got["over_limit_still"], True)
        check("...and the episode is still fully replayable",
              replay.reconstruct(store, episode)["replayable"], True)
        store.close()


def test_unpinning_lets_it_go_again() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.pin(episode, "keep for now")
        check("pinned", store.pinned(), [episode])
        store.unpin(episode, "the argument is settled")
        check("...and released", store.pinned(), [])
        retention.apply(store, retention.Policy(0.0, 0), now=NOW + DAY)
        check("...so retention can take it",
              store.evidence_state(refs.digest(PICTURE))["state"], DELETED)
        store.close()


def test_pinning_keeps_the_history_of_having_been_pinned() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.pin(episode, "for the write-up")
        store.unpin(episode, "written up")
        check("not pinned now", store.is_pinned(episode), False)
        check("...but both rows are there", store.summary()["pins"], 2)
        store.close()


def test_an_episode_whose_pictures_were_expired_says_so() -> None:
    """Reported as retention rather than as breakage, and never as though the
    episode could still be checked."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        retention.apply(store, retention.Policy(0.0, 10 ** 9),
                        now=NOW + 365 * DAY)
        got = replay.reconstruct(store, episode)
        check("no longer fully replayable", got["replayable"], False)
        check("...because evidence was deleted",
              got["missing"][0]["state"], DELETED)
        check("...by retention, and it says so",
              "retention" in got["missing"][0]["why"], True)
        check("...and the summary does not pretend otherwise",
              "no longer be replayed in full" in summary.of(store, episode), True)
        check("...while what the rover did is still readable",
              got["decision"]["chose"], "look_at(object:8)")
        store.close()


def test_a_dry_run_removes_nothing_and_reports_the_same() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        kept = _evidence(store, 3, first_at=NOW - 40 * DAY)
        planned = retention.apply(store, retention.Policy(14.0, 10 ** 9),
                                  now=NOW, dry_run=True)
        check("it says what it would take", planned["removed"], 3)
        check("...and took none of it",
              [store.evidence_state(one)["state"] for one in kept],
              [HELD, HELD, HELD])
        done = retention.apply(store, retention.Policy(14.0, 10 ** 9), now=NOW)
        check("...and doing it agrees", done["removed"], planned["removed"])
        store.close()


def test_retention_run_twice_does_not_delete_anything_twice() -> None:
    """One act, one deletion record."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _evidence(store, 2, first_at=NOW - 40 * DAY)
        retention.apply(store, retention.Policy(14.0, 10 ** 9), now=NOW)
        again = retention.apply(store, retention.Policy(14.0, 10 ** 9), now=NOW)
        check("nothing left to remove", again["removed"], 0)
        check("...and two deletion records, not four",
              store.summary()["deletions"], 2)
        store.close()


def test_an_empty_store_is_left_alone() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        got = retention.apply(store, retention.Policy(0.0, 0), now=NOW)
        check("nothing removed", got["removed"], 0)
        check("...and nothing claimed", got["held_after"], 0)
        store.close()


def test_the_rate_is_measured_from_what_is_in_the_store() -> None:
    """Rather than from an assumed frame size, which is how a retention policy
    ends up written against a number nobody measured."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _evidence(store, 11, first_at=NOW - 10 * 3600.0, every=3600.0,
                  size=1024 * 1024)
        got = retention.would_fill(store, hours=0,
                                   policy=retention.Policy(14.0, 10 * 1024 ** 3))
        check("it can say", got["known"], True)
        check("...roughly a megabyte an hour per frame, eleven frames over ten "
              "hours", round(got["megabytes_per_hour"]), 1)
        check("...and how long that leaves", got["days_to_limit"] > 100, True)
        store.close()


def test_a_rate_is_refused_when_the_span_is_too_short_to_mean_anything() -> None:
    """Seen on the rover: a recorder catching up copied 200 frames in a few
    seconds and the rate came back as 14 GB an hour, which is the shape of a
    number somebody quotes in a report. It refuses now."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _evidence(store, 200, first_at=NOW, every=0.02, size=30 * 1024)
        got = retention.would_fill(store)
        check("it will not answer", got["known"], False)
        check("...and says the span is the reason",
              "too short to extrapolate" in got["why"], True)
        check("...and how to get an answer anyway",
              "hours the recording really ran" in got["why"], True)
        told = retention.would_fill(store, hours=2.0)
        check("...which, told the real duration, it gives",
              told["known"], True)
        check("...as three megabytes an hour",
              round(told["megabytes_per_hour"]), 3)
        store.close()


def test_a_rate_cannot_be_measured_from_nothing() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        check("it says so rather than guessing",
              retention.would_fill(store, hours=1)["known"], False)
        store.close()


def test_the_policy_describes_itself() -> None:
    """So that a report says what was applied rather than naming a constant."""
    said = retention.DEFAULT.describe()
    for wanted in ("14 days", "GB", "oldest removed first", "pinned"):
        check(f"the policy mentions {wanted!r}", wanted in said, True)


def test_pinning_something_that_is_not_an_episode_is_refused() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            store.pin("au/9f2a1c04ffab3d21/episode:99", "no such thing")
            check("pinning thin air is refused", "allowed", "refused")
        except KeyError:
            check("pinning thin air is refused", "refused", "refused")
        store.close()


TESTS = (
    test_evidence_older_than_the_limit_goes,
    test_the_oldest_go_first_until_the_store_is_under_its_limit,
    test_a_pinned_episode_keeps_its_evidence_whatever_the_limit,
    test_unpinning_lets_it_go_again,
    test_pinning_keeps_the_history_of_having_been_pinned,
    test_an_episode_whose_pictures_were_expired_says_so,
    test_a_dry_run_removes_nothing_and_reports_the_same,
    test_retention_run_twice_does_not_delete_anything_twice,
    test_an_empty_store_is_left_alone,
    test_the_rate_is_measured_from_what_is_in_the_store,
    test_a_rate_is_refused_when_the_span_is_too_short_to_mean_anything,
    test_a_rate_cannot_be_measured_from_nothing,
    test_the_policy_describes_itself,
    test_pinning_something_that_is_not_an_episode_is_refused,
)
