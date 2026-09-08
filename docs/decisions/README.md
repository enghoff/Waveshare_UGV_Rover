# Decisions: why the rover is the way it is

A decision record answers a question somebody will otherwise ask again: why this
mapper, why that version pin, why the obvious fix was not the fix. It exists
because the reasoning is expensive and the conclusion alone is not enough to stop
the reasoning being redone.

| Record | Status | Question it settles |
|---|---|---|
| [jetson-orin-navigation.md](jetson-orin-navigation.md) | implemented | why `slam_toolbox` and Nav2 on the Orin, and what a replacement mapper or controller would have to beat |
| [cosmos-reason2.md](cosmos-reason2.md) | closed 2026-09-02 | why there is no local vision-language model in the inspection path |
| [doorway-pivot.md](doorway-pivot.md) | closed | why the rover locked up pivoting in narrow passages, and why two plausible fixes were wrong |
| [depthai-version-pin.md](depthai-version-pin.md) | standing, retested | why the depth camera's driver is pinned below 3.x |
| [rim-frontiers-are-not-cut-up.md](rim-frontiers-are-not-cut-up.md) | closed 2026-09-08 | why a rover ringed by unknown floor is not fixed by cutting the rim into pieces, and what a next attempt has to show |

## What a record owes

**A status line at the top, with a date.** Implemented, closed, superseded,
standing. A reader needs to know within one line whether this describes something
that runs.

**The alternatives that were actually tried, and how they failed.** This is the
valuable half. "RTAB-Map was tested and removed" is worth more than any amount of
argument for `slam_toolbox`, because it tells the next person what evidence a new
candidate needs to produce. A record listing only the chosen option is a
justification, not a decision.

**What would reopen it.** A decision closed on evidence should say what new
evidence would be enough to revisit it. Reviving a local vision-language model
is not forbidden; it has to start from evidence that addresses the failures that
removed the last one.

**Which requirements it supports or retires.** By identifier, so that a
requirement's `Superseded by` field has something to point at.

## What a record does not own

**Parameter values.** The source configuration and the deploy manifest are
authoritative. A record that copies a controller's tuning has created a second
place for it to go stale, and the stale one will be the one somebody reads. Say
what was decided and where the value lives.

**How the thing currently works.** That is the component README's job. These
records are read once, by somebody asking why.

## A record is frozen once closed

Do not rewrite a closed decision to reflect what happened later. Write a new
record that supersedes it and point the old one forward. The exception is
[depthai-version-pin.md](depthai-version-pin.md), which is a standing decision
rather than a closed one: it records the current pin and gets a new dated section
each time the pin is retested against a newer release.

A decision that was reversed keeps its record. What it is now evidence of is that
the reasoning looked sound at the time, which is the most useful thing it can
tell anybody.
