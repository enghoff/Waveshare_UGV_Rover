"""Which build of the rover produced what an episode is about.

**An episode that cannot say which rules made its evidence is a weaker record
than it looks.** The shadow run of 2026-09-08 is the case that showed it: a
deploy landed in the middle of it and changed the resolver's association rules,
so the looks recorded before about 09:43 and the ones after it were decided by
different rules. Nothing in the record said so. Anybody comparing the two halves
of that run would have been comparing two experiments while believing they had
one, and there would have been no way to find out from the record alone.

What is read is the deployer's own state file, which lives outside the deploy
tree at `~/.ugv/deploy-state.json` and holds the commit each component on this
machine was last deployed at. That is the honest answer to "which code was
running", because it is the same file the deployer refuses to advance until a
component's checks have passed.

Read from disk rather than asked for over the daemon's port, because the daemon
does not report its own commit and adding a call for it would mean changing a
component this one is deliberately kept out of. A recorder runs on the rover
beside the file, so there is nothing to reach for.
"""
from __future__ import annotations

import json
import os
from typing import Any

STATE = "~/.ugv/deploy-state.json"

#: The components whose build changes what an episode's evidence means. The
#: world state decides identity and placement; the daemon owns capture and the
#: gimbal; navigation owns the pose every bearing is measured from. A change to
#: any of the three makes a look mean something different from the look before
#: it.
WATCHED = ("world_state", "rover_daemon", "ros_nav")


def builds(path: str | None = None) -> dict[str, str]:
    """The commit each watched component was last deployed at, shortened.

    An empty answer rather than an error when the file is missing or unreadable:
    a recorder running on a machine that was never deployed to -- a developer's
    desk, a test -- should record what it can and say nothing it cannot.
    """
    where = os.path.expanduser(path or STATE)
    try:
        with open(where, encoding="utf-8") as handle:
            state = json.load(handle)
    except (OSError, ValueError):
        return {}
    got = state.get("components")
    if not isinstance(got, dict):
        return {}
    return {name: str(got[name])[:12] for name in WATCHED
            if isinstance(got.get(name), str)}


def changed(before: dict[str, str], after: dict[str, str]) -> list[str]:
    """Which watched components were redeployed between two readings."""
    return sorted(name for name in set(before) | set(after)
                  if before.get(name) != after.get(name))


def describe(what: dict[str, Any]) -> str:
    if not what:
        return "which build was running is not known"
    return ", ".join(f"{name} {commit}" for name, commit in sorted(what.items()))
