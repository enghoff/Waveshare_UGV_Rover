#!/usr/bin/env python3
"""Offline checks for the semantic world state. No rover, no GPU, no encoders.

    python world_state/selftest.py
    ssh orin 'cd ~/ugv/world_state && python3 selftest.py'

What is covered is the part where a bug is silent rather than loud. An inspection
that stores nothing says so out loud; an inspection that stores the *wrong* thing
looks exactly like one that worked. So: nothing becomes an identity that was not
measured, the provenance the rover measured really is on every row, a database
written by an older build still opens, and every failure path leaves the world
untouched.

Everything here runs against `FakeEyes` and a temporary directory. That is enough
to prove the store, the rules and the geometry, and nothing at all about what the
real encoders see.

This file is the runner. The checks live beside it, one module per part of the
component, and each exports a `TESTS` tuple: the database, the geometry that
turns a look into a place, the fit that places several things out of many
bearings at once, perception, one inspection end to end, identity, and search.
`test_fakes.py` holds the store, camera, pose and sighting they share.
"""
from __future__ import annotations

import sys

from test_harness import FAIL, PASS, SKIP
from test_cluster import TESTS as CLUSTER_TESTS
from test_inspect import TESTS as INSPECT_TESTS
from test_locate import TESTS as LOCATE_TESTS
from test_perceive import TESTS as PERCEIVE_TESTS
from test_resolve import TESTS as RESOLVE_TESTS
from test_search import TESTS as SEARCH_TESTS
from test_store import TESTS as STORE_TESTS

TESTS = (*STORE_TESTS, *LOCATE_TESTS, *CLUSTER_TESTS, *PERCEIVE_TESTS,
         *INSPECT_TESTS, *RESOLVE_TESTS, *SEARCH_TESTS)


def main() -> int:
    for test in TESTS:
        try:
            test()
        except Exception as exc:                       # noqa: BLE001
            FAIL.append(f"{test.__name__} raised {type(exc).__name__}: {exc}")

    for name in PASS:
        print(f"  ok   {name}")
    for name in SKIP:
        print(f"  skip {name}")
    for name in FAIL:
        print(f"  FAIL {name}")
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
