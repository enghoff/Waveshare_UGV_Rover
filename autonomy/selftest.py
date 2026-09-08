#!/usr/bin/env python3
"""Offline checks for the episode record. No rover, no daemon, no network.

    python autonomy/selftest.py
    ssh orin 'cd ~/ugv/autonomy && python3 selftest.py'

What is covered is the part where a bug is silent rather than loud. An episode
that fails to record says so; an episode that records the *wrong* thing, or that
quietly resolves a stale name against whatever holds it today, reads exactly like
one that worked. So: nothing already written is ever changed, a name from a world
state that has since been cleared fails closed rather than pointing at a
stranger, evidence somebody deleted reads as deleted, and a replay has nothing to
drive the rover with.

Everything runs against a temporary directory. That is enough to prove the
record, the names and the reconstruction, and nothing at all about the rover --
which has not run this yet.

This file is the runner. The checks live beside it, one module per part: the
names, the store, the reconstruction, and the few lines a person reads.
`test_fakes.py` holds the store and the one worked episode they share.
"""
from __future__ import annotations

import sys

from test_harness import FAIL, PASS, SKIP
from test_refs import TESTS as REFS_TESTS
from test_replay import TESTS as REPLAY_TESTS
from test_store import TESTS as STORE_TESTS
from test_summary import TESTS as SUMMARY_TESTS

TESTS = (*REFS_TESTS, *STORE_TESTS, *REPLAY_TESTS, *SUMMARY_TESTS)


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
