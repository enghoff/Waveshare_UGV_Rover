"""Shared pass/fail/skip lists for the world-state selftest.

A copy rather than an import, as `rover_daemon` and `voice_chat` each have: these
components are deployed to different directories on the rover and a shared module
would have to be copied there anyway, which is how the other two ended up with
one each.
"""
from __future__ import annotations

PASS, FAIL, SKIP = [], [], []


def check(name, got, want):
    if got == want:
        PASS.append(name)
    else:
        FAIL.append(f"{name}\n     got:  {got!r}\n     want: {want!r}")
