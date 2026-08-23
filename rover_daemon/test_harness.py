"""Shared pass/fail/skip lists for the rover daemon selftest split."""
from __future__ import annotations

PASS, FAIL, SKIP = [], [], []


def check(name, got, want):
    if got == want:
        PASS.append(name)
    else:
        FAIL.append(f"{name}\n     got:  {got!r}\n     want: {want!r}")
