"""Shared pass/fail/skip lists for the voice_chat selftest split."""
from __future__ import annotations

PASS, FAIL, SKIP = [], [], []


def check(name: str, got, want) -> None:
    if got == want:
        PASS.append(name)
    else:
        FAIL.append(f"{name}\n     got:  {got!r}\n     want: {want!r}")
