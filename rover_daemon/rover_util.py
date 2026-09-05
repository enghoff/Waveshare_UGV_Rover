"""Small argument coercers shared by Rover and its mixins."""
from __future__ import annotations

from typing import Any

from tool_schemas import LIGHT_MAX

def _level(value: Any) -> int:
    """Whatever the model produced -> a brightness, or ValueError.

    Tolerant on purpose. A small quantised model will hand over "255", 255.0 or
    "on" about as often as it hands over 255, and refusing those means the user
    hears "I could not do that" over a difference the tool does not care about.
    """
    if isinstance(value, bool):  # before int: bool is an int in Python
        return LIGHT_MAX if value else 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("on", "true", "full", "max"):
            return LIGHT_MAX
        if text in ("off", "false", "none"):
            return 0
        if text.endswith("%"):
            return round(float(text[:-1]) * LIGHT_MAX / 100)
        value = float(text)
    if not isinstance(value, (int, float)):
        raise ValueError(f"level must be a number from 0 to {LIGHT_MAX}")
    return int(min(max(round(value), 0), LIGHT_MAX))

def _number(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{what} must be a number")
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be a number, not {value!r}")


def _optional(value: Any, what: str) -> float | None:
    """A number the caller was allowed to leave out.

    None survives as None rather than becoming zero, because every caller of this
    reads a missing argument as "use the measured default" and a zero as a real
    request -- a `min_score` of 0 would accept any fit at all.
    """
    return None if value is None else _number(value, what)


def _flag(value: Any, what: str) -> bool:
    """Whatever the caller produced -> a yes or a no, or ValueError.

    Loose in the same way `_level` is, and for the same reason: a small quantised
    model writes "true", "yes" or 1 about as often as it writes a JSON boolean, and
    refusing those means refusing the tool. Only genuinely ambiguous input raises --
    silently reading an unrecognised word as False would turn a mistake into a picture
    that looks fine and faces the wrong way.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "yes", "on", "1"):
            return True
        if text in ("false", "no", "off", "0", ""):
            return False
    raise ValueError(f"{what} must be true or false, not {value!r}")
