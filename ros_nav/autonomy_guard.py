"""Checks for an autonomous journey, shared by the bridge and offline replay."""
try:
    from permission import fence_breach, FENCE_MARGIN_M
except ImportError:  # repository layout; deployed beside this module
    import importlib.util
    from pathlib import Path
    _spec = importlib.util.spec_from_file_location(
        "autonomy_permission", Path(__file__).resolve().parents[1] /
        "rover_daemon" / "permission.py")
    _rules = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_rules)
    fence_breach = _rules.fence_breach
    FENCE_MARGIN_M = _rules.FENCE_MARGIN_M


def refusal(guard, stop_seq, *, pose=None, goal=None, path=None):
    """A queued goal cannot survive a stop; a detour cannot escape the fence.

    The inset reserves room for the body and braking. Its adequacy is still a
    physical acceptance item, not something these arithmetic checks certify.
    """
    if guard is None:
        return ""
    if guard.get("stop_seq") is None or guard["stop_seq"] != stop_seq:
        return "a stop invalidated this autonomous goal before it could finish"
    fence = guard.get("geofence")
    if not fence:
        return ""
    if pose is None:
        return "the autonomous safe area cannot be checked without a position"
    points = [("rover", pose)]
    if goal is not None:
        points.append(("adjusted goal", goal))
    if path is not None:
        points.extend(("planned route", (p.pose.position.x, p.pose.position.y))
                      for p in path.poses)
    for name, point in points:
        why = fence_breach({"x_m": point[0], "y_m": point[1]}, fence,
                           margin_m=FENCE_MARGIN_M)
        if why:
            return name + " reaches the safe-area stopping margin: " + why
    return ""
