"""Whether one look shows something standing where a thing is believed to be.

This is the question an M0a inspection asks
([R-AUT-12](../docs/requirements/autonomy.md#r-aut-12)), and it is a question
about a place rather than about identity. **A single look cannot say which thing
it is looking at.** Measured on the labelled drive of 2026-09-08, a held-out look
of a real object matched the rest of its looks at 0.70 or better 43% of the time,
and a look at a *different* object, put where the first was claimed, matched it
that well 2.3% of the time; demanding 0.85 removed the false matches and kept 5%
of the true ones. Identity is R-WS-13's problem and M0b's gate, and nothing here
claims it. What a look *can* say is whether there is a surface where the looks
crossed, and the depth camera says that well.

So the claim is "something stands at (x, y), to within its uncertainty", and a
look answers it three ways:

- **supported** -- a region the look found lies on the line to the place, and
  its measured range puts it there;
- **contradicted** -- depth was measured *past* the place across the whole of its
  uncertainty, so the camera saw through where the thing should stand;
- **unresolved** -- everything else, each with its reason: a look whose direction
  was withheld, a place outside the depth camera's view, too little depth, a
  nearer surface in the way, or a surface about where the thing should be with
  no region found on it (a wall or the floor would give exactly that).

Not seeing a region is never a contradiction by itself. A detector miss is not
evidence of absence, and the only absence this module will assert is depth
measured beyond the place. Appearance against the claim's own looks is measured
and written into the evidence, because a reviewer wants it, and it decides
nothing.

Nothing here writes. It reads a stored look, its saved depth and the claim's
source looks, and returns a verdict with the numbers behind it.
"""
from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any

from . import oak

SUPPORTED = "supported"
CONTRADICTED = "contradicted"
UNRESOLVED = "unresolved"

#: How far a ranged point may land from where the claim puts the thing, on top
#: of the claim's own uncertainty: the thing's half-width, since a range is read
#: off the front of whatever the box is on and the claim is the middle of a
#: thing. **The measurement must be finer than the question**, so this is also
#: the most a look's own error may be -- twice the pointing error at that range
#: and twice the range's error, in quadrature. A noisier look cannot confirm a
#: place, because in a furnished room something stands within a noisy
#: allowance of almost anywhere: on the drive of 2026-10-01, claims moved 0.8 m
#: into what the store called open floor were confirmed half the time when the
#: allowance included the look's error.
THING_M = 0.15

#: The slack on the patch of depth that must all lie past the place before it
#: counts as seen through, on top of the claim's uncertainty: 15 cm for the
#: thing's own depth and three degrees of pointing. The same allowance
#: `negative_evidence.py` uses, which requires the whole patch behind it and
#: deliberately favours abstention over a false absence.
PATCH_SLACK_M = 0.15
POINTING_DEG = 3.0
#: How far past that the depth must reach before the place is called empty.
SEEN_PAST_M = 0.25

#: What a claim with no height is given for one, in metres up and down.
UNKNOWN_HEIGHT_SIGMA_M = 0.5

#: How much of the patch must carry depth at all. Below it the answer is "too
#: little depth", never a guess from the pixels that happen to be there.
DEPTH_SHARE = 0.8

#: Where the depth camera's distances can be believed. Inside half a metre the
#: stereo has no answer, and past four it is guessing on this rover's surfaces.
DEPTH_NEAR_M = 0.5
DEPTH_FAR_M = 4.0

#: A claim placed looser than this is too vague to test from one look: a
#: furnished room has something within three quarters of a metre of almost
#: anywhere, so "something is there" would be true of the claim by accident.
#: Half a metre keeps 51 of the 71 things the drive of 2026-10-01 placed, whose
#: median uncertainty was 0.28 m.
LOOSEST_M = 0.5

#: Bearing slack beyond a region's own half-width, as a multiple of the combined
#: pointing error: the look's bearing uncertainty and the claim's uncertainty
#: seen from where the rover stands.
BEARING_SIGMAS = 2.0


def check(look: list[dict[str, Any]], claim: dict[str, Any],
          depth: tuple[Any, dict[str, Any]] | None = None, lens: Any = None,
          source: list[dict[str, Any]] | None = None,
          pose: dict[str, Any] | None = None,
          withheld: str = "") -> dict[str, Any]:
    """The verdict on `claim` from one look, with the evidence behind it.

    `look` is every observation the look recorded (vectors included, as
    `store.observations(..., vectors=True)` returns them); it may be empty, which
    is a look that found no region at all. `claim` is the frozen hypothesis:
    `x_m`, `y_m`, and where known `height_m` (above the gimbal camera, as a
    placement carries it) and `uncertainty_m`. `depth` is `store.depth(frame)`'s
    answer and `lens` the depth camera's optics; either missing means the place
    cannot be seen into. `source` is the claim's own looks, for appearance only.
    `pose` stands in for the look's own when it recorded no region, and
    `withheld` is why the look's direction was taken away, when it was.
    """
    origin = _origin(look, pose)
    evidence: dict[str, Any] = {"claim": _rounded(claim), "regions": len(look)}
    if withheld or origin is None or any(one.get("bearing_deg") is None
                                         for one in look):
        why = withheld or ("the look recorded no pose" if origin is None else
                           "the look's direction was withheld")
        return _verdict(UNRESOLVED, f"the look cannot be pointed: {why}",
                        evidence, "direction")
    sigma = claim.get("uncertainty_m")
    sigma = 0.3 if sigma is None else float(sigma)
    evidence["origin"] = {k: origin.get(k) for k in ("x_m", "y_m", "heading_deg",
                                                    "pan_deg", "tilt_deg")}
    dx = float(claim["x_m"]) - float(origin["x_m"])
    dy = float(claim["y_m"]) - float(origin["y_m"])
    across = math.hypot(dx, dy)
    height = claim.get("height_m")
    distance = math.hypot(across, float(height or 0.0))
    bearing = math.degrees(math.atan2(dy, dx))
    evidence.update({"distance_m": round(distance, 3),
                     "bearing_deg": round(bearing, 2)})
    if sigma > LOOSEST_M:
        return _verdict(UNRESOLVED,
                        f"the claim is placed only to {sigma:.2f} m, too loosely "
                        f"for one look to test", evidence, "too loose")

    near = _regions_at_the_place(look, claim, origin, bearing, across, sigma,
                                 source)
    evidence["on_the_line"] = near
    there = [one for one in near if one.get("lands")]
    seen = _depth_at(claim, origin, distance, sigma, depth, lens)
    evidence["depth"] = seen

    if there:
        best = min(there, key=lambda one: one["off_m"])
        return _verdict(SUPPORTED,
                        f"a region ranged at {best['range_m']:.2f} m lands "
                        f"{best['off_m']:.2f} m from where the claim puts the "
                        f"thing, inside the {best['allowed_m']:.2f} m allowed",
                        evidence, "ranged")
    if seen["state"] == "clear":
        return _verdict(CONTRADICTED,
                        f"depth was measured past the place across its whole "
                        f"uncertainty: the nearest surface is "
                        f"{seen['nearest_m']:.2f} m away against "
                        f"{seen['expected_m']:.2f} m", evidence, "seen through")
    reasons = {
        "occluded": "something nearer stands in the way",
        "surface": ("depth finds a surface about where the thing should be, but "
                    "no region on the line was ranged there"),
        "outside": "the place is outside the depth camera's view",
        "sparse": "too little of the place carried depth",
        "range": "the place is outside the distances the depth camera measures",
        "none": seen.get("why") or "no depth was kept with this look",
    }
    return _verdict(UNRESOLVED, reasons.get(seen["state"], seen["state"]),
                    evidence, seen["state"])


def _verdict(outcome: str, why: str, evidence: dict[str, Any],
             code: str) -> dict[str, Any]:
    return {"outcome": outcome, "code": code, "why": why, "evidence": evidence}


def _origin(look: list[dict[str, Any]],
            pose: dict[str, Any] | None) -> dict[str, Any] | None:
    """Where the look was taken from and which way the gimbal pointed."""
    if look:
        one = look[0]
        where = one.get("pose")
        if not isinstance(where, dict) or where.get("x_m") is None:
            return None
        return {**where, "pan_deg": one.get("observer_pan_deg"),
                "tilt_deg": one.get("observer_tilt_deg"),
                "observed_at": one.get("observed_at"),
                "camera": one.get("camera") or oak.GIMBAL}
    if isinstance(pose, dict) and pose.get("x_m") is not None:
        return {"camera": oak.GIMBAL, **pose}
    return None


def _regions_at_the_place(look, claim, origin, bearing, across, sigma, source):
    """The look's regions near the line to the place, and whether each lands there.

    **A region supports the claim only where its ranged point lands inside the
    claim's uncertainty**: the bearing and the range together put a point on
    the map, and that point has to be within the claim's own uncertainty, the
    thing's half-width, the pointing error at that range and twice the range's
    error -- in height as well, when both have one. Sharing a bearing is not
    enough, and on the drive of 2026-10-01 a test that allowed for a box's whole
    width let regions 48 degrees off the line support a place.

    Unranged regions near the line are listed for the record and support
    nothing.
    """
    spread_deg = math.degrees(math.atan2(sigma, max(across, 0.3)))
    height = claim.get("height_m")
    height_sigma = claim.get("height_sigma_m")
    found = []
    for one in look:
        off_deg = (float(one["bearing_deg"]) - bearing + 180.0) % 360.0 - 180.0
        pointing_deg = BEARING_SIGMAS * float(one.get("bearing_sigma_deg") or 1.5)
        range_m = one.get("range_m")
        entry: dict[str, Any] = {"observation": one.get("id"),
                                 "off_deg": round(off_deg, 2),
                                 "range_m": range_m}
        if range_m is None:
            slack = (float(one.get("span_deg") or 0.0) / 2.0 + pointing_deg
                     + spread_deg)
            if abs(off_deg) > slack:
                continue
            entry["range_absent"] = one.get("range_absent")
        else:
            rise = math.radians(float(one.get("elevation_deg") or 0.0))
            flat = float(range_m) * math.cos(rise)
            theta = math.radians(float(one["bearing_deg"]))
            x = float(origin["x_m"]) + flat * math.cos(theta)
            y = float(origin["y_m"]) + flat * math.sin(theta)
            off_m = math.hypot(x - float(claim["x_m"]), y - float(claim["y_m"]))
            # Two independent errors, so combined in quadrature rather than
            # added: the pointing at this range, and the range itself.
            noise = math.hypot(flat * math.tan(math.radians(pointing_deg)),
                               2.0 * float(one.get("range_sigma_m") or 0.0))
            allowed = sigma + THING_M
            if off_m > allowed + 1.0:
                continue
            lands = off_m <= allowed and noise <= THING_M
            entry.update({"off_m": round(off_m, 3), "allowed_m": round(allowed, 3),
                          "noise_m": round(noise, 3)})
            if noise > THING_M:
                entry["too_noisy"] = True
            if height is not None and one.get("elevation_deg") is not None:
                up = float(range_m) * math.sin(rise) - float(height)
                up_allowed = (float(height_sigma if height_sigma is not None
                                    else sigma) + THING_M)
                entry.update({"up_m": round(up, 3),
                              "up_allowed_m": round(up_allowed, 3)})
                lands = lands and abs(up) <= up_allowed
            entry["lands"] = lands
        likeness = _likeness(one, source)
        if likeness is not None:
            entry["looks_like_claim"] = likeness
        found.append(entry)
    found.sort(key=lambda entry: entry.get("off_m", abs(entry["off_deg"])))
    return found


def _likeness(one: dict[str, Any], source: list[dict[str, Any]] | None):
    """The middle cosine between this region and the claim's own looks.

    Recorded and never decided on: see the module docstring for why one look
    cannot carry identity. Looks from the same frame are left out, so that a
    region is not compared with itself.
    """
    from . import appearance

    mine = one.get("dino_blob")
    if not mine or not source:
        return None
    scores = sorted(appearance.similarity(mine, other["dino_blob"])
                    for other in source
                    if other.get("dino_blob")
                    and other.get("frame_id") != one.get("frame_id"))
    if not scores:
        return None
    return round(scores[len(scores) // 2], 3)


def _depth_at(claim, origin, distance, sigma, depth, lens) -> dict[str, Any]:
    """What the saved depth shows across the patch where the thing should be."""
    if depth is None or depth[0] is None:
        why = depth[1] if depth is not None and isinstance(depth[1], str) else ""
        return {"state": "none", "why": why or "no depth was kept with this look"}
    millimetres, described = depth
    lens = lens or _lens_from(described)
    if lens is None:
        return {"state": "none",
                "why": "the depth camera's lens is not known for this look"}
    if origin.get("camera") not in (None, oak.GIMBAL):
        return {"state": "none",
                "why": "only a look through the gimbal camera can be projected "
                       "into its depth"}
    if not DEPTH_NEAR_M <= distance <= DEPTH_FAR_M:
        return {"state": "range", "expected_m": round(distance, 3)}
    heading = math.radians(float(origin.get("heading_deg") or 0.0))
    dx = float(claim["x_m"]) - float(origin["x_m"])
    dy = float(claim["y_m"]) - float(origin["y_m"])
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    left = -dx * math.sin(heading) + dy * math.cos(heading)
    up = float(claim.get("height_m") or 0.0)
    unit = (forward / distance, left / distance, up / distance)
    mount = oak.mount_at(origin.get("observed_at"))
    if mount.on_gimbal:
        unit = oak.camera_frame(unit, origin.get("pan_deg"), origin.get("tilt_deg"))
    xyz = oak._in_oak(unit, distance, mount)
    if xyz is None:
        return {"state": "outside", "why": "the place is behind the depth camera"}
    u, v = oak._project(xyz, lens)
    width, height = int(described["width"]), int(described["height"])
    margin = sigma + PATCH_SLACK_M + distance * math.tan(math.radians(POINTING_DEG))
    reach_x = max(2, math.ceil(lens.fx / lens.width * width * margin / xyz[2]))
    # Up and down the patch covers the claim's height and no more. The horizontal
    # margin is the place's uncertainty on the floor plan; spent vertically as
    # well it reaches the floor under anything standing on it, and the floor at
    # the place's distance would then stop any line of sight ever being clear.
    # A claim whose height is unknown gets half a metre, which reaches the floor
    # and so can only ever abstain -- the right answer for it.
    height_sigma = claim.get("height_sigma_m")
    if claim.get("height_m") is None:
        height_sigma = UNKNOWN_HEIGHT_SIGMA_M
    rise = (float(height_sigma or 0.0) + PATCH_SLACK_M
            + distance * math.tan(math.radians(POINTING_DEG)))
    reach_y = max(2, math.ceil(lens.fy / lens.height * height * rise / xyz[2]))
    x, y = round(u * width), round(v * height)
    seen: dict[str, Any] = {"pixel": [x, y], "patch": [reach_x, reach_y],
                            "expected_m": round(xyz[2], 3),
                            "margin_m": round(margin, 3), "rise_m": round(rise, 3)}
    if x - reach_x < 1 or y - reach_y < 1 or x + reach_x >= width - 1 \
            or y + reach_y >= height - 1:
        return {**seen, "state": "outside"}
    values = []
    for row in range(y - reach_y, y + reach_y + 1):
        start = (row * width + x - reach_x) * 2
        stop = (row * width + x + reach_x + 1) * 2
        chunk = millimetres[start:stop]
        values.extend(int.from_bytes(chunk[i:i + 2], "little") / 1000.0
                      for i in range(0, len(chunk), 2))
    valid = sorted(value for value in values if value > 0.15)
    seen["share"] = round(len(valid) / max(1, len(values)), 3)
    if not valid or len(valid) < DEPTH_SHARE * len(values):
        return {**seen, "state": "sparse"}
    nearest = valid[0]
    middle = valid[len(valid) // 2]
    seen.update({"nearest_m": round(nearest, 3), "middle_m": round(middle, 3)})
    if nearest > xyz[2] + margin + SEEN_PAST_M:
        return {**seen, "state": "clear"}
    if middle < xyz[2] - margin:
        return {**seen, "state": "occluded"}
    return {**seen, "state": "surface"}


def _lens_from(described: dict[str, Any]):
    """The lens a depth map was saved with, when it was saved with one."""
    lens = described.get("lens") if isinstance(described, dict) else None
    if not isinstance(lens, dict):
        return None
    try:
        return SimpleNamespace(**{k: float(lens[k]) for k in
                                  ("fx", "fy", "cx", "cy", "width", "height")})
    except (KeyError, TypeError, ValueError):
        return None


def _rounded(claim: dict[str, Any]) -> dict[str, Any]:
    return {k: (round(v, 3) if isinstance(v, float) else v)
            for k, v in claim.items()}
